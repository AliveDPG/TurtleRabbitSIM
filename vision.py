"""
vision.py - Binary threshold + blob finding for the shuttle cameras, and the
camera geometry needed to turn a blob centroid into a world-frame bearing.

Pure numpy; scipy.ndimage.label is used for connected components when it is
available (the sim's Python ships scipy) with a numpy/BFS fallback.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
import numpy.typing as npt

try:
    from scipy import ndimage as _ndi
except Exception:        # pragma: no cover
    _ndi = None


@dataclass
class Blob:
    cx: float           # centroid column (px, 0 = left)
    cy: float           # centroid row    (px, 0 = top)
    area: int           # px
    width: float        # px (bbox)
    height: float       # px (bbox)
    touches_edge: bool
    mean_intensity: float


# ---------------------------------------------------------------------------
# Image processing
# ---------------------------------------------------------------------------
def to_gray(r: npt.NDArray, g: npt.NDArray, b: npt.NDArray) -> npt.NDArray:
    return (0.299 * r + 0.587 * g + 0.114 * b).astype(np.float32)


def auto_threshold(gray: npt.NDArray, floor: float = 30.0) -> float:
    """
    Space background is near black and lit debris is bright, so a fixed
    threshold works; this nudges it up if the frame is unusually bright
    (e.g. Phobos' surface or the ship in view).
    """
    med = float(np.median(gray))
    return max(floor, med + 25.0)


def _label_numpy(mask: npt.NDArray) -> tuple[npt.NDArray, int]:
    """4-connected labelling without scipy (BFS, fine for sparse masks)."""
    h, w = mask.shape
    labels = np.zeros((h, w), dtype=np.int32)
    n = 0
    ys, xs = np.nonzero(mask)
    for y0, x0 in zip(ys, xs):
        if labels[y0, x0]:
            continue
        n += 1
        stack = [(y0, x0)]
        labels[y0, x0] = n
        while stack:
            y, x = stack.pop()
            for ny, nx in ((y - 1, x), (y + 1, x), (y, x - 1), (y, x + 1)):
                if 0 <= ny < h and 0 <= nx < w and mask[ny, nx] and not labels[ny, nx]:
                    labels[ny, nx] = n
                    stack.append((ny, nx))
    return labels, n


def find_blobs(gray: npt.NDArray, threshold: float = None, min_area: int = 3,
               max_area_frac: float = 0.15, max_mask_frac: float = 0.35) -> list[Blob]:
    """
    Threshold `gray` and return connected bright regions.
    Frames dominated by a bright surface (mask fraction > max_mask_frac) are
    skipped: that is Phobos / the ship, not debris.
    """
    if threshold is None:
        threshold = auto_threshold(gray)
    mask = gray > threshold
    h, w = mask.shape
    if mask.mean() > max_mask_frac or not mask.any():
        return []
    if _ndi is not None:
        labels, n = _ndi.label(mask)
    else:
        labels, n = _label_numpy(mask)
    if n == 0:
        return []
    idx = np.arange(1, n + 1)
    areas = np.bincount(labels.ravel(), minlength=n + 1)[1:]
    ys, xs = np.mgrid[0:h, 0:w]
    sum_y = np.bincount(labels.ravel(), weights=ys.ravel(), minlength=n + 1)[1:]
    sum_x = np.bincount(labels.ravel(), weights=xs.ravel(), minlength=n + 1)[1:]
    sum_i = np.bincount(labels.ravel(), weights=gray.ravel(), minlength=n + 1)[1:]
    blobs = []
    max_area = max_area_frac * h * w
    for k, a in zip(idx, areas):
        if a < min_area or a > max_area:
            continue
        sel = labels == k
        rows = np.nonzero(sel.any(axis=1))[0]
        cols = np.nonzero(sel.any(axis=0))[0]
        y0, y1 = rows[0], rows[-1]
        x0, x1 = cols[0], cols[-1]
        blobs.append(Blob(cx=sum_x[k - 1] / a, cy=sum_y[k - 1] / a, area=int(a),
                          width=float(x1 - x0 + 1), height=float(y1 - y0 + 1),
                          touches_edge=bool(y0 == 0 or x0 == 0 or y1 == h - 1 or x1 == w - 1),
                          mean_intensity=float(sum_i[k - 1] / a)))
    blobs.sort(key=lambda b: -b.area)
    return blobs


# ---------------------------------------------------------------------------
# Camera geometry
# ---------------------------------------------------------------------------
def focal_px(width_px: int, fov_deg: float) -> float:
    return 0.5 * width_px / math.tan(0.5 * math.radians(fov_deg))


def pixel_to_dir_cam(u: float, v: float, width: int, height: int, fov_deg: float,
                     right_is_minus_y: bool = True) -> npt.NDArray:
    """
    Unit bearing in the camera frame for pixel (u right, v down).
    Camera frame == shuttle body frame when yaw = pitch = 0: +x forward,
    +y left, +z up (positive camera yaw turns LEFT, per the wiki).  Image
    right therefore maps to -y and image down to -z.  Flip
    `right_is_minus_y` if the rendered image turns out mirrored.
    """
    f = focal_px(width, fov_deg)
    tx = (u - 0.5 * (width - 1)) / f
    ty = (v - 0.5 * (height - 1)) / f
    d = np.array([1.0, -tx if right_is_minus_y else tx, -ty])
    return d / np.linalg.norm(d)


def pixel_ang_sigma(width: int, fov_deg: float, px_sigma: float = 1.5) -> float:
    return px_sigma / focal_px(width, fov_deg)


def cam_to_body_dcm(yaw_deg: float, pitch_deg: float) -> npt.NDArray:
    """
    Wiki: T_body^cam = Ry(pitch) @ Rz(yaw) (3-2-1, zero roll), v_cam = T v_body.
    Returns the matrix that takes a camera-frame vector to the body frame.
    """
    psi, th = math.radians(yaw_deg), math.radians(pitch_deg)
    Rp = np.array([[math.cos(th), 0.0, -math.sin(th)],
                   [0.0, 1.0, 0.0],
                   [math.sin(th), 0.0, math.cos(th)]])
    Ry = np.array([[math.cos(psi), math.sin(psi), 0.0],
                   [-math.sin(psi), math.cos(psi), 0.0],
                   [0.0, 0.0, 1.0]])
    T_body_to_cam = Rp @ Ry
    return T_body_to_cam.T


def body_dir_to_yaw_pitch(d_body: npt.NDArray) -> tuple[float, float]:
    """Camera yaw/pitch (deg) that points the boresight along a body-frame direction."""
    d = d_body / np.linalg.norm(d_body)
    yaw = math.degrees(math.atan2(d[1], d[0]))
    pitch = -math.degrees(math.asin(np.clip(d[2], -1.0, 1.0)))   # positive pitch = down
    return yaw, pitch
