"""Pad watch: camera views for the rocks that rain onto the pad area during the launch.

The normal views follow the flight direction (up, at launch), but every rock that came within 30 m
of the shuttle in the first 40 s arrived from 30-95 deg off local up - a ring just above the
horizon - or from overhead.  While the shuttle is near the pad the controller therefore interleaves
wide views aimed at a ring of world azimuths around local up with its flight views.
"""
import math

import numpy as np

from depth_vision import boresight_body

PATTERN = "PF"          # per frame: P = next pad view, F = next flight view
N_AZ = 4                # ring azimuths: four 100-deg views tile the ring with ~10 deg overlap
EL_DEG = 30.0           # ring elevation above the local horizon
FOV, RES = 100.0, 256
MAX_OFF_DEG = 40.0      # skip a view whose clipped boresight is farther than this from its direction
                        # (a 100-deg view still covers a direction 40 deg off its centre)


def cam_for(u_b):
    """(camera, yaw, pitch) of the pan-tilt pointing closest to body direction u_b, clipped to the limits."""
    u = np.asarray(u_b, float) / max(float(np.linalg.norm(u_b)), 1e-9)
    yaw = math.degrees(math.atan2(float(u[1]), float(u[0])))
    pitch = -math.degrees(math.asin(float(np.clip(u[2], -1, 1))))
    yaw = float(np.clip(yaw, -89.0, 89.0))
    if u[2] >= 0.0:
        return "top", yaw, float(np.clip(pitch, -50.0, 0.0))
    return "bottom", yaw, float(np.clip(pitch, 0.0, 50.0))


def ring_dirs(p_world, R_w2b):
    """World unit directions of the pad ring at the shuttle's position (local up = radial).  The ring's
    reference azimuth follows the nose, so the half the cameras can reach comes first."""
    el = math.radians(EL_DEG)
    up = np.asarray(p_world, float) / max(float(np.linalg.norm(p_world)), 1e-9)
    ref = None
    for row in (R_w2b[0], R_w2b[1], R_w2b[2]):          # nose first, else body y, else body z
        h = row - float(row @ up) * up
        if float(np.linalg.norm(h)) > 0.3:
            ref = h / float(np.linalg.norm(h))
            break
    if ref is None:
        ref = np.cross(up, [1.0, 0.0, 0.0])
        ref = ref / max(float(np.linalg.norm(ref)), 1e-9)
    side = np.cross(up, ref)
    out = []
    for k in range(N_AZ):
        a = 2.0 * math.pi * k / N_AZ
        h = math.cos(a) * ref + math.sin(a) * side
        out.append(math.cos(el) * h + math.sin(el) * up)
    return out


class PadViews:
    """State of the pad-view cycle: which ring direction comes next and where the pattern is."""

    def __init__(self):
        self.k = 0               # ring index
        self.slot = 0            # pattern index

    def next_is_pad(self):
        c = PATTERN[self.slot % len(PATTERN)]
        self.slot += 1
        return c == "P"

    def next_view(self, p_world, R_w2b):
        """(camera, yaw, pitch, fov, res) of the next reachable pad view, or None when none is reachable."""
        dirs = ring_dirs(p_world, R_w2b)
        for _ in range(len(dirs)):
            u_b = R_w2b @ dirs[self.k % len(dirs)]
            self.k += 1
            cam, y, pt = cam_for(u_b)
            off = math.degrees(math.acos(float(np.clip(boresight_body(y, pt) @ (u_b / np.linalg.norm(u_b)), -1, 1))))
            if off <= MAX_OFF_DEG:
                return cam, y, pt, FOV, RES
        return None
