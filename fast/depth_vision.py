"""Depth-camera vision: rock detection in depth images, camera tracks, and their fusion with the radar.

Camera facts, calibrated in the sim:
  * a Depth_cm capture is a row-major res x res float image of planar depth (along the boresight) in
    centimetres; empty sky reads 1e8; rocks agree with their true range to 0.3-0.9 %;
  * camera.getRotation().DCM_WRT(frame) maps world -> camera; camera x = boresight, y = image left,
    z = image up; row 0 is the top of the image;
  * PanTiltCamera(yaw, pitch) puts the boresight at (cos p cos y, cos p sin y, -sin p) in body axes.
"""
import math
from collections import namedtuple

import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree

I3 = np.eye(3)


class Detection:
    __slots__ = ("t", "center", "radius", "range", "edge")

    def __init__(self, t, center, radius, rng, edge):
        self.t, self.center, self.radius, self.range, self.edge = t, center, radius, rng, edge


def _split(points, ray, depth, out, foot, ratio=2.6):
    """Bounding spheres (centroid, radius, pixel count) for one pixel cluster of world points, split while it is
    a chain of rocks receding in depth: spread along the ray much larger than across it."""
    m = points.mean(axis=0)
    q = points - m
    r = float(np.sqrt((q ** 2).sum(axis=1)).max()) + foot
    if depth < 5 and len(points) >= 8 and r > 4.0:
        along = q @ ray
        lat = q - np.outer(along, ray)
        s_al = float(along.std())
        s_lat = float(np.sqrt((lat ** 2).sum(axis=1).mean()))
        if s_al > ratio * max(s_lat, 0.5) or r > 30.0:
            side = along > 0
            if 2 <= side.sum() <= len(points) - 2:
                _split(points[side], ray, depth + 1, out, foot, ratio)
                _split(points[~side], ray, depth + 1, out, foot, ratio)
                return
    out.append((m, r, len(points)))


def detect_rocks(depth_m, fov_deg, cam_pos, R_w2c, t, max_range, min_range, ground_fn):
    """Segment a planar-depth image into rock detections (world-frame sphere estimates).
    ground_fn(center, radius) -> True rejects a blob (terrain, structures, the ship, our own hull)."""
    H, W = depth_m.shape
    f = (W / 2.0) / math.tan(math.radians(fov_deg) / 2.0)
    D = depth_m
    mask = (D > min_range) & (D < max_range)
    if not mask.any():
        return []
    # Break the mask at depth discontinuities.  The threshold scales with the pixel footprint d/f, so the steep
    # limb of one rock stays connected while a rock 4 m behind another separates; at every jump the FAR pixel is
    # removed, which disconnects the two components.
    Dm = np.where(mask, D, np.inf)
    jump = np.zeros_like(mask)
    with np.errstate(invalid="ignore"):
        for a, b, sa, sb in ((Dm[:, 1:], Dm[:, :-1], (slice(None), slice(1, None)), (slice(None), slice(None, -1))),
                             (Dm[1:, :], Dm[:-1, :], (slice(1, None), slice(None)), (slice(None, -1), slice(None)))):
            near = np.minimum(a, b)
            big = np.abs(a - b) > (1.0 + 4.0 * near / f + 0.01 * near)
            jump[sa] |= big & np.isfinite(a) & (a > b)
            jump[sb] |= big & np.isfinite(b) & (b > a)
    lbl, n = ndimage.label(mask & ~jump, structure=np.ones((3, 3)))
    if n == 0:
        return []
    vv, uu = np.nonzero(lbl)
    L = lbl[vv, uu]
    d = D[vv, uu]
    pts_c = np.column_stack([d, -((uu + 0.5) - W / 2.0) / f * d, -((vv + 0.5) - H / 2.0) / f * d])
    pts_w = cam_pos + pts_c @ R_w2c                       # (R_c2w @ p).T == p @ R_w2c
    edge_px = (uu <= 0) | (vv <= 0) | (uu >= W - 1) | (vv >= H - 1)
    order = np.argsort(L, kind="stable")
    bounds = np.searchsorted(L[order], np.arange(1, n + 2))
    out = []
    for k in range(n):
        ix = order[bounds[k]:bounds[k + 1]]
        if len(ix) == 0:
            continue
        P = pts_w[ix]
        dk = d[ix]
        ray = P.mean(axis=0) - cam_pos
        ray = ray / max(float(np.linalg.norm(ray)), 1e-6)
        foot = float(dk.mean()) / f
        edge = bool(edge_px[ix].any())
        parts = []
        _split(P, ray, 0, parts, foot)
        for m, r, npix in parts:
            # radius: the silhouette-area equivalent (median 1.00 x the actual radius on flight frames), but never
            # below 0.7 x the farthest point, which keeps merged or elongated blobs covered
            r_area = math.sqrt(npix / math.pi) * foot
            r = max(1.1 * r_area, 0.7 * r, foot)
            center = m + ray * (0.4 * r)                 # the visible-surface centroid lies in front of the centre
            if ground_fn(center, r):
                continue
            out.append(Detection(t, center, r, float(dk.min()), edge))
    return out


def _cv_prop(x, P, dt, sigma_acc):
    """Constant-velocity propagation of a 6-state (x, P) by dt with white-acceleration noise."""
    x = np.asarray(x, float)
    P = np.asarray(P, float)
    if dt <= 0.0:
        return x.copy(), P.copy()
    F = np.eye(6)
    F[:3, 3:] = I3 * dt
    q = sigma_acc ** 2
    Q = np.zeros((6, 6))
    Q[:3, :3] = I3 * q * dt ** 4 / 4
    Q[:3, 3:] = I3 * q * dt ** 3 / 2
    Q[3:, :3] = I3 * q * dt ** 3 / 2
    Q[3:, 3:] = I3 * q * dt ** 2
    return F @ x, F @ P @ F.T + Q


def _sig_pv(P):
    d = np.diag(P)
    return math.sqrt(max(float(d[:3].max()), 0.0)), math.sqrt(max(float(d[3:].max()), 0.0))


class CamTrack:
    """Constant-velocity Kalman track [p, v] of one rock seen by the cameras."""
    _next = 0
    # Rocks get knocked about by rock-rock contacts: on flight frames a responsive filter predicts 5 s ahead
    # about twice as well as a stiff one.
    SIGMA_ACC = 0.3
    FRAG_RATIO = 0.6        # a detection below this share of the track's radius is a fragment of the rock
    R_MAX = 0.0             # radius cap; the controller sets it from the largest diameter the radar reports

    def __init__(self, det, sigma_v0=6.0):
        self.id = CamTrack._next
        CamTrack._next += 1
        self.t = det.t
        self.x = np.hstack([det.center, np.zeros(3)])
        s_pos = 1.0 + 0.5 * det.radius + 0.01 * det.range
        self.P = np.diag([s_pos ** 2] * 3 + [sigma_v0 ** 2] * 3)
        self.radius = det.radius
        self.n = 1                       # sightings
        self.last_update = det.t
        self.t0 = det.t                  # first sighting
        self.t_kick_seen = -1e9          # radar contact time this track's velocity was last re-opened for ...
        self.n_kick = 0                  # ... and its sighting count then

    @property
    def pos(self):
        return self.x[:3]

    @property
    def vel(self):
        return self.x[3:]

    def update(self, det, cam_pos, pixel_rad):
        if det.t < self.t - 1e-6:
            return
        if det.t > self.t:
            self.x, self.P = _cv_prop(self.x, self.P, det.t - self.t, CamTrack.SIGMA_ACC)
            self.t = det.t
        ray = det.center - cam_pos
        rr = max(float(np.linalg.norm(ray)), 1.0)
        u = ray / rr
        s_along = 0.5 + 0.35 * det.radius + 0.005 * rr          # centre-depth uncertainty (visible-surface offset)
        s_lat = 0.3 + 0.25 * det.radius + rr * pixel_rad         # centroid wander as the rock turns / is clipped
        if det.edge:
            s_lat += det.radius
            s_along += det.radius
        if self.n >= 3 and det.radius < CamTrack.FRAG_RATIO * self.radius:
            # a big rock near by splits into several blobs; a fragment's centroid sits anywhere on the rock, so
            # it must not pull the track
            extra = self.radius - det.radius
            s_lat += extra
            s_along += 0.5 * extra
        R = s_along ** 2 * np.outer(u, u) + s_lat ** 2 * (I3 - np.outer(u, u))
        S = self.P[:3, :3] + R
        K = self.P[:, :3] @ np.linalg.inv(S)
        self.x = self.x + K @ (det.center - self.x[:3])
        self.P = self.P - K @ self.P[:3, :]
        self.P = 0.5 * (self.P + self.P.T)
        # size: running average of unclipped views (a clipped view only ever grows it), capped at the biggest
        # rock the radar has reported (merged views would otherwise ratchet it up)
        if det.edge:
            if det.radius > self.radius:
                self.radius = 0.8 * self.radius + 0.2 * det.radius
        else:
            w = 0.5 if self.n <= 3 else 0.2
            self.radius = (1 - w) * self.radius + w * det.radius
        if CamTrack.R_MAX > 0:
            self.radius = min(self.radius, CamTrack.R_MAX)
        self.n += 1
        self.last_update = det.t

    def pos_at(self, t):
        return self.x[:3] + self.x[3:] * (t - self.t)

    def sigma_pos(self):
        return math.sqrt(max(float(np.diag(self.P)[:3].max()), 0.0))

    def sigma_vel(self):
        return math.sqrt(max(float(np.diag(self.P)[3:].max()), 0.0))


class CamTracker:
    """Camera-only rock tracks; association is greedy, nearest-first by gate-normalised distance."""
    GATE_C, GATE_R, GATE_K = 3.0, 0.5, 2.0    # gate = C + R x the larger radius + K x the predicted sigma
    SIZE_GATE = 1.8                           # largest radius ratio of a detection to its track
    ABSORB_MIN_N, ABSORB_PAD = 3, 2.0         # a detection within radius + pad of a track this mature is a fragment
    MERGE_K = 0.9                             # a younger track centred within this x the radius of an older one is a duplicate

    def __init__(self, max_age_s):
        self.tracks = []
        self.max_age_s = max_age_s

    def ingest(self, dets, cam_pos, pixel_rad):
        if not dets:
            return
        t = dets[0].t
        Z = np.array([d.center for d in dets])
        Rd = np.array([d.radius for d in dets])
        assigned = np.full(len(dets), -1)
        absorbed = np.zeros(len(dets), bool)
        if self.tracks:
            X = np.array([tr.x for tr in self.tracks])
            dt = np.maximum(t - np.array([tr.t for tr in self.tracks]), 0.0)
            Pd = np.array([np.diag(tr.P) for tr in self.tracks])
            pred = X[:, :3] + X[:, 3:] * dt[:, None]
            sig = np.sqrt(Pd[:, :3].max(axis=1) + Pd[:, 3:].max(axis=1) * dt ** 2)
            Rt = np.array([tr.radius for tr in self.tracks])
            dist = np.linalg.norm(pred[:, None, :] - Z[None, :, :], axis=2)          # tracks x dets
            gate = self.GATE_C + self.GATE_R * np.maximum(Rt[:, None], Rd[None, :]) + self.GATE_K * sig[:, None]
            score = dist / gate
            ratio = np.maximum(Rt[:, None], Rd[None, :]) / np.maximum(np.minimum(Rt[:, None], Rd[None, :]), 0.5)
            score = np.where(ratio > self.SIZE_GATE, np.inf, score)
            ti, di = np.nonzero(score < 1.0)
            used_t = set()
            for k in np.argsort(score[ti, di]):
                a_, b_ = int(ti[k]), int(di[k])
                if a_ in used_t or assigned[b_] >= 0:
                    continue
                used_t.add(a_)
                assigned[b_] = a_
            # an unassigned detection inside a mature track's sphere is a piece of that rock, not a new one
            Nn = np.array([tr.n for tr in self.tracks])
            inside = (dist < Rt[:, None] + self.ABSORB_PAD) & (Rd[None, :] <= 1.2 * Rt[:, None]) \
                & (Nn[:, None] >= self.ABSORB_MIN_N)
            absorbed = (assigned < 0) & inside.any(axis=0)
        tracks = self.tracks
        for j, det in enumerate(dets):
            if assigned[j] >= 0:
                tracks[assigned[j]].update(det, cam_pos, pixel_rad)
            elif not absorbed[j]:
                self.tracks.append(CamTrack(det))

    def prune(self, t):
        self.tracks = [tr for tr in self.tracks
                       if (t - tr.last_update) < self.max_age_s or ((t - tr.last_update) < 2 * self.max_age_s and tr.n >= 4)]
        if len(self.tracks) > 1:
            # duplicates: two tracks on top of each other with similar velocity -> keep the more sighted one
            X = np.array([tr.pos_at(t) for tr in self.tracks])
            Vv = np.array([tr.vel for tr in self.tracks])
            Rr = np.array([tr.radius for tr in self.tracks])
            Nn = np.array([tr.n for tr in self.tracks])
            D = np.linalg.norm(X[:, None, :] - X[None, :, :], axis=2)
            DV = np.linalg.norm(Vv[:, None, :] - Vv[None, :, :], axis=2)
            close = (D < 0.6 * np.maximum(Rr[:, None], Rr[None, :]) + 1.0) & (DV < 2.5)
            # a young duplicate inside an older track's sphere has no converged velocity (3-6 m/s errors), so
            # its velocity test is widened by its own sigma
            Sv = np.array([tr.sigma_vel() for tr in self.tracks])
            Rbig = np.maximum(Rr[:, None], Rr[None, :])
            close |= (D < self.MERGE_K * Rbig + 1.0) & (DV < 2.5 + 2.0 * np.maximum(Sv[:, None], Sv[None, :]))
            np.fill_diagonal(close, False)
            drop = set()
            for i, j in zip(*np.nonzero(np.triu(close))):
                if i in drop or j in drop:
                    continue
                drop.add(j if Nn[i] >= Nn[j] else i)
            if drop:
                self.tracks = [tr for k, tr in enumerate(self.tracks) if k not in drop]


# ---------------------------------------------------------------------------------------------- radar / camera fusion
# The radar gives every rock to 3 m with a 0.02 m/s range rate and a 5 % diameter; the cameras see the same rocks,
# accurately across the line of sight.  The planner sums hit probabilities over obstacles, so a rock must appear
# once: fuse_obstacles associates camera tracks to radar tracks and presents one obstacle per radar rock.
ASSOC_POS_PAD, ASSOC_POS_K = 3.0, 2.0      # camera centre within R_radar + pad + k x sqrt(sp_r^2 + (lat_k sp_c)^2) of the radar centre
ASSOC_LAT_K = 0.3                          # a camera track's largest sigma is its depth sigma; across the line of sight it is ~0.3 x
ASSOC_VEL_PAD, ASSOC_VEL_K = 0.5, 3.0      # velocities within pad + k x sqrt(sv_r^2 + sv_c^2) ...
ASSOC_SV_FLOOR = 0.6                       # ... with this floor on the radar's velocity sigma (quiet tracks claim ~0.15 m/s)
ASSOC_SIZE_K, ASSOC_SIZE_PAD = 2.5, 6.0    # camera radius <= k x R_radar + pad: an oversized blob on a radar rock is an inflated estimate
FUSE_MIN_N = 4                             # the camera state enters the fused state only from a track with this many sightings,
FUSE_SIZE_K, FUSE_SIZE_PAD = 1.5, 2.0      # ... whose radius agrees with the radar's,
FUSE_CHI2_MAX = 40.0                       # ... and whose state agrees with the radar's (6-dof Mahalanobis)
FUSE_KC = 0.25                             # camera covariance scale in the fusion (its filter sigma is 2-3 x too wide)
FUSE_MAX_SVR = 1.2                         # m/s: a radar track whose velocity sigma is above this was opened by a contact;
FUSE_YOUNG_CAM_N = 60                      # ... it fuses only with a camera track of at most this many sightings, because
                                           # an older one still carries the velocity from before the contact
RADAR_SIGMA_ACC = 0.02                     # process noise used to bring a radar track's covariance to the fusion time
# Contact hand-over.  The radar sees a rock-rock contact at its first return afterwards and re-opens its velocity.
# A camera track born before the contact keeps the old velocity at a claimed sigma of ~0.1 m/s and would dominate
# the fused state (measured: 2.3 m/s of velocity error decaying over 9 s).  The contact is handed to that track once:
# velocity := the radar's, velocity sigma := KICK_SV_K x the radar's (at least KICK_SV_MIN), no position-velocity
# correlation.  It then re-learns the velocity from its next frames.
KICK_SV_K, KICK_SV_MIN = 2.0, 0.5
KICK_ONCE_S = 12.0                         # a track told within this time is already re-learning
KICK_MIN_NEW = 2                           # sightings after the hand-over before the track counts as young again

Obstacle = namedtuple("Obstacle", "name src pos vel radius n sp svf age kick_age")
# src: 'f' radar fused with a camera track, 'r' radar only, 'c' camera only; n: radar returns ('f', 'r') or camera
# sightings ('c'); sp / svf: filter position / velocity sigma (largest axis); age: s since the last camera update
# ('c'); kick_age: s since the last contact seen on the rock


def fuse_obstacles(radar_tracks, cam_tracks, t, p, min_hits_far, near_m):
    """Obstacles at time t with one obstacle per radar rock: the information-weighted fusion of the radar state
    and the best camera track on it, with the radar's radius.  Every other camera track on the rock (fragments,
    inflated blobs, young duplicates) is absorbed; camera tracks on no radar rock stand alone, de-duplicated.
    Camera tracks with fewer than min_hits_far sightings count only within near_m of the shuttle at p."""
    p = np.asarray(p, float)
    rad = [(tr, np.hstack([tr.pos_at(t), tr.vel])) for tr in radar_tracks]
    cams = [tr for tr in cam_tracks if not (tr.n < min_hits_far and float(np.linalg.norm(tr.pos - p)) > near_m)]
    assigned = {}
    free = []
    if rad and cams:
        RP = np.array([x[:3] for _, x in rad])
        RV = np.array([x[3:] for _, x in rad])
        RR = np.array([float(tr.radius) for tr, _ in rad])
        SR = np.array([_sig_pv(tr.P) for tr, _ in rad])
        CP = np.array([tr.pos_at(t) for tr in cams])
        CV = np.array([tr.vel for tr in cams])
        CR = np.array([float(tr.radius) for tr in cams])
        SPC = np.array([tr.sigma_pos() for tr in cams])
        SVC = np.array([tr.sigma_vel() for tr in cams])
        D = np.linalg.norm(CP[:, None, :] - RP[None, :, :], axis=2) - RR[None, :]          # (Nc, Nr) centre gap
        gate_p = ASSOC_POS_PAD + ASSOC_POS_K * np.sqrt(SR[None, :, 0] ** 2 + (ASSOC_LAT_K * SPC[:, None]) ** 2)
        ok = D < gate_p
        DV = np.linalg.norm(CV[:, None, :] - RV[None, :, :], axis=2)
        ok &= DV < ASSOC_VEL_PAD + ASSOC_VEL_K * np.sqrt(np.maximum(SR[None, :, 1], ASSOC_SV_FLOOR) ** 2 + SVC[:, None] ** 2)
        ok &= CR[:, None] <= ASSOC_SIZE_K * RR[None, :] + ASSOC_SIZE_PAD
        score = np.where(ok, D / gate_p, np.inf)
        best = np.argmin(score, axis=1)
        for i, tr in enumerate(cams):
            j = int(best[i])
            if np.isfinite(score[i, j]):
                assigned.setdefault(j, []).append(tr)
            else:
                free.append(tr)
    else:
        free = cams
    out = []
    for j, (tr, x) in enumerate(rad):
        name = f"r{tr.id}"
        lst = assigned.get(j)
        if not lst:
            out.append(Obstacle(name, "r", x[:3], x[3:], float(tr.radius), int(tr.hits), tr.pos_sigma(), tr.vel_sigma(), 0.0, 1e9))
            continue
        x, P = _cv_prop(tr.x, tr.P, t - tr.t, RADAR_SIGMA_ACC)
        sv_r = _sig_pv(P)[1]
        tk = tr.t_kick
        if tk > -1e8:
            for q in lst:
                # Only a track born before the contact can carry the old velocity, and each track is told once per
                # contact: one told recently is already re-learning and a second, late radar solution must not
                # overwrite it.
                if q.t0 < tr.t_kick_from and q.t_kick_seen < tk - KICK_ONCE_S:
                    sv = max(KICK_SV_K * sv_r, KICK_SV_MIN)
                    if tr.kick_solved:
                        q.x[3:] = x[3:]
                        q.P[:3, 3:] = 0.0
                        q.P[3:, :3] = 0.0
                        q.P[3:, 3:] = I3 * sv ** 2
                    else:                               # range-rate jump: the radar does not know the new velocity either
                        q.P[3:, 3:] = q.P[3:, 3:] + I3 * sv ** 2
                    q.t_kick_seen = tk
                    q.n_kick = q.n
        lst.sort(key=lambda q: -q.n)
        opened = sv_r > FUSE_MAX_SVR
        fused = None
        for q in lst:
            if opened and q.n > FUSE_YOUNG_CAM_N and not (t - q.t_kick_seen < KICK_ONCE_S and q.n - q.n_kick >= KICK_MIN_NEW):
                continue
            if q.n < FUSE_MIN_N or q.radius > FUSE_SIZE_K * tr.radius + FUSE_SIZE_PAD:
                continue
            xc, Pc = _cv_prop(q.x, q.P, t - q.t, CamTrack.SIGMA_ACC)
            Pc = Pc * FUSE_KC
            dx = xc - x
            try:
                d2 = float(dx @ np.linalg.solve(P + Pc, dx))
                if not (d2 <= FUSE_CHI2_MAX):
                    continue
                Ir = np.linalg.inv(P)
                Ic = np.linalg.inv(Pc)
                Pf = np.linalg.inv(Ir + Ic)
                Pf = 0.5 * (Pf + Pf.T)
                xf = Pf @ (Ir @ x + Ic @ xc)
            except np.linalg.LinAlgError:
                continue
            if not (np.all(np.isfinite(xf)) and np.all(np.isfinite(Pf))):
                continue
            fused = q
            break
        if fused is None:
            # camera tracks absorbed, none fit to fuse: the radar obstacle stands for the rock
            out.append(Obstacle(name, "r", x[:3], x[3:], float(tr.radius), int(tr.hits), tr.pos_sigma(), sv_r, 0.0, 1e9))
            continue
        sp_f, sv_f = _sig_pv(Pf)
        out.append(Obstacle(f"{name}+c{fused.id}", "f", xf[:3], xf[3:], float(tr.radius), int(tr.hits), sp_f, sv_f, 0.0, t - tk))
    cl = [Obstacle(f"c{tr.id}", "c", tr.pos_at(t), tr.vel, float(tr.radius), int(tr.n), tr.sigma_pos(), tr.sigma_vel(),
                   max(t - tr.last_update, 0.0), t - tr.t_kick_seen) for tr in free]
    if len(cl) > 1:
        # a camera track inside a more-sighted one's sphere is a duplicate of it
        cl.sort(key=lambda o: -o.n)
        KP = np.array([o.pos for o in cl])
        KR = np.array([o.radius for o in cl])
        KN = np.array([o.n for o in cl])
        keep = np.zeros(len(cl), bool)
        for i in range(len(cl)):
            dup = keep[:i] & (KN[:i] > KN[i]) & (np.linalg.norm(KP[:i] - KP[i], axis=1) < 0.9 * np.maximum(KR[:i], KR[i]) + 1.0)
            keep[i] = not dup.any()
        cl = [o for o, k in zip(cl, keep) if k]
    return out + cl


def boresight_body(yaw_deg, pitch_deg):
    y, p = math.radians(yaw_deg), math.radians(pitch_deg)
    return np.array([math.cos(p) * math.cos(y), math.cos(p) * math.sin(y), -math.sin(p)])


def yaw_pitch_for(d_body):
    d = d_body / max(float(np.linalg.norm(d_body)), 1e-9)
    pitch = -math.degrees(math.asin(float(np.clip(d[2], -1, 1))))
    yaw = math.degrees(math.atan2(float(d[1]), float(d[0])))
    return yaw, pitch


class StaticMap:
    """Voxel map of a static structure (the target ship) built from depth points.  The ship is ~250 m across
    and the target sits inside it, so it is mapped as solid cells and the planner keeps clear of them until the
    path is inside the keep-out sphere, where the run ends."""

    def __init__(self, center, radius=200.0, voxel=4.0, min_hits=2):
        self.center = np.asarray(center, float)
        self.radius = radius
        self.voxel = voxel
        self.min_hits = min_hits
        self.cells = {}
        self._dirty = False
        self._tree = None

    def add_depth(self, depth_m, fov_deg, cam_pos, R_w2c, stride=2):
        if float(np.linalg.norm(cam_pos - self.center)) > self.radius + 500.0:
            return
        H, W = depth_m.shape
        f = (W / 2.0) / math.tan(math.radians(fov_deg) / 2.0)
        D = depth_m[::stride, ::stride]
        vv, uu = np.nonzero((D > 4.0) & (D < self.radius + 600.0))
        if len(vv) == 0:
            return
        d = D[vv, uu]
        u = uu * stride + 0.5
        v = vv * stride + 0.5
        P = np.column_stack([d, -(u - W / 2.0) / f * d, -(v - H / 2.0) / f * d]) @ R_w2c + cam_pos
        near = np.linalg.norm(P - self.center, axis=1) < self.radius
        if not near.any():
            return
        keys = np.floor((P[near] - self.center) / self.voxel).astype(np.int32)
        uk, cnt = np.unique(keys, axis=0, return_counts=True)
        for k, c in zip(map(tuple, uk), cnt):
            self.cells[k] = self.cells.get(k, 0) + int(c)
        self._dirty = True

    def tree(self):
        """KD-tree of the centres of the cells seen at least min_hits times, or None while there are none."""
        if self._dirty:
            ks = [k for k, c in self.cells.items() if c >= self.min_hits]
            self._tree = cKDTree((np.array(ks, float) + 0.5) * self.voxel + self.center) if ks else None
            self._dirty = False
        return self._tree
