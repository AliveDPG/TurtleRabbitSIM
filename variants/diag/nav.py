"""
nav.py - Navigation, tracking and guidance for the Escape-from-Phobos shuttle.

Pure numpy (no spaceteams import) so every piece can be unit-tested offline
with sim_offline.py.  Controller.py is the thin glue to the simulator.

Contents
  ShuttleNav     6-state KF that fuses the sim state fix with the thruster
                 accelerometer (JetSelection/ResultantAccel) and keeps a
                 time history so radar returns can be de-skewed to world frame.
  CVTrack        constant-velocity Kalman track of one piece of debris,
                 kept in the Phobos-fixed (world) frame.
  DebrisTracker  association + track management for radar and camera data.
  Guidance       path/velocity decomposition along the start->target line,
                 tube (corridor) filter, closest-approach conflict check and
                 the "one burn, coast, trim only when worth it" command law.

All positions/velocities are in the Phobos-fixed frame unless stated.
Hot paths (association, tube filter, closest approach) are vectorised so the
60 Hz control loop stays well under its budget with 200+ tracks.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field

import numpy as np
import numpy.typing as npt

I3 = np.eye(3)


def unit(v: npt.NDArray) -> npt.NDArray:
    n = np.linalg.norm(v)
    return v / n if n > 1e-12 else np.zeros(3)


def perp_basis(axis: npt.NDArray) -> tuple[npt.NDArray, npt.NDArray]:
    """Two unit vectors spanning the plane perpendicular to `axis`."""
    a = unit(axis)
    helper = np.array([0.0, 0.0, 1.0]) if abs(a[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    e1 = unit(np.cross(a, helper))
    e2 = np.cross(a, e1)
    return e1, e2


def _cv_FQ(dt: float, sigma_acc: float):
    F = np.eye(6)
    F[:3, 3:] = I3 * dt
    q = sigma_acc ** 2
    Q = np.block([[q * dt ** 4 / 4 * I3, q * dt ** 3 / 2 * I3],
                  [q * dt ** 3 / 2 * I3, q * dt ** 2 * I3]])
    return F, Q


# ---------------------------------------------------------------------------
# Shuttle navigation filter
# ---------------------------------------------------------------------------
class ShuttleNav:
    """
    Fuses the (near perfect) simulator state fix with the thruster
    accelerometer.  The sim gives truth, so the main job here is to be a
    consistent, time-stamped source of the shuttle's own state:
      * pos_at(t) is needed to turn radar returns (relative to the shuttle at
        measurement time) into world-frame positions.
      * if the organisers ever add noise/latency to the state, the KF already
        has the right structure (process = IMU, measurement = position fix).
    State x = [p(3), v(3)].
    """

    def __init__(self, sigma_fix_pos=0.3, sigma_fix_vel=0.03, sigma_acc=0.05, history_s=30.0):
        self.x = None
        self.P = None
        self.t = None
        self.R = np.diag([sigma_fix_pos ** 2] * 3 + [sigma_fix_vel ** 2] * 3)
        self.sigma_acc = sigma_acc
        self.hist: deque = deque()
        self.history_s = history_s

    def propagate(self, t: float, acc_world: npt.NDArray) -> None:
        """IMU-style propagation with the acceleration the jets actually produced."""
        if self.x is None:
            return
        dt = t - self.t
        if dt <= 0.0:
            return
        F, Q = _cv_FQ(dt, self.sigma_acc)
        B = np.vstack([0.5 * dt * dt * I3, dt * I3])
        self.x = F @ self.x + B @ acc_world
        self.P = F @ self.P @ F.T + Q
        self.t = t

    def fix(self, t: float, pos: npt.NDArray, vel: npt.NDArray) -> None:
        """Position/velocity fix from the simulator ("high quality IMU/GNSS")."""
        z = np.hstack([pos, vel])
        if self.x is None:
            self.x = z.copy()
            self.P = self.R.copy()
            self.t = t
        else:
            if t > self.t:
                self.propagate(t, np.zeros(3))
            K = self.P @ np.linalg.inv(self.P + self.R)
            self.x = self.x + K @ (z - self.x)
            self.P = (np.eye(6) - K) @ self.P
        self.hist.append((t, self.x[:3].copy(), self.x[3:].copy()))
        while self.hist and self.hist[0][0] < t - self.history_s:
            self.hist.popleft()

    @property
    def pos(self) -> npt.NDArray:
        return self.x[:3]

    @property
    def vel(self) -> npt.NDArray:
        return self.x[3:]

    def pos_at(self, t: float):
        """Shuttle position at an earlier time (linear interpolation)."""
        if not self.hist:
            return None
        if t <= self.hist[0][0]:
            t0, p0, v0 = self.hist[0]
            return p0 + v0 * (t - t0) if t0 - t < 5.0 else p0
        if t >= self.hist[-1][0]:
            t1, p1, v1 = self.hist[-1]
            return p1 + v1 * min(t - t1, 0.5)
        lo, hi = 0, len(self.hist) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if self.hist[mid][0] <= t:
                lo = mid
            else:
                hi = mid
        t0, p0, _ = self.hist[lo]
        t1, p1, _ = self.hist[hi]
        w = (t - t0) / max(t1 - t0, 1e-9)
        return p0 + w * (p1 - p0)


# ---------------------------------------------------------------------------
# Debris tracking
# ---------------------------------------------------------------------------
class CVTrack:
    """Constant-velocity Kalman track in the world frame."""
    _next_id = 0

    def __init__(self, t: float, pos: npt.NDArray, R: npt.NDArray, diameter: float,
                 sigma_v0: float = 0.05, sigma_acc: float = 0.01, source: str = "radar"):
        self.id = CVTrack._next_id
        CVTrack._next_id += 1
        self.t = t                       # time the state is valid at
        self.x = np.hstack([pos, np.zeros(3)])
        self.P = np.zeros((6, 6))
        self.P[:3, :3] = R
        self.P[3:, 3:] = I3 * sigma_v0 ** 2   # prior: debris drifts slowly
        self.sigma_acc = sigma_acc
        self.diameter = float(diameter)
        self.hits = 1
        self.last_update = t
        self.source = source

    @property
    def pos(self) -> npt.NDArray:
        return self.x[:3]

    @property
    def vel(self) -> npt.NDArray:
        return self.x[3:]

    @property
    def radius(self) -> float:
        return 0.5 * self.diameter

    def pos_at(self, t: float) -> npt.NDArray:
        return self.x[:3] + self.x[3:] * (t - self.t)

    def pos_sigma(self, t: float = None) -> float:
        """1-sigma position uncertainty (largest axis), optionally extrapolated."""
        var = np.diag(self.P)[:3]
        if t is not None and t > self.t:
            var = var + (t - self.t) ** 2 * np.diag(self.P)[3:]
        return math.sqrt(max(float(var.max()), 0.0))

    def predict_to(self, t: float) -> None:
        dt = t - self.t
        if dt <= 0.0:
            return
        F, Q = _cv_FQ(dt, self.sigma_acc)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        self.t = t

    def update_pos(self, t: float, z: npt.NDArray, R: npt.NDArray) -> None:
        """Position measurement z (3,) with covariance R (3x3) taken at time t."""
        if t < self.t - 1e-6:
            return                        # out of order; ignore
        self.predict_to(t)
        PHt = self.P[:, :3]
        S = self.P[:3, :3] + R
        K = PHt @ np.linalg.inv(S)
        self.x = self.x + K @ (z - self.x[:3])
        self.P = self.P - K @ PHt.T
        self.hits += 1
        self.last_update = t

    def update_diameter(self, d: float, weight: float = 0.3) -> None:
        if d > 0.0:
            self.diameter = (1.0 - weight) * self.diameter + weight * d


@dataclass
class Snapshot:
    """All tracks propagated to one instant, as arrays (n,3)/(n,)."""
    tracks: list
    pos: npt.NDArray
    vel: npt.NDArray
    radius: npt.NDArray
    sigma: npt.NDArray       # 1-sigma position uncertainty now
    sigma_v: npt.NDArray     # 1-sigma velocity uncertainty


def make_snapshot(tracks: list, t: float) -> Snapshot:
    n = len(tracks)
    if n == 0:
        z = np.zeros((0, 3))
        return Snapshot([], z, z, np.zeros(0), np.zeros(0), np.zeros(0))
    X = np.array([tr.x for tr in tracks])
    T = np.array([tr.t for tr in tracks])
    D = np.array([np.diag(tr.P) for tr in tracks])
    dt = np.maximum(t - T, 0.0)[:, None]
    pos = X[:, :3] + X[:, 3:] * dt
    var = D[:, :3] + dt * dt * D[:, 3:]
    return Snapshot(list(tracks), pos, X[:, 3:],
                    0.5 * np.array([tr.diameter for tr in tracks]),
                    np.sqrt(np.maximum(var.max(axis=1), 0.0)),
                    np.sqrt(np.maximum(D[:, 3:].max(axis=1), 0.0)))


class DebrisTracker:
    """
    Keeps world-frame tracks of every piece of debris the sensors report.

    Radar entries are (3x3) matrices:
        [[x, y, z],           relative position, Phobos-fixed frame, at time dt
         [r_closest, rdot, d],
         [dt, 0, 0]]
    The sorted radar list carries no IDs and each object is refreshed in a
    round-robin, so association is nearest-neighbour in the *world* frame
    (debris barely moves there).  Returns already processed are recognised by
    their time stamp, so each read only costs the ~dozen fresh entries.
    """

    def __init__(self, sigma_pos=2.0, gate_m=12.0, max_age_s=12.0, time_offset=0.0,
                 radar_lag_mean_s=1.0, radar_lag_sigma_s=0.6, adapt_lag=True):
        self.tracks: list[CVTrack] = []
        self.R_radar = I3 * sigma_pos ** 2
        # The radar builds its relative vectors from a copy of the shuttle state that is 0-2 s
        # stale (entity physics sync), while stamping fresh time: returns are displaced along our
        # velocity by v*lag.  We de-skew with the shuttle position at (stamp - mean lag), treat the
        # spread as extra measurement noise along v, and refine the mean from the residuals.
        self.radar_lag_mean_s = radar_lag_mean_s
        self.radar_lag_sigma_s = radar_lag_sigma_s
        self.adapt_lag = adapt_lag
        self._lag_resid: deque = deque(maxlen=300)
        self.sigma_pos = sigma_pos
        self.gate_m = gate_m
        self.max_age_s = max_age_s
        self.time_offset = time_offset     # radar stamp -> controller time
        self.n_radar_meas = 0
        self.n_cam_meas = 0
        self._seen: set = set()
        self._initial_done = False
        self._offset_samples = 0
        self.last_stats = {}                 # per-read association diagnostics

    # -- array views --------------------------------------------------------------
    def snapshot(self, t: float) -> Snapshot:
        return make_snapshot(self.tracks, t)

    def _associate(self, t: float, z: npt.NDArray, diameter: float, gate: float, v_hat=None, gate_along=None):
        """
        Nearest track inside an ellipsoidal gate: `gate` across our velocity, `gate_along`
        along it (radar returns carry a timing skew relative to our own state, which shows
        up as an error along the flight direction that grows with speed).
        """
        if not self.tracks:
            return None
        snap = self.snapshot(t)
        rel = snap.pos - z
        if v_hat is not None and gate_along is not None and gate_along > gate:
            along = rel @ v_hat
            perp = np.linalg.norm(rel - along[:, None] * v_hat, axis=1)
            d = np.sqrt((perp / gate) ** 2 + (along / gate_along) ** 2) * gate   # normalised to `gate`
        else:
            d = np.linalg.norm(rel, axis=1)
        diam = 2.0 * snap.radius
        size_ok = np.abs(diam - diameter) < 0.6 * np.maximum(diam, diameter) + 2.0
        d = np.where(size_ok, d, np.inf)
        j = int(np.argmin(d))
        return self.tracks[j] if d[j] < gate else None

    def prune(self, t: float) -> None:
        self.tracks = [tr for tr in self.tracks if t - tr.last_update < self.max_age_s]

    # -- radar -----------------------------------------------------------------
    def ingest_radar(self, entries, shuttle_pos_at, t_now: float = None, v_now=None) -> int:
        """
        entries: iterable of 3x3 radar matrices.
        shuttle_pos_at: callable(t) -> shuttle world position at time t.
        t_now: controller time of this read.  The radar stamps measurements
               from its *own* start time; the freshest stamp in a read lags
               t_now by at most ~0.2 s, so the running minimum of
               (t_now - newest stamp) estimates the clock offset.
        Returns number of measurements actually applied.
        """
        entries = [np.asarray(m, dtype=float) for m in entries]
        if t_now is not None and entries:
            newest = max(float(m[2, 0]) for m in entries)
            if newest > 1e-6:
                lag = t_now - newest
                if self._offset_samples == 0 or lag < self.time_offset:
                    self.time_offset = lag
                self._offset_samples += 1
        applied = 0
        st = dict(entries=len(entries), seen=0, new=0, upd=0, stale=0, nopos=0, fail_d=[])
        speed = float(np.linalg.norm(v_now)) if v_now is not None else 0.0
        v_hat = unit(v_now) if speed > 1.0 else None
        # measurement covariance: radar noise, plus the lag spread along our velocity
        R = self.R_radar.copy()
        if v_hat is not None:
            R += (self.radar_lag_sigma_s * speed) ** 2 * np.outer(v_hat, v_hat)
        resids = []
        newest_stamp = max((float(m[2, 0]) for m in entries), default=0.0)
        for m in entries:
            m = np.asarray(m, dtype=float)
            ts = float(m[2, 0]) + self.time_offset
            rel = m[0, :]
            if ts <= self.time_offset + 1e-9:              # initial scan (stamp 0)
                if self._initial_done:
                    continue
            else:
                key = (round(ts, 4), round(float(rel[0]), 2), round(float(rel[1]), 2), round(float(rel[2]), 2))
                if key in self._seen:
                    st["seen"] += 1
                    continue
                self._seen.add(key)
            diameter = max(float(m[1, 2]), 0.2)
            sp = shuttle_pos_at(ts - self.radar_lag_mean_s)      # where the radar thought we were
            if sp is None:
                st["nopos"] += 1
                continue
            z = sp + rel
            gate = self.gate_m + 0.15 * diameter
            tr = self._associate(ts, z, diameter, gate, v_hat, gate + 3.0 * self.radar_lag_sigma_s * speed)
            if tr is None:
                if self.tracks:
                    snap = self.snapshot(ts)
                    st["fail_d"].append(float(np.min(np.linalg.norm(snap.pos - z, axis=1))))
                self.tracks.append(CVTrack(ts, z, R, diameter, source="radar"))
                st["new"] += 1
            elif ts > tr.last_update + 1e-6:
                if v_hat is not None and speed > 3.0:
                    resids.append(float((z - tr.pos_at(ts)) @ v_hat) / speed)   # implied extra lag, s
                tr.update_pos(ts, z, R)
                tr.update_diameter(diameter, 0.2)
                st["upd"] += 1
            else:
                st["stale"] += 1
                continue
            applied += 1
        fd = st.pop("fail_d")
        st["fail_med_m"] = round(float(np.median(fd)), 1) if fd else None
        # refine the mean lag from the along-track residuals of successful updates
        self._lag_resid.extend(resids)
        if self.adapt_lag and len(self._lag_resid) >= 40:
            med = float(np.median(self._lag_resid))
            self.radar_lag_mean_s = float(np.clip(self.radar_lag_mean_s + 0.2 * med, 0.0, 3.0))
        st["lag_mean"] = round(self.radar_lag_mean_s, 2)
        st["resid_med_s"] = round(float(np.median(resids)), 2) if resids else None
        if self.tracks:
            V = np.array([np.linalg.norm(tr.vel) for tr in self.tracks])
            st["v_med"] = round(float(np.median(V)), 3)
            st["v_max"] = round(float(V.max()), 2)
        st["offset"] = round(self.time_offset, 3)
        st["lag_now"] = round((t_now - newest_stamp) if t_now is not None else 0.0, 3)
        self.last_stats = st
        self._initial_done = True
        if len(self._seen) > 20000:
            self._seen = set(sorted(self._seen)[-5000:])
        self.n_radar_meas += applied
        return applied

    # -- camera ----------------------------------------------------------------
    def ingest_bearing(self, t: float, origin: npt.NDArray, direction: npt.NDArray,
                       ang_sigma: float, range_m: float = None, range_sigma: float = 1.0,
                       ang_size: float = None, ang_gate: float = math.radians(2.5)):
        """
        Fuse a camera detection: a unit bearing from `origin` (world frame).
        If the camera also provides depth (`range_m`) the measurement is a full
        3-D fix; otherwise the radar track supplies the range and the camera
        tightens the bearing ("radar range + camera direction").
        Returns the updated/created track or None.
        """
        d = unit(direction)
        best = None
        if self.tracks:
            snap = self.snapshot(t)
            rel = snap.pos - origin
            r = np.linalg.norm(rel, axis=1)
            cosang = np.clip((rel @ d) / np.maximum(r, 1e-6), -1.0, 1.0)
            ang = np.arccos(cosang)
            gate = np.maximum(ang_gate, 3.0 * snap.sigma / np.maximum(r, 1.0))
            ok = (r > 1.0) & (ang < gate)
            if ok.any():
                j = int(np.argmin(np.where(ok, ang, np.inf)))
                best = snap.tracks[j]
        if best is None:
            if range_m is None:
                return None
            z = origin + range_m * d
            R = range_sigma ** 2 * np.outer(d, d) + (range_m * ang_sigma) ** 2 * (I3 - np.outer(d, d))
            diameter = max(range_m * ang_size, 0.3) if ang_size else 2.0
            tr = CVTrack(t, z, R, diameter, source="camera")
            self.tracks.append(tr)
            self.n_cam_meas += 1
            return tr
        if range_m is None:
            rel = best.pos_at(t) - origin
            range_m = float(np.linalg.norm(rel))
            range_sigma = max(math.sqrt(max(d @ best.P[:3, :3] @ d, 1e-6)), 0.5)
        z = origin + range_m * d
        R = range_sigma ** 2 * np.outer(d, d) + (range_m * ang_sigma) ** 2 * (I3 - np.outer(d, d))
        best.update_pos(t, z, R)
        if ang_size:
            best.update_diameter(range_m * ang_size, 0.15)
        self.n_cam_meas += 1
        return best


# ---------------------------------------------------------------------------
# Guidance
# ---------------------------------------------------------------------------
@dataclass
class GuidanceParams:
    a_max: float = 0.5            # m/s^2, thruster cap (sim: 0.5)
    v_cruise: float = 14.0        # m/s, single main burn. Fuel is ~0.0765 kg per m/s of delta-v and the
                                  # nominal 1.15 kg is only ~15 m/s, so cruise + trims must fit that budget
    v_arrive: float = 14.0        # m/s, speed allowed at the keep-out boundary (== v_cruise: no brake)
    keep_out_m: float = 50.0      # m, run ends inside this radius of the target
    tube_radius: float = 80.0     # m, only debris this close to the flight line is considered
    horizon_s: float = 90.0       # s, how far ahead closest approaches are evaluated
    shuttle_radius: float = 3.0   # m, shuttle bounding sphere
    margin_m: float = 4.0         # m, extra clearance
    k_sigma: float = 3.0          # clearance grows with track uncertainty
    hysteresis_m: float = 2.0     # keep trimming until this much extra clearance exists
    dv_min_trim: float = 0.04     # m/s, trims smaller than this are not worth firing for
    dv_cap: float = 4.0           # m/s, lateral delta-v per manoeuvre
    n_dirs: int = 16              # candidate lateral escape directions
    speed_deadband: float = 0.15  # m/s, along-track speed error ignored during coast
    lat_vel_deadband: float = 1.0 # m/s, cross-track drift tolerated before cleanup trim
    lat_vel_deadband_near: float = 0.3  # m/s, same, inside `near_m` of the goal
    near_m: float = 300.0
    terminal_s: float = 20.0      # s to go below which the terminal steer may act
    goal_margin_m: float = 10.0   # aim for the keep-out sphere shrunk by this; anywhere inside wins
    # direction + speed coasting search
    plan_wait_s: float = 2.0            # s, max wait for the first radar scan before planning
    plan_min_tracks: int = 50           # plan as soon as this many tracks exist
    plan_offsets_m: tuple = (0.0, 30.0, 60.0, 100.0, 150.0, 220.0)   # aim-point rings around the target
    plan_ring_points: int = 8
    plan_speed_fracs: tuple = (1.0, 0.8, 0.6)   # of v_cruise; slower only wins if faster conflicts
    plan_corridor_m: float = 300.0      # m, rocks farther than this from the start->target line are ignored
    plan_offset_weight: float = 0.0     # extra raw-score per m of aim offset (fuel cost already counts it)
    plan_deficit_weight: float = 0.1    # raw-score per m of clearance deficit (soft collision penalty)
    fuel_kg_per_dv: float = 0.0706   # measured in the sim (1.27 kg for 18.0 m/s commanded); model was 60/(80*9.8)=0.0765
    fuel_nominal_kg: float = 1.15
    nominal_time_s: float = 120.0
    replan_dv: float = 0.05             # m/s, any conflict needing more than this triggers a search
    replan_period_s: float = 3.0        # s, minimum spacing between searches


@dataclass
class Threat:
    track: CVTrack
    tau: float            # s, time to closest approach
    miss: npt.NDArray     # m, obstacle minus shuttle at closest approach
    clearance: float      # m, required separation
    along: float          # m, distance along the flight line


@dataclass
class GuidanceOutput:
    accel_world: npt.NDArray
    mode: str
    threats: list = field(default_factory=list)
    in_tube: int = 0
    dv_needed: float = 0.0
    v_along: float = 0.0
    remaining: float = 0.0
    done: bool = False


@dataclass
class CoastPlan:
    aim: npt.NDArray          # point we fly the straight coast line toward
    dir: npt.NDArray          # unit direction of the coast
    speed: float              # m/s coast speed
    offset: float             # m, lateral offset of the aim point from the target
    t_est: float              # s, predicted time to the goal
    min_margin: float         # m, worst (miss - clearance) along the way
    planned_at: float         # s


class Guidance:
    """
    Path/velocity decomposition against a straight coast line s_hat towards an
    aim point:
        along-track : one burn to v_plan, coast, (brake only if v_arrive < v_cruise)
        cross-track : lateral offset e and drift v_lat relative to that line
    Direction + speed coasting search (plan_coast) picks the aim point and
    coast speed whose burn->coast trajectory clears every tracked rock with the
    best predicted time; it runs once at the start and again whenever a coast
    conflict would need more than `replan_dv` of trimming.
    Debris is screened by predicting OUR OWN planned trajectory (vector burn
    then coast, not constant velocity) against every track's constant-velocity
    motion.  Tracks whose closest approach falls inside the tube are threats;
    those inside the clearance are conflicts and get the cheapest lateral
    delta-v that clears all of them.
    """

    def __init__(self, p_start: npt.NDArray, p_goal: npt.NDArray, params: GuidanceParams = None):
        self.p_start = np.asarray(p_start, float)
        self.p_goal = np.asarray(p_goal, float)
        self.P = params or GuidanceParams()
        self.s0_hat = unit(self.p_goal - self.p_start)   # start->target line (reference)
        self.p_aim = self.p_goal.copy()
        self.s_hat = self.s0_hat.copy()                  # current coast line
        self.v_plan = self.P.v_cruise
        self.plan: CoastPlan = None
        self.plan_cost = None
        self.n_plans = 0
        self.last_plan_t = -1e9
        self.trimming = False
        self.burn_done = False
        self.last_threats: list[Threat] = []
        self.dv_used = 0.0                      # m/s of commanded delta-v so far (fuel proxy)
        self.fuel_used_kg = None                # real Stats/FuelUsed when the controller feeds it
        self._last_t = None
        self._last_a = np.zeros(3)
        self._taus = np.arange(0.0, self.P.horizon_s + 0.5, 0.5)

    # -- decomposition against the current coast line ---------------------------
    def decompose(self, p, v):
        s = self.s_hat
        rel = p - self.p_start
        along = float(rel @ s)
        e = rel - along * s                      # lateral offset from the line
        v_s = float(v @ s)
        v_lat = v - v_s * s
        s_go = float((self.p_goal - p) @ s)
        return along, e, v_s, v_lat, s_go

    # -- own trajectory: vector burn at a_max from v to v_t, then coast ---------
    @staticmethod
    def _burn_coast(p, v, v_t, a_max, taus):
        dv = v_t - v
        dvn = float(np.linalg.norm(dv))
        t_b = dvn / a_max
        if t_b < 1e-6:
            return p + taus[:, None] * v_t
        a = dv / t_b
        p_b = p + v * t_b + 0.5 * a * t_b * t_b
        tt = taus[:, None]
        burn = p + v * tt + 0.5 * a * tt * tt
        coast = p_b + v_t * (tt - t_b)
        return np.where(tt < t_b, burn, coast)

    def predict_self(self, p, v, taus):
        return self._burn_coast(p, v, self.v_plan * self.s_hat, self.P.a_max, taus)

    # -- closest approach of every track against a trajectory -------------------------
    def _closest(self, traj, snap: Snapshot, extra_clear: float):
        """traj(taus)->(K,3).  Returns tau*, miss (n,3), dist, clearance arrays."""
        P = self.P
        taus = self._taus
        us = traj(taus)
        obs = snap.pos[:, None, :] + snap.vel[:, None, :] * taus[None, :, None]
        d2 = ((obs - us[None, :, :]) ** 2).sum(-1)
        k = d2.argmin(1)
        n, K = d2.shape
        rows = np.arange(n)
        km, kp = np.clip(k - 1, 0, K - 1), np.clip(k + 1, 0, K - 1)
        y0, y1, y2 = d2[rows, km], d2[rows, k], d2[rows, kp]
        denom = y0 - 2.0 * y1 + y2
        safe_denom = np.where(np.abs(denom) > 1e-9, denom, 1.0)
        delta = np.where(np.abs(denom) > 1e-9, 0.5 * (y0 - y2) / safe_denom, 0.0)
        tau = np.clip(taus[k] + np.clip(delta, -1.0, 1.0) * 0.5, 0.0, P.horizon_s)
        # exact recompute at the refined time (one trajectory point per track)
        us_star = traj(tau)
        obs_star = snap.pos + snap.vel * tau[:, None]
        miss = obs_star - us_star
        dist = np.linalg.norm(miss, axis=1)
        sig = np.sqrt(snap.sigma ** 2 + (tau * snap.sigma_v) ** 2)
        clear = (snap.radius + P.shuttle_radius + P.margin_m
                 + P.k_sigma * np.minimum(sig, 6.0) + extra_clear)
        return tau, miss, dist, clear

    def closest_approaches(self, t, p, v, snap: Snapshot, extra_clear: float = 0.0) -> list[Threat]:
        if len(snap.tracks) == 0:
            return []
        traj = lambda taus: self.predict_self(p, v, taus)
        tau, miss, dist, clear = self._closest(traj, snap, extra_clear)
        keep = (dist < self.P.tube_radius + snap.radius) & (tau > 0.05)
        along = (snap.pos - p) @ self.s_hat
        return [Threat(snap.tracks[i], float(tau[i]), miss[i], float(clear[i]), float(along[i]))
                for i in np.nonzero(keep)[0]]

    def in_tube(self, t, p, v, tracks: list[CVTrack]) -> list[CVTrack]:
        ths = self.closest_approaches(t, p, v, make_snapshot(tracks, t))
        return [th.track for th in sorted(ths, key=lambda th: th.tau)]

    # -- direction + speed coasting search ---------------------------------------------
    def plan_coast(self, t, p, v, tracks: list[CVTrack]) -> CoastPlan:
        """
        Search aim points (rings around the target in the plane perpendicular to
        the start->target line) x coast speeds.  Each candidate is a vector burn
        from the current velocity to speed*dir followed by a coast; it is scored
        by predicted time plus penalties for clearance deficits and for how far
        the terminal phase will have to steer.  Adopts and returns the best plan.
        """
        P = self.P
        snap = make_snapshot(tracks, t)
        # only rocks anywhere near the corridor can matter
        if len(snap.tracks):
            rel = snap.pos - p
            along = rel @ self.s0_hat
            lat = np.linalg.norm(rel - along[:, None] * self.s0_hat, axis=1)
            keep = (along > -30.0) & (lat < P.plan_corridor_m + snap.radius)
            snap = Snapshot([snap.tracks[i] for i in np.nonzero(keep)[0]], snap.pos[keep], snap.vel[keep],
                            snap.radius[keep], snap.sigma[keep], snap.sigma_v[keep])
        e1, e2 = perp_basis(self.s0_hat)
        aims = [(self.p_goal.copy(), 0.0)]
        for r in P.plan_offsets_m:
            if r <= 0.0:
                continue
            for k in range(P.plan_ring_points):
                th = 2.0 * math.pi * k / P.plan_ring_points
                aims.append((self.p_goal + r * (math.cos(th) * e1 + math.sin(th) * e2), r))
        best, best_cost = None, None
        for aim, off in aims:
            d = unit(aim - p)
            dist_aim = float(np.linalg.norm(aim - p))
            for frac in P.plan_speed_fracs:
                speed = P.v_cruise * frac
                v_t = speed * d
                dvn = float(np.linalg.norm(v_t - v))
                t_b = dvn / P.a_max
                s_b = float(((v + v_t) * 0.5 * t_b) @ d)          # along-track distance covered in the burn
                t_est = t_b + max(dist_aim - P.keep_out_m - s_b, 0.0) / speed
                # terminal steer only for the part of the offset outside the keep-out sphere
                off_out = max(off - (P.keep_out_m - P.goal_margin_m), 0.0)
                dv_term = 2.0 * off_out / P.terminal_s
                t_est += math.sqrt(2.0 * off_out / P.a_max) * 0.5
                fuel_so_far = self.fuel_used_kg if self.fuel_used_kg is not None else self.dv_used * P.fuel_kg_per_dv
                fuel_est = fuel_so_far + (dvn + dv_term) * P.fuel_kg_per_dv
                fuel_pen = max(fuel_est / P.fuel_nominal_kg - 1.0, 0.0)
                cost = -P.nominal_time_s / max(t_est, 1.0) + fuel_pen + P.plan_offset_weight * off
                min_margin = 1e9
                if len(snap.tracks):
                    traj = lambda taus, p=p, v=v, v_t=v_t: self._burn_coast(p, v, v_t, P.a_max, taus)
                    tau, miss, dist, clear = self._closest(traj, snap, 0.0)
                    ok = (tau > 0.05) & (tau < t_est + 5.0)
                    if ok.any():
                        margin = dist[ok] - clear[ok]
                        min_margin = float(margin.min())
                        deficit = np.maximum(-margin, 0.0)
                        cost += P.plan_deficit_weight * float(deficit.sum())
                        hard = dist[ok] < snap.radius[ok] + P.shuttle_radius
                        if hard.any():
                            cost += 100.0
                if best_cost is None or cost < best_cost - 1e-9:
                    best, best_cost = CoastPlan(aim, d, speed, off, t_est, min_margin, t), cost
        self.plan_cost = best_cost
        self._adopt(best, p)
        return best

    def _adopt(self, plan: CoastPlan, p):
        self.plan = plan
        self.p_start = np.asarray(p, float).copy()
        self.p_aim = plan.aim
        self.s_hat = plan.dir
        self.v_plan = plan.speed
        self.burn_done = False
        self.trimming = False
        self.last_plan_t = plan.planned_at
        self.n_plans += 1

    # -- minimal lateral delta-v along a direction --------------------------
    @staticmethod
    def min_dv_along(n: npt.NDArray, threats: list[Threat], dv_cap: float):
        """
        A lateral delta-v x along n applied now shifts the miss vector by
        -x*tau*n (lateral motion is decoupled from the along-track profile).
        Safe iff |miss - x tau n| >= clearance: a quadratic in x whose roots
        bound a forbidden interval.  Return the smallest x >= 0 outside every
        forbidden interval, or None if that exceeds dv_cap.
        """
        intervals = []
        for th in threats:
            mn = float(np.dot(th.miss, n))
            disc = mn * mn - float(np.dot(th.miss, th.miss)) + th.clearance ** 2
            if disc <= 0.0:
                continue
            root = math.sqrt(disc)
            x1, x2 = (mn - root) / th.tau, (mn + root) / th.tau
            if x2 <= 0.0:
                continue
            intervals.append((max(x1, 0.0), x2))
        x = 0.0
        for lo, hi in sorted(intervals):
            if lo <= x + 1e-9 < hi:
                x = hi
            elif lo > x:
                break
        return x if x <= dv_cap else None

    @staticmethod
    def safe_up_to(n: npt.NDArray, threats: list[Threat], x_max: float) -> bool:
        """True if every delta-v in [0, x_max] along n keeps all threats clear."""
        for th in threats:
            mn = float(np.dot(th.miss, n))
            disc = mn * mn - float(np.dot(th.miss, th.miss)) + th.clearance ** 2
            if disc <= 0.0:
                continue
            root = math.sqrt(disc)
            x1, x2 = (mn - root) / th.tau, (mn + root) / th.tau
            if x2 > 0.0 and x1 < x_max:
                return False
        return True

    def plan_lateral(self, threats: list[Threat]):
        """Pick the cheapest lateral direction (perpendicular to the line) that clears every threat."""
        if not threats:
            return None, 0.0
        e1, e2 = perp_basis(self.s_hat)
        best_n, best_dv = None, None
        for k in range(self.P.n_dirs):
            th = 2.0 * math.pi * k / self.P.n_dirs
            n = math.cos(th) * e1 + math.sin(th) * e2
            dv = self.min_dv_along(n, threats, self.P.dv_cap)
            if dv is not None and (best_dv is None or dv < best_dv - 1e-6):
                best_n, best_dv = n, dv
        if best_n is None:
            # nothing clears within the cap: best effort away from the soonest conflict
            th0 = min(threats, key=lambda th: th.tau)
            m_perp = th0.miss - np.dot(th0.miss, self.s_hat) * self.s_hat
            best_n = -unit(m_perp) if np.linalg.norm(m_perp) > 1e-3 else e1
            best_dv = self.P.dv_cap
        return best_n, best_dv

    # -- main command law --------------------------------------------------------
    def command(self, t: float, p, v, tracks: list[CVTrack]) -> GuidanceOutput:
        out = self._command(t, p, v, tracks)
        # integrate commanded delta-v (what the fuel accounting charges for)
        if self._last_t is not None and t > self._last_t:
            self.dv_used += float(np.linalg.norm(self._last_a)) * (t - self._last_t)
        self._last_t, self._last_a = t, out.accel_world
        return out

    @property
    def fuel_est_kg(self) -> float:
        return self.dv_used * self.P.fuel_kg_per_dv

    def _command(self, t: float, p, v, tracks: list[CVTrack]) -> GuidanceOutput:
        P = self.P
        remaining = float(np.linalg.norm(self.p_goal - p)) - P.keep_out_m
        out = GuidanceOutput(np.zeros(3), "coast", remaining=remaining)
        if remaining <= 0.0:
            out.mode, out.done = "arrived", True
            return out

        # ---- initial direction + speed search (wait briefly for the first radar scan) ----
        if self.plan is None:
            if len(tracks) >= P.plan_min_tracks or t >= P.plan_wait_s:
                self.plan_coast(t, p, v, tracks)
                out.mode = "plan"
            else:
                out.mode = "wait-radar"
                return out

        along, e, v_s, v_lat, s_go = self.decompose(p, v)
        out.v_along = v_s
        snap = make_snapshot(tracks, t)
        extra = P.hysteresis_m if self.trimming else 0.0
        threats = self.closest_approaches(t, p, v, snap, extra)
        self.last_threats = threats
        out.in_tube = len(threats)
        conflicts = [th for th in threats if np.linalg.norm(th.miss) < th.clearance]
        out.threats = conflicts

        # ---- cross-track: avoidance trims, or a re-plan when the trim would be big ---------
        a_lat = np.zeros(3)
        if conflicts:
            n, dv = self.plan_lateral(threats)
            out.dv_needed = dv
            if (dv > P.replan_dv and t - self.last_plan_t > P.replan_period_s
                    and s_go / max(v_s, 1.0) > P.terminal_s):
                # cost of simply trimming on the current line, in the planner's raw-score units
                old_state = (self.plan, self.plan_cost, self.p_start.copy(), self.p_aim, self.s_hat.copy(),
                             self.v_plan, self.burn_done, self.last_plan_t, self.n_plans)
                rel_g = self.p_goal - p
                e_g = rel_g - float(rel_g @ self.s_hat) * self.s_hat
                off_out = max(float(np.linalg.norm(e_g)) - (P.keep_out_m - P.goal_margin_m), 0.0)
                t_trim = remaining / max(v_s, 1.0) + math.sqrt(2.0 * off_out / P.a_max) * 0.5
                fuel_so_far = self.fuel_used_kg if self.fuel_used_kg is not None else self.dv_used * P.fuel_kg_per_dv
                fuel_trim = fuel_so_far + (dv + 2.0 * off_out / P.terminal_s) * P.fuel_kg_per_dv
                cost_trim = -P.nominal_time_s / max(t_trim, 1.0) + max(fuel_trim / P.fuel_nominal_kg - 1.0, 0.0)
                plan = self.plan_coast(t, p, v, tracks)
                if plan.min_margin > 0.0 and self.plan_cost < cost_trim:
                    # new coast line clears everything and is cheaper than trimming: fly its burn
                    conflicts = []
                    out.threats = []
                    out.mode = "replan"
                    along, e, v_s, v_lat, s_go = self.decompose(p, v)
                else:
                    # keep the current line and trim (restore the plan the search overwrote)
                    (self.plan, self.plan_cost, self.p_start, self.p_aim, self.s_hat,
                     self.v_plan, self.burn_done, _, self.n_plans) = old_state
                    self.last_plan_t = t
            if conflicts and (dv > P.dv_min_trim or self.trimming):
                self.trimming = True
                a_lat = n * min(P.a_max, max(dv / 0.5, 0.05))   # taper at the end of the trim
                out.mode = "trim-avoid"
        else:
            self.trimming = False

        # ---- along-track speed allowed here (brake only if v_arrive < v_cruise) ---------
        v_allow = self.v_plan
        if P.v_arrive < self.v_plan:
            v_allow = min(v_allow, math.sqrt(P.v_arrive ** 2 + 2.0 * 0.9 * P.a_max * max(remaining - 10.0, 0.0)))

        # ---- terminal phase: steer the velocity straight at the target --------------------
        # Start early enough that half the thruster authority can still null the
        # lateral offset + drift before the goal: need |e_goal| + |v_lat| t_go <= 0.25 a t_go^2.
        t_go = s_go / max(v_s, 1.0)
        rel_goal = self.p_goal - p
        e_goal = rel_goal - float(rel_goal @ self.s_hat) * self.s_hat
        # predicted lateral miss of the sphere centre when we pass it, and the nearest
        # point of the (shrunk) keep-out sphere to that pass point: any point inside wins,
        # so we never steer for the centre itself
        R_ok = P.keep_out_m - P.goal_margin_m
        m_pass = e_goal - v_lat * t_go
        m_norm = float(np.linalg.norm(m_pass))
        lat_need = max(m_norm - R_ok, 0.0) + 0.0
        lat_have = 0.25 * P.a_max * t_go * t_go
        aim_pt = self.p_goal - (m_pass / m_norm) * min(R_ok, m_norm) if m_norm > 1e-6 else self.p_goal
        if s_go < 0.0 or (t_go < P.terminal_s and lat_need > 0.0) or lat_need > 0.8 * lat_have:
            v_des = v_allow * unit(aim_pt - p)
            dv = v_des - v
            dvn = float(np.linalg.norm(dv))
            a_p = np.zeros(3)
            if dvn > 0.2:
                a_p = dv / 1.0
                if np.linalg.norm(a_p) > P.a_max:
                    a_p = a_p / np.linalg.norm(a_p) * P.a_max
                if not conflicts:
                    out.mode = "terminal"
            # avoidance trim keeps priority; pursuit uses whatever authority is left
            a = a_lat + a_p
            if np.linalg.norm(a) > P.a_max:
                room = math.sqrt(max(P.a_max ** 2 - float(np.dot(a_lat, a_lat)), 0.0))
                a_p_perp = a_p - np.dot(a_p, unit(a_lat)) * unit(a_lat) if np.linalg.norm(a_lat) > 1e-9 else a_p
                a = a_lat + (a_p_perp / max(np.linalg.norm(a_p_perp), 1e-9)) * min(room, np.linalg.norm(a_p_perp))
            out.accel_world = a
            return out

        # ---- main burn: full vector burn to the planned coast velocity -------------------
        if not self.burn_done and not conflicts:
            v_t = v_allow * self.s_hat
            dv = v_t - v
            dvn = float(np.linalg.norm(dv))
            if dvn > P.speed_deadband:
                a = dv / 0.5
                if np.linalg.norm(a) > P.a_max:
                    a = a / np.linalg.norm(a) * P.a_max
                out.accel_world = a
                out.mode = "burn"
                return out
            self.burn_done = True

        # ---- cross-track: corridor return, only when worth it and only if safe -----------
        if not self.trimming:
            T_ret = float(np.clip(0.6 * t_go, 8.0, 40.0))
            v_lat_des = -e / T_ret
            nrm = np.linalg.norm(v_lat_des)
            if nrm > 3.0:
                v_lat_des *= 3.0 / nrm
            dv_vec = v_lat_des - v_lat
            dvn = float(np.linalg.norm(dv_vec))
            band = P.lat_vel_deadband_near if remaining < P.near_m else P.lat_vel_deadband
            if dvn > band:
                n = dv_vec / dvn
                threats_h = self.closest_approaches(t, p, v, snap, P.hysteresis_m + 1.0)
                # full correction if safe, else the largest safe fraction of it
                for frac in (1.0, 0.5, 0.25):
                    if self.safe_up_to(n, threats_h, frac * dvn):
                        a_lat = n * min(P.a_max, frac * dvn / 1.0)
                        out.mode = "trim-drift"
                        break

        # ---- along-track: coast with a deadband, brake only if asked ----------------------
        err = v_allow - v_s
        a_along = 0.0
        if err > P.speed_deadband:
            a_along = min(P.a_max, err / 0.5)
            if out.mode == "coast":
                out.mode = "trim-speed"
        elif err < -0.3:
            a_along = max(-P.a_max, err / 0.5)
            if out.mode == "coast":
                out.mode = "brake"

        # ---- combine; avoidance keeps priority inside the acceleration cap --------------
        a = a_along * self.s_hat + a_lat
        if np.linalg.norm(a) > P.a_max:
            if conflicts:
                room = math.sqrt(max(P.a_max ** 2 - float(np.dot(a_lat, a_lat)), 0.0))
                a = float(np.clip(a_along, -room, room)) * self.s_hat + a_lat
            else:
                a = a / np.linalg.norm(a) * P.a_max
        out.accel_world = a
        return out
