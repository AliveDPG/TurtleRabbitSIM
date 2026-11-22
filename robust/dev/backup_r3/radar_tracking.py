"""Radar tracking: the shuttle's own time-stamped state and one Kalman track per rock the radar reports.

The radar list carries no identities and refreshes each rock in a round-robin (every ~4.2 s), giving
a position relative to the shuttle at the time of measurement, a range rate and a diameter.
ShuttleNav keeps the shuttle's state history so a return can be placed in the Phobos-fixed frame at
its own time stamp; DebrisTracker associates returns to tracks there and follows rocks through the
velocity jumps that rock-rock contacts cause.
"""
import math
from collections import deque

import numpy as np

from guidance import coast_acc

I3 = np.eye(3)

# Between contacts a rock's velocity changes by 0.047 m/s per radar refresh, all of it gravity (98.5 % of
# intervals below 0.1 m/s), so tracks follow the gravity model with little process noise.  The other 1.2 %
# are contacts (above 1 m/s) and are handled as jumps:
SIGMA_ACC = 0.02          # m/s^2 process noise of a rock track
RDOT_JUMP_K = 5.0         # a range-rate innovation beyond this many sigma is a contact ...
RDOT_JUMP_GAIN = 2.0      # ... as strong across the line of sight as along it: velocity variance += (gain x innovation)^2
POS_JUMP_CHI2 = 8.0       # so is an associated return this far (chi-square) off the prediction of a track ...
POS_JUMP_SV_MAX = 0.35    # ... whose velocity the filter believes (sigma below this, m/s)
# Every rock is reported, so a return that fits no track belongs to a rock already tracked whose track no longer
# fits.  The old track of the same diameter that is due for its return is moved onto it (re-link):
RELINK_MIN_AGE = 2.0      # s the old track must have gone without a return
RELINK_DIAM_TOL = 0.15    # relative diameter difference
RELINK_REACH_MPS, RELINK_REACH_PAD = 14.0, 10.0    # the return lies within reach_mps x age + pad of the prediction
KICK_MIN_LOS_M = 4.0      # line-of-sight displacement needed to time the contact from the range rate
KICK_DV_MAX = 20.0        # m/s
KICK_SIG_D = 4.0          # m, 1-sigma of the displacement (radar 3 m, old track 1-2 m)
KICK_REL_TIMED, KICK_REL_UNTIMED = 0.25, 0.5       # relative sigma of the velocity change, contact time solved / not
KICK_S_UNTIMED = 0.667    # share of the gap assumed since the contact when it cannot be timed


def _cv_FQ(dt, sigma_acc):
    F = np.eye(6)
    F[:3, 3:] = I3 * dt
    q = sigma_acc ** 2
    Q = np.block([[q * dt ** 4 / 4 * I3, q * dt ** 3 / 2 * I3],
                  [q * dt ** 3 / 2 * I3, q * dt ** 2 * I3]])
    return F, Q


class ShuttleNav:
    """Shuttle state [p, v] with a time history: the simulator's state fix propagated with the acceleration the
    jets produced.  pos_at / vel_at place a radar return, which is relative to the shuttle at its measurement
    time, in the Phobos frame."""

    def __init__(self, sigma_fix_pos=0.3, sigma_fix_vel=0.03, sigma_acc=0.05, history_s=30.0):
        self.x = None
        self.P = None
        self.t = None
        self.R = np.diag([sigma_fix_pos ** 2] * 3 + [sigma_fix_vel ** 2] * 3)
        self.sigma_acc = sigma_acc
        self.hist = deque()
        self.history_s = history_s

    def propagate(self, t, acc_world):
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

    def fix(self, t, pos, vel):
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

    def _bracket(self, t):
        lo, hi = 0, len(self.hist) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if self.hist[mid][0] <= t:
                lo = mid
            else:
                hi = mid
        return self.hist[lo], self.hist[hi]

    def pos_at(self, t):
        """Shuttle position at an earlier time (linear interpolation), None before the first fix."""
        if not self.hist:
            return None
        if t <= self.hist[0][0]:
            t0, p0, v0 = self.hist[0]
            return p0 + v0 * (t - t0) if t0 - t < 5.0 else p0
        if t >= self.hist[-1][0]:
            t1, p1, v1 = self.hist[-1]
            return p1 + v1 * min(t - t1, 0.5)
        (t0, p0, _), (t1, p1, _) = self._bracket(t)
        w = (t - t0) / max(t1 - t0, 1e-9)
        return p0 + w * (p1 - p0)

    def vel_at(self, t):
        """Shuttle velocity at an earlier time (linear interpolation, clamped at the ends)."""
        if not self.hist:
            return None
        if t <= self.hist[0][0]:
            return self.hist[0][2]
        if t >= self.hist[-1][0]:
            return self.hist[-1][2]
        (t0, _, v0), (t1, _, v1) = self._bracket(t)
        w = (t - t0) / max(t1 - t0, 1e-9)
        return v0 + w * (v1 - v0)


class RadarTrack:
    """Kalman track [p, v] of one rock in the Phobos frame, coasting on the gravity model."""
    _next_id = 0

    def __init__(self, t, pos, R, diameter, sigma_v0=3.0):
        self.id = RadarTrack._next_id
        RadarTrack._next_id += 1
        self.t = t                       # time the state is valid at
        self.x = np.hstack([pos, np.zeros(3)])
        self.P = np.zeros((6, 6))
        self.P[:3, :3] = R
        self.P[3:, 3:] = I3 * sigma_v0 ** 2   # rocks move at a few m/s (median 2.7, p90 4.5)
        self.diameter = float(diameter)
        self.hits = 1
        self.last_update = t
        self.t_kick = -1e9               # time of the last contact seen on this rock
        self.t_kick_from = -1e9          # ... which happened some time after this
        self.kick_solved = True          # ... and whether the new velocity was solved (else only its line-of-sight part)

    @property
    def pos(self):
        return self.x[:3]

    @property
    def vel(self):
        return self.x[3:]

    @property
    def radius(self):
        return 0.5 * self.diameter

    def pos_at(self, t):
        return self.x[:3] + self.x[3:] * (t - self.t)

    def pos_sigma(self, t=None):
        """1-sigma position uncertainty (largest axis), optionally extrapolated to t."""
        var = np.diag(self.P)[:3]
        if t is not None and t > self.t:
            var = var + (t - self.t) ** 2 * np.diag(self.P)[3:]
        return math.sqrt(max(float(var.max()), 0.0))

    def vel_sigma(self):
        return math.sqrt(max(float(np.diag(self.P)[3:].max()), 0.0))

    def predict_to(self, t):
        dt = t - self.t
        if dt <= 0.0:
            return
        F, Q = _cv_FQ(dt, SIGMA_ACC)
        a = coast_acc(self.x[:3], self.x[3:])
        self.x = F @ self.x
        self.x[:3] += 0.5 * dt * dt * a
        self.x[3:] += dt * a
        self.P = F @ self.P @ F.T + Q
        self.t = t

    def update_pos(self, t, z, R):
        if t < self.t - 1e-6:
            return                        # out of order
        self.predict_to(t)
        PHt = self.P[:, :3]
        K = PHt @ np.linalg.inv(self.P[:3, :3] + R)
        self.x = self.x + K @ (z - self.x[:3])
        self.P = self.P - K @ PHt.T
        self.hits += 1
        self.last_update = t

    def range_rate(self, p_sh, v_sh):
        """Predicted radar range rate and its Jacobian (6,) at the current state."""
        rel = self.x[:3] - p_sh
        r = max(float(np.linalg.norm(rel)), 1.0)
        u = rel / r
        v_rel = self.x[3:] - v_sh
        h = float(v_rel @ u)
        return h, np.hstack([(v_rel - h * u) / r, u])

    def update_range_rate(self, t, rdot, p_sh, v_sh, sigma):
        """EKF update with the range rate (v_rock - v_shuttle) . u, right after the position update of the same
        return.  An innovation beyond RDOT_JUMP_K sigma is a contact: the velocity covariance is opened before
        the update, so the track follows instead of rejecting."""
        if t < self.t - 1e-6:
            return
        self.predict_to(t)
        h, H = self.range_rate(p_sh, v_sh)
        PHt = self.P @ H
        S = float(H @ PHt) + sigma ** 2
        innov = rdot - h
        if innov * innov > RDOT_JUMP_K ** 2 * S:
            self.P[3:, 3:] += I3 * (RDOT_JUMP_GAIN * innov) ** 2
            self.t_kick = t
            self.t_kick_from = t - 4.5   # some time since the return before this one
            self.kick_solved = False
            PHt = self.P @ H
            S = float(H @ PHt) + sigma ** 2
        K = PHt / S
        self.x = self.x + K * innov
        self.P = self.P - np.outer(K, PHt)
        self.P = 0.5 * (self.P + self.P.T)

    def update_diameter(self, d, weight):
        if d > 0.0:
            self.diameter = (1.0 - weight) * self.diameter + weight * d


class DebrisTracker:
    """Tracks of every rock the radar reports.

    A radar entry is a 3x3 matrix:
        [[x, y, z],            position relative to the shuttle, Phobos-fixed axes, at the entry's time stamp
         [r_closest, rdot, d],
         [stamp, 0, 0]]
    Association is nearest-neighbour in the Phobos frame, gated on size and range rate.  Entries already
    processed are recognised by their stamp, so each read costs only the fresh ones.
    """

    def __init__(self, sigma_pos=3.0, gate_m=6.0, max_age_s=12.0, sigma_rdot=0.03, rdot_gate_k=4.0, rdot_gate_floor=0.5):
        self.tracks = []
        self.R_radar = I3 * sigma_pos ** 2
        self.sigma_pos = sigma_pos
        self.sigma_rdot = sigma_rdot     # radar noise is 0.02 m/s; the rest covers stamp skew and our interpolated velocity
        self.rdot_gate_k = rdot_gate_k
        self.rdot_gate_floor = rdot_gate_floor
        self.gate_m = gate_m
        self.max_age_s = max_age_s
        self.time_offset = 0.0           # radar stamp -> controller time
        self._seen = set()
        self._initial_done = False
        self._offset_samples = 0

    def _associate(self, t, z, diameter, gate, rdot, p_sh, v_sh):
        if not self.tracks:
            return None
        X = np.array([tr.x for tr in self.tracks])
        D = np.array([np.diag(tr.P) for tr in self.tracks])
        dt = np.maximum(t - np.array([tr.t for tr in self.tracks]), 0.0)[:, None]
        pos = X[:, :3] + X[:, 3:] * dt
        vel = X[:, 3:]
        sigma = np.sqrt(np.maximum((D[:, :3] + dt * dt * D[:, 3:]).max(axis=1), 0.0))
        sigma_v = np.sqrt(np.maximum(D[:, 3:].max(axis=1), 0.0))
        diam = np.array([tr.diameter for tr in self.tracks])
        d = np.linalg.norm(pos - z, axis=1)
        ok = np.abs(diam - diameter) < 0.6 * np.maximum(diam, diameter) + 2.0
        # the range rate must agree too: it separates neighbouring rocks that move differently (wide open for
        # fresh tracks, whose velocity sigma is metres per second)
        rel = pos - p_sh
        r = np.maximum(np.linalg.norm(rel, axis=1), 1.0)
        u = rel / r[:, None]
        v_rel = vel - v_sh
        h = np.einsum("ij,ij->i", v_rel, u)
        v_perp = np.linalg.norm(v_rel - h[:, None] * u, axis=1)
        s_h = np.sqrt(sigma_v ** 2 + (v_perp / r * sigma) ** 2 + self.sigma_rdot ** 2)
        ok &= np.abs(rdot - h) < self.rdot_gate_k * s_h + self.rdot_gate_floor
        # the gate grows with the track's predicted uncertainty (a fresh track of unknown velocity may be tens of
        # metres off after one refresh, a converged one a few)
        gates = gate + 3.0 * np.sqrt(sigma ** 2 + self.sigma_pos ** 2)
        score = np.where(ok, d / gates, np.inf)
        j = int(np.argmin(score))
        return self.tracks[j] if score[j] < 1.0 else None

    def prune(self, t):
        self.tracks = [tr for tr in self.tracks if t - tr.last_update < self.max_age_s]

    def _relink(self, ts, z, diameter, rdot, p_sh, v_sh):
        """The track of the rock an unassociated return belongs to, moved onto the return; None if no track
        qualifies.  Without this the rock would get a new track of unknown velocity while the old one coasts on
        as a ghost."""
        best, best_d = None, None
        for tr in self.tracks:
            age = ts - tr.last_update
            if tr.hits < 2 or age < RELINK_MIN_AGE:
                continue
            if abs(diameter - tr.diameter) > RELINK_DIAM_TOL * tr.diameter:
                continue
            dn = float(np.linalg.norm(z - tr.pos_at(ts)))
            if dn < RELINK_REACH_MPS * age + RELINK_REACH_PAD and (best is None or dn < best_d):
                best, best_d = tr, dn
        if best is not None:
            self._kick(best, ts, z, diameter, rdot, p_sh, v_sh)
        return best

    def _kick(self, tr, ts, z, diameter, rdot, p_sh, v_sh):
        """Restart track tr on the return z after a contact.  The velocity change follows from the displacement
        d = dv x s, with the time since the contact s solved from the range-rate innovation u.dv when the
        line-of-sight displacement is large enough."""
        age = ts - tr.last_update
        d = z - tr.pos_at(ts)
        rel = z - p_sh
        u = rel / max(float(np.linalg.norm(rel)), 1.0)
        innov = rdot - float((tr.vel - v_sh) @ u)
        d_los = float(d @ u)
        s, rel_sig = KICK_S_UNTIMED * age, KICK_REL_UNTIMED
        if abs(d_los) > KICK_MIN_LOS_M and abs(innov) > 0.3:
            s_est = d_los / innov
            if 0.25 <= s_est <= 1.2 * age:
                s, rel_sig = min(s_est, age), KICK_REL_TIMED
        dv = d / s
        n = float(np.linalg.norm(dv))
        if n > KICK_DV_MAX:
            dv *= KICK_DV_MAX / n
            n = KICK_DV_MAX
        sig_v = min(math.sqrt((KICK_SIG_D / s) ** 2 + (rel_sig * n) ** 2), 3.0)
        tr.x = np.hstack([z, tr.vel + dv])
        tr.P = np.zeros((6, 6))
        tr.P[:3, :3] = self.R_radar
        tr.P[3:, 3:] = I3 * sig_v ** 2
        tr.t = ts
        tr.last_update = ts
        tr.hits = 1                      # a new velocity history: a young track again for the fusion and the risk model
        tr.t_kick = ts
        tr.t_kick_from = ts - age
        tr.kick_solved = True
        tr.update_diameter(diameter, 0.2)

    def ingest_radar(self, entries, shuttle_pos_at, t_now, shuttle_vel_at):
        """entries: 3x3 radar matrices; shuttle_pos_at / shuttle_vel_at: callables of time; t_now: controller
        time of this read.  The radar stamps returns from its own start time and the freshest stamp of a read lags
        t_now by at most ~0.2 s, so the running minimum of (t_now - newest stamp) is the clock offset."""
        entries = [np.asarray(m, dtype=float) for m in entries]
        if entries:
            newest = max(float(m[2, 0]) for m in entries)
            if newest > 1e-6:
                lag = t_now - newest
                if self._offset_samples == 0 or lag < self.time_offset:
                    self.time_offset = lag
                self._offset_samples += 1
        for m in entries:
            ts = float(m[2, 0]) + self.time_offset
            rel = m[0, :]
            first_scan = ts <= self.time_offset + 1e-9       # stamp 0: the scan taken at the start
            if first_scan:
                if self._initial_done:
                    continue
            else:
                key = (round(ts, 4), round(float(rel[0]), 2), round(float(rel[1]), 2), round(float(rel[2]), 2))
                if key in self._seen:
                    continue
                self._seen.add(key)
            diameter = max(float(m[1, 2]), 0.2)
            sp = shuttle_pos_at(ts)
            if sp is None:
                continue
            z = sp + rel
            sv = shuttle_vel_at(ts)
            rdot = float(m[1, 1])
            tr = self._associate(ts, z, diameter, self.gate_m + 0.15 * diameter, rdot, sp, sv)
            if tr is None:
                tr = None if first_scan else self._relink(ts, z, diameter, rdot, sp, sv)
                if tr is None:
                    tr = RadarTrack(ts, z, self.R_radar, diameter)
                    self.tracks.append(tr)
            elif ts > tr.last_update + 1e-6:
                if (tr.hits >= 2 and ts - tr.last_update >= RELINK_MIN_AGE
                        and float(np.diag(tr.P)[3:].max()) < POS_JUMP_SV_MAX ** 2
                        and float(np.sum((z - tr.pos_at(ts)) ** 2)) > POS_JUMP_CHI2 * (tr.pos_sigma(ts) ** 2 + self.sigma_pos ** 2)):
                    # the track's own rock (it passed the gates) but far off the prediction: a contact the range
                    # rate did not show, e.g. 7.8 m/s across the line of sight
                    self._kick(tr, ts, z, diameter, rdot, sp, sv)
                else:
                    tr.update_pos(ts, z, self.R_radar)
                    tr.update_diameter(diameter, 0.2)
            else:
                continue
            tr.update_range_rate(ts, rdot, sp, sv, self.sigma_rdot)
        self._initial_done = True
        if len(self._seen) > 20000:
            self._seen = set(sorted(self._seen)[-5000:])
