"""Thrusters: a replica of the simulator's jet-selection actuator, and a pulser built on it.

For every command the actuator solves a linear programme over the 38 jets and, for anything well
above ~0.003 m/s^2, fires the busiest jet for the whole tick and scales the others to hold the
commanded direction.  The achieved acceleration is therefore the jets' physical limit in that
direction, whatever magnitude was asked for.  JetModel repeats that LP to know the capability and
fuel rate in any direction; Pulser turns the control law's (acceleration, angular acceleration)
request into per-tick on/off commands whose average matches it.
"""
import csv
import os

import numpy as np
from scipy import optimize

ISP_G0 = 80.0 * 9.8
DT_TICK = 1.0 / 60.0       # the actuator's loop period
FIRE_MAG = 1.0             # commanded magnitude of a pulse: far above the jets' limit, so always saturated
JET_CSV = os.path.expandvars(r"%LOCALAPPDATA%\SimDynamX\STPro\Assets\Repos\STU4-Sim\Data\SpaceBuggyJetConfigNoMains.csv")
# The simulator's jet table (x, y, z, nx, ny, nz, thrust N), used when the file above cannot be read.
JETS_EMBEDDED = [
    (0.973422, 0.755998, -0.844252, -5e-06, -3e-06, -1, 163.419),
    (0.973421, 0.855013, -0.724565, 1e-06, 1, -9e-06, 163.417),
    (-1.2, 1.24216, 2e-06, 2e-06, 1, 2e-05, 163.412),
    (-1.4, 1.24216, 2e-06, 2e-06, 1, 2e-05, 163.411),
    (-1.2, 1.09179, 0.457795, -2e-06, -4e-06, 1, 163.412),
    (-1.4, 1.09179, 0.457795, -2e-06, -4e-06, 1, 163.413),
    (-1.2, -1.24216, 2e-06, 2e-06, -1, 5e-06, 163.416),
    (-1.4, -1.24216, 2e-06, 2e-06, -1, 5e-06, 163.407),
    (-1.2, -1.09179, 0.457795, 2e-06, 4e-06, 1, 163.412),
    (-1.4, -1.09179, 0.457795, 2e-06, 4e-06, 1, 163.412),
    (-1.64038, 0.991784, 0.377421, -1, -7e-06, 0, 163.413),
    (-1.64038, -0.991784, 0.377421, -1, -7e-06, 0, 163.412),
    (0.973421, 1.05501, 0.724565, 2e-06, 1, -1.2e-05, 163.414),
    (1.05552, 0.991447, 0.782521, 1, 1.5e-05, 0, 163.415),
    (0.973422, 0.955998, 0.844252, -2e-05, -3e-06, 1, 163.414),
    (0.973421, -1.05501, 0.724565, 2e-06, -1, 9e-06, 163.415),
    (1.05552, -0.991447, 0.782521, 1, 1.5e-05, 0, 163.414),
    (0.973422, -0.955998, 0.844252, -7e-06, 3e-06, 1, 163.414),
    (1.8793, 0.099999, -0.784352, 1, 7e-06, 0, 163.413),
    (1.8793, -0.100001, -0.784352, 1, 7e-06, 0, 163.415),
    (1.77562, 0.149999, -0.903213, 4e-06, -1e-06, -1, 163.415),
    (1.77562, -0.150001, -0.903213, 4e-06, -1e-06, -1, 163.415),
    (-0.833503, 1.05501, 0.724565, -2e-06, 1, 2.1e-05, 163.413),
    (-0.833503, 0.955998, 0.844252, 7e-06, -3e-06, 1, 163.414),
    (-0.833503, -1.05501, 0.724565, -2e-06, -1, 2e-06, 163.413),
    (-0.833503, -0.955998, 0.844252, 2e-05, 3e-06, 1, 163.414),
    (1.15552, 1.34145, 1e-06, 1, 1.5e-05, 5e-06, 163.415),
    (1.15552, -1.34145, 1e-06, 1, 2.2e-05, 5e-06, 163.415),
    (-1.77562, 0.150001, -0.903213, -5e-06, 1e-06, -1, 163.415),
    (-1.77562, -0.149999, -0.903213, -5e-06, 1e-06, -1, 163.415),
    (0.973421, -0.855013, -0.724565, 1e-06, -1, -2e-06, 163.413),
    (0.973422, -0.755998, -0.844252, -2e-05, 3e-06, -1, 163.414),
    (-0.833503, 1.05501, -0.724565, -2e-06, 1, -2e-06, 163.413),
    (-0.833503, 0.955998, -0.844252, 2e-05, -3e-06, -1, 163.414),
    (-0.833503, -1.05501, -0.724565, -2e-06, -1, -2.1e-05, 163.413),
    (-0.833503, -0.955998, -0.844252, 7e-06, 3e-06, -1, 163.414),
    (-1.64038, 0.991784, -0.37742, -1, -2.1e-05, 0, 163.415),
    (-1.64038, -0.991784, -0.37742, -1, 2.1e-05, 0, 163.415),
]


def load_jets():
    try:
        with open(JET_CSV) as fh:
            jets = [tuple(float(x) for x in row) for row in list(csv.reader(fh))[1:]]
        return jets if jets else JETS_EMBEDDED
    except (OSError, ValueError):
        return JETS_EMBEDDED


class JetModel:
    """Per-jet (acceleration, angular acceleration) for a given mass and inertia, with the actuator's LP."""

    def __init__(self, mass, inertia):
        self.mass = float(mass)
        Iinv = np.linalg.inv(np.asarray(inertia, float))
        A = []
        for x, y, z, nx, ny, nz, T in load_jets():
            rr = np.array([x, y, z])
            n = np.array([nx, ny, nz])
            n = n / np.linalg.norm(n)
            f = -T * n
            A.append(np.concatenate([f / self.mass, Iinv @ np.cross(rr, f)]))
        self.A = np.array(A)
        self.a_norm = np.linalg.norm(self.A[:, :3], axis=1)
        self._cache = {}

    def _solve(self, b):
        # same method order as the actuator: older SciPy offers "revised simplex", newer only "highs"
        for method in ("revised simplex", "highs"):
            try:
                r = optimize.linprog(np.ones(self.A.shape[0]), A_eq=self.A.T, b_eq=b, method=method)
                if r.success:
                    return r.x
            except (ValueError, TypeError):
                continue
        return None

    def realised(self, b6, dt=DT_TICK):
        """(accel, ang accel, fuel kg per tick) the actuator produces for a commanded 6-vector."""
        b = np.asarray(b6, float)
        if float(np.linalg.norm(b)) < 1e-12:
            return np.zeros(3), np.zeros(3), 0.0
        x = self._solve(b)
        if x is None:
            return np.zeros(3), np.zeros(3), float("nan")
        m = max(float(x.max()), 0.0)
        ttf = x if m <= dt else x * dt / m            # jet on-times, the busiest one clipped to the tick
        ttf = np.clip(ttf, 0.0, dt)
        act = self.A.T @ ttf
        fuel = self.mass * float(np.sum(ttf * self.a_norm)) / ISP_G0
        return act[:3] / dt, act[3:] / dt, fuel

    def cap(self, d6):
        """Saturated (accel, ang accel, fuel kg/s) for a 6-D direction; cached on a ~2 deg grid."""
        d = np.asarray(d6, float)
        n = float(np.linalg.norm(d))
        if n < 1e-12:
            return np.zeros(3), np.zeros(3), 0.0
        d = d / n
        key = tuple(np.round(d * 30.0).astype(int))
        hit = self._cache.get(key)
        if hit is None:
            a, al, f = self.realised(FIRE_MAG * d)
            hit = (a, al, f / DT_TICK)
            if len(self._cache) > 4000:
                self._cache.clear()
            self._cache[key] = hit
        return hit

    def cap_trans(self, d_body):
        a, _, f = self.cap(np.concatenate([d_body, np.zeros(3)]))
        return a, f

    def cap_rot(self, axis_body):
        _, al, f = self.cap(np.concatenate([np.zeros(3), axis_body]))
        return al, f


class Pulser:
    """Turns a desired body-frame (accel, ang accel) into on/off jet commands, one body axis per tick.

    Why one axis: the actuator's LP holds the commanded direction exactly, and anything off a body axis
    lands on a vertex that uses a different, weaker jet set (1 deg off +x the thrust drops from 0.175 to
    0.096-0.14 m/s^2).  Pure-axis pulses get each axis's full capability (+x 0.175, -x 0.097, +y 0.143,
    -y 0.096, +z 0.096, -z 0.117 m/s^2) at 4.32 kg per m/s, so a vector request costs its L1 norm, which
    is what the planner prices.

    Six translation and six rotation channels (+-x, +-y, +-z) each carry a sigma-delta accumulator.  A
    channel's duty is |component| / capability; each tick fires the channel whose accumulator is furthest
    past 1.  The one exception to "one axis" is a combined pulse (see _combo_pick).
    """

    COMBO_RATIOS = (0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 1.0, 1.3, 1.6, 2.0, 3.0)   # |ang accel| / |accel| of the command
    COMBO_SHARE = 0.6        # the translation channel must carry at least this share of the translation duty
    COMBO_OVERSHOOT = 0.5    # pulse units a combined pulse may fire beyond what is owed
    COMBO_GAIN = 0.05        # a combined pulse must beat the plain pulse by this many units

    def __init__(self, mass, inertia):
        self.jets = JetModel(mass, inertia)
        E = np.eye(3)
        self.cap_t = []      # per channel: (direction, realised accel, |accel|, fuel kg/s)
        self.cap_r = []
        for i in range(3):
            for s in (1.0, -1.0):
                a, f = self.jets.cap_trans(s * E[i])
                self.cap_t.append((s * E[i], np.asarray(a, float), float(np.linalg.norm(a)), f))
                al, f = self.jets.cap_rot(s * E[i])
                self.cap_r.append((s * E[i], np.asarray(al, float), float(np.linalg.norm(al)), f))
        self.acc = np.zeros(12)              # channels 0-5 translation (+x -x +y -y +z -z), 6-11 rotation
        self.realised_body = np.zeros(3)     # acceleration produced on the last tick, body axes (for the nav filter)
        self.trans_min_share = 0.0           # share of the ticks kept for translation; the law sets it during the launch
        self.combo_table = {}                # (trans channel, rot channel) -> [(units trans, units rot, accel, cmd_a, cmd_al)]

    def step(self, a_des_body, alpha_des_body):
        a = np.asarray(a_des_body, float)
        al = np.asarray(alpha_des_body, float)
        f = np.zeros(12)
        want = np.zeros(12, bool)
        for i in range(3):
            for k, s in ((0, 1.0), (1, -1.0)):
                c = 2 * i + k
                comp = s * a[i]
                if np.isfinite(comp) and comp > 1e-6 and self.cap_t[c][2] > 1e-9:
                    f[c] = comp / self.cap_t[c][2]
                    want[c] = True
                comp = s * al[i]
                if np.isfinite(comp) and comp > 1e-6 and self.cap_r[c][2] > 1e-9:
                    f[6 + c] = comp / self.cap_r[c][2]
                    want[6 + c] = True
        # Duties are not clipped one by one: a saturated request must keep its direction, so the channels
        # of a group share the ticks in proportion and only the group total is capped at one pulse per tick.
        saturated = float(f[:6].sum()) > 1.0
        for g in (slice(0, 6), slice(6, 12)):
            tot = float(f[g].sum())
            if tot > 1.0:
                f[g] /= tot
        # Rotation first, translation gets the ticks left over: shared in proportion, a slew braking during
        # a burn got ~60 % of the angular acceleration its braking curve assumed and the nose rang for 20 s.
        # During the launch the translation still keeps trans_min_share, or the first burn seconds starve.
        rot = float(f[6:].sum())
        tr = float(f[:6].sum())
        rmax = 1.0 - min(self.trans_min_share, tr)
        if rot > rmax:
            f[6:] *= rmax / rot
            rot = rmax
        if tr > max(1.0 - rot, 0.0):
            f[:6] *= max(1.0 - rot, 0.0) / max(tr, 1e-12)
        tot = float(f.sum())
        if tot > 1.0:
            f /= tot
        self.acc += f
        self.acc[~want] = np.minimum(self.acc[~want], 0.999)   # a withdrawn request must not fire on old credit
        cmd_a = np.zeros(3)
        cmd_al = np.zeros(3)
        self.realised_body = np.zeros(3)
        c = int(np.argmax(self.acc))
        base_fires = bool(self.acc[c] >= 1.0 and want[c])
        pick = self._combo_pick(f, want, saturated, base_fires)
        if pick is not None:
            ct, cr, (ut, ur, act, ca, cal) = pick
            self.acc[6 + cr] -= ur
            if not saturated:
                self.acc[ct] -= ut
            self.realised_body = np.asarray(act, float)
            return np.asarray(ca, float).copy(), np.asarray(cal, float).copy()
        if base_fires:
            self.acc[c] -= 1.0
            if c < 6:
                d, act, _, _ = self.cap_t[c]
                cmd_a = FIRE_MAG * d
                self.realised_body = act
            else:
                cmd_al = FIRE_MAG * self.cap_r[c - 6][0]
        return cmd_a, cmd_al

    def combo_entries(self, ct, cr):
        """Combined commands of translation channel ct with rotation channel cr, one per ratio in COMBO_RATIOS,
        solved with the actuator's LP and cached.  The command is scaled under the actuator's input clips
        (|accel| <= 0.5, |ang accel| <= 0.05) so the actuator sees exactly this direction, and it is far above
        the proportional threshold, so the busiest jet still fires for the whole tick."""
        key = (int(ct), int(cr))
        hit = self.combo_table.get(key)
        if hit is not None:
            return hit
        et = self.cap_t[ct][0]
        er = self.cap_r[cr][0]
        out = []
        for k in self.COMBO_RATIOS:
            sc = min(0.45, 0.045 / k)
            ca = sc * et
            cal = sc * k * er
            a, al, fuel = self.jets.realised(np.concatenate([ca, cal]))
            if not (np.all(np.isfinite(a)) and np.all(np.isfinite(al)) and np.isfinite(fuel)):
                continue
            ut = float(a @ et) / max(self.cap_t[ct][2], 1e-9)
            ur = float(al @ er) / max(self.cap_r[cr][2], 1e-9)
            # the LP holds the commanded direction: an entry with anything off the two channels is not used
            off = float(np.linalg.norm(a - (a @ et) * et)) + float(np.linalg.norm(al - (al @ er) * er))
            if ut <= 0.0 or ur <= 0.0 or off > 1e-6:
                continue
            out.append((ut, ur, a, ca, cal))
        self.combo_table[key] = out
        return out

    def _combo_pick(self, f, want, saturated, base_fires):
        """One tick may carry one translation and one rotation channel together.  The LP then shares jets
        between them, and for many pairs the pair gives more than a tick of either alone: +z with -y rotation
        at ratio 0.5 gives 1.24 x the +z thrust plus 0.66 x the rotation.  So when both are wanted, the best
        (translation channel, rotation channel, ratio) is fired instead of the plain pulse if it pays more
        units: translation units (all of them when the translation request is saturated, else up to its
        accumulated debt) plus rotation units up to the rotation debt, minus the units fired beyond the debts.
        Returns (ct, cr, entry) or None for the plain pulse."""
        wt = [c for c in range(6) if want[c]]
        wr = [c for c in range(6) if want[6 + c]]
        if not wt or not wr:
            return None
        ft = float(sum(f[c] for c in wt))
        ct = max(wt, key=lambda c: f[c])
        if ft <= 0.0 or f[ct] < self.COMBO_SHARE * ft:
            return None
        cr = max(wr, key=lambda c: self.acc[6 + c])
        debt_r = max(float(self.acc[6 + cr]), 0.0)
        debt_t = max(float(self.acc[ct]), 0.0)
        best, best_v = None, (1.0 if base_fires else 0.0) + self.COMBO_GAIN
        for e in self.combo_entries(ct, cr):
            ut, ur = e[0], e[1]
            over_r = max(ur - debt_r, 0.0)
            over_t = 0.0 if saturated else max(ut - debt_t, 0.0)
            if over_r > self.COMBO_OVERSHOOT or over_t > self.COMBO_OVERSHOOT:
                continue
            v = (ut - over_t) + (ur - over_r) - over_t - over_r
            if v > best_v:
                best, best_v = e, v
        if best is None:
            return None
        return ct, cr, best
