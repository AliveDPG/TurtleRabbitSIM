"""Guidance: a velocity-target planner that weighs every candidate manoeuvre in score points.

A plan is a coast velocity.  Each cycle the planner prices the plan being flown and a fixed family
of alternatives (re-aim, turns, pausing the burn) by fuel, arrival time, final aim, collision risk
and clearance from the target ship, and switches only when an alternative is clearly cheaper.
The scoring facts behind the weights: fuel is the L1 norm of the body-frame velocity change
(4.32 kg per m/s per axis), time is worth ~0.25 points per second around 220 s, and a rock hit
costs tens of kJ of damage even at 4 m/s.
"""
import math
from dataclasses import dataclass, field

import numpy as np
from scipy.stats import ncx2

MU = 707.5e-6 * 1e9          # Phobos GM, m^3/s^2
# The sim's gravity is not a point mass.  Fitted on thrust-free stretches of 15 flights, the acceleration in
# the Phobos-fixed frame is point mass + GRAV_BIAS - 2 OMEGA x v (residual 0.02-0.04 mm/s^2 between 150 and
# 1300 m above the pad).  OMEGA is Phobos' spin; the bias is 30 % of local g and constant along the corridor.
GRAV_BIAS = np.array([2.25e-3, -1.10e-3, 1.60e-3])     # m/s^2
OMEGA = np.array([0.0, 0.0, 2.24e-4])                  # rad/s


def coast_acc(p, v):
    """Thrust-free acceleration in the Phobos frame at position p with velocity v (v may be (N,3))."""
    r = float(np.linalg.norm(p))
    return -MU * p / r ** 3 + GRAV_BIAS - 2.0 * np.cross(OMEGA, v)


def unit(v):
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-12 else np.zeros_like(v)


def perp_basis(axis):
    a = unit(axis)
    h = np.array([1.0, 0, 0]) if abs(a[0]) < 0.9 else np.array([0, 1.0, 0])
    e1 = unit(np.cross(a, h))
    e2 = np.cross(a, e1)
    return e1, e2


def p_disc(m, Rc, s):
    """Probability that a 2-D isotropic Gaussian miss (per-axis sigma s) centred m from the shuttle falls
    inside the contact radius Rc: the noncentral chi-square(2) cdf."""
    s = np.maximum(s, 1e-3)
    return np.clip(np.nan_to_num(ncx2.cdf((Rc / s) ** 2, 2, (m / s) ** 2), nan=0.0), 0.0, 1.0)


@dataclass
class GuideParams:
    v_cruise: float = 7.0            # m/s
    keep_out_m: float = 40.0         # the run ends inside this radius of the target
    horizon_s: float = 35.0          # collision look-ahead: at 0.15 m/s^2 a dodge needs time to open up
    dt_tau: float = 0.4
    latency_s: float = 0.2           # from plan to thrust
    # score weights
    fuel_pts_per_mps: float = 5.76   # 100 x 4.32 kg per m/s per body axis / 75 kg nominal fuel
    time_pts_per_s: float = 0.25     # 100 x nominal time / t^2 at t ~ 220 s
    hit_pts: float = 600.0           # a hit at 7-9 m/s costs several hundred points of damage
    # final aim and the approach to the ship
    aim_ok_m: float = 26.0           # predicted miss of the centre below this needs no correction
    aim_ok_near_m: float = 20.0      # ... of the chosen entry point (15 + 20 m still lands inside 40 m)
    aim_ramp_m: float = 8.0          # the 15-point aim step ramps in over this many metres (no cliff)
    near_ship_m: float = 600.0       # inside this, aim at a clear entry point instead of the centre
    entry_r: float = 15.0            # entry points lie this far from the centre
    entry_clear_m: float = 9.0       # the straight line to the entry point keeps this clear of mapped ship cells
    entry_period_s: float = 2.0      # re-choose at most this often (unless the current line gets blocked)
    cone_deg: float = 30.0           # entry points lie within this of the ship axis, on our side
    band_deg: float = 50.0           # 40-130 m from the centre the line must stay within this of the axis
    approach_from_m: float = 1000.0  # inside this, candidates pay for crossing approach_r_cross far off the ship axis
    approach_r_cross: float = 130.0
    approach_ok_deg: float = 30.0
    approach_bad_deg: float = 50.0
    approach_pts: float = 200.0
    near_cooldown_s: float = 1.5     # near the ship, hold a plan this long after a switch ...
    near_urgent_pts: float = 50.0    # ... unless the new one is this much better
    # robust (failure mode 2): a miss the side jets cannot correct before the ship is a pass-by and a turn back
    # (a 2-5 m/s reversal, or an unfinished flight), not "15 points plus fuel later".  Off by default.
    near_fix: bool = False
    near_lag_from_m: float = 1e9     # lag-aware miss only inside this distance of the ship (milestone 4: 600 m;
                                     # farther out it made the instant re-aim look ~30 m short and a 400 m miss
                                     # was kept for 30-60 s, then corrected late and dearly)
    near_alat: float = 0.06          # m/s^2 of sideways correction counted on (side jets 0.143, one axis per tick)
    near_lead_s: float = 5.0         # the correction must be done this long before closest approach
    near_miss_pts: float = 80.0      # price of an uncorrectable miss (ramps in over near_ramp_m)
    near_ramp_m: float = 10.0
    near_slow: tuple = ()            # inside slow_from_m: re-aim candidates at these fractions of the aim speed
                                     # (tried (0.5, 0.3): never better offline, the braking lag eats the gain) ...
    near_slow_min: float = 1.5       # ... but no slower than this (m/s)
    slow_from_m: float = 450.0
    # collision probability
    shuttle_r: float = 3.2           # m, bounding sphere of the hull
    margin_m: float = 4.0            # measured contacts reach 9.8 m from the rock centre
    small_rad_m: float = 5.2         # rocks below this radar radius touch from further out than they measure:
    margin_small_m: float = 7.0      # ... contacts at a median 7.7 m,
    small_k: float = 4.7             # ... and up to this many radii from the centre
    sig_floor: float = 0.5           # m, added in quadrature to the predicted miss sigma
    sigma_cap: float = 80.0          # m; only bounds the numerics (young-track errors do reach 100 m)
    urgent_s: float = 16.0           # conflicts sooner than this count fully, later ones less ...
    far_s: float = 36.0              # ... down to a quarter weight from here on
    short_pts: float = 10.0          # points per metre of predicted overlap for a confident, urgent hit
    edge_drop: bool = False          # leave out rocks whose closest approach lies beyond the look-ahead window ...
    edge_pmax: float = 0.0           # ... while their hit probability is below this
    p_gate: float = 0.0              # hit probability on the plan in force below which a rock is ignored (0 = off)
    # robust (failure mode 1): gate memory.  The gate looks at the plan in force, so right after a dodge the rock
    # fell under it and vanished from every candidate, the re-aim straight back into it looked free, and the plan
    # flipped dodge / re-aim every 0.2-0.5 s (7 of 10 contact flights; 37 of 104 clean flights, median 40 vs 52).
    # A rock that passed the gate keeps counting for max(gate_hold_s, its closest approach + 2 s).  0 = off.
    gate_hold_s: float = 0.0
    # how a held rock counts (milestone 2):
    #   "hold"   it counts fully against every candidate for max(gate_hold_s, closest approach + 2 s) (milestone 1)
    #   "excess" same hold time, but only the extra risk of a candidate over the plan in force counts
    #            (P_c - P_0)+: it blocks a re-aim back into the rock but cannot start or enlarge a dodge
    #   "short"  counts fully for gate_hold_s only (no closest-approach term)
    #   "hyst"   no timer: a rock that passed p_gate stays in while the plan in force has P >= gate_leave
    gate_mode: str = "hold"
    recover_slow: tuple = (0.6, 0.8)  # re-aim speeds (x aim speed) offered after a hit
    recover_score: bool = False       # after a hit: time priced by the score formula, finish past the cap priced as lost
    time_cap_s: float = 400.0         # the run's time limit (the harness cap in testing)
    cap_margin_s: float = 15.0
    cap_pts: float = 300.0
    gate_leave: float = 0.15
    # candidates and switching
    lat_mags: tuple = (0.4, 0.8, 1.3, 2.0, 3.0, 4.5, 6.5, 9.0)   # m/s turns at cruise speed
    lat_dirs: int = 12
    switch_pts: float = 8.0          # a new plan must be this much cheaper than the current one (burns take a minute) ...
    switch_frac: float = 0.25        # ... or this fraction of the current plan's aim + risk cost, if larger,
    switch_cap: float = 40.0         # ... but never more than this: a near-certain hit must not block its own escape
    noop_mps: float = 0.05           # a candidate this close to the current plan is not a switch
    # burn pause: while a big burn is pending (the launch), "the same plan, burn resumed H s from now" is a dodge
    # that costs no fuel, only arrival time.  A turn at 4 m/s costs 5-20 kg and swings the burn.
    pause_steps: tuple = ()          # s from now
    pause_min_dv: float = 1.5        # m/s of burn still pending for a pause to mean anything
    pause_min_speed: float = 1.0     # m/s: never stall on or just above the pad
    pause_budget_s: float = 25.0     # total pause per flight
    pause_slack_pts: float = 8.0     # a pause may carry this much more risk than the safest plan that burns at once
    # robust (milestone 5): extra contact margin for large radar-tracked rocks (radius >= big_rad_m, id with a radar
    # track 'r..').  Most losses on seed 8193 are such rocks (Debris406, 26 m) knocked by a neighbour after the dodge
    # left a few metres of clearance.  0 = off.
    big_margin_m: float = 0.0
    big_rad_m: float = 10.0
    # ... and a lower P gate for such rocks while their velocity is uncertain (after a contact the radar track's
    # velocity sigma is metres per second, P spreads below the 0.3 gate and the rock vanished from planning until
    # 1-8 s before the hit).  0 = off.
    kick_gate: float = 0.0
    kick_gate_sv: float = 0.4
    # ... and a longer look-ahead (s) for such rocks while their track is settled (velocity sigma < big_look_sv):
    # Debris406 / Debris208 are radar-tracked from t = 3 s, predicted to 1-6 m 30 s ahead, but were first priced at
    # the 22 s window edge, where only a 15 s burn pause or a large turn still clears them.  0 = off.
    big_look_s: float = 0.0
    big_look_sv: float = 0.15
    big_look_edge_p: float = 0.0     # ... rocks at the far window edge count only above this P (0 = no edge rule)


@dataclass
class PlanInfo:
    reason: str = ""                 # why the plan changed this cycle: avoid / aim / cheaper ("" = no change)
    kind: str = ""                   # candidate family of the new plan
    worst_id: object = None          # most likely hit on the plan in force before the decision
    worst_p: float = 0.0
    worst_tau: float = 0.0
    threats: list = field(default_factory=list)
    best_coll: float = 0.0
    miss_m: float = 0.0
    costs: dict = field(default_factory=dict)


class Planner:
    def __init__(self, target, params, R_w2b_fn, caps_body):
        self.T = np.asarray(target, float)
        self.P = params
        self.R_w2b_fn = R_w2b_fn
        # per-axis jet capability (+x -x +y -y +z -z, m/s^2): the pulser fires one body axis per tick, so a
        # velocity change takes sum_i |dv_i| / cap_i seconds
        self.caps_body = np.asarray(caps_body, float).reshape(3, 2)
        self.v_tgt = None
        self.ship_axis = None
        self.entry = None
        self.entry_t = -1e9
        self.last_switch_t = -1e9
        self.n_switch = 0
        self.hold_until = -1e9       # no translation burn before this time (burn pause)
        self.pause_used = 0.0
        self.recover_until = -1e9    # set by the controller after a hit: slower re-aim candidates until then
        self.gate_mem = {}           # track-id token -> time until which the rock passes the gate (gate memory)
        k = np.arange(300) + 0.5     # 300 directions spread evenly over the sphere
        phi = np.arccos(1 - 2 * k / 300)
        th = math.pi * (1 + 5 ** 0.5) * k
        self._dirs = np.column_stack([np.cos(th) * np.sin(phi), np.sin(th) * np.sin(phi), np.cos(phi)])

    def aim_point(self):
        return self.entry if self.entry is not None else self.T

    def aim_velocity(self, p, v_speed):
        """Velocity of magnitude ~v_speed whose gravity-bent coast passes through the aim point."""
        A = self.aim_point()
        tau = float(np.linalg.norm(A - p)) / max(v_speed, 0.5)
        g = coast_acc(p, (A - p) / tau)
        return (A - p - 0.5 * g * tau * tau) / tau

    def _segment_clear(self, p, E, tree, band=True):
        """Is the straight line from p to E clear of mapped ship cells and (band) of the ship's equatorial ring?"""
        n = max(int(float(np.linalg.norm(E - p)) / 2.0), 2)
        pts = p[None, :] + (E - p)[None, :] * np.linspace(0.0, 1.0, n)[:, None]
        rr = np.linalg.norm(pts - self.T[None, :], axis=1)
        outside = rr > self.P.keep_out_m - 1.0
        if band and self.ship_axis is not None:
            # the ship's ring / disc sits around its equator 40-112 m out: stay near the axis there
            ring = outside & (rr < 130.0)
            if ring.any():
                cosb = ((pts[ring] - self.T[None, :]) @ self.ship_axis) / rr[ring]
                if float(cosb.min()) < math.cos(math.radians(self.P.band_deg)):
                    return False
        if not outside.any() or tree is None:
            return True
        d, _ = tree.query(pts[outside], k=1, distance_upper_bound=50.0)
        return float(np.min(np.where(np.isfinite(d), d, 50.0))) >= self.P.entry_clear_m

    def choose_entry(self, t, p, v, tree):
        """Entry point on a sphere of entry_r around the centre, on our side, whose straight line from here stays
        clear of the ship; the one needing the smallest turn wins."""
        P = self.P
        if (tree is None and self.ship_axis is None) or float(np.linalg.norm(p - self.T)) > P.near_ship_m:
            self.entry = None
            return
        if self.entry is not None and t - self.entry_t < P.entry_period_s and self._segment_clear(p, self.entry, tree):
            return
        dirs = self._dirs
        toward = unit(p - self.T)
        facing = dirs @ toward > 0.2
        in_cone = facing
        if self.ship_axis is not None:
            in_cone = facing & (dirs @ self.ship_axis >= math.cos(math.radians(P.cone_deg)))
        vhat = unit(v) if float(np.linalg.norm(v)) > 1.0 else unit(self.T - p)
        best, best_c = None, 1e9
        for d in dirs[in_cone]:
            E = self.T + P.entry_r * d
            turn = math.acos(float(np.clip(unit(E - p) @ vhat, -1.0, 1.0)))
            if turn < best_c and self._segment_clear(p, E, tree):
                best, best_c = E, turn
        if best is None and self.ship_axis is not None:
            # nothing clear inside the cone: the best clear entry point on our side, cone or not
            for d in dirs[facing]:
                E = self.T + P.entry_r * d
                turn = math.acos(float(np.clip(unit(E - p) @ vhat, -1.0, 1.0)))
                if turn < best_c and self._segment_clear(p, E, tree, band=False):
                    best, best_c = E, turn
        if best is not None:                 # nothing provably clear: keep the old point (or the centre)
            self.entry, self.entry_t = best, t

    def predicted_miss(self, p, vels, lag=None):
        """Closest approach to the aim point, and its time, of a coast from p at each velocity in vels (C,3).
        lag = (v, lat (C,), t_burn (C,)): fly v until lat, then change velocity at a constant rate over t_burn
        (the velocity change is not instant) instead of starting at vels."""
        g = coast_acc(p, vels)
        rel = self.aim_point() - p
        taus = np.arange(0.0, 400.0, 0.5)
        if lag is None:
            pos = -rel[None, None, :] + vels[:, None, :] * taus[None, :, None] + 0.5 * g[:, None, :] * (taus ** 2)[None, :, None]
        else:
            v0, lat, tb = lag
            dv = vels - np.asarray(v0, float)[None, :]
            tl = np.maximum(taus[None, :] - lat[:, None], 0.0)
            tbc = np.maximum(tb, 1e-6)[:, None]
            frac = np.where(tl < tbc, 0.5 * tl ** 2 / tbc, tl - 0.5 * tbc)            # (C,T) share of dv flown
            pos = (-rel[None, None, :] + np.asarray(v0, float)[None, None, :] * taus[None, :, None]
                   + dv[:, None, :] * frac[:, :, None] + 0.5 * g[:, None, :] * (taus ** 2)[None, :, None])
        dist = np.linalg.norm(pos, axis=2)
        k = np.argmin(dist, axis=1)
        return dist[np.arange(len(vels)), k], taus[k]

    def plan(self, t, p, v, obs, t_elapsed, static_tree=None):
        """obs: dict of ids (list), pos (N,3) at time t, vel (N,3), rad, sig, sigv, eps, wide (N), or None.
        static_tree: KD-tree of mapped ship cells, or None.  Returns (target velocity, PlanInfo)."""
        P = self.P
        p = np.asarray(p, float)
        v = np.asarray(v, float)
        self.choose_entry(t, p, v, static_tree)
        if self.v_tgt is None:
            self.v_tgt = self.aim_velocity(p, P.v_cruise)
        aim_v = self.aim_velocity(p, P.v_cruise)

        # Candidate 0 is the plan being flown, exactly as flown: the "keep" cost then describes a velocity the
        # shuttle really has, and no candidate is preferred merely for matching a nominal speed.
        base = self.v_tgt.copy()
        s_base = max(float(np.linalg.norm(base)), 0.5)
        s_aim = max(float(np.linalg.norm(aim_v)), 0.5)
        u = unit(base)
        e1, e2 = perp_basis(u)
        cands = [base, aim_v, u * s_aim]            # keep / re-aim / current direction at the aim speed
        for m in P.lat_mags:                        # pure turns of the current plan (no along-track braking)
            for k in range(P.lat_dirs):
                ang = 2 * math.pi * k / P.lat_dirs
                cands.append(unit(base + (m * s_base / max(P.v_cruise, 0.5)) * (math.cos(ang) * e1 + math.sin(ang) * e2)) * s_base)
        f1, f2 = perp_basis(unit(aim_v))
        for m in (1.0, 2.0, 3.5, 6.0):              # turns from the aim line, at the aim speed
            for k in range(8):
                ang = 2 * math.pi * k / 8
                cands.append(unit(aim_v + (m * s_aim / max(P.v_cruise, 0.5)) * (math.cos(ang) * f1 + math.sin(ang) * f2)) * s_aim)
        n_turns = 3 + len(P.lat_mags) * P.lat_dirs
        i_slow = len(cands)
        if P.near_fix and float(np.linalg.norm(p - self.T)) < P.slow_from_m:
            for f in P.near_slow:            # slow down and re-aim: buys time for the side jets near the ship
                cands.append(self.aim_velocity(p, max(P.near_slow_min, f * s_aim)))
        if t < self.recover_until:
            for f in P.recover_slow:         # after a hit: slower re-aims, priced like everything else
                cands.append(self.aim_velocity(p, max(P.near_slow_min, f * s_aim)))
        n_slow = len(cands) - i_slow
        # burn-pause candidates: the plan in force with its burn resumed H s from now
        h0 = max(self.hold_until - t, 0.0)
        i_pause = len(cands)
        pause_H = []
        if P.pause_steps and float(np.linalg.norm(base - v)) > P.pause_min_dv and float(np.linalg.norm(v)) > P.pause_min_speed:
            for H in P.pause_steps:
                if H > h0 + 1.0 and self.pause_used + (H - h0) <= P.pause_budget_s:
                    cands.append(base.copy())
                    pause_H.append(float(H))
        C = np.array(cands)
        hold = np.zeros(len(C))
        hold[0] = h0
        if pause_H:
            hold[i_pause:] = pause_H
        lat = np.full(len(C), P.latency_s) + hold     # when each candidate's thrust starts

        # --- costs that do not depend on obstacles
        dvC = C - v[None, :]
        dvn = np.linalg.norm(dvC, axis=1)
        dvb = dvC @ self.R_w2b_fn().T
        caps = np.where(dvb > 0, self.caps_body[:, 0][None, :], self.caps_body[:, 1][None, :])
        t_burn = (np.abs(dvb) / caps).sum(axis=1)                       # one axis per tick: burns add up
        a_eff = dvn / np.maximum(t_burn, 1e-6)                          # mean acceleration along dv while burning
        fuel = P.fuel_pts_per_mps * np.abs(dvb).sum(axis=1)
        if P.near_fix and float(np.linalg.norm(p - self.T)) < P.near_lag_from_m:
            # The velocity change takes t_burn on the jets (one axis per tick) after the latency: until then the
            # shuttle keeps flying at v.  Close to the ship that lag is most of the time left, and a re-aim that
            # looked like a 11 m miss was a 50 m one (late-dodge flight, 2.3-3.2 m/s asked 14-22 s out).
            miss, tca = self.predicted_miss(p, C, lag=(v, lat, t_burn))
        else:
            miss, tca = self.predicted_miss(p, C)
        # final approach: enter the keep-out sphere near its centre line (the ship's structure starts just
        # outside the 40 m sphere)
        aim_ok = P.aim_ok_near_m if self.entry is not None else P.aim_ok_m
        step = 15.0 * np.clip((miss - aim_ok) / P.aim_ramp_m, 0.0, 1.0)
        aim = np.where(miss < aim_ok, 0.0, step + 1.3 * P.fuel_pts_per_mps * (miss - 8.0) / np.maximum(tca - 8.0, 5.0))
        if P.near_fix:
            t_corr = np.maximum(tca - P.near_lead_s, 0.0)
            reach = 0.5 * P.near_alat * t_corr ** 2       # sideways move the jets can still make before the ship
            aim = aim + P.near_miss_pts * np.clip((miss - aim_ok - reach) / P.near_ramp_m, 0.0, 1.0)
        speed = np.maximum(np.linalg.norm(C, axis=1), 0.5)
        t_arr = t_elapsed + np.maximum(tca - P.keep_out_m / speed, 0.0) + (lat - hold - P.latency_s)
        t_arr = t_arr + 0.5 * t_burn                # a velocity change is not instant: ramp delay
        # H s of pause flown at v instead of C arrive H (1 - v.C / |C|^2) s later
        t_arr = t_arr + hold * np.clip(1.0 - (C @ v) / np.maximum((C * C).sum(axis=1), 0.25), 0.0, 2.0)
        time_c = P.time_pts_per_s * (t_arr - float(t_arr[0]))
        if P.recover_score and t < self.recover_until:
            # After a hit the arrival is late, where a second is worth 12000 / t^2 (0.075 points at 400 s, not
            # 0.25): price time with the score formula itself, and a finish past the run's time cap as lost.
            ta = np.maximum(t_arr, 1.0)
            time_c = 12000.0 / max(float(ta[0]), 1.0) - 12000.0 / ta
            time_c = time_c + P.cap_pts * np.clip((ta - (P.time_cap_s - P.cap_margin_s)) / 10.0, 0.0, 1.0)

        # --- collision cost: sum over rocks of P(hit) x the cost of a hit
        coll = np.zeros(len(C))
        info = PlanInfo()
        if obs is not None and len(obs["rad"]):
            big_look = None
            if P.big_look_s > P.horizon_s:
                big_look = (np.array([str(i).startswith("r") for i in obs["ids"]], bool) & (obs["rad"] >= P.big_rad_m)
                            & (np.asarray(obs["sigv"], float) < P.big_look_sv))
                if not big_look.any():
                    big_look = None
            taus = np.arange(0.0, (P.big_look_s if big_look is not None else P.horizon_s) + 1e-9, P.dt_tau)
            e = dvC / np.maximum(dvn, 1e-9)[:, None]
            tl = np.maximum(taus[None, :] - lat[:, None], 0.0)
            s = np.where(tl < t_burn[:, None], 0.5 * a_eff[:, None] * tl ** 2, dvn[:, None] * (tl - 0.5 * t_burn[:, None]))   # (C,T)
            own = (v[None, None, :] * taus[None, :, None]) + e[:, None, :] * s[:, :, None]                    # (C,T,3)
            rel0 = p[None, :] - obs["pos"]                                                                     # (N,3)
            rock = rel0[:, None, :] - obs["vel"][:, None, :] * taus[None, :, None]                           # (N,T,3)
            sep = rock[None, :, :, :] + own[:, None, :, :]                                                     # (C,N,T,3)
            dist = np.sqrt((sep ** 2).sum(axis=3))                                                             # (C,N,T)
            if big_look is not None:
                # other rocks keep the normal window: samples beyond it do not exist for them
                beyond = taus > P.horizon_s + 1e-9
                dist[:, ~big_look, :] = np.where(beyond[None, None, :], np.inf, dist[:, ~big_look, :])
            rad = obs["rad"]
            margin = np.where(rad < P.small_rad_m, P.margin_small_m, P.margin_m)
            margin = np.where(rad < P.small_rad_m, np.maximum(margin, P.small_k * rad - rad - P.shuttle_r), margin)
            if P.big_margin_m > 0.0:
                isr = np.array([str(i).startswith("r") for i in obs["ids"]], bool)
                margin = margin + np.where(isr & (rad >= P.big_rad_m), P.big_margin_m, 0.0)
            Rc = (rad + P.shuttle_r + margin)[None, :]                   # contact radius
            kmin = np.argmin(dist, axis=2)                                                                     # (C,N)
            m = np.take_along_axis(dist, kmin[:, :, None], axis=2)[:, :, 0]
            tau_star = taus[kmin]
            # The miss-plane prediction error is a core Gaussian (the track's calibrated sigmas, grown to the
            # time of closest approach) plus a heavy tail of weight eps and sigma x wide: rock-rock kicks and
            # young or fragmented tracks.
            sig_t = np.sqrt(obs["sig"][None, :] ** 2 + (obs["sigv"][None, :] * tau_star) ** 2 + P.sig_floor ** 2)
            sig_t = np.minimum(sig_t, P.sigma_cap)
            eps = np.asarray(obs["eps"], float)[None, :]
            wide = np.asarray(obs["wide"], float)[None, :]
            Phit = (1.0 - eps) * p_disc(m, Rc, sig_t)
            Phit = Phit + eps * p_disc(m, Rc, np.minimum(sig_t * wide, P.sigma_cap))
            if big_look is not None and P.big_look_edge_p > 0.0:
                # A long-window rock whose closest approach is still beyond the window end asked for one small turn
                # per window slide (0.46 m/s at 16, 17, 20 s for Debris208 in flight, aim miss 80 -> 240 m): it is
                # judged once its closest approach is inside the window, unless the hit is already likely.
                k_last = len(taus) - 1
                far_edge = big_look[None, :] & (kmin == k_last) & (kmin[0] == k_last)[None, :] & (Phit < P.big_look_edge_p)
                Phit = np.where(far_edge, 0.0, Phit)
            if P.p_gate > 0.0:
                # The dodge is sized on the full probability, so a turn cannot stop just under the gate.
                gate_v = P.p_gate
                if P.kick_gate > 0.0:
                    isr_k = np.array([str(i).startswith("r") for i in obs["ids"]], bool)
                    gate_v = np.where(isr_k & (rad >= P.big_rad_m) & (np.asarray(obs["sigv"], float) >= P.kick_gate_sv), P.kick_gate, P.p_gate)
                on = Phit[0] >= gate_v
                held = np.zeros_like(on)
                if P.gate_hold_s > 0.0 or P.gate_mode == "hyst":
                    toks = [str(i).split("+") for i in obs["ids"]]
                    if P.gate_mode == "hyst":
                        # in while P_0 >= gate_leave once it has passed p_gate; the memory is the last cycle's set
                        was = np.array([any(self.gate_mem.get(k, -1e9) >= t - 1.0 for k in tk) for tk in toks], bool)
                        held = was & ~on & (Phit[0] >= P.gate_leave)
                        for q in np.nonzero(on | held)[0]:
                            for k in toks[q]:
                                self.gate_mem[k] = t
                    else:
                        for q in np.nonzero(on)[0]:
                            dt_hold = P.gate_hold_s if P.gate_mode == "short" else max(P.gate_hold_s, float(tau_star[0, q]) + 2.0)
                            for k in toks[q]:
                                self.gate_mem[k] = max(self.gate_mem.get(k, -1e9), t + dt_hold)
                        held = ~on & np.array([any(self.gate_mem.get(k, -1e9) > t for k in tk) for tk in toks], bool)
                    if len(self.gate_mem) > 200:
                        self.gate_mem = {k: u for k, u in self.gate_mem.items() if u > t - 2.0}
                if P.gate_mode == "excess":
                    Phit = np.where(on[None, :], Phit, np.where(held[None, :], np.clip(Phit - Phit[0][None, :], 0.0, None), 0.0))
                else:
                    Phit = Phit * (on | held)[None, :]
            if P.edge_drop:
                # A rock still closing at the end of the window has its closest approach beyond it, and the
                # window reports the distance at its last sample as the miss.  A turn sized for that sample looks
                # like a dodge, but a moment later the window has slid on and the same rock asks for the next
                # turn.  Such a rock is judged once its closest approach is inside the window - unless the hit is
                # already likely.
                k_last = len(taus) - 1
                at_edge = (kmin == k_last) & (kmin[0] == k_last)[None, :] & (Phit < P.edge_pmax)
                Phit = np.where(at_edge, 0.0, Phit)
            # Far conflicts count less: their predictions are poor and they can be fixed later, airborne, for
            # little fuel; at full weight they kept swinging the burn direction while the shuttle sat on the pad.
            w_t = np.clip(1.0 - 0.75 * (tau_star - P.urgent_s) / max(P.far_s - P.urgent_s, 1e-6), 0.25, 1.0)
            coll = P.hit_pts * (Phit * w_t).sum(axis=1)
            # P saturates near 0.88 when every candidate hits; a slope on the overlap then prefers the larger miss
            overlap = np.clip(Rc - m, 0.0, None)
            coll = coll + P.short_pts * (overlap * (Phit > 0.5) * (tau_star < P.urgent_s)).sum(axis=1)
            j = int(np.argmax(Phit[0]))
            top = np.argsort(-Phit[0])[:3]
            info.threats = [(obs["ids"][q], float(Phit[0, q]), float(tau_star[0, q]), float(m[0, q]), float(rad[q]),
                             float(np.linalg.norm(obs["pos"][q] - p)), float(sig_t[0, q]), float(Phit[:, q].min()))
                            for q in top if Phit[0, q] > 0.2]
            info.worst_p = float(Phit[0, j])
            info.worst_id = obs["ids"][j]
            info.worst_tau = float(tau_star[0, j])

        # --- the ship: stay clear of mapped cells until inside the keep-out sphere
        stat = np.zeros(len(C))
        if static_tree is not None:
            taus_s = np.arange(0.0, P.horizon_s + 1e-9, 0.5)
            e_s = dvC / np.maximum(dvn, 1e-9)[:, None]
            tl_s = np.maximum(taus_s[None, :] - lat[:, None], 0.0)
            s_s = np.where(tl_s < t_burn[:, None], 0.5 * a_eff[:, None] * tl_s ** 2, dvn[:, None] * (tl_s - 0.5 * t_burn[:, None]))
            path = p[None, None, :] + v[None, None, :] * taus_s[None, :, None] + e_s[:, None, :] * s_s[:, :, None]
            inside = np.linalg.norm(path - self.T[None, None, :], axis=2) < P.keep_out_m - 1.0
            after = np.cumsum(inside, axis=1) > 0          # the run is over once inside: ignore the rest
            dmin, _ = static_tree.query(path.reshape(-1, 3), k=1, distance_upper_bound=30.0)
            dmin = np.where(np.isfinite(dmin), dmin, 30.0).reshape(path.shape[:2])
            dmin = np.where(after, 30.0, dmin)
            stat = P.hit_pts * np.clip((P.shuttle_r + 4.0 - dmin.min(axis=1)) / 3.0, 0.0, 1.0)
        # approach angle: where each candidate's coast crosses approach_r_cross from the ship, and how far off
        # the ship axis that point is
        appr = np.zeros(len(C))
        if self.ship_axis is not None and float(np.linalg.norm(p - self.T)) < P.approach_from_m:
            ta = np.arange(0.0, 120.0, 1.0)
            ga = coast_acc(p, C)
            pa = (p - self.T)[None, None, :] + C[:, None, :] * ta[None, :, None] + 0.5 * ga[:, None, :] * (ta ** 2)[None, :, None]
            inside = np.linalg.norm(pa, axis=2) < P.approach_r_cross
            xa = pa[np.arange(len(C)), np.argmax(inside, axis=1)]
            cosang = (xa @ self.ship_axis) / np.maximum(np.linalg.norm(xa, axis=1), 1e-6)
            ang = np.degrees(np.arccos(np.clip(cosang, -1.0, 1.0)))
            frac = np.clip((ang - P.approach_ok_deg) / (P.approach_bad_deg - P.approach_ok_deg), 0.0, 1.0)
            appr = np.where(inside.any(axis=1), P.approach_pts * frac, 0.0)

        total = fuel + aim + time_c + coll + stat + appr
        self.dbg = (C, fuel, aim, time_c, coll + stat, appr, miss, tca, i_slow, n_slow)   # for offline tests
        risk = coll + stat
        if pause_H:
            # A pause may only win as a dodge, and only when it is about as safe as the safest plan that burns at
            # once.  A pause that merely lowers the risk wins on fuel, keeps the shuttle slow and leaves no way out.
            bad = (risk[i_pause:] > risk[0] - P.switch_pts) | (risk[i_pause:] > float(risk[1:i_pause].min()) + P.pause_slack_pts)
            total[i_pause:] = np.where(bad, np.inf, total[i_pause:])
        best = int(np.argmin(total))
        gain = float(total[0] - total[best])
        # near the ship a tight aim made the plan re-aim every cycle: hold each plan for near_cooldown_s there
        cooling = self.entry is not None and (t - self.last_switch_t) < P.near_cooldown_s and gain < P.near_urgent_pts
        is_pause = bool(pause_H) and best >= i_pause
        noop = float(np.linalg.norm(C[best] - C[0])) < P.noop_mps and not is_pause
        # The threshold scales with the current plan's aim + risk cost but not its remaining fuel: with fuel in it
        # a half-flown dodge blocked its own re-aim for 100 s.
        thr = max(P.switch_pts, min(P.switch_frac * (aim[0] + coll[0] + stat[0] + appr[0]), P.switch_cap))
        chosen = 0
        if is_pause and not cooling and gain > thr:
            H = pause_H[best - i_pause]
            self.pause_used += H - h0
            self.hold_until = t + H
            info.reason = "avoid"
            info.kind = f"pause{H:.0f}"
            chosen = best
        elif best != 0 and not cooling and gain > thr and not noop:
            self.last_switch_t = t
            self.v_tgt = C[best].copy()
            self.hold_until = t                 # a new plan burns at once
            self.n_switch += 1
            # name the switch after the cost term that improved most
            d_terms = {"avoid": risk[0] - risk[best],
                       "aim": (aim[0] + appr[0]) - (aim[best] + appr[best]),
                       "cheaper": (fuel[0] + time_c[0]) - (fuel[best] + time_c[best])}
            info.reason = max(d_terms, key=d_terms.get)
            info.kind = ("aim" if best == 1 else "dir@aim-speed" if best == 2 else "turn" if best < n_turns
                         else "slow" if i_slow <= best < i_slow + n_slow else "aim-ring")
            chosen = best
        info.best_coll = float(coll.min())
        info.miss_m = float(miss[chosen])
        info.costs = dict(fuel=float(fuel[chosen]), aim=float(aim[chosen]), time=float(time_c[chosen]),
                          coll=float(coll[chosen]), stat=float(stat[chosen]), coll_cur=float(coll[0]))
        return self.v_tgt, info
