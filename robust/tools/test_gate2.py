"""Offline closed-loop benchmark of the gate variants (milestone 2).
Point-mass shuttle follows the planner's target velocity at 0.143 m/s^2 (no motion while a burn pause holds),
one rock per episode crossing the launch line.  Tracking error = a fixed per-episode bias (3 m, 0.1 m/s) plus
per-cycle noise.  Per config: plan switches, avoid->aim->avoid flips, velocity change flown (fuel proxy, kg at
4.32 kg per m/s), contacts (gap to the hull sphere < 0), for the 'threat' episodes (offset < contact radius) and
the 'pass' episodes (rock passes outside the contact radius: any fuel there is a needless dodge).
usage: RB_SRC=robust/dev <sim py> robust/tools/test_gate2.py [config ...]"""
import os, sys, itertools
import numpy as np
from multiprocessing import Pool
sys.path.insert(0, os.environ.get("RB_SRC", "robust/dev"))
import guidance as G

T = np.array([-850.0, 120.0, -9530.0])
P0 = np.array([-651.0, 95.2, -8239.7])
CAPS = [0.175, 0.175, 0.143, 0.143, 0.143, 0.143]
A_MAX = 0.143
CONFIGS = {
    "off":      dict(gate_hold_s=0.0),
    "hold10":   dict(gate_hold_s=10.0, gate_mode="hold"),
    "excess10": dict(gate_hold_s=10.0, gate_mode="excess"),
    "short4":   dict(gate_hold_s=4.0, gate_mode="short"),
    "hyst15":   dict(gate_mode="hyst", gate_leave=0.15),
    "hyst15_pause": dict(gate_mode="hyst", gate_leave=0.15, pause_steps=(3.0, 6.0, 10.0, 15.0)),
    "excess10_pause": dict(gate_hold_s=10.0, gate_mode="excess", pause_steps=(3.0, 6.0, 10.0, 15.0)),
    "off_pause": dict(gate_hold_s=0.0, pause_steps=(3.0, 6.0, 10.0, 15.0)),
}


def episode(args):
    cfg, rock_r, offset, tc, seed = args
    rng = np.random.default_rng(seed)
    P = G.GuideParams(p_gate=0.3, horizon_s=22.0, **CONFIGS[cfg])
    pl = G.Planner(T, P, R_w2b_fn=lambda: np.eye(3), caps_body=CAPS)
    u = G.unit(T - P0)
    v0 = 3.0
    p = P0 + u * 40.0
    v = u * v0
    pl.v_tgt = pl.aim_velocity(p, P.v_cruise)
    e1, e2 = G.perp_basis(u)
    side = e1 if rng.random() < 0.5 else e2
    meet = p + u * (v0 * tc + 0.5 * 0.12 * tc * tc) + side * offset
    rv = -(e1 if side is e2 else e2) * 1.4 + rng.normal(0, 0.2, 3)
    rp = meet - rv * tc
    bias_p, bias_v = rng.normal(0, 3.0 / 3 ** 0.5, 3), rng.normal(0, 0.1 / 3 ** 0.5, 3)
    ev, dmin, dv_flown, t = [], 1e9, 0.0, 30.0
    for k in range(int(45 / 0.2)):
        obs = dict(ids=["r13+c27"], pos=(rp + bias_p + rng.normal(0, 0.7, 3))[None, :], vel=(rv + bias_v + rng.normal(0, 0.05, 3))[None, :],
                   rad=np.array([rock_r]), sig=np.array([2.0]), sigv=np.array([0.15]), eps=np.array([0.06]), wide=np.array([10.6]))
        vt, info = pl.plan(t, p, v, obs, t)
        if info.reason:
            ev.append(info.reason)
        for _ in range(10):
            if t >= pl.hold_until:
                dv = pl.v_tgt - v
                n = float(np.linalg.norm(dv))
                if n > 0.06:
                    step = min(n, A_MAX * 0.02)
                    v = v + dv / n * step
                    dv_flown += step
            p = p + v * 0.02
            rp = rp + rv * 0.02
            t += 0.02
            dmin = min(dmin, float(np.linalg.norm(p - rp)))
    flips = sum(1 for i in range(1, len(ev) - 1) if ev[i] == "aim" and ev[i - 1] == "avoid" and ev[i + 1] == "avoid")
    return cfg, rock_r, offset, len(ev), flips, dv_flown, dmin - (rock_r + 3.2)


if __name__ == "__main__":
    cfgs = sys.argv[1:] or ["off", "hold10", "excess10", "short4", "hyst15"]
    cases = [(15.5, o, tc) for o in (0.0, 8.0, 16.0, 26.0, 34.0) for tc in (14.0, 20.0, 26.0)] + \
            [(5.0, o, tc) for o in (0.0, 8.0, 16.0, 24.0) for tc in (14.0, 20.0, 26.0)]
    jobs = [(c, r, o, tc, 1000 + i) for c in cfgs for i, (r, o, tc) in enumerate(cases)]
    with Pool(int(os.environ.get("NPROC", "4"))) as pool:
        res = pool.map(episode, jobs)
    v_aim = 0.0                     # baseline: velocity change flown with no rock at all
    base = episode(("off", 15.5, 500.0, 20.0, 1))[5]
    print(f"no-rock episode flies {base:.2f} m/s (the launch burn); fuel below is the extra over that, kg")
    for c in cfgs:
        R = [x for x in res if x[0] == c]
        for name, sel in (("threat", lambda x: x[2] < x[1] + 3.2 + 4.0), ("pass", lambda x: x[2] >= x[1] + 3.2 + 4.0)):
            S = [x for x in R if sel(x)]
            if not S:
                continue
            sw = np.array([x[3] for x in S]); fl = np.array([x[4] for x in S]); kg = 4.32 * (np.array([x[5] for x in S]) - base)
            gap = np.array([x[6] for x in S])
            print(f"{c:15s} {name:6s} n={len(S):2d} switches/ep={sw.mean():5.1f} flips={fl.sum():4d} extra kg mean={kg.mean():5.1f} "
                  f"max={kg.max():5.1f} contacts={int((gap < 0).sum())} gap med={np.median(gap):5.1f}")
