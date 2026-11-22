"""Milestone 5 D: offline closed loop of RB_BIG_LOOK (longer look-ahead for settled radar rocks of radius >= 10 m).
Point mass in the launch burn (starts 60 m out at 3.0 m/s accelerating to 7.6 m/s at 0.12 m/s^2 nominal; follows the
planner at 0.143 m/s^2; burn pauses allowed from the start, as after the pad watch), round-3 planner settings, one rock:
  "d406": r 13 m crossing at 5 m/s,   "d208": r 15.7 m crossing at 1.4 m/s,
nominal closest approach tc = 26 / 32 / 38 s after the start, centre offset 0-40 m (>= contact radius + 4 m = 'pass').
Track: bias 1.7 m / 0.06 m/s per axis, noise 0.5 m / 0.03 m/s, sig 2 m, sigv 0.07 m/s (settled radar).
Per variant: contacts, gap median, kg = 4.32 x (velocity change flown + velocity error left to the aim velocity at
the end, tc + 40 s), arrival delay (s, vs the no-rock run), points = 1.333 kg + 0.27 s.
usage: RB_SRC=robust/dev <sim py> robust/tools/test_biglook.py"""
import os, sys
import numpy as np
from multiprocessing import Pool
sys.path.insert(0, os.environ.get("RB_SRC", "robust/dev"))
import guidance as G

T = np.array([-850.0, 120.0, -9530.0]); P0 = np.array([-651.0, 95.2, -8239.7])
CAPS = [0.175, 0.175, 0.143, 0.143, 0.143, 0.143]; A_MAX = 0.143


def episode(args):
    look, kind, offset, tc, seed = args
    rng = np.random.default_rng(seed)
    P = G.GuideParams(p_gate=0.3, horizon_s=22.0, gate_hold_s=10.0, gate_mode="excess", near_fix=True,
                      pause_steps=(3.0, 6.0, 10.0, 15.0), big_look_s=look,
                      big_look_edge_p=float(os.environ.get("BL_EDGE", "0")))
    pl = G.Planner(T, P, R_w2b_fn=lambda: np.eye(3), caps_body=CAPS)
    u = G.unit(T - P0); v0, a0 = 3.0, 0.12
    p0 = P0 + u * 60.0; p = p0.copy(); v = u * v0
    pl.v_tgt = pl.aim_velocity(p, P.v_cruise)
    e1, e2 = G.perp_basis(u); ang = rng.uniform(0, 2 * np.pi)
    side = np.cos(ang) * e1 + np.sin(ang) * e2; cross = np.cross(u, side)
    tb = (7.6 - v0) / a0
    s = v0 * min(tc, tb) + 0.5 * a0 * min(tc, tb) ** 2 + 7.6 * max(tc - tb, 0.0)
    rr, sp = (13.0, 5.0) if kind == "d406" else (15.7, 1.4)
    rv = cross * sp + rng.normal(0, 0.1, 3)
    rp = p0 + u * s + side * offset - rv * tc
    bias_p, bias_v = rng.normal(0, 1.7, 3), rng.normal(0, 0.06, 3)
    dmin, dvf, t = 1e9, 0.0, 0.0
    t_end = tc + 40.0
    while t < t_end:
        obs = None
        if offset < 900:
            obs = dict(ids=["r158+c72"], pos=(rp + bias_p + rng.normal(0, 0.5, 3))[None, :], vel=(rv + bias_v + rng.normal(0, 0.03, 3))[None, :],
                       rad=np.array([rr]), sig=np.array([2.0]), sigv=np.array([0.07]), eps=np.array([0.06]), wide=np.array([10.6]))
        pl.plan(40.0 + t, p, v, obs, 40.0 + t)
        for _ in range(10):
            if 40.0 + t >= pl.hold_until:
                dv = pl.v_tgt - v; n = float(np.linalg.norm(dv))
                if n > 0.06:
                    st = min(n, A_MAX * 0.02); v = v + dv / n * st; dvf += st
            p = p + v * 0.02; rp = rp + rv * 0.02; t += 0.02
            dmin = min(dmin, float(np.linalg.norm(p - rp)))
    resid = float(np.linalg.norm(pl.aim_velocity(p, P.v_cruise) - v))
    prog = float(np.dot(p - p0, u))
    return look, kind, offset, tc, dvf + resid, prog, dmin - (rr + 3.2)


if __name__ == "__main__":
    looks = [float(x) for x in (sys.argv[1:] or ["0", "35"])]
    cases = [(k, o, tc) for k in ("d406", "d208") for o in (0.0, 8.0, 16.0, 24.0, 32.0, 40.0) for tc in (26.0, 32.0, 38.0) for _ in range(2)]
    jobs = [(lk, k, o, tc, 7000 + i) for lk in looks for i, (k, o, tc) in enumerate(cases)]
    jobs += [(lk, "d406", 999.0, tc, 1) for lk in looks for tc in (26.0, 32.0, 38.0)]
    with Pool(int(os.environ.get("NPROC", "2"))) as pool:
        res = pool.map(episode, jobs)
    ref = {(x[0], x[3]): x for x in res if x[2] == 999.0}
    for lk in looks:
        for kind in ("d406", "d208"):
            for name, sel in (("threat", lambda x, r: x[2] < r + 3.2 + 4.0), ("pass", lambda x, r: x[2] >= r + 3.2 + 4.0)):
                rr = 13.0 if kind == "d406" else 15.7
                R = [x for x in res if x[0] == lk and x[1] == kind and x[2] < 900 and sel(x, rr)]
                if not R: continue
                kg = np.array([4.32 * (x[4] - ref[(lk, x[3])][4]) for x in R])
                delay = np.array([(ref[(lk, x[3])][5] - x[5]) / 7.6 for x in R])
                gap = np.array([x[6] for x in R])
                pts = 1.333 * kg + 0.27 * delay
                print(f"look={lk:4.0f} {kind} {name:6s} n={len(R):2d} contacts={int((gap < 0).sum())} gap med {np.median(gap):5.1f} min {gap.min():5.1f} | "
                      f"extra kg mean {kg.mean():5.2f} | delay s mean {delay.mean():5.1f} | points mean {pts.mean():5.2f}")
