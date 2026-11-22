"""Milestone 5 offline closed-loop benchmark: a Debris406-like rock (r 13 m, crossing at 5 m/s) that the planner dodges,
then a rock-rock knock sends it toward the shuttle's no-dodge position (aimed knock, lateral scatter 15 m), and the
radar re-converges in refresh epochs (first return 0-4.2 s after the knock, then every 4.2 s; velocity error / claimed
sigma 2.0, 0.8, 0.35, 0.15 m/s per axis).  Point-mass shuttle follows the planner at 0.143 m/s^2, burn pauses allowed,
round-3 planner settings (p_gate 0.3, look-ahead 22 s, gate memory 10 s excess, near_fix).
Per variant: contacts (centre distance < r + 3.2 m) in knock episodes, median clearance, extra velocity change (kg) in
episodes WITHOUT a knock (the cost on clean flights) and with one.
usage: RB_SRC=robust/dev <sim py> robust/tools/test_knock.py [variant ...]   variants: m0 m8 m15 ..."""
import os, sys
import numpy as np
from multiprocessing import Pool
sys.path.insert(0, os.environ.get("RB_SRC", "robust/dev"))
import guidance as G

T = np.array([-850.0, 120.0, -9530.0])
P0 = np.array([-651.0, 95.2, -8239.7])
CAPS = [0.175, 0.175, 0.143, 0.143, 0.143, 0.143]
A_MAX = 0.143
RR = 13.0
TRACE = os.environ.get("KNOCK_TRACE", "0") == "1"
RAND = os.environ.get("KNOCK_RAND", "0") == "1"   # random-direction knocks (most miss): many episodes needed


def params(var):
    import re
    m = re.search(r"m(\d+)", var); g = re.search(r"g(\d+)", var)
    return G.GuideParams(p_gate=0.3, horizon_s=22.0, gate_hold_s=10.0, gate_mode="excess", near_fix=True,
                         pause_steps=(3.0, 6.0, 10.0, 15.0), big_margin_m=float(m.group(1)) if m else 0.0,
                         kick_gate=float(g.group(1)) / 100.0 if g else 0.0)


def episode(args):
    var, offset, tc, knock, L, kmag, seed = args
    rng = np.random.default_rng(seed)
    P = params(var)
    pl = G.Planner(T, P, R_w2b_fn=lambda: np.eye(3), caps_body=CAPS)
    u = G.unit(T - P0)
    v0, a0 = 4.3, 0.12
    p = P0 + u * 60.0
    v = u * v0
    pl.v_tgt = pl.aim_velocity(p, P.v_cruise)
    e1, e2 = G.perp_basis(u)
    ang = rng.uniform(0, 2 * np.pi)
    side = np.cos(ang) * e1 + np.sin(ang) * e2
    cross = np.cross(u, side)
    def nominal(tt):                     # no-dodge shuttle position tt s after the start (accelerating to 7.6 m/s)
        tb = (7.6 - v0) / a0
        s = v0 * min(tt, tb) + 0.5 * a0 * min(tt, tb) ** 2 + 7.6 * max(tt - tb, 0.0)
        return p + u * s
    meet = nominal(tc) + side * offset
    rv = cross * 5.0 + rng.normal(0, 0.3, 3)
    rp = meet - rv * tc
    tk = tc - L
    bias_p, bias_v = rng.normal(0, 1.7, 3), rng.normal(0, 0.06, 3)
    epochs = [tk + rng.uniform(0, 4.2) + 4.2 * i for i in range(4)] if knock else []
    errs = [2.0, 0.8, 0.35, 0.15]
    trk_p, trk_v, trk_t, claim = rp + bias_p, rv + bias_v, 0.0, 0.15
    ev, dmin, dv_flown, t = [], 1e9, 0.0, 0.0
    kicked, ep_i = False, 0
    while t < tc + 8.0:
        if knock and not kicked and t >= tk:
            if RAND:
                d = rng.normal(0, 1, 3); d /= np.linalg.norm(d)
                rv = rv + d * rng.uniform(3.0, 12.0)            # random knock: uniform direction, 3-12 m/s
            else:
                target = nominal(tc) + rng.normal(0, 15.0, 3)
                rv_new = (target - rp) / max(tc - t, 2.0)
                d = rv_new - rv
                n = float(np.linalg.norm(d))
                rv = rv + d * min(1.0, kmag / max(n, 1e-9))      # a knock of at most kmag m/s toward the shuttle
            kicked = True
        if knock and ep_i < len(epochs) and t >= epochs[ep_i]:
            e = errs[ep_i]
            trk_p, trk_v, trk_t, claim = rp + rng.normal(0, 3.0, 3), rv + rng.normal(0, e, 3), t, e
            ep_i += 1
        o_pos = trk_p + trk_v * (t - trk_t)
        young = claim >= 0.5
        obs = dict(ids=["r158+c72"], pos=(o_pos + rng.normal(0, 0.5, 3))[None, :], vel=(trk_v + rng.normal(0, 0.03, 3))[None, :],
                   rad=np.array([RR]), sig=np.array([3.0 if young else 2.0]), sigv=np.array([claim]),
                   eps=np.array([0.10 if young else 0.06]), wide=np.array([6.4 if young else 10.6]))
        vt, info = pl.plan(40.0 + t, p, v, obs, 40.0 + t)
        if info.reason:
            ev.append(info.reason)
        if TRACE and (info.reason or int(t * 5) % 5 == 0):
            th = info.threats[0] if info.threats else None
            print(f"t={t:5.1f} claim={claim:4.2f} dist={np.linalg.norm(p - rp) - RR - 3.2:6.1f} truth_miss_dir P={th[1] if th else 0:.2f} tau={th[2] if th else 0:.1f} miss={th[3] if th else 0:.1f} sig={th[6] if th else 0:.1f} reason={info.reason} hold={pl.hold_until - 40 - t:.1f}")
        for _ in range(10):
            if 40.0 + t >= pl.hold_until:
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
    return var, offset, tc, knock, L, kmag, len(ev), dv_flown, dmin - (RR + 3.2)


def _fix_pre(args):
    return episode(args)


if __name__ == "__main__":
    vars_ = sys.argv[1:] or ["m0", "m8", "m15"]
    cases = []
    seed = 5000
    reps = int(os.environ.get("KNOCK_REPS", "3"))
    for off in (0.0, 10.0, 20.0, 30.0):
        for tc in (20.0, 26.0):
            cases.append((off, tc, False, 0.0, 0.0, seed)); seed += 1
            for L in (6.0, 10.0, 14.0):
                for kmag in ((0.0,) if RAND else (5.0, 10.0)):
                    for rep in range(reps):
                        cases.append((off, tc, True, L, kmag, seed)); seed += 1
    jobs = [(vr,) + c for vr in vars_ for c in cases]
    with Pool(int(os.environ.get("NPROC", "4"))) as pool:
        res = pool.map(_fix_pre, jobs)
    base = np.median([x[7] for x in res if x[0] == vars_[0] and not x[3] and x[1] >= 30.0])
    print(f"reference velocity change (no knock, 30 m offset, first variant) {base:.2f} m/s; kg below are over that")
    for vr in vars_:
        R = [x for x in res if x[0] == vr]
        nk = [x for x in R if not x[3]]; kk = [x for x in R if x[3]]
        kg_n = 4.32 * (np.array([x[7] for x in nk]) - base); kg_k = 4.32 * (np.array([x[7] for x in kk]) - base)
        gap = np.array([x[8] for x in kk])
        byL = " ".join(f"L{int(L)}:{sum(1 for x in kk if x[4] == L and x[8] < 0)}/{sum(1 for x in kk if x[4] == L)}" for L in (6.0, 10.0, 14.0))
        print(f"{vr:5s} no-knock: n={len(nk)} extra kg mean {kg_n.mean():5.2f} max {kg_n.max():5.2f} contacts {sum(1 for x in nk if x[8] < 0)} | "
              f"knock: n={len(kk)} contacts {int((gap < 0).sum())} ({byL}) gap med {np.median(gap):5.1f} extra kg mean {kg_k.mean():5.2f}")
