"""Offline closed-loop check of the gate memory (GuideParams.gate_hold_s) against the dodge / re-aim flip-flop.
A point-mass shuttle follows the planner's target velocity with 0.143 m/s^2 (one axis at a time is not modelled)
after 0.2 s latency; one large rock crosses the launch line.  Counts plan switches, avoid->aim->avoid flips and
the closest approach minus the contact radius.  Run with the sim's python:  RB_SRC=robust <sim py> robust/tools/test_gate.py"""
import os, sys
import numpy as np
sys.path.insert(0, os.environ.get("RB_SRC", "robust"))
import guidance as G

T = np.array([-850.0, 120.0, -9530.0])
P0 = np.array([-651.0, 95.2, -8239.7])
CAPS = [0.175, 0.175, 0.143, 0.143, 0.143, 0.143]
A_MAX = 0.143


def episode(hold, rng, v0=3.0, t0=30.0, rock_r=15.5, rock_v=1.4, tc=18.0, offset=0.0, noise=1.0):
    P = G.GuideParams(p_gate=0.3, gate_hold_s=hold, horizon_s=22.0)
    pl = G.Planner(T, P, R_w2b_fn=lambda: np.eye(3), caps_body=CAPS)
    u = G.unit(T - P0)
    p = P0 + u * 40.0
    v = u * v0
    pl.v_tgt = pl.aim_velocity(p, P.v_cruise)
    # rock: crosses the line, its centre reaching the line at time tc, with a miss of `offset` m
    e1, _ = G.perp_basis(u)
    meet = p + u * (v0 * tc + 0.5 * 0.1 * tc * tc) + e1 * offset
    rv = -e1 * rock_v
    rp = meet - rv * tc
    Rc = rock_r + 3.2 + 4.0
    ev = []
    dmin = 1e9
    t = t0
    for k in range(int(40 / 0.2)):
        rp_meas = rp + rng.normal(0, noise, 3)
        rv_meas = rv + rng.normal(0, 0.1, 3)
        obs = dict(ids=["r13+c27"], pos=rp_meas[None, :], vel=rv_meas[None, :], rad=np.array([rock_r]),
                   sig=np.array([2.0]), sigv=np.array([0.25]), eps=np.array([0.06]), wide=np.array([10.6]))
        vt, info = pl.plan(t, p, v, obs, t)
        if info.reason:
            ev.append((t, info.reason))
        for _ in range(10):                       # 0.2 s in 10 steps
            dv = pl.v_tgt - v
            n = float(np.linalg.norm(dv))
            if n > 1e-9:
                v = v + dv / n * min(n, A_MAX * 0.02)
            p = p + v * 0.02
            rp = rp + rv * 0.02
            t += 0.02
            dmin = min(dmin, float(np.linalg.norm(p - rp)))
    flips = sum(1 for i in range(1, len(ev) - 1) if ev[i][1] == "aim" and ev[i - 1][1] == "avoid" and ev[i + 1][1] == "avoid")
    return len(ev), flips, dmin - (rock_r + 3.2)


rng = np.random.default_rng(1)
for hold in [float(x) for x in os.environ.get("HOLDS", "0,10").split(",")]:
    rows = []
    for off in (0.0, 5.0, 10.0, -5.0):
        for tc in (14.0, 18.0, 22.0):
            rows.append(episode(hold, rng, tc=tc, offset=off)); print(f"   off={off:4.0f} tc={tc:3.0f} switches={rows[-1][0]:3d} flips={rows[-1][1]:3d} gap={rows[-1][2]:6.1f}")
    sw, fl, gap = map(np.array, zip(*rows))
    print(f"gate_hold={hold:4.0f}: episodes={len(rows)} switches mean={sw.mean():5.1f} flips total={fl.sum():3d} "
          f"contacts(gap<0)={int((gap < 0).sum())} gap min={gap.min():5.1f} median={np.median(gap):5.1f}")
