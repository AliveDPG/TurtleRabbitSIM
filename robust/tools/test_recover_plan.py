"""Offline: planner choice right after the 170 kJ hit (flight 261121_223159, states from its trajectory), with and
without recover_score (score-formula time price + cap guard + slower re-aims).  No rocks.
<sim py> robust/tools/test_recover_plan.py"""
import csv, os, sys
import numpy as np
sys.path.insert(0, os.environ.get("RB_SRC", "robust/dev"))
import guidance as G
T = np.array([-850.0, 120.0, -9530.0]); AX = np.array([-0.02000996, 0.09146282, 0.99560743])
rows = list(csv.DictReader(open("results/runs/261121_223159_default_s8193_ts1_offscreen_L1/trajectory.csv")))
def state(t):
    r = min(rows, key=lambda r: abs(float(r["t_sim"]) - t))
    return np.array([float(r[k]) for k in "xyz"]), np.array([float(r[k]) for k in ("vx", "vy", "vz")]), float(r["fuel_kg"])
for t in (90.0, 116.0, 140.0):
    p, v, fuel = state(t)
    for rs, slow in ((False, (0.6, 0.8)), (True, (0.4, 0.6, 0.8))):
        P = G.GuideParams(p_gate=0.3, recover_score=rs, recover_slow=slow)
        pl = G.Planner(T, P, R_w2b_fn=lambda: np.eye(3), caps_body=[0.175, 0.175, 0.143, 0.143, 0.143, 0.143])
        pl.ship_axis = AX; pl.recover_until = t + 90
        pl.v_tgt = pl.aim_velocity(p, 7.0)          # the plan in force: the cruise re-aim
        vt, info = pl.plan(t, p, v, None, t)
        C, fu, aim, tc, risk, appr, miss, tca, i0, n = pl.dbg
        k = int(np.argmin(np.linalg.norm(C - vt, axis=1)))
        dv = np.linalg.norm(vt - v); spd = np.linalg.norm(vt)
        t_end = t + tca[k] + dv / 0.175
        sc = 100 * (120 / t_end - (fuel + 4.32 * dv * 1.1) / 75) + 50
        print(f"t={t:.0f} rem={np.linalg.norm(T - p):.0f} |v|={np.linalg.norm(v):.1f} recover_score={int(rs)}: chose {info.kind or 'keep'} |vt|={spd:.2f} dv={dv:.1f} "
              f"est arrival {t_end:.0f} s, est score {sc:.0f} (fuel so far {fuel:.0f} kg)")
