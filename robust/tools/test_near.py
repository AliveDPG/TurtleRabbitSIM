"""Offline check of the near-ship fix (RB_NEAR / GuideParams.near_fix) on the late-dodge flight
results/runs/261121_120001_default_s8193_ts1_offscreen: replays shuttle states from its trajectory, with the
plan in force = the velocity actually flown, a synthetic block of ship cells on the straight line in, and no rocks.
Run with the sim's python (needs scipy):  <sim py> robust/test_near.py"""
import csv, sys, os
import numpy as np
from scipy.spatial import cKDTree
sys.path.insert(0, os.environ.get("RB_SRC", "robust"))
import guidance as G

RUN = "results/runs/261121_120001_default_s8193_ts1_offscreen/trajectory.csv"
T = np.array([-850.0, 120.0, -9530.0])
AX = np.array([-0.02000996, 0.09146282, 0.99560743])
rows = list(csv.DictReader(open(RUN)))
def state(t):
    r = min(rows, key=lambda r: abs(float(r["t_sim"]) - t))
    return np.array([float(r[k]) for k in "xyz"]), np.array([float(r[k]) for k in ("vx", "vy", "vz")])

def run(t, near, tree=None, v_plan=None):
    p, v = state(t)
    P = G.GuideParams(near_fix=near, p_gate=0.3)
    pl = G.Planner(T, P, R_w2b_fn=lambda: np.eye(3), caps_body=[0.175, 0.175, 0.143, 0.143, 0.143, 0.143])
    pl.ship_axis = AX
    pl.v_tgt = (v if v_plan is None else v_plan).copy()
    vt, info = pl.plan(t, p, v, None, t, static_tree=tree)
    miss, tca = pl.predicted_miss(p, vt[None, :], lag=(v, np.array([0.2]), np.array([float(np.abs(vt - v).sum() / 0.15)])))
    return p, v, vt, info, float(miss[0]), float(tca[0])

def show(tag, t, tree=None):
    for near in (False, True):
        p, v, vt, info, miss, tca = run(t, near, tree)
        print(f"{tag} t={t:5.1f} rem={np.linalg.norm(T - p):5.0f} near={int(near)} reason={info.reason or '-':6s} kind={info.kind or '-':13s}"
              f" dv={np.linalg.norm(vt - v):4.2f} |vt|={np.linalg.norm(vt):4.2f} miss(centre/entry)={miss:5.1f} tca={tca:5.1f}"
              f" costs={ {k: round(x, 1) for k, x in info.costs.items() if k != 'coll_cur'} }")

# 1. far from the ship (no entry point): the fix must change nothing
same = 0; n = 0
for t in np.arange(40.0, 170.0, 5.0):
    a = run(t, False); b = run(t, True); n += 1
    same += int(np.allclose(a[2], b[2]) and a[3].reason == b[3].reason)
print(f"far states t=40..165: identical choice in {same}/{n}")

# 2. late states of the failing flight, plan in force = velocity flown, no structure
for t in (196.0, 206.0, 211.0, 215.0, 220.0):
    show("free ", t)

# 3. a block of ship cells across the straight line in, 50-70 m from the centre: the structure 'avoid' at 206 s
p, v = state(206.0)
d = G.unit(p - T)
e1, e2 = G.perp_basis(d)
cells = [T + d * r + e1 * a + e2 * b for r in np.arange(45.0, 75.0, 4.0) for a in np.arange(-12, 13, 4.0) for b in np.arange(-12, 13, 4.0)]
tree = cKDTree(np.array(cells))
for t in (200.0, 206.0, 211.0):
    show("block", t, tree)
