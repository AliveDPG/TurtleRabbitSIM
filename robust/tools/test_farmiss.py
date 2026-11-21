"""Offline: does near_fix (lag-aware miss) delay the re-aim of a large mid-cruise miss?  Replays the states of the
candidate flights with a >= 90 m mid-cruise miss (plan in force = velocity flown, no rocks) and reports the planner's
choice with near_fix off / on / on-but-only-inside-near_lag_from_m.  <sim py> robust/tools/test_farmiss.py"""
import csv, json, glob, os, sys
import numpy as np
sys.path.insert(0, os.environ.get("RB_SRC", "robust/dev"))
import guidance as G
T = np.array([-850.0, 120.0, -9530.0]); AX = np.array([-0.02000996, 0.09146282, 0.99560743])
runs = []
for f in ["results/eval/m2fin_b/runs.jsonl"] + glob.glob("results/eval/conf*_?/runs.jsonl"):
    b = os.path.basename(os.path.dirname(f))
    if b.startswith("conf") and json.load(open(f"results/eval/{b}.arm"))["arm"] != "cand": continue
    runs += [json.loads(l) for l in open(f) if json.loads(l).get("score") is not None]
picked = [r for r in runs if r["score"] in (-6.4, -2.3, 12.4, 17.5)]
cfgs = {"off": dict(near_fix=False), "on": dict(near_fix=True), "on<600m": dict(near_fix=True, near_lag_from_m=600.0)}
for r in picked:
    rows = list(csv.DictReader(open(os.path.join(r["out_dir"], "trajectory.csv"))))
    print(f"== score {r['score']}")
    for t in range(80, 141, 15):
        row = min(rows, key=lambda x: abs(float(x["t_sim"]) - t))
        p = np.array([float(row[k]) for k in "xyz"]); v = np.array([float(row[k]) for k in ("vx", "vy", "vz")])
        out = []
        for name, kw in cfgs.items():
            try:
                P = G.GuideParams(p_gate=0.3, **kw)
            except TypeError:
                out.append(f"{name}: n/a"); continue
            pl = G.Planner(T, P, R_w2b_fn=lambda: np.eye(3), caps_body=[0.175, 0.175, 0.143, 0.143, 0.143, 0.143])
            pl.ship_axis = AX; pl.v_tgt = v.copy()
            vt, info = pl.plan(float(t), p, v, None, float(t))
            miss0 = float(pl.predicted_miss(p, v[None, :])[0][0])
            out.append(f"{name}: {info.kind or 'keep'} dv={np.linalg.norm(vt - v):.2f}")
        print(f"   t={t} rem={np.linalg.norm(T - p):.0f} miss(keep)={miss0:.0f} | " + " | ".join(out))
