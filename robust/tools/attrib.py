"""Milestone 4 attribution: per flight, launch fuel/speed at 60 s, fuel after 60 s, plan switches by reason, avoid
switches whose worst rock on the plan in force was below the 0.3 gate (gate-memory dodges), near-ship aim switches
(rem < 600 m), RECOVERY lines, seconds with nose > 60 deg outside tumble (RB_RECOVER_DEG re-points)."""
import json, glob, os, re, sys
import numpy as np
ST = re.compile(r"t=\s*([\d.]+) rem=\s*([\d.]+) \|v\|=\s*([\d.]+).*?att=\s*([\d.]+) w=([\d.]+) mode=(\w+).*?fuel=\s*([\d.]+)")
PL = re.compile(r"t=\s*([\d.]+) PLAN (\w+): dv=([\d.]+).*?worst=(\S+) P=([\d.]+).*?miss=([\d.]+)m")
def flights(arm):
    out = []
    srcs = [("results/eval/m2fin_a/runs.jsonl", "base"), ("results/eval/m2fin_b/runs.jsonl", "cand")]
    for f in glob.glob("results/eval/conf*_?/runs.jsonl"):
        b = os.path.basename(os.path.dirname(f)); srcs.append((f, json.load(open(f"results/eval/{b}.arm"))["arm"]))
    for f, a in srcs:
        if a != arm: continue
        for l in open(f):
            r = json.loads(l)
            if r.get("score") is None: continue
            lg = glob.glob(os.path.join(r["out_dir"], "*_PySys_Controller.log"))
            if lg: out.append((r, lg[0]))
    return out
def analyse(r, lg):
    L = [x.split("OnScreenLogMessage: ", 1)[1] for x in open(lg, errors="ignore") if "OnScreenLogMessage: " in x]
    S = [(float(a), float(b), float(c), float(d), float(e), f, float(g)) for a, b, c, d, e, f, g in (m.groups() for m in map(ST.search, L) if m)]
    P = [(float(a), b, float(c), d, float(e), float(f)) for a, b, c, d, e, f in (m.groups() for m in map(PL.search, L) if m)]
    f60 = next((x[6] for x in S if x[0] >= 60), np.nan); v60 = next((x[2] for x in S if x[0] >= 60), np.nan)
    vmax = max((x[2] for x in S if x[0] < 90), default=np.nan)
    rem_at = lambda t: next((x[1] for x in S if x[0] >= t), 0)
    fuel_at = lambda t: next((x[6] for x in S if x[0] >= t), S[-1][6])
    d = dict(score=r["score"], fuel=r["fuel_kg"], f60=f60, v60=v60, vmax=vmax, man=r["fuel_kg"] - f60,
             n_plan=len(P), n_avoid=sum(1 for p in P if p[1] == "avoid"),
             avoid_lowP=sum(1 for p in P if p[1] == "avoid" and p[4] < 0.3 and not p[3].startswith("None")),
             dv_avoid_lowP=sum(p[2] for p in P if p[1] == "avoid" and p[4] < 0.3),
             dv_avoid_hiP=sum(p[2] for p in P if p[1] == "avoid" and p[4] >= 0.3),
             near_aim=sum(1 for p in P if p[1] == "aim" and rem_at(p[0]) < 600),
             dv_near_aim=sum(p[2] for p in P if p[1] == "aim" and rem_at(p[0]) < 600),
             rec=sum(1 for x in L if "RECOVERY" in x), att60=sum(1 for x in S if x[0] > 15 and x[3] > 60 and x[5] != "tumble"),
             fuel_near=r["fuel_kg"] - fuel_at(min((x[0] for x in S if x[1] < 600), default=1e9)),
             dmg=r["damage_J"] or 0)
    return d
for arm in ("base", "cand"):
    D = [analyse(r, lg) for r, lg in flights(arm)]
    D = [d for d in D if d["dmg"] == 0]
    k = lambda key: np.array([d[key] for d in D])
    print(f"{arm}: n={len(D)} undamaged | score med {np.median(k('score')):.1f} | fuel med {np.median(k('fuel')):.1f} | fuel@60 med {np.nanmedian(k('f60')):.1f} "
          f"| v@60 med {np.nanmedian(k('v60')):.2f} vmax med {np.nanmedian(k('vmax')):.2f} | fuel after 60 s med {np.nanmedian(k('man')):.1f} "
          f"| fuel inside 600 m med {np.median(k('fuel_near')):.1f}")
    print(f"   per flight mean: plan switches {k('n_plan').mean():.1f}, avoid {k('n_avoid').mean():.1f}, avoid with worst P<0.3 {k('avoid_lowP').mean():.1f} "
          f"(dv sum {k('dv_avoid_lowP').mean():.2f}), avoid P>=0.3 dv sum {k('dv_avoid_hiP').mean():.2f}, near-ship aim switches {k('near_aim').mean():.1f} "
          f"(dv {k('dv_near_aim').mean():.2f}), RECOVERY lines {k('rec').sum()} total, s with nose>60 outside tumble {k('att60').mean():.1f}")
    if "-v" in sys.argv:
        for d in sorted(D, key=lambda d: d["score"]):
            if d["score"] < 48:
                print("   " + " ".join(f"{a}={(round(b, 2) if isinstance(b, float) else b)}" for a, b in d.items()))
