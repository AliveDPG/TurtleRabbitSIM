"""Summarise eval batches: outcome classes, score stats and per-flight log signatures.
usage: python robust/tools/summ.py BATCH [BATCH ...]   (-v for per-flight lines)"""
import json, os, glob, re, sys, statistics as stt
import numpy as np

ST = re.compile(r"t=\s*([\d.]+) rem=\s*([\d.]+) \|v\|=\s*([\d.]+).*?att=\s*([\d.]+) w=([\d.]+) mode=(\w+).*?fuel=\s*([\d.]+)")
PL = re.compile(r"t=\s*([\d.]+) PLAN (\w+):")
FT = re.compile(r"first loop tick at t=([\d.]+)s")
verbose = "-v" in sys.argv
for b in [a for a in sys.argv[1:] if a != "-v"]:
    f = f"results/eval/{b}/runs.jsonl"
    if not os.path.exists(f):
        print(b, "no runs.jsonl"); continue
    R = [json.loads(l) for l in open(f)]
    sc, cls = [], {"clean43": 0, "clean<43": 0, "contact": 0, "unfinished": 0, "rig_fail": 0}
    sig = []
    for r in R:
        s = r.get("score")
        d = r.get("damage_J") or 0
        lg = glob.glob(os.path.join(r.get("out_dir") or "x", "*_PySys_Controller.log"))
        if s is None:
            if not lg:
                cls["rig_fail"] += 1; continue
            cls["unfinished"] += 1
        elif d > 0:
            cls["contact"] += 1
        elif s >= 43:
            cls["clean43"] += 1
        else:
            cls["clean<43"] += 1
        if s is not None:
            sc.append(s)
        if not lg:
            continue
        L = [x.split("OnScreenLogMessage: ", 1)[1] for x in open(lg[0], errors="ignore") if "OnScreenLogMessage: " in x]
        S = [m.groups() for m in map(ST.search, L) if m]
        S = [(float(a), float(b_), float(c), float(d_), float(e), g, float(h)) for a, b_, c, d_, e, g, h in S]
        evt = [(float(m.group(1)), m.group(2)) for m in map(PL.search, L) if m]
        ev = [e for _, e in evt]
        t_plan1 = evt[0][0] if evt else 1e9          # pitch-over overshoot only before the first plan change
        flips = sum(1 for i in range(1, len(ev) - 1) if ev[i] == "aim" and ev[i - 1] == "avoid" and ev[i + 1] == "avoid")
        ft = next((float(m.group(1)) for m in map(FT.search, L) if m), S[0][0] if S else -1)
        w15 = max((x[4] for x in S if x[0] < 15), default=-1)
        w0 = S[0][4] if S else -1
        i0 = next((i for i, x in enumerate(S) if x[3] < 8.0), None)
        over = max((x[3] for x in S[i0:] if x[0] < min(30, t_plan1)), default=0) if i0 is not None else -1
        f50 = next((x[6] for x in S if x[0] >= 50), -1)
        rmin = min((x[1] for x in S), default=-1)
        sig.append((ft, w15, over, f50, flips))
        if verbose:
            print(f"   {b} score={s if s is None else round(s, 1)} t={r.get('elapsed_s')} fuel={r.get('fuel_kg')} dmg={d:.0f} "
                  f"tick0={ft:.2f} w@tick0={w0:.2f} w<15s={w15:.2f} overshoot={over:.1f} fuel@50={f50:.1f} flips={flips} rem_min={rmin:.0f}")
    a = np.array(sc) if sc else np.array([np.nan])
    sg = np.array(sig) if sig else np.full((1, 5), np.nan)
    print(f"{b}: n={len(R)} {cls} median={np.median(a):.1f} mean={a.mean():.1f} worst={a.min():.1f} p10={np.percentile(a, 10):.1f} "
          f"| tick0 med={np.median(sg[:, 0]):.2f} w<15 max={np.max(sg[:, 1]):.2f} overshoot med={np.median(sg[:, 2]):.1f} "
          f">=8deg:{int((sg[:, 2] >= 8).sum())} fuel@50 med={np.median(sg[:, 3]):.1f} flips tot={int(sg[:, 4].sum())} flights_with_flips={int((sg[:, 4] > 0).sum())}")
