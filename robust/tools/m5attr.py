"""Milestone 5 D: where undamaged flights of one arm lose points against a clean flight.
Per flight: score, time, fuel; fuel in phases (launch to 60 s, 60 s to 600 m from the ship, inside 600 m), launch pause s,
plan switches by reason with dv sums, |v| at 600 m / at 200 m, and the points of each phase against the arm's best
quartile (fuel 1.333 points/kg, time 12000/t^2 points/s).
usage: python robust/tools/m5attr.py PREFIX ARM [-v]"""
import json, glob, os, re, sys
import numpy as np
ST = re.compile(r"t=\s*([\d.]+) rem=\s*([\d.]+) \|v\|=\s*([\d.]+).*?mode=(\w+).*?paused=(\d+)s fuel=\s*([\d.]+) dmg=(\d+)")
PL = re.compile(r"t=\s*([\d.]+) PLAN (\w+): dv=([\d.]+).*?P=([\d.]+).*?kind=(\S+)")
prefix, arm = sys.argv[1], sys.argv[2]
rows = []
for f in sorted(glob.glob(f"results/eval/{prefix}[0-9]*_?/runs.jsonl")):
    b = os.path.basename(os.path.dirname(f)); a = f"results/eval/{b}.arm"
    if not os.path.exists(a) or json.load(open(a))["arm"] != arm: continue
    for l in open(f):
        r = json.loads(l)
        if r.get("score") is None or r.get("damage_J"): continue
        lg = glob.glob(os.path.join(r["out_dir"], "*_PySys_Controller.log"))[0]
        L = [x for x in open(lg, errors="ignore") if "OnScreenLogMessage" in x]
        S = [(float(m.group(1)), float(m.group(2)), float(m.group(3)), m.group(4), int(m.group(5)), float(m.group(6))) for m in map(ST.search, L) if m]
        P = [(float(m.group(1)), m.group(2), float(m.group(3)), float(m.group(4)), m.group(5)) for m in map(PL.search, L) if m]
        fat = lambda t: next((s[5] for s in S if s[0] >= t), S[-1][5])
        t600 = next((s[0] for s in S if s[1] < 600), S[-1][0]); t200 = next((s[0] for s in S if s[1] < 200), S[-1][0])
        v600 = next((s[2] for s in S if s[1] < 600), np.nan); v200 = next((s[2] for s in S if s[1] < 200), np.nan)
        pause = max(s[4] for s in S)
        dv = lambda reason, t0=0, t1=1e9: sum(p[2] for p in P if p[1] == reason and t0 <= p[0] < t1 and not p[4].startswith("pause"))
        rows.append(dict(score=r["score"], t=r["elapsed_s"], fuel=r["fuel_kg"], f60=fat(60), fmid=fat(t600) - fat(60), fend=r["fuel_kg"] - fat(t600),
                         t600=t600, tend=r["elapsed_s"] - t600, pause=pause, n_sw=len(P),
                         dv_avoid=dv("avoid"), dv_aim_mid=dv("aim", 60, t600) + dv("cheaper", 60, t600), dv_aim_end=dv("aim", t600) + dv("cheaper", t600),
                         dv_avoid_launch=dv("avoid", 0, 60), v600=v600, v200=v200))
D = {k: np.array([r[k] for r in rows]) for k in rows[0]}
q = D["score"] >= np.percentile(D["score"], 75)
print(f"{arm}: {len(rows)} undamaged flights; score median {np.median(D['score']):.1f}, best quartile median {np.median(D['score'][q]):.1f}")
for k in ("t", "fuel", "f60", "fmid", "fend", "t600", "tend", "pause", "n_sw", "dv_avoid", "dv_avoid_launch", "dv_aim_mid", "dv_aim_end", "v600", "v200"):
    print(f"   {k:16s} median {np.nanmedian(D[k]):7.2f}  best-quartile {np.nanmedian(D[k][q]):7.2f}  p90 {np.nanpercentile(D[k], 90):7.2f}")
tq = np.median(D["t"][q])
pts = dict(launch=1.333 * (D["f60"] - np.median(D["f60"][q])), cruise_fuel=1.333 * (D["fmid"] - np.median(D["fmid"][q])),
           arrival_fuel=1.333 * (D["fend"] - np.median(D["fend"][q])), time=12000 / tq ** 2 * (D["t"] - tq))
print("   points lost vs the best quartile (median / mean / sum over flights):")
for k, v in sorted(pts.items(), key=lambda kv: -kv[1].sum()):
    print(f"      {k:13s} {np.median(v):6.2f} / {v.mean():6.2f} / {v.sum():7.1f}")
if "-v" in sys.argv:
    for r in sorted(rows, key=lambda r: r["score"]):
        print("   " + " ".join(f"{k}={round(v, 1) if isinstance(v, float) else v}" for k, v in r.items()))
