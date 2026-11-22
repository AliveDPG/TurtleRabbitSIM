"""Milestone 5 table for batches PREFIX<i>_{a,b} with arms from results/eval/<batch>.arm.
Rows: flights, undamaged 43+, undamaged <43, contacts, lost, unfinished, median, mean, worst, p10, median fuel, Fisher p;
plus launch numbers (fuel and time at the end of the launch burn, fuel at 15 s / 30 s, speed at 60 s) for item C.
usage: python robust/tools/m5table.py PREFIX [ARM_X ARM_Y] [-v]"""
import json, glob, os, re, sys
import numpy as np
from math import comb
ST = re.compile(r"t=\s*([\d.]+) rem=\s*([\d.]+) \|v\|=\s*([\d.]+).*?mode=(\w+).*?paused=(\d+)s fuel=\s*([\d.]+)")


def fisher(a, n1, b, n2):
    K = a + b; N = n1 + n2
    pr = lambda x: comb(n1, x) * comb(n2, K - x) / comb(N, K)
    p0 = pr(a)
    return sum(pr(x) for x in range(max(0, K - n2), min(K, n1) + 1) if pr(x) <= p0 + 1e-12)


def dmg(r):
    if r.get("damage_J"): return r["damage_J"]
    p = os.path.join(r.get("out_dir") or "", "result.json")
    try:
        return json.load(open(p))["harness"].get("damage_J", 0) if os.path.exists(p) else 0
    except Exception:
        h = json.load(open(p)).get("harness")
        m = re.search(r"'damage_J': ([\d.e+]+)", str(h))
        return float(m.group(1)) if m else 0


def launch(r):
    lg = glob.glob(os.path.join(r.get("out_dir") or "", "*_PySys_Controller.log"))
    if not lg: return None
    S = [(float(a), float(b), float(c), d, int(e), float(f)) for a, b, c, d, e, f in
         (m.groups() for m in map(ST.search, (x for x in open(lg[0], errors="ignore") if "OnScreenLogMessage" in x)) if m)]
    if not S: return None
    end = next((x for x in S if x[0] > 10 and x[3] not in ("burn", "pad")), None)
    at = lambda t, k: next((x[k] for x in S if x[0] >= t), np.nan)
    return dict(t_end=end[0] if end else np.nan, f_end=end[5] if end else np.nan, f15=at(15, 5), f30=at(30, 5), v60=at(60, 2),
                paused=max(x[4] for x in S if x[0] < 90))


def collect(prefix):
    arms = {}
    for f in sorted(glob.glob(f"results/eval/{prefix}[0-9]*_?/runs.jsonl")):
        b = os.path.basename(os.path.dirname(f))
        a = f"results/eval/{b}.arm"
        if not os.path.exists(a): continue
        arm = json.load(open(a))["arm"]
        for l in open(f):
            r = json.loads(l); r["_batch"] = b
            arms.setdefault(arm, []).append(r)
    return arms


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    prefix = args[0]
    arms = collect(prefix)
    order = args[1:3] if len(args) >= 3 else sorted(arms)
    out = {}
    for arm in order:
        R = arms.get(arm, [])
        S = [r for r in R if r.get("score") is not None]
        unf = [r for r in R if r.get("score") is None and "MaxSimTime" in str(r.get("error"))]
        rig = [r for r in R if r.get("score") is None and r not in unf]
        contacts = sum(1 for r in S + unf if dmg(r) > 0)
        lost = sum(1 for r in S if r.get("outcome") == "loss")
        sc = np.array([r["score"] for r in S]); fu = np.array([r["fuel_kg"] for r in S])
        n = len(S) + len(unf)
        out[arm] = (contacts, lost + len(unf), n)
        print(f"{arm}: flights {n} (scored {len(S)}, unfinished {len(unf)}; rig failures not counted {len(rig)})")
        if len(S):
            print(f"   undamaged 43+ {sum(1 for r in S if not dmg(r) and r['score'] >= 43)}, undamaged <43 {sum(1 for r in S if not dmg(r) and r['score'] < 43)}, "
                  f"contacts {contacts}, lost {lost}, unfinished {len(unf)}")
            print(f"   median {np.median(sc):.1f} mean {sc.mean():.1f} worst {sc.min():.1f} p10 {np.percentile(sc, 10):.1f} | fuel median {np.median(fu):.1f} kg")
            L = [launch(r) for r in S if not dmg(r)]
            L = [x for x in L if x]
            if L:
                k = lambda key: np.array([x[key] for x in L], float)
                print(f"   launch (undamaged, n={len(L)}): burn end t med {np.nanmedian(k('t_end')):.1f} s, fuel at burn end med {np.nanmedian(k('f_end')):.2f} kg, "
                      f"fuel@15 {np.nanmedian(k('f15')):.2f}, fuel@30 {np.nanmedian(k('f30')):.2f}, v@60 {np.nanmedian(k('v60')):.2f}, flights with a launch pause {int((k('paused') > 0).sum())}")
            if "-v" in sys.argv:
                print("   scores: " + " ".join(f"{s:.1f}" for s in sorted(sc)))
    if len(order) == 2 and all(a in out for a in order):
        (cx, lx, nx), (cy, ly, ny) = out[order[0]], out[order[1]]
        print(f"Fisher two-sided: contacts {cx}/{nx} vs {cy}/{ny} p={fisher(cx, nx, cy, ny):.3f}; lost-or-unfinished {lx}/{nx} vs {ly}/{ny} p={fisher(lx, nx, ly, ny):.3f}")
