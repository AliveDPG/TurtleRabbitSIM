"""Milestone 4 comparison (m4c* batches): drops a flight whose controller log shows a MemoryError together with the
other lane's flight of the same repeat slot (the concurrent pair)."""
import json, glob, os, sys
import numpy as np
from math import comb
def fisher(a, n1, b, n2):
    K = a + b; N = n1 + n2; pr = lambda x: comb(n1, x) * comb(n2, K - x) / comb(N, K); p0 = pr(a)
    return sum(pr(x) for x in range(max(0, K - n2), min(K, n1) + 1) if pr(x) <= p0 + 1e-12)
prefix = sys.argv[1] if len(sys.argv) > 1 else "m4c"
A = {"base": [], "cand": []}; dropped = []
for d in sorted(glob.glob(f"results/eval/{prefix}*_a")):
    b = os.path.basename(d)[:-2]
    lanes = {}
    for ln in "ab":
        f = f"results/eval/{b}_{ln}/runs.jsonl"
        lanes[ln] = [json.loads(l) for l in open(f)] if os.path.exists(f) else []
    arm = {ln: json.load(open(f"results/eval/{b}_{ln}.arm"))["arm"] for ln in "ab"}
    def memerr(r):
        lg = glob.glob(os.path.join(r.get("out_dir") or "x", "*_PySys_Controller.log"))
        return bool(lg) and "MemoryError" in open(lg[0], errors="ignore").read()
    for i in range(max(len(lanes["a"]), len(lanes["b"]))):
        pair = [lanes[ln][i] if i < len(lanes[ln]) else None for ln in "ab"]
        if any(r is not None and memerr(r) for r in pair):
            dropped.append(b + f"#{i}"); continue
        for ln, r in zip("ab", pair):
            if r is not None: A[arm[ln]].append(r)
res = {}
for k, R in A.items():
    S = [r for r in R if r.get("score") is not None]
    unf = [r for r in R if r.get("score") is None and "MaxSimTime" in str(r.get("error"))]
    rig = len(R) - len(S) - len(unf)
    sc = np.array([r["score"] for r in S]); fu = np.array([r["fuel_kg"] for r in S])
    con = sum(1 for r in S if r["damage_J"] > 0); lost = sum(1 for r in S if r.get("outcome") == "loss")
    res[k] = (con, lost + len(unf), len(S) + len(unf))
    print(f"{k}: scored {len(S)} unfinished {len(unf)} rig-fail {rig} | undamaged 43+ {sum(1 for r in S if not r['damage_J'] and r['score'] >= 43)} "
          f"undamaged<43 {sum(1 for r in S if not r['damage_J'] and r['score'] < 43)} contacts {con} lost {lost} | median {np.median(sc):.1f} mean {sc.mean():.1f} "
          f"worst {sc.min():.1f} p10 {np.percentile(sc, 10):.1f} | fuel med {np.median(fu):.1f} >70 {int((fu > 70).sum())}")
    print("   ", sorted(round(x, 1) for x in sc))
(cc, lc, nc), (cb, lb, nb) = res["cand"], res["base"]
print(f"dropped pairs (MemoryError): {dropped}")
print(f"Fisher: contacts {cc}/{nc} vs {cb}/{nb} p={fisher(cc, nc, cb, nb):.2f}; lost-or-unfinished {lc}/{nc} vs {lb}/{nb} p={fisher(lc, nc, lb, nb):.2f}")
