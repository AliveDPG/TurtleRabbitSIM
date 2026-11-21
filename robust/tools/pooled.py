"""Pooled final table: m2fin (single lane: a = frozen submission/, b = candidate) + conf* (two lanes, arms per
.arm files).  Rows as before plus Fisher exact p (two-sided) for contacts and for lost-or-unfinished."""
import json, glob, os, math
import numpy as np
def fisher(a, n1, b, n2):
    # two-sided Fisher exact test for a/n1 vs b/n2
    from math import comb
    K = a + b; N = n1 + n2
    p0 = comb(n1, a) * comb(n2, b) / comb(N, K)
    return sum(comb(n1, x) * comb(n2, K - x) / comb(N, K) for x in range(max(0, K - n2), min(K, n1) + 1)
               if comb(n1, x) * comb(n2, K - x) / comb(N, K) <= p0 + 1e-12)
arms = {"base": [], "cand": []}
src = {"base": {}, "cand": {}}
def add(arm, f, tag):
    for l in open(f):
        r = json.loads(l); r["_src"] = tag; arms[arm].append(r)
        src[arm][tag] = src[arm].get(tag, 0) + 1
add("base", "results/eval/m2fin_a/runs.jsonl", "m2fin single-lane")
add("cand", "results/eval/m2fin_b/runs.jsonl", "m2fin single-lane")
for f in sorted(glob.glob("results/eval/conf*_?/runs.jsonl")):
    b = os.path.basename(os.path.dirname(f))
    arm = json.load(open(f"results/eval/{b}.arm"))["arm"]
    lane = "two-lane" if os.path.exists(f"results/eval/{b[:-2]}.pair.log") else "single-lane"
    add(arm, f, f"conf {lane}")
out = {}
for arm, R in arms.items():
    S = [r for r in R if r.get("score") is not None]
    unf = [r for r in R if r.get("score") is None and "MaxSimTime" in str(r.get("error"))]
    rig = [r for r in R if r.get("score") is None and r not in unf]
    def dmg(r):
        if r.get("damage_J"): return r["damage_J"]
        p = os.path.join(r.get("out_dir") or "", "result.json")
        return json.load(open(p))["harness"].get("damage_J", 0) if os.path.exists(p) else 0
    contacts = sum(1 for r in S + unf if dmg(r) > 0)
    lost = sum(1 for r in S if r.get("outcome") == "loss") + len(unf)
    sc = np.array([r["score"] for r in S]); fu = np.array([r["fuel_kg"] for r in S])
    n = len(S) + len(unf)
    out[arm] = (contacts, lost, n)
    print(f"{arm}: flights={n} (scored {len(S)}, unfinished {len(unf)}; rig fails not counted: {len(rig)}) sources={src[arm]}")
    print(f"   undamaged 43+ {sum(1 for r in S if not r['damage_J'] and r['score'] >= 43)}, undamaged <43 {sum(1 for r in S if not r['damage_J'] and r['score'] < 43)}, "
          f"contacts {contacts}, lost {sum(1 for r in S if r.get('outcome') == 'loss')}, unfinished {len(unf)}")
    print(f"   median {np.median(sc):.1f} mean {sc.mean():.1f} worst {sc.min():.1f} p10 {np.percentile(sc, 10):.1f} | fuel median {np.median(fu):.1f} kg, >70 kg {int((fu > 70).sum())}")
cb, lb, nb = out["base"]; cc, lc, nc = out["cand"]
print(f"Fisher two-sided: contacts {cc}/{nc} vs {cb}/{nb}: p={fisher(cc, nc, cb, nb):.3f}; lost-or-unfinished {lc}/{nc} vs {lb}/{nb}: p={fisher(lc, nc, lb, nb):.3f}")
