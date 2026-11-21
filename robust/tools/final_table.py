"""Final table for an alternating batch: per arm outcome classes, score stats, fuel stats.
usage: python robust/tools/final_table.py BATCH_a BATCH_b"""
import json, sys, numpy as np
for b in sys.argv[1:]:
    R = [json.loads(l) for l in open(f"results/eval/{b}/runs.jsonl")]
    S = [r for r in R if r.get("score") is not None]
    rig = [r for r in R if r.get("score") is None]
    unf = [r for r in rig if "MaxSimTime" in str(r.get("error"))]
    sc = np.array([r["score"] for r in S]); fu = np.array([r["fuel_kg"] for r in S])
    c43 = sum(1 for r in S if not r["damage_J"] and r["score"] >= 43)
    cl = sum(1 for r in S if not r["damage_J"] and r["score"] < 43)
    ct = sum(1 for r in S if r["damage_J"] > 0)
    print(f"{b}: scored={len(S)} rig_fail={len(rig) - len(unf)} unfinished={len(unf)} clean43={c43} clean<43={cl} contact={ct} "
          f"median={np.median(sc):.1f} mean={sc.mean():.1f} worst={sc.min():.1f} p10={np.percentile(sc, 10):.1f} "
          f"fuel_med={np.median(fu):.1f} fuel>70={int((fu > 70).sum())}")
    print("   scores:", sorted(round(x, 1) for x in sc))
    print("   errors:", [r.get("error") for r in rig])
