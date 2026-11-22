"""Per-arm table for batches PREFIX<i>_{a,b} (arms from results/eval/<batch>.arm).
'clean' = undamaged and no avoid plan change at all (no dodge, no burn pause).
usage: python fast/tools/table.py PREFIX [PREFIX2 ...] [-v]"""
import glob, json, os, re, sys
import numpy as np

AV = re.compile(r"PLAN avoid")


def flights(prefixes):
    arms = {}
    for pre in prefixes:
        for f in sorted(glob.glob(f"results/eval/{pre}[0-9]*_?/runs.jsonl")):
            b = os.path.basename(os.path.dirname(f))
            a = f"results/eval/{b}.arm"
            if not os.path.exists(a):
                continue
            arm = json.load(open(a))["arm"]
            for l in open(f):
                r = json.loads(l)
                h = r.get("harness") or {}
                if r.get("score") is None:
                    continue
                lg = glob.glob(os.path.join(r.get("out_dir") or "", "*_PySys_Controller.log"))
                nav = sum(1 for x in open(lg[0], errors="ignore") if AV.search(x)) if lg else -1
                arms.setdefault(arm, []).append(dict(score=float(r["score"]), dmg=float(h.get("damage_J") or 0),
                                                     end=h.get("end_kind"), t=h.get("t_end"), fuel=h.get("fuel_kg"),
                                                     nav=nav, batch=b, run=r.get("run_id")))
    return arms


def row(F):
    s = np.array([f["score"] for f in F])
    und = [f for f in F if f["dmg"] == 0 and f["end"] == "win"]
    clean = [f for f in und if f["nav"] == 0]
    n = len(F)
    sh = lambda x: np.mean(s >= x)
    return {
        "flights": n, "best": s.max(),
        "clean n / median score": f"{len(clean)} / " + (f"{np.median([f['score'] for f in clean]):.2f}" if clean else "-"),
        "57+ / 60+ / 65+": f"{sh(57):.0%} / {sh(60):.0%} / {sh(65):.0%}",
        "undamaged 43+ / <43": f"{sum(f['score'] >= 43 for f in und)} / {sum(f['score'] < 43 for f in und)}",
        "contacts / lost": f"{sum(f['dmg'] > 0 for f in F)} / {sum(f['end'] != 'win' for f in F)}",
        "median / mean / worst": f"{np.median(s):.2f} / {s.mean():.2f} / {s.min():.1f}",
        "median fuel / time": f"{np.median([f['fuel'] for f in F]):.1f} kg / {np.median([f['t'] for f in F if f['t']]):.1f} s",
        "pair both 57+ / 60+": f"{sh(57) ** 2:.1%} / {sh(60) ** 2:.1%}",
    }


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if a != "-v"]
    arms = flights(args)
    rows = {a: row(F) for a, F in arms.items()}
    keys = list(next(iter(rows.values())).keys())
    print("| | " + " | ".join(rows) + " |")
    print("|---|" + "---|" * len(rows))
    for k in keys:
        print(f"| {k} | " + " | ".join(str(rows[a][k]) if not isinstance(rows[a][k], float) else f"{rows[a][k]:.2f}" for a in rows) + " |")
    if "-v" in sys.argv:
        for a, F in arms.items():
            print(a, " ".join(f"{f['score']:.1f}({f['t']:.0f}s,{f['fuel']:.0f}kg,av{f['nav']})" for f in sorted(F, key=lambda z: -z['score'])))
