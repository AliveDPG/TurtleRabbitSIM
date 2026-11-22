"""Milestone 5 item C: launch efficiency per arm. waste(t) = fuel(t) - 4.32 kg/(m/s) x speed(t) at t = 15/20/30/45 s,
plus pitch-over numbers (time to att < 5 deg, fuel then). Undamaged scored flights; arms from .arm files.
usage: python robust/tools/launch_waste.py 'conf*' 'm4c*' 'm5a*'"""
import json, glob, os, re, sys
import numpy as np
ST = re.compile(r"t=\s*([\d.]+) rem=\s*([\d.]+) \|v\|=\s*([\d.]+).*?att=\s*([\d.]+) w=([\d.]+) mode=(\w+).*?fuel=\s*([\d.]+) dmg=(\d+)")
def rows(pat):
    for f in glob.glob(f"results/eval/{pat}_?/runs.jsonl"):
        b = os.path.basename(os.path.dirname(f)); a = f"results/eval/{b}.arm"
        if not os.path.exists(a): continue
        arm = json.load(open(a))["arm"]; arm = "cand" if arm in ("cand", "r3") else arm
        for l in open(f):
            r = json.loads(l)
            if r.get("score") is None or r.get("damage_J"): continue
            lg = glob.glob(os.path.join(r["out_dir"], "*_PySys_Controller.log"))
            if not lg: continue
            S = [(float(m.group(1)), float(m.group(3)), float(m.group(4)), m.group(6), float(m.group(7))) for m in map(ST.search, open(lg[0], errors="ignore")) if m]
            if not S: continue
            at = lambda t, k: next((x[k] for x in S if x[0] >= t), np.nan)
            po = next((x for x in S if x[0] > 6 and x[2] < 5), None)
            yield arm, [at(t, 4) - 4.32 * at(t, 1) for t in (15, 20, 30, 45)] + [po[0] if po else np.nan, po[4] if po else np.nan, at(30, 1)]
D = {}
for pat in sys.argv[1:]:
    for arm, x in rows(pat):
        D.setdefault(arm, []).append(x)
for arm, X in sorted(D.items()):
    X = np.array(X)
    print(f"{arm}: n={len(X)} waste@15/20/30/45 s median {np.nanmedian(X[:, 0]):.2f} {np.nanmedian(X[:, 1]):.2f} {np.nanmedian(X[:, 2]):.2f} {np.nanmedian(X[:, 3]):.2f} kg | "
          f"pitch-over done (att<5) t {np.nanmedian(X[:, 4]):.1f} s fuel {np.nanmedian(X[:, 5]):.2f} kg | v@30 {np.nanmedian(X[:, 6]):.2f} m/s")
