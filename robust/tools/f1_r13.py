"""Milestone 5 F1: Debris208 (r13) conflicts vs launch timing. Per flight: overshoot, time to reach 1170 m from the
target (Debris208's crossing, ~135 m from the pad), and whether r13 reached P >= 0.3 / contact.
usage: python robust/tools/f1_r13.py"""
import sys, re, glob, os, json, numpy as np
sys.path.insert(0, os.path.dirname(__file__)); sys.argv = sys.argv[:1]
import f1_timing as F
ST = re.compile(r"t=\s*([\d.]+) rem=\s*([\d.]+) \|v\|=\s*([\d.]+).*?att=\s*([\d.]+)")
def info(r):
    lg = glob.glob(os.path.join(r.get("out_dir") or "", "*_PySys_Controller.log"))
    if r.get("score") is None or not lg: return None
    txt = open(lg[0], errors="ignore").read()
    S = [(float(a), float(b), float(c), float(d)) for a, b, c, d in ST.findall(txt)]
    i5 = next((k for k, s in enumerate(S) if s[0] > 6 and s[3] < 5), None)
    over = max((s[3] for s in S[i5:] if s[0] < S[i5][0] + 15), default=np.nan) if i5 is not None else np.nan
    t1170 = next((s[0] for s in S if s[1] < 1170), np.nan)
    v30 = next((s[2] for s in S if s[0] >= 30), np.nan)
    r13 = bool(re.search(r"THREATS r13\+?\S* P=0\.[3-9]", txt))
    hit208 = "Debris208" in txt and "COLLISION" in txt and bool(re.search(r"COLLISION.*Debris208", txt))
    return over, t1170, v30, r13, hit208, r.get("score")
for name, gen in (("pre", F.runs_old()), ("today base", F.runs_arm("m5a", "base")), ("today r3", F.runs_arm("m5a", "r3"))):
    X = np.array([x for x in map(info, gen) if x], float)
    print(f"{name}: n={len(X)} overshoot median {np.nanmedian(X[:,0]):.1f} deg, v@30 {np.nanmedian(X[:,2]):.2f} m/s, t(1170 m) median {np.nanmedian(X[:,1]):.1f} s | "
          f"r13 P>=0.3 {int(X[:,3].sum())} ({100*X[:,3].mean():.0f} %), Debris208 hits {int(X[:,4].sum())}")
    for lo, hi in ((0, 8), (8, 20), (20, 999)):
        m = (X[:, 0] >= lo) & (X[:, 0] < hi)
        if m.any(): print(f"   overshoot {lo}-{hi} deg: n={m.sum()} r13 conflict {100*X[m,3].mean():.0f} % hits {int(X[m,4].sum())} t(1170) {np.nanmedian(X[m,1]):.1f} s")
