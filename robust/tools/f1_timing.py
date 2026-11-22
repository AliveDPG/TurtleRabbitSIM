"""Milestone 5 F1: loop-timing proxies vs outcome, pre-submission set vs today's control arm (logs only, no flights).
The controller does not log tick intervals; proxies from its own log:
  tick0     time of the first loop tick (s)
  gap       spacing of the ~1 s status lines (s): a long tick delays the next status line; median / p95 / max and the
            count of gaps > 1.5 s and > 2.0 s, for 0-70 s and for the whole flight
  fps       depth frames captured per second (max 4/s; each capture costs a tick), 10-60 s and 60 s-end
  wall      harness wall seconds per sim second (trajectory.csv), the sim's own pace
  over      pitch-over overshoot (largest att within 15 s after the nose first gets within 5 deg)
  f60       fuel at 60 s;  nsw  plan switches
usage: python robust/tools/f1_timing.py"""
import json, glob, os, re, sys
import numpy as np
ST = re.compile(r"t=\s*([\d.]+) rem=\s*([\d.]+) \|v\|=\s*([\d.]+).*?att=\s*([\d.]+) w=([\d.]+) mode=(\w+).*?frames=(\d+).*?fuel=\s*([\d.]+) dmg=(\d+)")
OLD = ["soak_2111_a", "soak_2111_b", "r7_gate30_h22_cam", "r7_gcam_c1", "r7_gcam_c2", "sub_gate_1", "sub_gate_2", "clean_room"]


def runs_old():
    for b in OLD:
        for l in open(f"results/eval/{b}/runs.jsonl"):
            r = json.loads(l); r["_b"] = b; yield r


def runs_arm(prefix, arm):
    for f in sorted(glob.glob(f"results/eval/{prefix}[0-9]*_?/runs.jsonl")):
        b = os.path.basename(os.path.dirname(f)); a = f"results/eval/{b}.arm"
        if os.path.exists(a) and json.load(open(a))["arm"] == arm:
            for l in open(f):
                r = json.loads(l); r["_b"] = b; yield r


def feat(r):
    od = r.get("out_dir") or ""
    lg = glob.glob(os.path.join(od, "*_PySys_Controller.log"))
    if r.get("score") is None or not lg: return None
    L = [x for x in open(lg[0], errors="ignore") if "OnScreenLogMessage" in x]
    t0 = next((float(m.group(1)) for m in (re.search(r"first loop tick at t=([\d.]+)", x) for x in L) if m), np.nan)
    S = [(float(m.group(1)), float(m.group(4)), int(m.group(7)), float(m.group(8))) for m in map(ST.search, L) if m]
    if len(S) < 20: return None
    t = np.array([s[0] for s in S]); g = np.diff(t); g70 = g[t[1:] <= 70]
    fr = np.array([s[2] for s in S])
    def fps(a, b):
        i, j = np.searchsorted(t, a), min(np.searchsorted(t, b), len(t) - 1)
        return (fr[j] - fr[i]) / max(t[j] - t[i], 1e-6) if j > i else np.nan
    i5 = next((k for k, s in enumerate(S) if s[0] > 6 and s[1] < 5), None)
    over = max((s[1] for s in S[i5:] if s[0] < S[i5][0] + 15), default=np.nan) if i5 is not None else np.nan
    f60 = next((s[3] for s in S if s[0] >= 60), np.nan)
    nsw = sum(1 for x in L if " PLAN " in x)
    wall = np.nan
    tc = os.path.join(od, "trajectory.csv")
    if os.path.exists(tc):
        T = np.genfromtxt(tc, delimiter=",", names=True)
        if len(T) > 50: wall = float((T["wall"][-1] - T["wall"][0]) / max(T["t_sim"][-1] - T["t_sim"][0], 1e-6))
    dmg = r.get("damage_J") or 0
    return dict(score=r["score"], contact=int(dmg > 0), tick0=t0, gmed=np.median(g70), gp95=np.percentile(g70, 95), gmax70=g70.max(),
                n15=int((g70 > 1.5).sum()), n20=int((g70 > 2.0).sum()), gmax=g.max(), fps1=fps(10, 60), fps2=fps(60, t[-1]),
                wall=wall, over=over, f60=f60, nsw=nsw, b=r["_b"])


def table(name, F):
    k = lambda key: np.array([f[key] for f in F], float)
    sc = k("score"); c = k("contact")
    print(f"{name}: n={len(F)} contacts {int(c.sum())} ({100 * c.mean():.0f} %) median {np.median(sc):.1f}")
    for key in ("tick0", "gmed", "gp95", "gmax70", "n15", "n20", "gmax", "fps1", "fps2", "wall", "over", "f60", "nsw"):
        v = k(key); print(f"   {key:7s} median {np.nanmedian(v):7.3f}  p90 {np.nanpercentile(v, 90):7.3f}  mean {np.nanmean(v):7.3f}")


def terciles(name, F, key):
    v = np.array([f[key] for f in F], float); ok = np.isfinite(v)
    G = [f for f, o in zip(F, ok) if o]; v = v[ok]
    q = np.percentile(v, [33.3, 66.7])
    out = []
    for lo, hi, lab in ((-np.inf, q[0], "low"), (q[0], q[1], "mid"), (q[1], np.inf, "high")):
        S = [f for f, x in zip(G, v) if lo < x <= hi] if lab != "low" else [f for f, x in zip(G, v) if x <= hi]
        if not S: continue
        c = np.mean([f["contact"] for f in S]); m = np.median([f["score"] for f in S])
        out.append(f"{lab} (<= {hi:.2f}) n={len(S)} contacts {100 * c:.0f} % median {m:.1f}")
    print(f"   {name} by {key} tercile: " + " | ".join(out))


if __name__ == "__main__":
    old = [f for f in map(feat, runs_old()) if f]
    new = [f for f in map(feat, runs_arm("m5a", "base")) if f]
    r3 = [f for f in map(feat, runs_arm("m5a", "r3")) if f]
    table("pre-submission submission/ sets", old); table("today A control (robust off)", new); table("today A r3", r3)
    for key in ("gp95", "n15", "fps1", "over", "nsw"):
        terciles("pre", old, key); terciles("today ctrl", new, key)
    if "-v" in sys.argv:
        for f in new: print(f)
