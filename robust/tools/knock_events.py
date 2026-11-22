"""Milestone 5 (OFFLINE ONLY, truth): every rock-rock contact in the 12 gt flights after the first 10 s, whether the
knocked rock then came near the flown path, and how early a straight-line prediction from radar-quality tracks
(truth + noise: 3 m position, 0.15 m/s velocity per axis, 1 sigma) would have flagged the pair's close approach.
usage: python robust/tools/knock_events.py"""
import glob, os, numpy as np
ROOT = "C:/Users/darpan/Documents/SpaceTeams/TurtleRabbitSIM"
HULL = 3.2
rng = np.random.default_rng(1)
SP, SV = 3.0, 0.15
tot = dict(events=0, near=0)
rows = []
for f in sorted(glob.glob(f"{ROOT}/results/tracking_audit/cache/gt_*.npz")):
    stamp = os.path.basename(f)[3:-4]
    z = np.load(f, allow_pickle=True); t = z["times"]; P = z["pos"]; V = z["vel"].astype(float); R = z["radii"]; nm = list(z["names"])
    T = np.genfromtxt(f"{ROOT}/results/runs/{stamp}/trajectory.csv", delimiter=",", names=True)
    sp = np.stack([np.interp(t, T["t_sim"], T[c]) for c in "xyz"], 1)
    dvs = np.linalg.norm(np.diff(V, axis=0), axis=2)            # (n-1, 500)
    done = set()
    for i in range(len(t) - 1):
        if t[i] < 10.0: continue
        ks = np.where(dvs[i] > 1.0)[0]
        for a in ks:
            if (a, i // 10) in done: continue
            g = np.linalg.norm(P[i] - P[i, a], axis=1) - R - R[a]; g[a] = 1e9
            b = int(np.argmin(g))
            if g[b] > 3.0:          # shuttle contact or unexplained
                continue
            for x in (a, b):
                for k in range(-2, 3): done.add((x, (i + k * 10) // 10))
            tot["events"] += 1
            # did either rock come within contact radius + 20 m of the shuttle within 40 s after the knock?
            w = (t > t[i]) & (t < t[i] + 40)
            near = []
            for x in (a, b):
                d = np.linalg.norm(P[w, x] - sp[w], axis=1) - R[x] - HULL
                if len(d) and d.min() < 20:
                    k = int(np.argmin(d)); near.append((nm[x], float(d[k]), float(t[w][k] - t[i]), float(dvs[i, x])))
            dsh = float(np.linalg.norm(P[i, a] - sp[i]))
            if not near: continue
            tot["near"] += 1
            # predictability: from the states at t_k - L (truth + track noise), closest approach of a and b
            pred = {}
            for L in (20, 15, 10, 6, 3):
                j = np.searchsorted(t, t[i] - L)
                if j < 1 or t[j] < 10: pred[L] = None; continue
                hits, gaps = 0, []
                for _ in range(200):
                    pa = P[j, a] + rng.normal(0, SP, 3); va = V[j, a] + rng.normal(0, SV, 3)
                    pb = P[j, b] + rng.normal(0, SP, 3); vb = V[j, b] + rng.normal(0, SV, 3)
                    dp, dv = pa - pb, va - vb
                    tc = np.clip(-dp @ dv / max(dv @ dv, 1e-9), 0, L + 10)
                    gap = np.linalg.norm(dp + dv * tc) - R[a] - R[b]
                    gaps.append(gap); hits += gap < 0
                pred[L] = (hits / 200.0, float(np.median(gaps)), float(np.std(gaps)))
            rows.append((stamp[:13], round(t[i], 1), nm[a], round(R[a], 1), nm[b], round(R[b], 1), round(dsh), near, pred))
for r in rows:
    print(*r[:7])
    for n in r[7]:
        print(f"      near: {n[0]} closest {n[1]:.1f} m outside contact radius, {n[2]:.1f} s after the knock, dv {n[3]:.1f} m/s")
    print("      predicted pair contact prob / median gap / sd at lead 20,15,10,6,3 s:",
          " | ".join("-" if r[8][L] is None else f"{r[8][L][0]:.2f} {r[8][L][1]:.0f} {r[8][L][2]:.0f}" for L in (20, 15, 10, 6, 3)))
print(tot)
