"""Milestone 5 knock study on truth recordings (OFFLINE ONLY): for each gt flight, the kick history of chosen rocks
(velocity jumps between truth sweeps), the partner rock of each kick, and the closest approach to the shuttle.
usage: python robust/tools/knock_gt.py [Debris406 Debris208 ...]"""
import glob, os, sys, json, numpy as np
ROOT = "C:/Users/darpan/Documents/SpaceTeams/TurtleRabbitSIM"
names = sys.argv[1:] or ["Debris406", "Debris208", "Debris109"]
def load_traj(stamp):
    d = f"{ROOT}/results/runs/{stamp}"
    T = np.genfromtxt(f"{d}/trajectory.csv", delimiter=",", names=True)
    return T
def kicks(V, t, j, thr=0.5):
    dv = np.linalg.norm(np.diff(V[:, j, :], axis=0), axis=1)
    idx = np.where(dv > thr)[0]
    out = []
    for i in idx:
        if out and i - out[-1][0] <= 2: 
            if dv[i] > out[-1][1]: out[-1] = (i, dv[i])
            continue
        out.append((i, dv[i]))
    return out
for f in sorted(glob.glob(f"{ROOT}/results/tracking_audit/cache/gt_*.npz")):
    stamp = os.path.basename(f)[3:-4]
    z = np.load(f, allow_pickle=True); t = z["times"]; P = z["pos"]; V = z["vel"].astype(float); R = z["radii"]; nm = list(z["names"])
    T = load_traj(stamp)
    sp = np.stack([np.interp(t, T["t_sim"], T[c]) for c in "xyz"], 1)
    print(f"=== {stamp[:13]} t_end {t[-1]:.1f}")
    for n in names:
        j = nm.index(n)
        d = np.linalg.norm(P[:, j] - sp, axis=1) - R[j]
        k = int(np.argmin(d))
        line = f"  {n} r={R[j]:.1f} closest surf {d[k]:.1f} m at t={t[k]:.1f}; speed at 5/30/45/50/55 s: " + " ".join(f"{np.linalg.norm(V[np.searchsorted(t, s), j]):.1f}" for s in (5, 30, 45, 50, 55) if s < t[-1])
        print(line)
        for i, dvm in kicks(V, t, j):
            if t[i] > t[k] + 3: break
            g = np.linalg.norm(P[i, :] - P[i, j], axis=1) - R - R[j]; g[j] = 1e9
            p = int(np.argmin(g))
            dsh = np.linalg.norm(P[i, j] - sp[i])
            print(f"    kick t={t[i]:.1f} dv={dvm:.2f} v {np.linalg.norm(V[i,j]):.1f}->{np.linalg.norm(V[i+1,j]):.1f}  partner {nm[p]} r={R[p]:.1f} gap {g[p]:.1f}  dist to shuttle {dsh:.0f} m")
