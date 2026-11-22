"""Milestone 5 (OFFLINE ONLY, truth): exposure of a nominal no-dodge flight path to the recorded rock fields of the 12 gt
flights, for launch holds D and lateral offsets. Nominal path = straight pad->target line flown with the along-track
speed profile of a clean flight (time shifted by D); lateral offset ramps in over the first 300 m.
Per variant: conflicts (rock centre within contact radius r + 3.2 + 2 m of the path) per flight in 0-120 s, and the
clearance to Debris406 / Debris208.
usage: python robust/tools/exposure.py"""
import glob, os, numpy as np
ROOT = "C:/Users/darpan/Documents/SpaceTeams/TurtleRabbitSIM"
P0 = np.array([-651.0, 95.2, -8239.7]); TG = np.array([-850.0, 120.0, -9530.0])
u = (TG - P0) / np.linalg.norm(TG - P0)
up = P0 / np.linalg.norm(P0); e1 = np.cross(u, up); e1 /= np.linalg.norm(e1); e2 = np.cross(e1, u)
ref = np.genfromtxt(f"{ROOT}/results/runs/261119_012548_default_s8193_ts1_offscreen_L1/trajectory.csv", delimiter=",", names=True)
s_ref = (np.stack([ref[c] for c in "xyz"], 1) - P0) @ u
files = sorted(glob.glob(f"{ROOT}/results/tracking_audit/cache/gt_*.npz"))
F = [np.load(f, allow_pickle=True) for f in files]
def path(t, D, off):
    s = np.interp(t - D, ref["t_sim"], s_ref, left=0.0)
    lat = off * np.clip(s / 300.0, 0, 1)
    return P0[None, :] + s[:, None] * u[None, :] + lat[:, None] * e1[None, :]   # e1 roughly horizontal sideways
print("variant           conflicts/flight (0-120 s)  flights with 406 clr<10 m | 406 clr med  | 208 clr<10 m  | conflict times")
for D, off in [(0, 0), (3, 0), (6, 0), (10, 0), (15, 0), (0, 40), (0, -40), (0, 80), (0, -80)]:
    nconf, c406, c208, ctimes = [], [], [], []
    for z in F:
        t = z["times"]; P = z["pos"]; R = z["radii"]; nm = list(z["names"])
        w = t <= min(120.0, t[-1])
        if t[-1] < 100: continue          # only flights recorded past 100 s
        sp = path(t[w], D, off)
        d = np.linalg.norm(P[w] - sp[:, None, :], axis=2) - R[None, :] - 3.2
        dm = d.min(0)
        nconf.append(int((dm < 2.0).sum()))
        ctimes += [round(float(t[w][np.argmin(d[:, j])])) for j in np.where(dm < 2.0)[0]]
        c406.append(dm[nm.index("Debris406")]); c208.append(dm[nm.index("Debris208")])
    c406 = np.array(c406); c208 = np.array(c208)
    print(f"D={D:2d} off={off:4d}   {np.mean(nconf):5.2f} ({len(nconf)} fl)   {int((c406 < 10).sum()):2d}   {np.median(c406):6.1f}   {int((c208 < 10).sum()):2d}   {sorted(ctimes)}")
