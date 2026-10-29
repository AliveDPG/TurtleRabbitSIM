"""
analyze_dump.py - What is wrong with the radar returns?  Offline analysis of results/radar_dump.npz
(written by the `diag` controller variant: every radar read with our own position/velocity).

For each fresh return (unique stamp+relative vector) we form the world position
    z(tau) = shuttle_pos(stamp + offset + tau) + rel
and cluster returns into rocks (rocks are ~static, ~90 m apart).  The right de-skew time tau is the
one that makes repeated returns of the same rock agree, so we scan tau and report the within-rock
scatter; then we decompose the residuals along our velocity / radial / perpendicular directions,
and look at how they depend on speed and range.
"""
import sys
from pathlib import Path

import numpy as np

path = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parents[1] / "results" / "radar_dump.npz"
d = np.load(path)
t, E, P, V = d["t"], d["e"], d["p"], d["v"]
N = len(t)
print(f"{N} radar reads, t = {t[0]:.1f} .. {t[-1]:.1f} s, speed {np.linalg.norm(V[0]):.1f} .. {np.linalg.norm(V[-1]):.1f} m/s, entries/read {E.shape[1]}")

# clock offset: radar stamps vs our time (running min of t_now - newest stamp)
newest = E[:, :, 2, 0].max(axis=1)
offset = float(np.min(t - newest))
print(f"radar clock offset (ours - radar) = {offset:.3f} s")

# fresh returns only
seen = set()
fresh = []          # (read index, stamp, rel, diameter)
for i in range(N):
    for j in range(E.shape[1]):
        ts = float(E[i, j, 2, 0])
        if ts <= 1e-9:
            continue
        key = (round(ts, 4), round(float(E[i, j, 0, 0]), 2), round(float(E[i, j, 0, 1]), 2))
        if key in seen:
            continue
        seen.add(key)
        fresh.append((i, ts, E[i, j, 0].copy(), float(E[i, j, 1, 2])))
print(f"{len(fresh)} fresh returns")
if len(fresh) < 50:
    sys.exit("not enough data yet")

stamps = np.array([f[1] for f in fresh]) + offset
rels = np.array([f[2] for f in fresh])
diam = np.array([f[3] for f in fresh])
read_i = np.array([f[0] for f in fresh])


def pos_at(times):
    return np.column_stack([np.interp(times, t, P[:, k]) for k in range(3)])


def vel_at(times):
    return np.column_stack([np.interp(times, t, V[:, k]) for k in range(3)])


def cluster(z, radius=45.0):
    """Greedy clustering by distance (rocks are ~90 m apart, errors ~20 m)."""
    labels = -np.ones(len(z), int)
    centers = []
    for k in np.argsort(stamps):
        if centers:
            C = np.array(centers)
            dd = np.linalg.norm(C - z[k], axis=1)
            j = int(np.argmin(dd))
            if dd[j] < radius:
                labels[k] = j
                n = np.sum(labels == j)
                centers[j] = C[j] + (z[k] - C[j]) / n
                continue
        labels[k] = len(centers)
        centers.append(z[k].copy())
    return labels, np.array(centers)


def scatter_for(tau):
    z = pos_at(stamps + tau) + rels
    labels, centers = cluster(z)
    res = z - centers[labels]
    sizes = np.bincount(labels)
    multi = sizes[labels] >= 3
    rms = float(np.sqrt(np.mean(np.sum(res[multi] ** 2, axis=1)))) if multi.any() else np.nan
    return rms, len(centers), z, labels, centers, res


print("\n--- de-skew time scan: which shuttle-position time makes repeated returns of a rock agree? ---")
print(f"{'tau (s)':>8} {'rms scatter m':>14} {'clusters':>9}")
best = None
for tau in np.arange(-3.0, 3.01, 0.25):
    rms, nc, *_ = scatter_for(tau)
    print(f"{tau:>8.2f} {rms:>14.2f} {nc:>9}")
    if best is None or rms < best[0]:
        best = (rms, tau)
print(f"best tau = {best[1]:.2f} s (rms {best[0]:.2f} m); tau=0 means 'shuttle position at the stamp time'")

print("\n--- residual structure at tau = 0 ---")
rms0, nc, z, labels, centers, res = scatter_for(0.0)
sizes = np.bincount(labels)
multi = sizes[labels] >= 3
r = res[multi]
vv = vel_at(stamps[multi])
spd = np.linalg.norm(vv, axis=1)
vhat = vv / np.maximum(spd[:, None], 1e-6)
along = np.einsum("ij,ij->i", r, vhat)
radial = np.einsum("ij,ij->i", r, pos_at(stamps[multi]) / np.linalg.norm(pos_at(stamps[multi]), axis=1)[:, None])
perp = np.sqrt(np.maximum(np.sum(r ** 2, axis=1) - along ** 2, 0.0))
mag = np.linalg.norm(r, axis=1)
rng = np.linalg.norm(rels[multi], axis=1)
print(f"clusters: {nc} (expected ~200 rocks); returns in clusters with >=3 members: {multi.sum()}")
print(f"|residual|  median {np.median(mag):.1f} m   p90 {np.percentile(mag, 90):.1f} m   max {mag.max():.1f} m")
print(f"along-velocity component: median {np.median(along):+.1f} m, std {along.std():.1f} m")
print(f"perpendicular component : median {np.median(perp):.1f} m")
print(f"radial (Phobos-up) comp : median {np.median(radial):+.1f} m, std {radial.std():.1f} m")
for lo, hi in ((0, 3), (3, 8), (8, 12), (12, 20)):
    m = (spd >= lo) & (spd < hi)
    if m.sum() > 10:
        print(f"  speed {lo:>2}-{hi:<2} m/s: n={m.sum():4d}  |res| median {np.median(mag[m]):5.1f} m  p90 {np.percentile(mag[m], 90):5.1f}  along std {along[m].std():5.1f}  perp median {np.median(perp[m]):5.1f}")
for lo, hi in ((0, 100), (100, 200), (200, 400), (400, 2000)):
    m = (rng >= lo) & (rng < hi)
    if m.sum() > 10:
        print(f"  range {lo:>4}-{hi:<4} m: n={m.sum():4d}  |res| median {np.median(mag[m]):5.1f} m  p90 {np.percentile(mag[m], 90):5.1f}")
# is the error bimodal (a few rocks jump) or everywhere?
per_cluster = np.array([np.sqrt(np.mean(np.sum(res[labels == c] ** 2, axis=1))) for c in range(nc) if sizes[c] >= 3])
print(f"per-rock rms: median {np.median(per_cluster):.1f} m, p90 {np.percentile(per_cluster, 90):.1f} m, rocks with rms > 10 m: {np.mean(per_cluster > 10) * 100:.0f}%")
# stamp spacing per rock (refresh period)
gaps = []
for c in range(nc):
    s = np.sort(stamps[labels == c])
    if len(s) > 2:
        gaps.extend(np.diff(s))
print(f"refresh period per rock: median {np.median(gaps):.2f} s (radar design: 200 objects / 120 Hz = 1.67 s)")
# raw radar diameter consistency
dd = np.array([np.std(diam[labels == c]) / max(np.mean(diam[labels == c]), 1e-6) for c in range(nc) if sizes[c] >= 3])
print(f"diameter noise within rock: median {np.median(dd) * 100:.1f} % (design 5 %)")
