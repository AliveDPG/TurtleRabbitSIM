"""
sim_offline.py - Point-mass rehearsal of nav.py without Space Teams PRO.

Reproduces the competition numbers (0.5 m/s^2 cap, 500 debris of which the
radar reports 200, 120 Hz round-robin radar with 2 m / 0.02 m/s / 5 % noise,
1.3 km climb to a 50 m keep-out) so the tracker and guidance can be tuned and
regression-tested from the command line:

    python sim_offline.py            # 5 seeds
    python sim_offline.py 20         # 20 seeds
"""
import math
import sys
import time

import numpy as np

from nav import DebrisTracker, Guidance, GuidanceParams, ShuttleNav, unit

MU = 7.075e-4 * 1e9          # m^3/s^2
R_PHOBOS = 11_080.0          # m
P_START = np.array([-651.02, 95.20, -8239.67])
P_GOAL = np.array([-850.0, 120.0, -9530.0])
SHUTTLE_R = 3.0


def spawn_debris(rng, n=500):
    up = unit(P_START)
    north = unit(np.cross(np.cross(up, np.array([0, 0, 1.0])), up))
    west = np.cross(up, north)
    centre = P_START + 500.0 * up
    spread = np.array([300.0, 300.0, 500.0])
    pos = centre + (rng.random((n, 3)) * 2 - 1) * spread
    vmax = math.sqrt(MU / R_PHOBOS)
    vmag = rng.uniform(0.05 * vmax, 0.1 * vmax, n)
    az = rng.uniform(0, 2 * math.pi, n)
    vel = (vmag * np.cos(az))[:, None] * north + (vmag * np.sin(az))[:, None] * west
    vel += rng.normal(0, 0.02, (n, 3))
    kinds = rng.choice(4, n, p=[0.45, 0.35, 0.15, 0.05])
    base = np.array([0.08, 0.05, 0.01, 17.0])
    smin = np.array([10, 20, 40, 1.0])
    smax = np.array([400, 800, 1000, 1.0])
    scale = smin[kinds] + rng.random(n) * (smax[kinds] - smin[kinds])
    diam = base[kinds] * scale
    return pos, vel, diam


def run(seed, params=None, verbose=False, radar_n=200, dt=1 / 60, camera=False, radar_offset=0.0, range_rate=True):
    """radar_offset: seconds the radar's clock lags the controller's (stamps = t - radar_offset).
    range_rate: feed the radar range-rate to the tracker (gate + update)."""
    rng = np.random.default_rng(seed)
    pos_d, vel_d, diam = spawn_debris(rng)
    visible = np.arange(radar_n)                       # radar only publishes the first N
    last_cam, ang_sig, cam_meas = -1e9, 1.5 / 333.0, 0

    p, v = P_START.copy(), np.zeros(3)
    nav = ShuttleNav()
    tracker = DebrisTracker()
    guide = Guidance(P_START, P_GOAL, params or GuidanceParams())

    # radar model (Radar.py): 120 Hz round-robin, sorted list republished every N/10 ticks
    radar_data = {}
    radar_tick = 0
    radar_published = []
    for i in visible:
        rr0 = float(np.dot(vel_d[i] - v, unit(pos_d[i] - p)))      # initial scan carries range-rate too
        radar_data[i] = (pos_d[i] - p + rng.normal(0, 2, 3), rr0 + rng.normal(0, 0.02),
                         diam[i] * (1 + rng.normal(0, 0.05)), 0.0)
    radar_published = sorted(radar_data.items(), key=lambda kv: np.linalg.norm(kv[1][0]))

    t = 0.0
    min_clear = 1e9
    min_clear_hidden = 1e9
    collisions = 0
    fuel = 0.0
    modes = {}
    hit_ids = set()
    err_v, err_p = [], []                              # track errors vs truth, tracks within 300 m
    while t < 400.0:
        # shuttle nav fix
        nav.fix(t, p, v)

        # radar: 2 radar ticks per 60 Hz controller tick, each with its own stamp
        for k in range(2):
            i = visible[radar_tick % radar_n]
            rel = pos_d[i] - p
            rr = float(np.dot(vel_d[i] - v, unit(rel)))
            radar_data[i] = (rel + rng.normal(0, 2, 3), rr + rng.normal(0, 0.02),
                             diam[i] * (1 + rng.normal(0, 0.05)), t + k / 120.0 - radar_offset)
            radar_tick += 1
            if radar_tick % (radar_n // 10) == 0:
                radar_published = sorted(radar_data.items(), key=lambda kv: np.linalg.norm(kv[1][0]))
        if int(round(t / dt)) % 6 == 0:
            entries = [np.array([[d[0][0], d[0][1], d[0][2]], [0.0, d[1], d[2]], [d[3], 0, 0]])
                       for _, d in radar_published]
            tracker.ingest_radar(entries, nav.pos_at, t, nav.vel_at if range_rate else None)
            tracker.prune(t)
            if int(round(t / dt)) % 60 == 0 and t > 5.0 and tracker.tracks:
                snap = tracker.snapshot(t)
                near = np.linalg.norm(snap.pos - p, axis=1) < 300.0
                for k in np.nonzero(near)[0]:
                    dd = np.linalg.norm(pos_d[visible] - snap.pos[k], axis=1)
                    j = visible[int(np.argmin(dd))]
                    if dd.min() < 15.0:
                        err_p.append(dd.min())
                        err_v.append(np.linalg.norm(snap.vel[k] - vel_d[j]))

        # camera model: forward-looking, 60 deg FOV, depth available, 1.5 s cadence,
        # detects anything subtending > 1.5 px at 384 px / 60 deg (f ~ 333 px)
        if camera and t - last_cam >= 1.5 and np.linalg.norm(v) > 2.0:
            last_cam = t
            boresight = unit(v)
            rel = pos_d - p
            rng_all = np.linalg.norm(rel, axis=1)
            cosang = (rel @ boresight) / np.maximum(rng_all, 1e-6)
            ang_px = diam / np.maximum(rng_all, 1e-6) * 333.0
            seen = np.nonzero((cosang > math.cos(math.radians(30))) & (rng_all < 600.0) & (ang_px > 1.5))[0]
            for i in seen:
                d_true = rel[i] / rng_all[i]
                e1, e2 = unit(np.cross(d_true, boresight if abs(d_true @ boresight) < 0.99 else np.array([0, 1.0, 0]))), None
                e2 = np.cross(d_true, e1)
                d_meas = unit(d_true + ang_sig * (rng.normal() * e1 + rng.normal() * e2))
                tracker.ingest_bearing(t, p, d_meas, ang_sig, range_m=rng_all[i] + rng.normal(0, 1.0),
                                       range_sigma=1.0, ang_size=diam[i] / rng_all[i])
                cam_meas += 1

        out = guide.command(t, nav.pos, nav.vel, tracker.tracks)
        modes[out.mode] = modes.get(out.mode, 0) + 1
        if out.done:
            break
        a = out.accel_world
        fuel += np.linalg.norm(a) * dt / (80.0 * 9.8)

        # integrate (gravity ~1e-5 m/s^2, ignored); debris drift
        p = p + v * dt + 0.5 * a * dt * dt
        v = v + a * dt
        pos_d = pos_d + vel_d * dt
        t += dt

        # clearance bookkeeping
        d = np.linalg.norm(pos_d - p, axis=1) - 0.5 * diam - SHUTTLE_R
        j = int(np.argmin(d))
        if j < radar_n:
            min_clear = min(min_clear, d[j])
        else:
            min_clear_hidden = min(min_clear_hidden, d[j])
        for k in np.nonzero(d < 0)[0]:
            if k not in hit_ids:
                hit_ids.add(int(k))
                collisions += 1
                if verbose:
                    print(f"  HIT debris {k} ({'radar' if k < radar_n else 'hidden'}) diam={diam[k]:.1f} t={t:.1f}")

    arrived = out.done
    return dict(seed=seed, t=t, arrived=arrived, collisions=collisions, hits_radar=sum(1 for k in hit_ids if k < radar_n),
                hits_hidden=sum(1 for k in hit_ids if k >= radar_n), min_clear=min_clear,
                min_clear_hidden=min_clear_hidden, fuel=fuel, tracks=len(tracker.tracks),
                cam_tracks=sum(1 for tr in tracker.tracks if tr.source == "camera"), cam_meas=cam_meas,
                offset_est=tracker.time_offset,
                trim_ticks=modes.get("trim-avoid", 0), modes=modes,
                err_p=np.median(err_p) if err_p else float('nan'), err_v=np.median(err_v) if err_v else float('nan'),
                err_v90=np.percentile(err_v, 90) if err_v else float('nan'))


def report(label, results):
    print(f"--- {label} ---")
    for r in results:
        print(f"seed {r['seed']:2d}: {'ARRIVED' if r['arrived'] else 'FAILED '} t={r['t']:6.1f}s "
              f"hits radar/hidden={r['hits_radar']}/{r['hits_hidden']} minClear radar={r['min_clear']:6.1f}m "
              f"hidden={r['min_clear_hidden']:6.1f}m fuel={r['fuel']:.3f}kg tracks={r['tracks']}(+{r['cam_tracks']}cam, "
              f"{r['cam_meas']} cam meas) offset_est={r['offset_est']:.3f}s trim_ticks={r['trim_ticks']} "
              f"track err pos/vel/vel90={r['err_p']:.2f}m/{r['err_v']:.3f}/{r['err_v90']:.3f}m/s modes={r['modes']}")
    ts = [r['t'] for r in results if r['arrived']]
    print(f"{len(ts)}/{len(results)} arrived, mean time {np.mean(ts) if ts else float('nan'):.1f}s, "
          f"radar-visible hits {sum(r['hits_radar'] for r in results)}, hidden hits {sum(r['hits_hidden'] for r in results)}, "
          f"track err median pos {np.nanmedian([r['err_p'] for r in results]):.2f} m, "
          f"vel {np.nanmedian([r['err_v'] for r in results]):.3f} m/s (p90 {np.nanmedian([r['err_v90'] for r in results]):.3f})\n")


if __name__ == "__main__":
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 5
    t_wall = time.time()
    report("radar only, positions only (no range-rate)", [run(s, range_rate=False) for s in range(n)])
    report("radar only", [run(s) for s in range(n)])
    report("radar + depth camera", [run(s, camera=True) for s in range(n)])
    report("radar only, radar clock lags controller by 0.4 s", [run(s, radar_offset=0.4) for s in range(n)])
    print(f"wall {time.time() - t_wall:.1f}s")
