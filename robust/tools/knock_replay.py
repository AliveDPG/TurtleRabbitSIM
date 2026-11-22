"""Milestone 5 (OFFLINE ONLY, uses truth): replay the recorded radar stream of a gt flight through robust/'s own
ShuttleNav + DebrisTracker and compare the controller's radar track of a chosen rock with the truth:
track error, claimed sigma, kicks seen, and the closest approach to the shuttle's actual future path predicted from
the track at several lead times before the true closest approach.
usage: python robust/tools/knock_replay.py [STAMP ...] [--rocks Debris406,Debris208]"""
import glob, json, os, sys
import numpy as np
ROOT = "C:/Users/darpan/Documents/SpaceTeams/TurtleRabbitSIM"
sys.path.insert(0, ROOT + "/robust")
import radar_tracking as rt

REC = "D:/STU4-recordings/STU4-evaluation"


def load_events(stamp):
    S = glob.glob(f"{REC}/eval-{stamp}/sensors/*")[0]
    ev = [json.loads(l) for l in open(S + "/events.jsonl")]
    return S, ev


def replay(stamp, rocks, leads=(30, 20, 15, 10, 6, 3), verbose=False):
    z = np.load(f"{ROOT}/results/tracking_audit/cache/gt_{stamp}.npz", allow_pickle=True)
    gt_t, gt_p, gt_v, gt_r, names = z["times"], z["pos"], z["vel"].astype(float), z["radii"], list(z["names"])
    T = np.genfromtxt(f"{ROOT}/results/runs/{stamp}/trajectory.csv", delimiter=",", names=True)
    S, ev = load_events(stamp)
    nav = rt.ShuttleNav(); trk = rt.DebrisTracker()
    rt.RadarTrack._next_id = 0
    J = [names.index(n) for n in rocks]
    log = {n: [] for n in rocks}
    for e in ev:
        k = e["kind"]; d = e["data"]
        if k == "shuttle_state":
            nav.fix(d["read_start_sim_s"], np.array(d["position_m"]), np.array(d["velocity_m_s"]))
        elif k == "radar":
            t = d["read_start_sim_s"]
            A = np.load(S + "/" + e["array"])
            trk.ingest_radar(list(A), nav.pos_at, t, nav.vel_at)
            trk.prune(t)
            i = min(np.searchsorted(gt_t, t), len(gt_t) - 1)
            for n, j in zip(rocks, J):
                tp, tv = gt_p[i, j], gt_v[i, j]
                best, bd = None, 1e9
                for tr in trk.tracks:
                    dd = np.linalg.norm(tr.pos_at(t) - tp)
                    if dd < bd and abs(tr.diameter - 2 * gt_r[j]) < 0.5 * 2 * gt_r[j] + 3:
                        best, bd = tr, dd
                if best is None or bd > 25:
                    log[n].append((t, None)); continue
                log[n].append((t, dict(id=best.id, perr=bd, verr=float(np.linalg.norm(best.vel - tv)), psig=best.pos_sigma(t),
                                       vsig=best.vel_sigma(), hits=best.hits, tkick=best.t_kick, p=best.pos_at(t).copy(), v=best.vel.copy())))
    # true closest approach and predictions
    out = []
    for n, j in zip(rocks, J):
        sp = np.stack([np.interp(gt_t, T["t_sim"], T[c]) for c in "xyz"], 1)
        dist = np.linalg.norm(gt_p[:, j] - sp, axis=1) - gt_r[j]
        kca = int(np.argmin(dist)); tca = gt_t[kca]; true_miss = dist[kca]
        rows = []
        for L in leads:
            tq = tca - L
            cand = [x for x in log[n] if x[0] <= tq]
            if not cand:
                rows.append((L, None)); continue
            t0, s = cand[-1]
            if s is None:
                rows.append((L, "untracked")); continue
            tt = np.linspace(t0, min(t0 + 60, T["t_sim"][-1]), 600)
            shp = np.stack([np.interp(tt, T["t_sim"], T[c]) for c in "xyz"], 1)
            rp = s["p"][None, :] + s["v"][None, :] * (tt - t0)[:, None]
            dd = np.linalg.norm(rp - shp, axis=1) - gt_r[j]
            m = int(np.argmin(dd))
            sig_ca = np.hypot(s["psig"], s["vsig"] * (tt[m] - t0))
            rows.append((L, dict(pred_miss=dd[m], pred_tca=tt[m], sig=sig_ca, perr=s["perr"], verr=s["verr"], vsig=s["vsig"], hits=s["hits"], id=s["id"])))
        first = next((x[0] for x in log[n] if x[1] is not None), None)
        ids = sorted(set(x[1]["id"] for x in log[n] if x[1] is not None and x[0] <= tca))
        kicks = sorted(set(round(x[1]["tkick"], 1) for x in log[n] if x[1] is not None and x[1]["tkick"] > 0 and x[0] <= tca + 1))
        out.append((n, tca, true_miss, first, ids, kicks, rows))
    return out


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    rocks = ["Debris406", "Debris208", "Debris109"]
    for a in sys.argv[1:]:
        if a.startswith("--rocks="):
            rocks = a.split("=", 1)[1].split(",")
    stamps = args or [os.path.basename(f)[3:-4] for f in sorted(glob.glob(f"{ROOT}/results/tracking_audit/cache/gt_*.npz"))]
    for st in stamps:
        try:
            res = replay(st, rocks)
        except Exception as ex:
            print(st, "FAILED", ex); continue
        for n, tca, tm, first, ids, kicks, rows in res:
            print(f"{st[:13]} {n} true CA t={tca:.1f} surf miss {tm:.1f} m; first tracked {first and round(first,1)} s; track ids {ids[:6]}; kicks seen {kicks[:6]}")
            for L, s in rows:
                if not isinstance(s, dict):
                    print(f"      lead {L:2d}s: {s}"); continue
                print(f"      lead {L:2d}s: pred miss {s['pred_miss']:6.1f} (tca {s['pred_tca']:.1f}) sig {s['sig']:5.1f} | pos err {s['perr']:4.1f} vel err {s['verr']:4.2f} vsig {s['vsig']:4.2f} hits {s['hits']}")
