"""
Harness.py - passive observer / run manager injected next to the competition sim.

It is a normal Space Teams Python system (the sim launches it like Radar or
Scoring).  It never touches the shuttle or the controller.  It:

  * optionally sets the sim timescale at Runtime start (Inst param Timescale),
  * samples the shuttle state at SampleHz in *sim* time and writes a CSV,
  * detects the end of the run the same way Scoring.py does (keep-out radius or
    damage threshold), waits StopAfterEnd_s more sim seconds so the Scoring
    system has certainly printed its score, writes summary.json, and stops the
    sim (st.stop_sim) so the launcher can exit,
  * stops the sim after MaxSimTime_s if nothing happened.

Inst_Parameters (set by run_sim.py):
    Shuttle, Phobos, Target   entity refs (same as the Scoring system)
    OutputDir                 absolute directory for trajectory.csv / summary.json
    Timescale                 1.0 = leave the clock alone
    SampleHz                  trajectory sample rate in sim time (default 10)
    KeepOutRadius_m, DamageThreshold_J   copied from the Scoring system params
    StopAfterEnd_s            sim seconds to wait after the end condition (default 3)
    MaxSimTime_s              hard stop (default 400)
    LoopFreqHz                wall-clock loop rate of this process
"""
# DON'T CHANGE ANY OF THE BELOW; NECESSARY FOR JOINING SIMULATION
import os, sys, time, datetime, traceback
import spaceteams as st
def custom_exception_handler(exctype, value, tb):
    error_message = "".join(traceback.format_exception(exctype, value, tb))
    st.logger_fatal(error_message)
sys.excepthook = custom_exception_handler
st.connect_to_sim(sys.argv)
import numpy as np
# DON'T CHANGE ANY OF THE ABOVE; NECESSARY FOR JOINING SIMULATION

import json
import pathlib

this = st.GetThisSystem()


def param(key, vartype, default):
    try:
        if this.HasParam(key):
            return this.GetParam(vartype, key)
    except Exception:
        pass
    return default


phobos: st.Entity = this.GetParam(st.VarType.entityRef, "Phobos")
phobosFrame = phobos.GetBodyFixedFrame()
shuttle: st.Entity = this.GetParam(st.VarType.entityRef, "Shuttle")
target: st.Entity = this.GetParam(st.VarType.entityRef, "Target")
targetFrame = target.GetBodyFixedFrame()

out_dir = pathlib.Path(param("OutputDir", st.VarType.string, str(st.SimGlobals.GetRunLogsDir())))
out_dir.mkdir(parents=True, exist_ok=True)
timescale = float(param("Timescale", st.VarType.double, 1.0))
sample_hz = float(param("SampleHz", st.VarType.double, 10.0))
keep_out = float(param("KeepOutRadius_m", st.VarType.double, 50.0))
dmg_thresh = float(param("DamageThreshold_J", st.VarType.double, 209200.0))
stop_after = float(param("StopAfterEnd_s", st.VarType.double, 3.0))
max_sim_t = float(param("MaxSimTime_s", st.VarType.double, 400.0))
loop_hz = float(param("LoopFreqHz", st.VarType.double, 60.0))
num_debris = int(param("NumDebris", st.VarType.int32, 500))      # Debris0..N-1 exist in the scenario
radar_num = int(param("RadarNum", st.VarType.int32, 200))        # Debris0..radar_num-1 are radar-visible
shuttleFrame = shuttle.GetBodyFixedFrame()
debris_entities = []
for i in range(num_debris):
    try:
        debris_entities.append((i, st.SimGlobals.GetSimEntity().GetParam(st.VarType.entityRef, ["Debris", f"Debris{i}"])))
    except Exception:
        break


def nearest_debris(p_sh, v_sh):
    """Scan every rock (only done on a damage event): the one closest to the shuttle hull, with its state."""
    best = None
    for i, en in debris_entities:
        try:
            lva = en.getLocVelAcc()
            d_loc = np.asarray(lva.loc_WRT_ExprIn(phobosFrame), float)
            d_vel = np.asarray(lva.vel_WRT_ExprIn(phobosFrame), float)
            diam = float(en.GetParam(st.VarType.double, "RadarDiameter_m"))
        except Exception:
            continue
        gap = float(np.linalg.norm(d_loc - p_sh)) - 0.5 * diam
        if best is None or gap < best["gap_m"]:
            rel = d_loc - p_sh
            v_rel = d_vel - v_sh
            best = dict(index=i, name=f"Debris{i}", radar_visible=i < radar_num, diameter_m=diam,
                        mass_kg=float(en.getMass()), gap_m=gap, range_m=float(np.linalg.norm(rel)),
                        rel_pos=[round(float(x), 2) for x in rel], debris_vel=[round(float(x), 3) for x in d_vel],
                        rel_vel=[round(float(x), 3) for x in v_rel], rel_speed_m_s=float(np.linalg.norm(v_rel)),
                        closing_m_s=float(-np.dot(v_rel, rel) / max(np.linalg.norm(rel), 1e-6)))
    return best


t0 = st.SimGlobals.SimClock.GetTimeNow().as_datetime()
wall0 = time.time()
try:   # the controller's own clock starts at StartDateTAI; ours at Runtime.  Offset lets logs be aligned.
    _start_tai = st.SimGlobals.GetSimEntity().GetParam(st.VarType.timestamp, ["Time", "StartDateTAI"]).as_datetime()
    controller_t_offset = (t0 - _start_tai).total_seconds()
except Exception:
    controller_t_offset = None


def sim_time() -> float:
    return (st.SimGlobals.SimClock.GetTimeNow().as_datetime() - t0).total_seconds()


def log(msg):
    st.OnScreenLogMessage(msg, "Harness", st.Severity.Info)


if abs(timescale - 1.0) > 1e-9:
    st.SimGlobals.SimClock.SetTimescale(timescale)
    time.sleep(0.2)
log(f"Harness: timescale requested {timescale:g}, clock reports {st.SimGlobals.SimClock.GetTimescale():g}; "
    f"sampling {sample_hz:g} Hz -> {out_dir}")

p_goal = np.asarray(target.getLocVelAcc().loc_WRT_ExprIn(phobosFrame), float)
dist0 = None

csv_path = out_dir / "trajectory.csv"
csv_f = csv_path.open("w", buffering=1)
csv_f.write("t_sim,wall,x,y,z,vx,vy,vz,dist,damage_J,fuel_kg,cmd_ax,cmd_ay,cmd_az,timescale\n")

next_sample = 0.0
stall_abort_s = float(param("StallAbort_s", st.VarType.double, 4.0))   # frozen velocity under thrust for this long -> abort
v_prev, t_prev, frozen_s = None, None, 0.0
last_damage = 0.0
collisions = []
coll_path = out_dir / "collisions.json"
recent = []          # (t, p, v, cmd) ring of the last ~5 s for the pre-impact history
end_t = None
end_kind = None
samples = 0
max_damage = 0.0
min_dist = 1e9
ts_seen = set()
stopped = False

while not stopped:
    time.sleep(1.0 / loop_hz)
    t = sim_time()
    lva = shuttle.getLocVelAcc()
    p = np.asarray(lva.loc_WRT_ExprIn(phobosFrame), float)
    v = np.asarray(lva.vel_WRT_ExprIn(phobosFrame), float)
    dist = float(np.linalg.norm(shuttle.getLocation().WRT_ExprIn(targetFrame)))
    damage = float(shuttle.GetParam(st.VarType.double, ["Stats", "Damage"]))
    fuel = float(shuttle.GetParam(st.VarType.double, ["Stats", "FuelUsed"]))
    if dist0 is None:
        dist0 = dist
    min_dist = min(min_dist, dist)
    max_damage = max(max_damage, damage)
    ts_now = float(st.SimGlobals.SimClock.GetTimescale())
    ts_seen.add(round(ts_now, 6))

    if t >= next_sample:
        next_sample = t + 1.0 / sample_hz
        try:
            cmd = np.asarray(shuttle.GetParam(st.VarType.doubleV3, ["JetSelection", "CommandedAccel"]), float)
        except Exception:
            cmd = np.zeros(3)
        csv_f.write(f"{t:.4f},{time.time() - wall0:.3f},{p[0]:.4f},{p[1]:.4f},{p[2]:.4f},"
                    f"{v[0]:.5f},{v[1]:.5f},{v[2]:.5f},{dist:.3f},{damage:.3f},{fuel:.6f},"
                    f"{cmd[0]:.5f},{cmd[1]:.5f},{cmd[2]:.5f},{ts_now:.4f}\n")
        samples += 1

    try:
        cmd_now = np.asarray(shuttle.GetParam(st.VarType.doubleV3, ["JetSelection", "CommandedAccel"]), float)
    except Exception:
        cmd_now = np.zeros(3)
    # sim thrust-stall detector: velocity bit-identical between ticks while thrust is commanded
    if v_prev is not None and t > t_prev:
        if np.linalg.norm(v - v_prev) < 1e-12 and np.linalg.norm(cmd_now) > 0.05 and t > 1.0:
            frozen_s += t - t_prev
        else:
            frozen_s = 0.0
    v_prev, t_prev = v.copy(), t
    recent.append((t, p.copy(), v.copy(), cmd_now))
    if len(recent) > int(5 * loop_hz):
        recent.pop(0)
    if damage > last_damage + 1.0:
        d_dmg = damage - last_damage
        last_damage = damage
        if collisions and t - collisions[-1]["t_last"] < 1.0:
            ev = collisions[-1]                      # same contact still bouncing: merge
            ev["damage_J"] += d_dmg
            ev["t_last"] = t
            ev["hits"] += 1
        else:
            hit = nearest_debris(p, v)
            if hit is not None and hit["gap_m"] > 10.0:
                hit = None                      # nothing rocky within 10 m: the ship / a structure, not debris
            ev = dict(t=round(t, 3), t_last=t, hits=1, damage_J=d_dmg, damage_total_J=damage,
                      dist_to_target_m=round(dist, 1), progress=round(1.0 - dist / max(dist0, 1.0), 3),
                      shuttle_pos=[round(float(x), 2) for x in p], shuttle_vel=[round(float(x), 3) for x in v],
                      shuttle_speed_m_s=round(float(np.linalg.norm(v)), 3),
                      cmd_accel=[round(float(x), 3) for x in cmd_now],
                      before_win=end_t is None, debris=hit,
                      history_5s=[dict(t=round(a, 2), pos=[round(float(x), 1) for x in b], speed=round(float(np.linalg.norm(c)), 2),
                                       vel=[round(float(x), 3) for x in c], cmd=[round(float(x), 2) for x in d])
                                  for a, b, c, d in recent[::int(loop_hz // 2) or 1]])
            collisions.append(ev)
            who = f"{hit['name']} d={hit['diameter_m']:.1f}m {'radar' if hit['radar_visible'] else 'HIDDEN'} vrel={hit['rel_speed_m_s']:.1f}m/s" if hit else "no rock within 10 m (ship/structure)"
            log(f"Harness: COLLISION t={t:.1f}s +{d_dmg:.0f}J (total {damage:.0f}J) with {who} at {dist:.0f}m from target")
        coll_path.write_text(json.dumps(collisions, indent=1))

    if end_t is None:
        if dist <= keep_out:
            end_t, end_kind = t, "win"
        elif damage > dmg_thresh:
            end_t, end_kind = t, "loss"
        elif t >= max_sim_t:
            end_t, end_kind = t, "timeout"
        elif stall_abort_s > 0 and frozen_s >= stall_abort_s:
            end_t, end_kind = t, "stall"
            stop_after = 0.0
            log(f"Harness: SIM STALL - velocity frozen for {frozen_s:.1f}s while {np.linalg.norm(cmd_now):.2f} m/s^2 is commanded; aborting")
        if end_t is not None:
            log(f"Harness: end condition '{end_kind}' at t={t:.2f}s dist={dist:.1f}m damage={damage:.0f}J fuel={fuel:.3f}kg")
    elif t - end_t >= stop_after:
        csv_f.close()
        summary = dict(end_kind=end_kind, collisions=len(collisions), controller_t_offset=controller_t_offset,
                       collisions_before_end=sum(1 for c in collisions if c["before_win"]), t_end=end_t, dist_end=dist, dist0=dist0, min_dist=min_dist,
                       damage_J=damage, max_damage_J=max_damage, fuel_kg=fuel, samples=samples,
                       timescale_requested=timescale, timescales_seen=sorted(ts_seen),
                       wall_s=time.time() - wall0, sim_s=t)
        (out_dir / "summary.json").write_text(json.dumps(summary, indent=1))
        coll_path.write_text(json.dumps(collisions, indent=1))
        log(f"Harness: wrote {out_dir / 'summary.json'}; stopping sim")
        stopped = True
        time.sleep(0.5)
        st.stop_sim()

st.leave_sim()
