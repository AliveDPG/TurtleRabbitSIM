# Fast controller: notes

Baseline: `robust/` round 3 (copied 22 Nov 17:37, after robust/NOTES.md "State at stop"). Switches prefix `FA_`,
default off (= robust behaviour). Tools in `fast/tools/` (not part of the controller). Python for tools that import
guidance.py: `%LOCALAPPDATA%/SimDynamX/STPro/Versions/v0.55.0/py/Scripts/python.exe`.

## Switches
| Env var | Where | What |
|---|---|---|
| `FA_SPEED=v` | Controller.py `CRUISE_SPEED_MPS` | cruise speed the launch burn aims for (default 7.0) |
| `FA_TIMEPTS=x` / `auto` | Controller.py -> guidance `time_pts_per_s` | planner time price; auto = 12000 / (1570 / v)^2 (0.239 at 7, 0.36 at 8.6) |

## Phase 1: ceiling of a clean flight (offline)

### Physical limits (sim files, STU4-Sim)
- Shuttle mass 3388.7 kg (constant: fuel is only a stat), inertia diag ~3026 / 5928 / 6756 kg m^2, 38 jets of 163.4 N.
- Actuator: LP (min sum of on-times), busiest jet clipped to one 1/60 s tick, others scaled; fuel = sum over jets of
  |impulse| / (80 x 9.8) -> 4.322 kg per m/s on any pure axis. Command norms clipped to 0.5 m/s^2 / 0.05 rad/s^2,
  irrelevant (the scaling saturates any command above ~0.003).
- Replica (`thrusters.JetModel`): +x 0.1746 (LP vertex fires the 4 rear jets at 1, 0.81, 0.81, 1 - not 0.193),
  -x 0.097, +y 0.143, -y 0.096, +z 0.096, -z 0.117 m/s^2; pitch +0.076 / -0.090, roll 0.125, yaw 0.081 rad/s^2.
- Combined directions are WORSE, not better: x+y at 39 deg gives |a| 0.124 at 6.08 kg/(m/s); +x plus any rotation
  command drops +x to ~0.10. One axis at a time is the only efficient way; 0.1746 is the acceleration ceiling.
- Gravity along the pad->ship line (fitted on coasts of empty-field flights): g(s) = 0.01225 - 1.95e-6 s m/s^2,
  i.e. 0.0115 at 300 m, 0.0099 at 1200 m. Sideways part ~0.0014 (aimed in for free). Finish 50 m sphere: 1255.8 m.

### Measured clean flights (frozen-type controller, empty field / no dodge)
| peak speed | t (50 m) | fuel | score |
|---|---|---|---|
| 7.45 | 218.9 | 37.14 | 55.30 |
| 9.08 | 182.0 | 44.56 | 56.53 |
| 9.93 | 170.5 | 48.67 | 55.49 |
Launch: burn effectively starts at t0 ~7.8 s (speed 1.96 m/s and 11.5 kg at 20 s; net 0.162 m/s^2 after that),
launch waste ~2.4 kg, cruise trims ~0.3 kg.

### Model (`tools/ceiling1d.py`, 1-D along the line; reads ~1.2 points low on all three flights, same shape)
| lever | change | seconds | kg | points (model) |
|---|---|---|---|---|
| cruise speed 7.45 -> 9.0 m/s peak | | -36 | +7.2 | +1.0 (measured +1.2) |
| best pitch-over (`tools/launch_opt.py`, planar, per-tick actions on the LP caps) | t0 7.8 -> 6.0 s | -1.8 | +0.7 | -0.4 (worse) |
| robust's RB_BRAKE launch waste removed (RB_BRAKE_KEEP, robust D2/D4) | | ~0 | -0.5..-0.7 | +0.7..+0.9 vs robust r3 |
| lofted burn / sustainer / second burn (1 m/s) | gravity is along the line | +7 | +6 | -1.8 |
| extra jet groups (x+y combo) | LP vertex 0.124 m/s^2 | | | -22 |
| arrival at the nearest point of the 50 m sphere | already the straight line | 0 | 0 | 0 |
| unphysical bound: instant, zero-waste launch at best speed | t0 0, waste 0 | -9 | -2.7 | +6.2 (62.4 model, ~63.5 real) |

Result: best achievable clean flight ~56.5-57 (measured 56.53 at 9.1 m/s peak; +0.5 at most from launch waste).
65 is not reachable: even an instant launch with no waste gives ~63. The limit is the 0.175 m/s^2 main jet
(~50 s to 9 m/s, half the speed on average meanwhile) and 4.32 kg per m/s; gravity is along the route, so lofting
cannot help. A 57.45 pair needs two clean ~9 m/s flights (~180 s, ~44 kg) with ~0.7 kg less launch waste than
ours, or one at ~57.9 and one at ~57.0.

## Phase 2 log
- fa1 (17:38): s86 = r3 + FA_SPEED=8.6 FA_TIMEPTS=auto vs r3, two lanes, 3+3.
  - fa11 lane a s86: 4.2 (78 kg: window-edge turns for r7 at 21-25 s, r13 at 30 s, r56 at 41 s, P 0.3-0.7 at
    tau 22.0 s, aim miss up to 810 m, re-aimed by small turns until 131 s), 50.8 (198.5 s, 44.8 kg: one 15 s burn
    pause for r157 = Debris406 at 38 s; model for a 9.1 m/s peak + 15 s pause: ~196 s / 44.5 kg - matches),
    44.3 (194.7 s, 50.5 kg: camera track c579 at 17 s, 4.5 m/s turn and back). Lane b r3: 1 rig fail, 50.9, 49.5.
  - The launch burn is identical up to ~50 s at any cruise speed (same thrust, same direction), so the Debris208
    (~135 m, 47-55 s) and Debris406 (200-260 m, 55-75 s) crossings meet the shuttle at the same times and places:
    nothing to retune there for speed. Look-ahead, urgency, error model and near-ship rules are all in seconds,
    which is what matters for the side jets; distance covered in the 22 s window rises 154 -> 198 m (radar 500
    rocks / cameras 450 m reach are not the limit).
  - fa12 never flew: gpt-oss:20b was reloaded seconds after the stop (free 2.9-3.4 GB), runner killed at 18:00.
- fa3 (Phase 3 start, 18:00): fast = r3 + FA_SPEED=8.6 FA_TIMEPTS=auto RB_BRAKE_KEEP=1 vs r3, two lanes, swapped.
  - fa31 (a = fast, b = r3): fast 55.0 (188.9 s, 43.9 kg), 50.1 (200.9 s, 44.7 kg), 52.1 (197.8 s, 43.9 kg);
    r3 -21.6 (91 kg, undamaged), 36.3, 46.6. Launch (m5table): fuel at 15 / 30 s 7.96 / 18.74 kg fast vs
    7.35 / 18.19 r3 (n = 3, no bkeep benefit visible yet); 2 of 3 fast flights paused for a crossing rock.
  - fa32 (a = r3, b = fast): r3 54.4 (219.2 s, 37.7 kg); fast lane killed mid-flight.

## State at stop (owner dropped the fast work, ~18:19 22 Nov)
All fast runners, evaluate.py, run_sim.py and SpaceCRAFT processes of fa3 killed; queue lock removed at that point.
Totals: s86/fast 6 flights scored (4.2, 50.8, 44.3, 55.0, 50.1, 52.1; 0 contacts; median 50.5); r3 control 6
(50.9, 49.5, -21.6, 36.3, 46.6, 54.4; 0 contacts). Far too few to tell the arms apart; Phase 3 not done.
The 7 frozen submission/*.py were copied into %LOCALAPPDATA%/SimDynamX/STPro/Assets/Repos/STU4-Solution and
cmp-verified at 18:19:42. 5 s later another session's chain (robust/tools/chain_q3.sh, m5g3) started and its
lane a installs robust/ there with --install-controller, so that folder now holds robust/ files again until
that session restores them. By mistake the same 7 files were ALSO copied into
Documents/SpaceTeams/STU4-Solution (a git working copy; the 7 files are untracked there except
Controller.py, which shows as modified). What those files held before is not known.

