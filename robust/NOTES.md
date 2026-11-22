# Robust controller: notes

Baseline: `submission/` (frozen, 7 files). This folder: the same 7 files plus environment switches. All
switches default to the baseline behaviour. `robust/dev/` is the editing copy (the rig compares every top-level
`.py` of `robust/` with the installed sim copy before each flight, so `robust/*.py` must not change while a
batch runs); `robust/tools/` holds test and batch scripts (not part of the controller).

## Switches

| Env var | File / function | What it does | Failure mode |
|---|---|---|---|
| `RB_WATCH=1` | Controller.py, pre-launch block (`_prelaunch_watch` thread) | runs `prelaunch_check` every 50 ms in a daemon thread from the moment the held launch command is sent until the first loop tick (lock + handover flag so it never commands after the loop has started) | 3 slow start |
| `RB_DELAY=N` | Controller.py | TEST ONLY: `time.sleep(N)` right after the pre-launch command | 3 test |
| `RB_BRAKE=1` | thrust_law.py `ThrustLaw.step` | the `XP_BRAKE` change from `explore/knobs`: above the braking curve of the pitch-over, brake with full rotation authority and every tick | 4 overshoot |
| `RB_NEAR=1` | guidance.py `Planner.plan`, `predicted_miss` | (a) the aim miss of each candidate is computed with the burn lag (fly v for the latency, then the velocity change at a constant rate over t_burn) instead of an instant velocity change; (b) +80 points (ramped over 10 m) for a miss beyond `aim_ok + 0.5 * 0.06 m/s^2 * (tca - 5 s)^2`, i.e. one the side jets can no longer correct before the ship | 2 late dodge |
| `RB_GATE_HOLD=S` | guidance.py `Planner.plan` (p_gate block) | gate memory: a rock that passed the 0.3 gate keeps counting for max(S, its closest approach + 2 s) | 1 contacts (dodge / re-aim flip-flop) |
| `RB_RECOVER=1` | thrust_law.py `ThrustLaw.step` | after a tumble ends, re-point at the pitch-over rates (0.05 rad/s^2, 0.2 rad/s) until the nose is within 5 deg | post-contact / unfinished |

The first loop tick logs `first loop tick at t=..s (watchdog .., injected delay .., pre-launch cut ..; near .. gate_hold .. brake .. recover ..)`.

## Analysis

### Mode 2 (late dodge near the ship), flight 261121_120001
Not a 64 m pass that turned back and finished: the flight never finished (timeout at 400 s, 137 kg, min distance
64 m). Chain: rock dodge at 173-186 s (r460, P 0.91) left a predicted miss of 84-88 m; re-aim at 196 s (rem 190 m,
2.4 m/s); a 4.3 m/s "avoid" of mapped ship cells at 206 s; re-aims of 3-5 m/s from 217 s. The planner prices a
candidate's miss as if the velocity change were instant. Offline replay of the flight's own states
(`tools/test_near.py`, no rocks): the baseline re-aims at 206-220 s claim a miss of 11-19 m from the entry point,
but with the burn lag the same plans miss by 40-49 m, which is what happened. With `near_fix` the plan in force
already costs 19.5 points at 181 s and 25.7 at 186 s (rem 276 / 246 m), so the correction is made there for
2-8 points; at 211 s and later no cruise-speed plan can still make it and the planner says so (aim cost 150+).
Far from the ship the fix changes nothing (26/26 identical choices at 40-165 s), and replaying the 60-500 m part
of 8 clean flights gave identical decisions (0 extra switches in 7, 14 = 14 in one).
Tried and dropped: slow-down re-aim candidates (0.5 and 0.3 x aim speed): never cheaper in the replay, because
braking adds to the lag.

After the pass the turn back failed: the re-point is capped at 0.01 rad/s in flight, the nose was 85-146 deg off
for 40+ s. The same slow re-point kept the unfinished soak_2111_a flight (261121_154344) 100-200 s with the nose
90-160 deg off after a tumble at 84-110 s. `RB_RECOVER` addresses the post-tumble case only.

### Mode 1 (rock contact), 10 contact flights in sub_gate_1/2, soak_2111_a/b, r7_gcam_c2
Contact times 50-56 s (5 flights, 1160-1180 m out, launch burn) and 74-92 s (5 flights). In the 50-56 s group the
colliding rock (31 m diameter, 1.4 m/s; the same rock each time) had P = 0.85-0.88 on the plan in force for 20 s
or more before contact (from about 29 s). The warning was there; the plan did not commit. In 7 of 10 contact
flights the PLAN log shows the planner flipping avoid -> aim -> avoid within 1 s, 8 to 122 times
(script `tools/summ.py`, "flips"). Cause: the 0.3 gate is tested on the plan in force, so right after a dodge
the rock is under the gate and is dropped from every candidate; the re-aim straight back into it then looks free.
The same flip-flop occurs in 37 of 104 undamaged flights of those batches, which score lower (median about 40
against 52 without flips). Pauses are not available while the pad watch runs (first 45 s), which is when the
launch-group contacts started.
Offline closed-loop test (`tools/test_gate.py`: point mass following the plan at 0.143 m/s^2, one 31 m rock
crossing the launch line, 12 geometries): gate memory 10 s cut plan switches from 36.3 to 7.1 per episode, flips
from 183 to 15, and raised the median clearance (gap to the hull sphere) from 1.3 to 5.9 m. Contacts 4 -> 3: the
three with 14 s warning hit in both (not avoidable at 3 m/s with 0.143 m/s^2); the one 18 s case that hit
without memory cleared by 5.7 m with it.
The other three contacts: one with no flips (sub_gate_1, avoid at 44 s then re-aim at 47.5 s with P 0.84, a single
undo), one never warned before 86 s (P 0.97 from 86 s on, contact 92 s, no plan change at all at 7 m/s: the
dodge was judged too expensive or impossible), one with a rock unseen by the log summary.

## Test results
(batch name: lane env -> outcome counts; Roblox was running on this PC during all batches unless noted)

- eq1: aborted (rig refused lane b: a test script in `robust/` differed from the installed copy; moved tools to
  `robust/tools/`). An orphaned eq1 sim kept running into eq2 for its first 4 minutes (3 sims at once): eq2 flight
  1 of lane a and the failed first launch of lane b are affected.
- eq2 (6+6, both lanes all switches off, same installed code; Roblox running):
  - lane a: 2 clean 43+, 2 clean <43, 1 contact (-89.0, 70 kJ), 1 unfinished (flight 1: the sim froze at 52 s
    sim time while 3 sims ran; rig/load, not a controller result). Scores 48.0, 44.9, 30.0, -39.9 (102 kg, no
    damage, 143 flips), -89.0. Pitch-over overshoot median 36 deg (all 6 >= 8 deg).
  - lane b: 4 clean 43+, 1 clean <43, 0 contacts, 1 rig fail (never reached Runtime). Scores 50.7, 50.0, 48.4,
    48.3, 33.0. Overshoot median 7.7 deg.
  - Same code in both lanes, so the gap is lane / load, not code: lane a (the installing lane) launched under
    heavier load (camera 2.4-2.7 frames/s at 100 s vs 2.2-3.3 in lane b, 2.7-3.1 in the 21 Nov soak). Lane b
    matches the baseline (median 48.4, overshoot as in soak_2111). Lane a shows the load sensitivity of mode 4/5:
    overshoot 10-62 deg and flip-flops in every flight. Consequence: the final comparison swaps lanes halfway.
- dly6 (3+3, injected 6 s start delay; lane a RB_WATCH=1, lane b watchdog off; Roblox running):
  - first loop tick 7.7-8.5 s (a) / 8.3-9.2 s (b). Body rate at the first tick: 0.16, 0.17, 0.16 rad/s with the
    watchdog (the cut at 0.15 happened on time) against 0.21, 0.27, 0.18 without. Pitch-over overshoot 15, 7.5,
    0 deg against 41, 50, 17 deg.
  - outcomes: a: 44.9, 4.1 (clean, 61 flips), 1 sim thrust stall at 148 s (rig fault); b: 49.6, 17.9 (clean),
    1 sim thrust stall at 89 s. Scores are dominated by dodges; 2 flights per lane say nothing about score.
- dly25 (3+3, injected 2.5 s; a watchdog on, b off). Roblox Player AND Roblox Studio running (Studio 1.9 GB).
  - tick0 4.0-4.9 s in both lanes; rate at first tick 0.15-0.16 (a) vs 0.15-0.19 (b): at this delay the
    set-up checkpoints still catch the cut, as the baseline docs say (fine up to about 5 s).
  - outcomes: a: -4.4, 17.2 (clean, 81 / 66 kg), -124.7 (contact 103 kJ); b: 3 contacts (-146.0, -185.6, -50.0).
    Overshoot 15-52 deg in all six, 31-63 flips per flight: the heavy machine load wrecks the launch and the
    planner in both lanes. Not a test of the watchdog's score effect.
- fin1 (6+6; lane a ALL = RB_WATCH=1 RB_BRAKE=1 RB_NEAR=1 RB_GATE_HOLD=10 RB_RECOVER=1, lane b all off; Roblox
  Player + Studio running):
  - a (ALL): 4 flights + 2 rig fails (never reached Runtime): 39.8, 30.9, -8.3, -10.4; 0 contacts. Pitch-over
    overshoot (before the first plan change) 3.1-6.2 deg, none >= 8. Flips 5 in total. The two low scores burned
    81-84 kg: dodges of 3-6 m/s at 15-34 s against rocks at P 0.14-0.30, and fuel kept rising in cruise.
  - b (off): 51.8, 49.6, 45.8, 32.1, -236.4 (contact 118 kJ, 87 flips), -1772.1 (lost, 1.77 MJ at 70 s, no flip,
    no warning in the log); overshoot >= 8 deg in 2 of 6; flips 168 in total.
- fin2 (6+6, lanes swapped: lane a all off, lane b ALL):
  - b (ALL): 53.9, 51.1, 44.2, 1.5 (77 kg), -10.0 (83 kg); 0 contacts; 1 sim freeze (MemoryError in both sims
    at the same wall second 18:46:35: the PC ran out of memory with Roblox Studio open). Flips 4 in total.
  - a (off): 49.3, -20.7 (16 kJ), -72.5 (18 kJ), -105.5 (35 kJ); 2 sim freezes (same MemoryError event and one
    more). Flips 204.

### Final comparison (fin1 + fin2, lanes swapped halfway, flown concurrently, heavy machine load)
| | ALL fixes | baseline (all off) |
|---|---|---|
| attempts / flights with a score | 12 / 9 | 12 / 10 |
| rig or sim failures (no Runtime, memory freeze) | 3 | 2 |
| undamaged 43+ | 3 | 4 |
| undamaged below 43 | 6 | 1 |
| rock contact | 0 | 5 (one lost, -1772) |
| unfinished (controller) | 0 | 0 |
| median / mean | 30.9 / 21.4 | 5.7 / -197.9 |
| worst / 10th percentile | -10.4 / -10.1 | -1772.1 / -390.0 |
| pitch-over overshoot >= 8 deg | 0 of 9 | 4 of 10 |
| avoid/aim flips | 9 in total | 372 in total |
Scores ALL: 53.9, 51.1, 44.2, 39.8, 30.9, 1.5, -8.3, -10.0, -10.4.
Scores off: 51.8, 49.6, 49.3, 45.8, 32.1, -20.7, -72.5, -105.5, -236.4, -1772.1.
0 vs 5 contacts in about 10 flights each is a strong signal (Fisher exact p about 0.03) but under unusual load;
the cost is visible: 6 of 9 candidate flights were undamaged below 43, four of them at 77-84 kg, from dodges at
P 0.14-0.30 that the gate memory keeps alive. Not yet acceptable as is: the gate memory needs to be narrower.

Cleanup: no sim / evaluate processes left, lock removed, frozen `submission/*.py` copied back into STU4-Solution
and verified with cmp; the stray `test_near.py` that eq1 had installed there was removed.

# Milestone 2

Load: Roblox Player running (2-3.5 GB), Studio closed. With two sims + Roblox the commit charge left only about
1 GB free (offline numpy runs failed with MemoryError, the same failure that froze both sims at 18:46 in
milestone 1), so offline tests now run only while at most one sim flies. From the user's instruction mid-way:
ONE sim at a time; arms alternate flight by flight in one lane (`tools/run_alt.sh`, each flight installs
`robust/`, waits while free memory < 4 GB and logs free memory per flight to `results/eval/<name>.load`).

## 1. Gate memory variants (offline, `tools/test_gate2.py`)
Closed loop, point mass at 0.143 m/s^2, one rock crossing the launch line, per-episode track bias 3 m / 0.1 m/s,
27 episodes per variant: 15 'threat' (offset inside the contact radius) and 12 'pass' (outside: any dodge there
is needless). Output `tools/gate2_out.txt`.

| variant | threat: switches/ep, flips, extra kg, contacts, gap med | pass: switches/ep, flips, extra kg |
|---|---|---|
| off (baseline) | 57.9, 397, 4.3, 3, 4.8 m | 24.5, 139, 0.5 |
| hold10 (milestone 1) | 7.0, 13, 3.6, 2, 6.7 m | 5.1, 19, 0.4 |
| (a) excess10 | 4.1, 0, 3.5, 2, 6.4 m | 1.9, 1, 0.4 |
| (b) short4 | 10.3, 42, 3.6, 2, 6.3 m | 4.7, 16, 0.4 |
| (c) hyst 0.30/0.15 | 54.9, 374, 4.3, 2, 5.2 m | 29.5, 169, 0.5 |

Hysteresis does almost nothing: after a dodge P on the plan in force drops below 0.15 at once, so the rock
still vanishes. (a) "excess" is best on every count. The synthetic benchmark does NOT reproduce the in-flight
fuel problem (all variants spend < 0.5 kg on pass episodes): in flight, hold10 kept a held rock's full residual
risk (P 0.14-0.30, 85-180 points) on the plan in force, which made the planner enlarge dodges to drive it to
zero (3-6 m/s turns in fin1_a). "excess" charges the plan in force nothing for a held rock and a larger dodge
nothing either (its P is lower), so that mechanism is removed by construction. Replaying the 104 clean flights'
decisions is not possible: the logs hold the top-3 threats, not the obstacle sets the planner saw.
Chosen: `RB_GATE_HOLD=10 RB_GATE_MODE=excess`.

## 3. Recovery after a hit / nose far off
### Past contacts (`tools/contact_recovery.py`, 18 contact flights)
- 5 flights ended at the hit (657 kJ, 253 kJ, 267 kJ, 1.77 MJ, ...): the sim reports a lost / destroyed
  shuttle; nothing to recover. These are most of the score loss; only avoidance helps.
- 13 survivable hits (2-154 kJ, at 46-82 s, mostly 50-55 s at 2.5-6.8 m/s): spin 0.4-2.3 rad/s; tumble mode
  5-36 s; ~16 kg in the first 30 s; nose within 20 deg of target only after 5-230 s (median about 130 s);
  velocity error below 0.5 m/s after 46-187 s; 25-125 kg burned after the hit (median about 60 kg).
- Where it goes wrong (timelines of dly25_b and fin2_a): the hit leaves a 9.6-10 m/s velocity error and the nose
  80-135 deg off. After the tumble the in-flight re-point runs at 0.01 rad/s (0.5 deg/s), so the nose takes
  100-230 s to come round, while the law burns the error on whatever axes face it: dv fell 9.8 -> 4.5 m/s for
  45 kg, 8.5 kg per m/s against 4.3 on the main jets. The planner keeps re-aiming at 8 m/s, and cameras were
  fine (cam tracks 100-260 during the tumble), planner switches 2-12 in the minute after the hit (two with 32-35:
  flip-flop). Repeat contacts in the same event: 'hits' 2-5 in 9 of 18 (the shuttle still overlapping the rock).
### Design (all behind switches, thrust_law.py `step`, guidance.py `plan`, Controller.py)
- `RB_RECOVER=1`: when a tumble ends, re-point at the pitch-over rates (0.05 rad/s^2, 0.2 rad/s).
- `RB_RECOVER_DEG=60`: the same fast re-point whenever the nose is > 60 deg off in flight (not on the pad, not in
  the launch).
- `RB_RECOVER_BURN_DEG=30`: while recovering, no translation until the nose is within 30 deg of the burn.
- `RB_RECOVER_PLAN=1`: hit detected from the controller's own data (Stats/Damage jump or tumble mode); for 90 s
  the planner also offers re-aims at 0.6 and 0.8 x the aim speed, priced like any other candidate.
- Test-only `RB_KICK_T=T` (yaw the hull 90 deg off at 0.1 rad/s, then release) and `RB_KICK=T` (brake on -x
  for 20 s and spin to ~0.5 rad/s: a hit-like upset of ~30 deg/s and a few m/s off the plan).
### kick1 (RB_KICK_T=110, nose 90 deg off, two lanes; lane 2 stopped on the user's instruction)
- with RB_RECOVER=1 RB_RECOVER_DEG=60: nose < 10 deg 12 s after release, 2.3-2.6 kg; scores 48.3, 48.6.
- without: nose never back within 20 deg in the remaining ~93 s of flight (forward cameras looking 20-90 deg
  off the flight line); score 52.1 (39.4 kg). One more flight per arm lost to rig faults.
  So the fast re-point costs ~2.5 kg (~3 points) when nothing threatens; it buys forward vision.

## 2. Launch pauses during the pad watch
Logs: in the 5 launch-group contacts the warning (P 0.85+) came at 21-29 s, at 2-3 m/s with 5-6 m/s of burn
pending, i.e. pause candidates would have been eligible (speed > 1 m/s, burn > 1.5 m/s) but were switched off by
the pad watch (until 45 s / 60 m). Offline (`tools/gate2_pause_out.txt`, same 27 episodes, pauses allowed):
excess10 + pauses: threat extra fuel 3.5 -> 2.0 kg, median gap 6.4 -> 8.1 m, but contacts 2 -> 3 (one 14 s case
that cleared without pauses hit with them). baseline + pauses: 4.3 -> 2.2 kg, contacts 3 -> 3. Cheaper, not
safer: not adopted, not flown (`RB_PADPAUSE=1` exists, default off).

### kick3 (single lane, alternating; RB_KICK=110: 15 s sideways on body +y, then spin-up to 0.5 rad/s)
Arm a = kick only, arm b = kick + RB_RECOVER=1 RB_RECOVER_DEG=60 RB_RECOVER_BURN_DEG=30 RB_RECOVER_PLAN=1.
(kick2 with a braking kick was abandoned after 1+1 flights: braking along the path only changes speed, the
planner simply re-plans at the lower speed (0.6 m/s error left); a: 29.4, nose never back; b: 23.9, nose and
rate back after 25 s / 7.4 kg. Note: in kick1/kick2 robust/Controller.py had lost its CRLF line endings through
a sed edit (header text identical, line endings not); fixed before kick3, header byte-identical again.)
- a: -0.5 and 0.5: upset 2.6 m/s off the plan at 0.50 rad/s (kick itself 14 kg); nose never back within 20 deg
  for the rest of the flight; 27-28 kg from release to the end.
- b, flight 1: lost (-1113.9, 3.0 MJ at 121 s) DURING the kick (blind 15 s sideways push into a rock), before
  any recovery code acted. A test artefact, but a warning that the injection itself is risky.
- 20:14: free memory fell to ~1 GB with no sim running: a local `llama-server` process (17.6 GB, not part of
  this work) had started. The runner waits while free memory < 4 GB (logged in results/eval/kick3.load).
- b, flight 2: upset 2.6 m/s, 0.50 rad/s: nose and rate back after 17 s, fully back on the plan (velocity error
  < 0.5 m/s) after 59 s and 23.3 kg; 26.4 kg from release to the end; score -2.5 (a: -0.5, 0.5).
  So the recovery mode restores the attitude (17 s vs never) at no extra fuel, but saves no fuel or points
  in this upset: the burn after the upset costs the same with or without the burn hold / slower re-aims.
- 20:41-: flights paused again, free memory 0.5-0.9 GB (llama-server).

### Estimate (not measured) for the 14 survivable past contacts
Assume the recovery mode makes the post-despin correction cost 2.5 kg (re-point) + 4.32 kg/(m/s) x 1.15 x the
velocity error left after the despin, instead of what was burned; arrival time unchanged. Script inline in the
session; numbers: fuel after the hit 25-125 kg (median 64) -> 17-64 kg (median 54); score change +1 to +93,
median about +10 (e.g. fin1_b -236 -> about -143, sub_gate_1 -68 -> about -33, fin2_a -21 -> about -9). The
damage term stays (2-154 kJ = 2-147 points), so a contact remains far worse than any dodge. The kick test
does NOT confirm these savings (equal fuel with and without), so treat the estimate as an upper bound.
- a, flight 3: -2.9, nose never back, 30.5 kg release->end. kick3 stopped there (b's third flight not flown:
  memory waits) to start the final comparison. kick3 totals: a 3 flights (nose never back, 27-31 kg after
  release, -2.9..0.5), b 1 valid flight (nose back 17 s, recovered 59 s, 26.4 kg, -2.5) + 1 lost during the kick.

## 4. Final comparison m2fin (single lane, alternating, order swapped after 5 scores; up to 14 attempts/arm)
arm a = frozen `submission/` installed each flight; arm b = `robust/` with
RB_WATCH=1 RB_BRAKE=1 RB_NEAR=1 RB_GATE_HOLD=10 RB_GATE_MODE=excess RB_RECOVER=1 RB_RECOVER_DEG=60
RB_RECOVER_BURN_DEG=30 RB_RECOVER_PLAN=1. Free memory per flight in results/eval/m2fin.load.
- m2fin candidate flight 6 (22:32): UNFINISHED. Hit at 77 s (170 kJ, 5 hits in one event, 7.2 m/s) that threw
  the shuttle to 10.8 m/s away from the ship and spun it at 2.85 rad/s. Recovery: RECOVERY logged at 77.1 s,
  tumble mode ~40 s (spin 2.85 -> 0.09 rad/s by 116 s), nose on the burn by 124 s (fast re-point worked), then
  a 16 m/s reversal flown at 6.7 kg per m/s (ideal 4.3) from 124 to 327 s; the shuttle kept receding to
  1768 m and was 1218 m out at the 400 s cap with 190 kg burned. The slower re-aims were offered (|vt| 6.0 at
  84 s) but the plan went back to ~7.9 m/s by 100 s. A reversal of this size needs >= 90 s of main-jet burn even
  when perfect; the remaining loss is burn efficiency during the reversal (not investigated further).

### m2fin result (ALL single-lane, alternating; two lanes were allowed again at ~22:40 but free RAM with no
sim was 2.4-5.6 GB because of a loaded llama-server / Ollama model, below the 5 GB two-sim rule, so the whole
final stayed single-lane. One arm-a flight was aborted when the runner was restarted at 22:42.)
| | frozen baseline (submission/) | candidate (robust/, env below) |
|---|---|---|
| attempts / scored | 13 / 12 (1 rig fail) | 13 / 12 |
| undamaged 43+ | 4 | 7 |
| undamaged below 43 | 5 | 5 |
| rock contact | 3 (all lost: -1642, -1155, -767) | 1 (the unfinished flight, 170 kJ) |
| unfinished | 0 | 1 |
| median / mean (scored) | 34.9 / -264.8 | 48.6 / 41.1 |
| median counting the unfinished as worst | 34.9 | 46.6 |
| worst / 10th pct (scored) | -1642.0 / -1115.8 | -2.3 / 19.1 |
| median fuel / flights > 70 kg | 41.2 kg / 0 | 43.3 kg / 1 (+ the unfinished one at 190 kg) |
| pitch-over overshoot >= 8 deg | 3 | 0 |
| avoid/aim flips (total, flights with any) | 234, 8 | 8, 4 |
Baseline scores: -1642.0 -1154.5 -767.2 25.8 33.6 33.6 36.2 41.5 53.0 53.5 54.2 54.4.
Candidate scores: -2.3 17.5 33.0 33.8 40.6 46.6 50.6 53.9 54.4 54.5 55.0 55.1, + unfinished.
Contacts 1 vs 3 in 13 attempts each: fewer, but not statistically clear (Fisher p about 0.3). Median within
3 points: yes (candidate higher). The gate-memory fuel problem of milestone 1 did not recur: 1 flight > 70 kg
(77 kg, -2.3) against 4 of 9 in milestone 1.

Cleanup: no sim or rig processes left, no lock folder, frozen submission/*.py copied back into STU4-Solution
and verified with cmp; robust/Controller.py header byte-identical to the original.

Recommended env: RB_WATCH=1 RB_BRAKE=1 RB_NEAR=1 RB_GATE_HOLD=10 RB_GATE_MODE=excess RB_RECOVER=1
RB_RECOVER_DEG=60 RB_RECOVER_BURN_DEG=30 RB_RECOVER_PLAN=1

# Milestone 3

Load: Ollama/llama-server model resident (~14 GB); Roblox Player idle (165 MB). Free RAM 5.4-6.6 GB at batch
starts, so two lanes per the 5 GB rule.

## 1. Heavy-hit recovery: why the 170 kJ flight reversed at 6.7 kg per m/s
Log of 261121_223159 (m2fin candidate 6), 116-327 s: mode stayed `cruise` the whole reversal, att (nose vs the
target velocity) 1-14 deg, dv 16.4 -> 0.06 m/s for 110 kg. In cruise mode the law points the nose at v_tgt and
fires the velocity error on whatever axes face it; burn mode (nose along the error, main jets) needs the error
within 25 deg of v_tgt, and after the hit the shuttle's velocity had a large sideways part, so the error was
more than 25 deg off v_tgt for 200 s. Not the planner speed: an offline replay (`tools/test_recover_plan.py`)
at 90/116/140 s shows the 7.9 m/s re-aim is also what the score formula prefers there (a slower return cannot
make the 400 s cap from 1060-1410 m out), with an efficient burn arriving at ~335 s.
Fixes (switches):
- `RB_RECOVER_DVBURN=1` (thrust_law.py `step`): after a tumble, burn mode (nose along the velocity error, main
  jets) whatever its angle to v_tgt, until the error is below 1 m/s. With `RB_RECOVER_BURN_DEG=30` the
  translation waits until the nose is within 30 deg of the error: re-point first, then main-jet burn.
- `RB_RECOVER_SCORE=1` (guidance.py `plan`): for 90 s after a hit, time is priced with the score formula
  (12000 / t, so 0.075 points/s at 400 s instead of the constant 0.25), a finish later than the time cap - 15 s
  costs up to 300 points, and re-aims at 0.4 / 0.6 / 0.8 x the aim speed are offered.
- Fuel reserve: the sim has no fuel limit (fuel is only a score cost; flights have burned 190 kg), so the
  shuttle cannot run dry; no reserve rule needed.
- Test switch `RB_KICK_STRONG=S` (with `RB_KICK=T`): brake on -x for S s (60 s ~ 10 m/s, ending ~4 m/s away from
  the ship), then spin to 0.5 rad/s.

## 2. Lost flights: what the recommended config would have done (offline estimate)
- m2fin baseline -767 (758 kJ at 158 s): rock r38 at P 0.53-0.65 for 20 s from 133 s with a cheap way out
  (best candidate P 0.02-0.03), but 28 plan switches and 13 dodge/re-aim flips in the last 40 s: the gate
  flip-flop. The excess gate memory removes exactly this; likely avoided (estimate).
- m2fin baseline -1642 (1.63 MJ at 60 s, 8.9 m/s relative): first warning 8 s before (P 0.34, best candidate
  already 0.30, all candidates 0.82+ 4 s later). No plan could clear it; recovery is moot (flight lost at the
  hit). Recommended config: same outcome.
- m2fin baseline -1155 (1.13 MJ at 35 s, launch): warning at 25 s, tau 10.8 s, best candidate 0.30 -> 0.64
  within 3 s. Same as above: too late for the jets; same outcome.
- kick3 test loss (3.0 MJ at 121 s): the test kick overrode the planner for 15 s while r246 closed from P 0.34
  to 0.87; an artefact of the injection, not a controller decision (the arm also lacked the gate fix: 10 flips).
- Lost flights end at the hit (the sim scores them as losses), so post-hit recovery cannot change them.

## Heavy-kick tests (two lanes, swapped halfway; all flights with the milestone-2 recommended config)
### kickS1/kickS2: RB_KICK=70 RB_KICK_STRONG=60 (60 s brake on -x then spin to 0.5 rad/s; ~10 m/s, ending ~4 m/s
away from the ship), fix = RB_RECOVER_DVBURN=1 RB_RECOVER_SCORE=1
| arm | flights | finished | scores | arrival | fuel | release -> error < 1 m/s |
|---|---|---|---|---|---|---|
| fix | 4 | 4 | -42.1, -45.3, -63.6, -45.6 (mean -49.2) | 305-354 s | 97-113 kg | 46-53 s, 27-35 kg |
| no fix | 4 | 4 | -61.6, -49.5, -42.3, -44.5 (mean -49.5) | 300-358 s | 96-110 kg | 43-55 s, 25-34 kg |
No difference. This kick leaves the velocity error along the flight line, so both arms fly it in burn mode
(modes after release: tumble then burn, plus 13-16 s of 'pad' mode while the speed passed through ~0, in both
arms): it does not reproduce the sideways error of the 170 kJ flight. Lane effect again: lane a scored better
in both halves. Follow-up kickY: the same with a 50 s SIDEWAYS push (RB_KICK_DIR=y, ~7 m/s across the line).
### kickY1/kickY2: RB_KICK=70 RB_KICK_STRONG=50 RB_KICK_DIR=y (50 s SIDEWAYS on body +y, ~7 m/s across the line,
then spin to 0.5 rad/s), two lanes, swapped halfway, 4 flights per arm
| arm | finished (400 s cap) | score / closest distance to the ship at the cap | release -> velocity error < 1 m/s | modes after release |
|---|---|---|---|---|
| fix | 2 of 4 | -127.0 (302 s, 163 kg), -144.0 (359 s, 171 kg); unfinished at 70 m; unfinished after 2 rock contacts (166 s, 204 s, 82 kJ) | 73, 48, 104, 36 s (50, 33, 62, 15 kg) | burn |
| no fix | 0 of 4 | unfinished at 343, 487, 143, 346 m | 127, 215, never, 201 s (50-89 kg) | cruise |
This kick reproduces the 170 kJ case (error far off the target velocity): without the fix the law stays in
cruise mode and needs 2-4 times as long to null the error; with it, burn mode on the main jets. 2 of 4 fix flights
finish against 0 of 4; the remaining unfinished fix flight with no contact ended 70 m out (the finish is 50 m).
Fix adopted for the confirmation.

## 3. Confirmation (conf1-conf5, two lanes, swapped every batch, 3 flights per lane per batch, 00:36-01:50)
Candidate = milestone-2 config + RB_RECOVER_DVBURN=1 RB_RECOVER_SCORE=1; baseline = robust/ with all switches off
(same code paths as the frozen submission, checked in milestone 1 eq2). Rig/sim failures: 1 baseline thrust
stall at 11 s, 1 candidate sim freeze (MemoryError in the controller at 01:04, commit exhausted again although
9 GB were free at the batch start). Confirmation alone (14 scored per arm):
- baseline: median 45.9, mean 16.6, p10 19.2, fuel median 44.6 kg, >70 kg 0, contacts 1 (lost, -309.7)
- candidate: median 40.8, mean 37.1, p10 17.4, fuel median 46.4 kg, >70 kg 1, contacts 0
So in the clean two-lane part the candidate's median is 5 points BELOW the baseline (outside the 3-point target);
it has more undamaged-below-43 flights (expensive dodges).

### Pooled m2fin (single lane) + conf (two lanes)
| | baseline | candidate |
|---|---|---|
| flights (scored + unfinished) | 26 (+2 rig failures) | 27 (+1 rig failure) |
| undamaged 43+ | 12 | 12 |
| undamaged below 43 | 10 | 14 |
| contacts | 4 (all lost) | 1 (unfinished) |
| lost / unfinished | 4 / 0 | 0 / 1 |
| median / mean | 39.8 / -113.3 | 41.8 / 39.0 |
| worst / 10th pct | -1642.0 / -538.5 | -6.4 / 14.9 |
| median fuel / > 70 kg | 43.7 kg / 0 | 46.1 kg / 2 |
Fisher exact (two-sided): contacts 1/27 vs 4/26, p = 0.19; lost-or-unfinished 1/27 vs 4/26, p = 0.19.

Cleanup: no sim / rig processes, no lock folder, frozen submission/*.py copied back into STU4-Solution and
verified with cmp; robust/Controller.py header byte-identical.

Final env: RB_WATCH=1 RB_BRAKE=1 RB_NEAR=1 RB_GATE_HOLD=10 RB_GATE_MODE=excess RB_RECOVER=1 RB_RECOVER_DEG=60
RB_RECOVER_BURN_DEG=30 RB_RECOVER_PLAN=1 RB_RECOVER_DVBURN=1 RB_RECOVER_SCORE=1

# Milestone 4: where the candidate loses median

## Attribution (`tools/attrib.py`, `tools/test_farmiss.py`; undamaged flights: 26 candidate, 22 baseline)
| cause | evidence | flights | points |
|---|---|---|---|
| RB_NEAR lag-aware miss applied far from the ship: the instant re-aim candidate looks ~30 m short under the lag model, so a 400 m mid-cruise miss is KEPT (offline replay of the flights' own states: at 80-110 s, 800-1000 m out, near_fix keeps the plan where the baseline formula re-aims with 2.8-3.2 m/s), then corrected late in 0.4 m/s steps and a forced 2.5-2.8 m/s turn at 460-560 m | candidate -6.4 and -2.3: 24 kg each inside 600 m, 38-44 kg after 60 s | 2 clear, up to 4 | ~30-40 each in those 2; the rest small |
| launch (RB_BRAKE / watchdog) | fuel at the end of the launch burn median 35.5 vs 34.5 kg; at 15 s 7.6 vs 8.0 kg; speed at 60 s 5.88 vs 5.92 m/s | all | ~1.3 (within noise) |
| gate memory dodges at P < 0.3 | avoid switches with worst P < 0.3: 0.2 per flight vs 0.8 in the baseline; avoid dv sums far lower (4.6 vs 90 m/s of switching) | 0 | 0 |
| recovery mode without a hit | no RECOVERY line in any undamaged flight; nose > 60 deg outside tumble 0.0 s/flight | 0 | 0 |
| RB_RECOVER_SCORE pricing | active only within 90 s of a hit | 0 | 0 |
| watchdog | first-tick rate normal; no effect without a slow start | 0 | 0 |
Mid-cruise misses >= 90 m occur in both arms (8/26 candidate, 6/22 baseline; median post-60 s fuel of the other
flights 8.7 vs 7.7 kg), so most of the remaining gap is the dodge lottery, not a switch.

## Change
`RB_NEAR_FROM=600` (guidance.py `GuideParams.near_lag_from_m`): the lag-aware miss (and so the uncorrectable-miss
price built on it) applies only within 600 m of the ship. Offline: the 400 m misses are re-aimed at once again
(test_farmiss: 'aim 2.8-3.2 m/s' at 80-110 s, as the baseline formula); the fly-past replay (181-211 s,
114-276 m out) is unchanged (same decisions and keep costs as before); gate and recovery code untouched, so the
back-and-forth counts and kick tests still apply.

## m4c comparison (01:56-03:21; m4c1, m4c3-m4c5 two lanes swapped each batch, m4c2 single lane alternating because
free RAM was 4.4 GB; no MemoryError freezes; 3 rig failures)
Candidate = milestone-3 config + RB_NEAR_FROM=600; baseline = robust/ all switches off.
| | baseline | revised candidate |
|---|---|---|
| scored / unfinished | 12 / 1 | 14 / 0 |
| undamaged 43+ | 7 | 3 |
| undamaged below 43 | 4 | 8 |
| contacts | 1 (+1 in the unfinished flight) | 3 |
| lost | 0 (+1 unfinished) | 2 (-297, -4034) |
| median / mean | 46.2 / 35.5 | 24.6 / -283.1 |
| worst / p10 | -62.3 / 31.4 | -4034.0 / -203.7 |
| median fuel / > 70 kg | 44.6 / 0 | 54.1 / 0 |
Fisher contacts 3/14 vs 2/13 p = 1.0 (counting the unfinished baseline hit); lost-or-unfinished 2/14 vs 1/13 p = 1.0.
The change made the median much worse: with the re-aim no longer deferred, the big post-launch dodges (500-800 m
miss after the launch rock, r2/r13) are corrected at once with ~3 m/s at 60-80 s (fuel at 110 s ~60 kg), where
the deferred, piecewise correction of milestone 3 was cheaper on average. The two losses were fast knocked rocks
(18 and 34.5 m/s relative, r157/r158, first threat 1-6 s before) that also produced the baseline's unfinished
flight (r158, 21 m/s): luck of the knock, not the change. Also this session was harder for both arms (baseline
median 46 here vs 46 in conf, but candidate fell from 41 to 25).
Conclusion: RB_NEAR_FROM=600 rejected. Recommended config stays the milestone-3 one (RB_NEAR_FROM unset).
Cleanup done: no rig processes, no lock, frozen files restored and verified with cmp, header byte-identical.

# Milestone 5

Load at start (06:36): free RAM 19.3 GB, Ollama running with no model loaded (`ollama ps` empty), Roblox Player
idle (2 processes, ~190 MB), no other sims. Python interpreter for offline tools that import guidance.py (scipy):
`%LOCALAPPDATA%/SimDynamX/STPro/Versions/v0.55.0/py/Scripts/python.exe` (the system python has no scipy).

## A. Reference head-to-head (running): `tools/chain_m5.sh`
`bash robust/tools/chain_m5.sh m5a "<r3 env>" r3 "RB_NONE=1" base 40 6 18000`: two lanes, 6 flights per lane per
batch, arms swapped every batch, until 40 scored per arm, 5 h, or `results/eval/m5a.stop` exists (checked between
batches); single-lane fallback below 5 GB free; load logged per batch in `results/eval/m5a.progress`. Baseline arm =
robust/ with all switches off (same code paths as submission/, milestone 1 eq2). Table: `tools/m5table.py m5a r3 base`.

### A result (m5a1-m5a11, 06:36-12:15; 7 two-lane batches + 4 single-lane batches)
Load: Ollama gpt-oss:20b (12 GB) was loaded by another user from 07:04 on and off (free RAM 0.4-3.6 GB at times):
m5a2, m5a4, m5a6, m5a8, m5a9 ran single-lane alternating with memory waits; m5a3/m5a5 started two-lane at 5.3-6.5
GB free and ran with 0.7 GB free (1 rig failure). From 11:14 the model was stopped with `ollama stop gpt-oss:20b`
(owner allowed). The first chain process was stopped at 10:59 (script edited for continuation) and continued with
I0=9; robust/*.py were updated to the dev copy at 11:07 between single-lane flights (new switches default off,
identical behaviour, regression-checked).
| | round 3 (r3) | baseline (all off) |
|---|---|---|
| flights scored (+ rig failures) | 46 (+0) | 43 (+3) |
| undamaged 43+ / undamaged < 43 | 30 / 15 | 20 / 11 |
| contacts / lost / unfinished | 1 / 1 / 0 | 12 / 2 / 0 |
| median / mean | 48.0 / -5.6 | 42.0 / -39.8 |
| worst / p10 | -2022.9 / 6.6 | -1903.3 / -119.1 |
| median fuel | 40.5 kg | 45.0 kg |
Fisher (two-sided): contacts 1/46 vs 12/43 p = 0.001; lost-or-unfinished 1/46 vs 2/43 p = 0.61.
Contacts: r3: Debris406 knocked (7.8 m/s) at 63 s, 175 kJ, scored as a loss. Baseline: Debris208 x8 (52-58 s,
3-63 kJ, all during the dodge flip-flop the gate memory removes), Debris406 knocked x2 (10.0 m/s 118 kJ; 6.3 m/s
1.9 MJ lost), Debris386 7.8 m/s (128 kJ, lost), Debris311 (7 kJ).
Launch waste (C) on the unloaded A data: 2.93 vs 2.35 kg at 15 s, 4.63 vs 4.02 at 30 s, speed at 30 s 3.10 vs
3.36 m/s: the RB_BRAKE cost persists unloaded.

## B. Knock study (offline; truth recordings used for analysis only)
Data: 203 logged contacts with a rock name (`collisions.json` of every run under results/runs), 12 truth recordings
(`results/tracking_audit/cache/gt_*.npz`, 19 Nov, 500 rocks at ~10 Hz) with their sensor recordings
(`D:/STU4-recordings/STU4-evaluation/eval-<stamp>/sensors`).
Tools: `tools/knock_gt.py` (kick history of chosen rocks in the truth), `tools/knock_replay.py` (recorded radar stream
through robust/'s own ShuttleNav + DebrisTracker, track vs truth, closest approach predicted from the track at 30-3 s
lead), `tools/knock_events.py` (every rock-rock contact in the truth, which ones put a rock near the flown path, and
their pair-contact predictability with radar-quality tracks), `tools/hit_view.py RUN_DIR` (controller messages before
a contact, rock speed and the true pre-impact relative speed), `tools/exposure.py` (launch hold / lateral offset vs
the recorded fields).

1. The "18-35 m/s knocked rock" figure is an artefact: the harness `rel_speed_m_s` uses the shuttle velocity AFTER the
   impact (the hit throws the 3.4 t shuttle to 17-38 m/s). Pre-impact relative speed from `history_5s`: median 7.8,
   p90 13.7, max 24.3 m/s over 203 contacts. The m4c losses "r157/r158" ARE Debris406 (26 m), tracked by radar
   from t = 3 s in every flight (track id 157/158).
2. Seed-8193 era (since 17 Nov, 90 named contacts): Debris406 27 contacts and 21 of the 29 contacts above 500 kJ;
   Debris208 (31 m, 1.4 m/s) 23 contacts, all below 500 kJ. Debris406 hits at 55-75 s (median 61 s), 1080-1130 m
   from the target, i.e. 200-260 m from the pad, during or just after the launch burn. Its speed at the hit was
   3.3-5.5 m/s in 18 of 27 (its normal, unknocked ~4.9-5.5 m/s) and 7.3-20.4 m/s in 9 (knocked).
3. Truth (12 flights, 1544 s): 6 contacts (Debris406 x2, Debris208 x2, Debris352, Debris353); in all six the rock
   had no rock-rock contact for >= 50 s before (straight line). 895 rock-rock contacts after t = 10 s (~75 per
   flight); only 2 put the knocked rock within 20 m of the contact radius of the flown path, 1 inside it (no hit
   recorded). Debris406 was knocked by Debris311 (r 17 m) at 40-42 s in 2 of 12 flights (sent away, 158-186 m).
4. Tracking (radar replay, 2 Debris406 losses 012405/013849): one radar track all flight, position error 2-5 m,
   velocity error 0.05-0.12 m/s, claimed sigma 0.17-0.22 m/s; the closest approach predicted from the track and the
   flown path was within 1-6 m of the truth from 30 s before (pred 11.7 / 5.7 / 4.2 m at 30 / 20 / 15 s lead vs
   true 5.1 m). The unknocked hazard is visible 30 s ahead; the older v84 controller in those flights had P 0.7-0.86
   on it for 30 s and did not switch (coll cost 0 in its log), the current planner sees it at the window edge (tau
   22 s, P 0.3-0.75) at 38-41 s in ~75 % of flights and dodges once (mostly a burn pause).
5. Current-code Debris406 contacts: r3 m4c1_a (12.2 m/s, knocked after the dodge), m4c5_a (3.3 m/s, first warning
   6 s before), m5a1_a (7.8 m/s, knocked after a pause15 dodge at 40 s; the radar obstacle never reappeared, only
   camera tracks with sigma 11-18 m at tau 5 / 3 s); baseline m2fin_a 212242 (4.1 m/s, first warning 8 s before at
   sigma 17 m), m4c3_b (20.4), fin1_b (7.3). Pattern: after a contact on the rock the track's velocity sigma is
   metres per second, P on the plan spreads below the 0.3 gate and the rock drops out of the planning until 1-8 s
   before the hit.
6. Knock prediction from tracks (question B1/B2): not useful. Pair-contact prediction for the one relevant truth knock
   (Debris284/409, 013801) from tracks with 3 m / 0.15 m/s errors gives P 0.34-0.40 at 20-3 s lead (gap median 1-2 m,
   sd 4-6 m); the outgoing direction depends on the contact geometry known to +-4 m against radii sums of 15-30 m.
   With 2 relevant out of 895 contacts, a pairwise hazard term would price hundreds of harmless pairs per real
   event; 0 of 6 truth contacts and at most 9 of 27 Debris406 contacts would be in its scope. Not implemented.
7. Emergency manoeuvre (B3): axis caps +x 0.175, +y 0.143, -y 0.096, +z 0.096, -z 0.117 m/s^2, one axis per tick.
   Sideways in 2 / 4 / 6 s: 0.3 / 1.1 / 2.6 m at best (0.12 / 0.5 / 1.1 m at the 0.06 m/s^2 the planner counts on).
   The main jet needs a re-point first (0.05-0.1 rad/s^2, 0.2 rad/s: 46 deg in 4 s at most, 90 deg in ~10-12 s), so
   it buys nothing in 2-4 s. Against a 13 m rock plus 3.2 m hull no last-second manoeuvre helps; >= 10-12 s of
   warning is needed for ~6-10 m.
8. Route / timing exposure (B3/B4, `tools/exposure.py`, nominal no-dodge path on the 10 truth fields recorded past
   100 s): conflicts per flight 0.60 (no hold), 0.40-0.60 (hold 3-15 s), 0.30-1.00 (lateral offset +-40/80 m);
   Debris406 clearance < 10 m in 3/10 flights with no hold, 0/10 with a 6 s hold (median 16.5 -> 26.8 m), but
   Debris208 conflicts stay 3/10 for every hold. 10 fields is too few to separate the variants, the knocked-rock
   risk cannot be evaluated this way, and an 80 m offset at s ~ 200 m costs ~5 m/s. Not implemented.

9. What was implemented and tested offline (dev copy, switches default off): `RB_BIG_MARGIN=M` (guidance
   `big_margin_m`: +M m contact margin for radar-tracked rocks of radius >= 10 m, i.e. a wider berth for Debris406-like
   rocks) and `RB_KICK_GATE=P` (guidance `kick_gate`: P gate P instead of 0.3 for such rocks while their velocity
   sigma is >= 0.4 m/s, i.e. after a contact). Closed-loop benchmark `tools/test_knock.py` (point mass on the
   round-3 planner settings incl. burn pauses; a 26 m rock crossing at 5 m/s that the planner dodges, then a knock
   L = 6/10/14 s before the closest approach; radar re-convergence in 4.2 s epochs with velocity errors 2.0 / 0.8 /
   0.35 / 0.15 m/s, matching report_x_kicks): 
   - aimed knocks (toward the no-dodge position +-15 m, <= 5 or 10 m/s; 144 episodes per variant): contacts
     m0 6, m8 6, m15 4; g10 6, g20 6 (identical to m0). Extra fuel on the no-knock episodes: m8 +1.3 kg, m15 +2.2 kg.
   - random knocks (uniform direction, 3-12 m/s; 600 per variant): contacts m0 28 (L6 16/200, L10 10/200, L14 2/200),
     m15 26; m15 costs +1.7 kg per episode.
   Trace of a contact (`KNOCK_TRACE=1`): for 4 s after the knock the track still shows the old course, the next 4 s
   its velocity sigma is 2 m/s and P on the plan is ~0, and when the second post-knock return gives a usable state
   (P 0.46 -> 0.95) 7 s are left, too little for 0.143 m/s^2 against a 16 m contact radius. Knocks >= 14 s before
   the closest approach are survived already; knocks <= 10 s before are not, with any of the variants. Neither
   switch passes; no third arm flown (A continues to 40 per arm). The switches stay in robust/ default off.

## C. Launch fuel of the round-3 config (`tools/launch_waste.py`, `tools/launch_sim.py`)
Launch waste = fuel - 4.32 kg/(m/s) x speed. Undamaged flights, medians (candidate vs baseline): conf 2.99 vs 2.42 kg
at 15 s, m4c 3.00 vs 2.41, m5a (unloaded, first 5+5) 3.06 vs 2.33; pooled 30 vs 29 flights: 3.01 vs 2.41 at 15 s,
4.73 vs 3.98 at 30 s, speed at 15 s 1.05 vs 1.23 m/s at equal fuel (7.56 vs 7.77 kg). The pre-launch state at the
first tick is identical (no pre-launch cut in any flight, att 91-94 deg), the pitch-over timing too (nose within
20 deg at 10.6 vs 10.9 s), so it is not the watchdog (dly6 / dly25, watchdog only: no difference). The old
single-switch pair xp_brake vs xp_ctrl (XP_BRAKE = RB_BRAKE only) shows the same: speed at 15 s 0.91 vs 1.19 m/s,
waste 2.98 vs 2.44 kg. Cause: RB_BRAKE. While the rate is above the braking curve it brakes at gain 20 with every
tick (translation share 0), so the brake chatters through the end of the pitch-over, when the burn has started
(nose within the 20 deg pad gate): rotation fuel up, translation ticks lost. ~0.6-0.75 kg (~1 point) per flight.
Fix (dev copy, behind a switch, default off): `RB_BRAKE_KEEP=1` (thrust_law.py): while braking, translation keeps
its launch share once the nose is within PAD_ALIGN (20 deg) of the burn, and the brake gain is `RB_BRAKE_K`
(default 5) instead of 20. Offline replica `tools/launch_sim.py` (ThrustLaw + Pulser on the actuator LP replica,
rigid body, pad, recorded tick intervals, 1-tick actuator delay, 4 draws): waste at 15 / 30 s 2.45 / 3.20 kg with
RB_BRAKE, 2.21 / 2.93 with RB_BRAKE + RB_BRAKE_KEEP, 2.37 / 3.31 with no brake; overshoot after the nose first
reaches 5 deg 4.9 / 4.9 / 5.4 deg. Limits: the replica reproduces the baseline waste (2.37 vs 2.41 kg in flight)
but only +0.08 kg for RB_BRAKE (flights +0.6), and it cannot reproduce the loaded-machine overshoots (9-40 deg)
that RB_BRAKE was made for, so the fix's flight value and its overshoot protection are NOT verified. Not flown
(item C: no flights of its own).

## Regression of the dev copy (new switches default off)
`robust/dev` = robust/ + RB_BIG_MARGIN, RB_KICK_GATE (guidance.py, Controller.py) + RB_BRAKE_KEEP / RB_BRAKE_K
(thrust_law.py); first-tick log line gains `bigm .. kgate .. bkeep ..`; Controller.py header byte-identical.
With the switches off: test_gate2 (off, excess10), test_near, test_recover_plan, test_farmiss give output identical
to robust/ (and test_gate2 identical to milestone 2's gate2_out.txt), so the earlier offline safety evidence holds.

## D. Score work (owner extension, 11:00): where undamaged r3 flights lose points (`tools/m5attr.py m5a r3`)
31 undamaged r3 flights of A (at the time): median 48.4, best quartile 53.0 (clean flights: 37.7-38.6 kg, 220 s).
- Launch-phase dodges (mostly r13 = Debris208, 31 m, 1.4 m/s, conflict at 47-55 s, and r7/r2): planned turns of
  3-16 m/s in sum at 20-45 s, then 400-1000 m aim misses and 2-6 m/s re-aims: 6 of 31 flights at -21..35 points;
  the largest loss in sum (cruise fuel 387 points over 31 flights vs the best quartile).
- Debris406 burn pause (pause15 at 38-41 s, the window edge tau = 22 s): 14 of 31 flights, arrival ~229-232 s
  instead of ~220 s and +0.5-1 kg: ~3 points each (median 50 vs 53.5 for the no-pause clean flights).
- Launch waste of RB_BRAKE: ~0.6-0.75 kg (~1 point) in every flight (item C).
- Arrival: small (median 0.1 kg inside 600 m; 3 flights 11-25 kg after earlier misses). Re-aims after a dodge already
  aim at the ship from the current position (`aim_velocity(p, ...)`), not back to the old line.
Changes tried (offline first, then flown against r3, 12+ scored per arm, two lanes swapped):
1. `RB_BIG_MARGIN=8 RB_KICK_GATE=0.1` (Debris406 knock fixes; offline: no gain, flown on the owner's request).
2. `RB_BRAKE_KEEP=1` (launch waste).
3. `RB_BIG_LOOK=35` (guidance `big_look_s`; Controller relevant() reach): settled radar rocks of radius >= 10 m
   (velocity sigma < 0.15 m/s) are priced up to 35 s ahead instead of 22 s; other rocks keep the 22 s window
   (samples beyond it are masked, so their decisions are unchanged). Offline closed loop `tools/test_biglook.py`
   (launch-phase point mass, 18 threat + 18 pass episodes per rock type):
   | rock | variant | contacts | gap median / min | extra kg | delay s | points |
   |---|---|---|---|---|---|---|
   | Debris406-like (13 m, 5 m/s) threat | 22 s | 0 | 7.6 / 4.5 m | 0.98 | 7.6 | 3.37 |
   | | 35 s | 0 | 8.9 / 3.9 m | -0.15 | 8.6 | 2.14 |
   | Debris208-like (15.7 m, 1.4 m/s) threat | 22 s | 0 | 7.0 / 4.2 m | 3.38 | 6.3 | 6.21 |
   | | 35 s | 0 | 7.2 / 3.3 m | 1.34 | 4.4 | 2.97 |
   | pass episodes (both) | 22 s / 35 s | 0 / 0 | unchanged | ~0 / ~0 | 0.3-1.1 | 0.06-0.23 |
   With the switch off, test_gate2 / test_near / test_recover_plan / test_farmiss are unchanged (same as above).
E seeds (picked at random from 1000-99999, none used before): 48386 51806 49223 64895 67041 15495 62891 10657 62013 60938.

### D1 flown: k406 = r3 + RB_BIG_MARGIN=8 RB_KICK_GATE=0.1 vs r3 (m5d11-m5d13, 12:16-13:44; m5d12 single-lane:
gpt-oss reloaded; from 13:12 `tools/ollama_guard.sh` stops gpt-oss:20b every time it is loaded while flying,
log results/eval/ollama_guard.log; Roblox 4-6 processes at times)
| | k406 | r3 |
|---|---|---|
| scored (+rig fail) | 13 (+1) | 14 |
| undamaged 43+ / < 43 | 8 / 4 | 8 / 6 |
| contacts / lost | 1 / 0 | 0 / 0 |
| median / mean / worst | 45.7 / 23.8 / -102.7 | 46.0 / 31.8 / -19.4 |
| median fuel | 42.8 kg | 44.3 kg |
The k406 contact: Debris208 (r13) at 58 s, 33 kJ, after a launch-dodge sequence at 33-40 s (turns of 1.5-5.9 m/s, aim
miss 1040-1160 m, nose 70-83 deg off the burn at 3.3 m/s); the +8 m margin also applies to Debris208 (r 15.7).
Rejected: no gain, one more contact (rule).

### D2 flown (run 1, m5d21-m5d23, 13:44-15:00): bkeep = r3 + RB_BRAKE_KEEP=1 vs r3 - COMPROMISED BY LOAD
The ollama guard (13:12-14:59) stopped gpt-oss:20b 80 times: the other user's client reloaded the 12 GB model within
30-60 s every time, so the machine spent the whole run loading it (disk / memory churn). Both arms collapsed (median
29.6 bkeep vs 20.5 r3, median fuel 56 vs 63 kg; launch overshoot and dodge chaos). Guard stopped at 14:59 (stop file);
from then on gpt-oss is only stopped once before each batch by chain_m5.sh. Launch numbers (robust to the load,
concurrent): waste at 15 / 30 s 2.17 / 4.10 kg bkeep vs 2.74 / 4.90 kg r3; speed at 30 s 3.19 vs 2.96 m/s, i.e. the
launch waste is back at the baseline level (2.35 / 4.02 kg in A). Contacts 1/11 vs 0/12: Debris481 (16 kJ, 61 s) in a
dodge sequence after a pause, not in the launch. Re-run queued as m5d4 (after D3).

## F. Why the frozen baseline did worse today (owner question, 15:00)
### F1 (logs only; `tools/f1_timing.py`, `tools/f1_r13.py`)
Sets: pre-submission = soak_2111_a/b, r7_gate30_h22_cam, r7_gcam_c1/c2, sub_gate_1/2, clean_room (144 scored flights
with logs; 108 with complete status lines), today = A control arm (robust/ all off, 43 flights). The controller logs no
tick intervals; proxies: spacing of the ~1 s status lines, depth frames per second, harness wall s per sim s, first
tick time (robust only).
| proxy (median / p90) | pre-submission | today control | today r3 |
|---|---|---|---|
| status-line gap p95 within 0-70 s | 1.20 / 1.40 s | 1.22 / 1.48 s | 1.20 / 1.50 s |
| status gaps > 1.5 s / > 2.0 s (0-70 s) | 1.0 / 0.0 per flight | 1.0 / 0.0 | 1.0 / 0.0 |
| depth frames/s 10-60 s / after 60 s | 2.72 / 3.28 | 2.70 / 3.24 | 2.79 / 3.21 |
| wall s per sim s | 1.000 | 1.000 | 1.000 |
| pitch-over overshoot (1 Hz att) | 7.8 / 19.9 deg | 8.8 / 24.1 deg | 5.5 / 26.7 deg |
| fuel at 60 s | 30.2 kg | 30.4 kg | 34.8 kg |
| plan switches per flight | 3 / 83 | 39 / 129 | 2 / 17 |
| Debris208 (r13) conflict, P >= 0.3 | 12 / 144 (8 %) | 12 / 43 (28 %) | 7 / 46 (15 %) |
| Debris208 hits | 4 / 144 | 8 / 43 | 0 / 46 |
At 1 s resolution today's control flights were NOT slower than the 145 (same status-line spacing, frame rate and sim
pace). The difference is in the launch: overshoot >= 20 deg in 8/43 (19 %) vs 13/144 (9 %), and Debris208 conflicts
rise steeply with overshoot (later arrival at its crossing, ~135 m from the pad): pre 5 / 7 / 31 % conflicts for
overshoot < 8 / 8-20 / >= 20 deg; today 11 / 31 / 62 %. The baseline then flip-flops on Debris208 (plan switches by
tercile: contacts 0 / 4 / 20 % pre, 7 / 13 / 64 % today) - 8 of today's 12 baseline contacts are Debris208 at
52-58 s. Status-line gaps do not predict contacts (terciles 5/7/11 % pre, 27/29/29 % today). Load recorded per batch:
Ollama gpt-oss:20b (12 GB) loaded in 9 of 11 A batches' windows, Roblox 2-6 processes; the pre-submission sets ran
with Roblox only. Code paths: submission/ vs robust/ (all switches off) differ only in switch-guarded blocks
(guidance gate memory / near_fix / recover, thrust_law brake / recovery, Controller env reads and logging); with
switches off every guarded block is inactive (checked line by line; offline planner tests identical).
Interpretation: the pitch-over (3-13 s) is sensitive to loop jitter that 1 s proxies cannot see; today's heavier
load gives more overshoot, a later launch and more Debris208 conflicts, which the baseline cannot handle
(flip-flop) and round 3 can (gate memory: 0 Debris208 hits in 46). F2/F3 test this with flights.

### D3 in-flight observation and fix (15:40)
In the first D3 batch the long window made Debris208 (r13) appear at 16 s at the window edge (tau 34.8 s), and the
planner turned 0.46 m/s at 16, 17 and 20 s (aim miss 80 -> 240 m): the window-edge slide the baseline's `edge_drop`
comment describes. Added `RB_BIG_LOOK_EDGE=P` (guidance `big_look_edge_p`): a long-window rock whose closest approach
on the plan in force is still at the last window sample counts only if P >= the value. Offline (test_biglook, 35 s,
edge 0.6): Debris406-like threat 2.26 points (gap median 11.5, min 3.9 m), Debris208-like 3.17 points (7.2 / 3.3 m),
0 contacts, pass episodes 0.08 / 0.30 points; vs 3.37 / 6.21 points with the 22 s window. Regression with defaults
off: identical. Flown as m5d5 (blook2 = r3 + RB_BIG_LOOK=35 RB_BIG_LOOK_EDGE=0.6). Also added test-only
`RB_TICK_SLEEP=ms` (Controller: extra sleep per loop tick) for F3.
Queue (tools/chain_rest.sh): m5d4 bkeep re-run, m5d5 blook2, m5f31 / m5f32 (+50 / +150 ms per tick, off vs r3, 6+6
each), m5f2 (submission/ vs robust/ off, single lane alternating, 12 each).

### D3 flown: blook = r3 + RB_BIG_LOOK=35 (no edge rule) vs r3 (m5d31-m5d33, 15:08-16:45; m5d33 single-lane)
| | blook | r3 |
|---|---|---|
| scored (+rig fail) | 13 (+1) | 13 (+2) |
| undamaged 43+ / < 43 | 6 / 7 | 8 / 4 |
| contacts / lost | 0 / 0 | 1 / 1 |
| median / mean / worst | 38.2 / 34.6 / -8.5 | 48.6 / -159.0 / -2534.5 |
| median fuel | 47.3 kg | 40.4 kg |
r3 loss: Debris310 at 79.8 s moving at 33.9 m/s (pre-impact relative 36.1 m/s, 2.56 MJ), no warning in the log: a
genuine late knock. blook: safety held, but 7 kg more median fuel - the window-edge turns described above (fixed
version flown as m5d5). Rejected as it is.

## G. Hits of the robust arms in milestone 5 and recovery (owner change: rare unavoidable hits accepted, recovery must be good)
### G1 classification (`tools/hit_view.py`, `tools/contact_recovery.py`)
| flight | arm | rock, speed | first hit | warning (P >= 0.3 on the rock) | avoidable? | score |
|---|---|---|---|---|---|---|
| m5a1_a 261122_064053 | r3 | Debris406 knocked (4.9 -> 7.8 m/s) after a pause15 dodge at 40 s | 63.0 s, 207 kJ, thrown to 35.4 m/s, spin 1.6 rad/s | radar obstacle vanished after the knock; camera track P 0.32-0.38 at tau 4.8 / 2.8 s | no (knock < 8 s before; 0.143 m/s^2 buys < 2 m in 5 s) | -2023 (the loss came 8.2 s later: Debris162, 1.81 MJ, hit at 21 m/s while tumbling backwards at 35 m/s; the planner saw it at P 0.97, tau 8.4 s, but tumble mode allows no translation and a 1.6 rad/s spin cannot be stopped in 8 s) |
| m5d32_a 261122_15xx | r3 | Debris310 at 33.9 m/s (pre-impact relative 36.1 m/s) | 79.8 s, 2.57 MJ, loss at the hit | none in the log | no (a rock this fast crosses the 22 s window in < 1 s of warning per 36 m) | -2535 |
| m5d13_a 261122_133729 | k406 (candidate) | Debris208, 1.4 m/s | 58.2 s, 36 kJ (2 hits) | r13 P 0.36-0.87 from 33.8 s (tau 22 s) | yes: planner chaos (turns 1.5-5.9 m/s, aim miss > 1000 m, nose 70-83 deg off at 3.3 m/s), worsened by the +8 m big margin | -103 |
| m5d22_b | bkeep (run 1, load churn) | Debris481, 2.1 m/s | 60.8 s, 16 kJ | r35 P 0.30-0.53 from tau 11 s, Pbest 0.23-0.40; the planner kept the plan, then switched for another rock | partly (a 0.5-1 m/s dodge at tau 8-11 s was cheaper by its own costs only by < switch threshold) | -28 |
Rarity: in 46 + 13 + 13 + 11 + 13 r3-config flights of milestone 5 (96), knocked-rock losses 2 (2 %); with the
earlier sessions, Debris406 knocked contacts ~9 of ~400 logged flights of all versions and 2 truth-recorded knocks in 12 flights.
### G2 recovery quality
- RECOVERY line present in every robust hit flight (damage jump detected within 0.2 s).
- Small hits (16-36 kJ, shuttle left at 1-4 m/s): spin < 0.1 rad/s after 15-22 s, nose within 20 deg after 24-37 s,
  velocity error < 0.5 m/s after 74-79 s, 47-89 kg after the hit, both finished. Baseline arm (no recovery switches):
  nose within 20 deg after 55-225 s, 54-82 kg. The post-hit burn of m5d13_a ran at 4.4 kg per m/s (8.5 -> 0.04 m/s
  for 37.6 kg, ideal 4.32): the main-jet burn is near ideal; the loss is the physics (the hit stopped the shuttle at
  1.6 m/s, 1180 m out: a second launch of ~6 m/s = 26 kg) plus ~10 s of despin / re-point delay (~2 points) and the
  pre-hit dodge chaos. Ideal recovery from 1.6 m/s at 1180 m: re-accelerate to ~7.4 m/s (25 kg) and arrive ~20 s
  later than an undisturbed flight: score about -35 at 36 kJ vs -103 actual; the difference is the 37 kg of pre-hit
  launch dodges in that flight, not the recovery.
- Heavy hits: thrown to 21-35 m/s. Reversing 35 m/s on the main jets (0.175 m/s^2) takes >= 200 s and ~150 kg plus
  ~1300 m of return: finishing inside 400 s is marginal at best and the damage (1.8-2.6 MJ) already ends the run
  ('loss' end condition at the hit or the next hit). No controller action changes those two losses.

## State at stop (owner request, ~17:35 22 Nov)
All chains, runners, guards and sims stopped; no queue lock; the 7 frozen submission/*.py copied back into
STU4-Solution and verified with cmp (BasicController.py / UserAPI.py there are framework files); STU4-Sim
SpawnDebris.py has seed = 8193.
Finished: A (r3 46 vs base 43: contacts 1 vs 12, p = 0.001; lost 1 vs 2; median 48.0 vs 42.0), D1 (k406 rejected),
D2 run 1 (load-compromised), D3 (blook without edge rule rejected), F1 (logs), G1/G2 (analysis) - tables above.
D4 (bkeep re-run, m5d41-42, partial, rig failures 6 under memory pressure): bkeep 5 scored, 0 contacts, median 48.2,
mean 44.0, worst 27.0, fuel 42.9 kg; r3 3 scored, 0 contacts, median 38.3. Pooled D2 + D4 launch waste (17 vs 17):
2.25 / 4.18 kg at 15 / 30 s bkeep vs 2.72 / 4.85 r3. Contacts pooled bkeep 1/16 (Debris481, load run) vs r3 0/15.
Not flown: D4 to 12 per arm, D5 (blook2 = RB_BIG_LOOK=35 RB_BIG_LOOK_EDGE=0.6), F2 (submission/ vs robust/ off,
single lane), F3 (RB_TICK_SLEEP=50 / 150, off vs r3), G3 (kickY recovery on vs off, 3+3), the stack vs baseline
(20 per arm), E (seeds 48386 51806 49223 64895 67041 15495 62891 10657 62013 60938). Queue scripts:
tools/chain_q2.sh (D5, F3, G3), F2 line in tools/chain_rest.sh.
Recommended env (unchanged, round 3): RB_WATCH=1 RB_BRAKE=1 RB_NEAR=1 RB_GATE_HOLD=10 RB_GATE_MODE=excess
RB_RECOVER=1 RB_RECOVER_DEG=60 RB_RECOVER_BURN_DEG=30 RB_RECOVER_PLAN=1 RB_RECOVER_DVBURN=1 RB_RECOVER_SCORE=1.
robust/*.py = robust/dev/*.py (identical): round 3 code plus default-off switches. Unverified in flight / not accepted:
RB_BRAKE_KEEP (+RB_BRAKE_K; launch fix, safety not shown at 12 per arm), RB_BIG_LOOK_EDGE with RB_BIG_LOOK (offline
only), RB_BIG_MARGIN / RB_KICK_GATE (rejected), RB_TICK_SLEEP (test only). With all of them off, offline planner tests
are identical to round 3. Pre-milestone-5 robust copy: robust/dev/backup_r3/.

# Milestone 5, resumed (18:19, after the owner's stop)
Rig free at 18:19 (fast-mode work cleaned up); queue `tools/chain_q3.sh` (G3, G4, F3, D5, D4 finish, F2).
### G3 kick tests (m5g31-m5g33): RB_KICK=70 RB_KICK_STRONG=50 RB_KICK_DIR=y (50 s blind push on +y, then spin-up to
0.5 rad/s, released at ~120-129 s), krec = r3 vs knorec = r3 without the RB_RECOVER* switches
Rig notes: m5g31 lane b failed its 3 flights at setup (installed copy still the frozen submission/ until lane a's
install); m5g31_a flight 1 (261122_181947) ran without the kick and without robust's first-tick line (the frozen
copy): excluded. 4 flights hit rocks during the blind push itself (test artefact): krec 648 kJ at 47 s (before the
kick: Debris406 knocked to 18.9 m/s, 4 s warning), 235 / 81 / 91 kJ; knorec 1.64 MJ at 82 s.
| | recovery on (krec) | recovery off (knorec) |
|---|---|---|
| clean kicks (no hit during the push) | 3 (+1 lightly hit, 4 kJ: -22.6) | 5 |
| finished | 3 of 3 | 3 of 5 (1 unfinished at 400 s, 137 m out; 1 sim thrust stall at 325 s, 176 kg) |
| scores of finished | -181.3, -123.4, -120.4 (mean -141.7) | -167.0, -163.4, -189.6 (mean -173.3) |
| arrival / fuel of finished | 334-397 s / 153-197 kg (mean 169) | 356-388 s / 187-205 kg (mean 193) |
Recovery on: every clean kick finished, ~24 kg and ~30 points better on average; off: 2 of 5 did not finish.
### G4 heavy kick (m5g41, one batch): RB_KICK=50 RB_KICK_STRONG=120 RB_KICK_DIR=y (120 s blind push on +y + spin-up;
released at 179 s with |v| 16.6-17.9 m/s and a 10-13 m/s velocity error, the real post-hit state of the heavy hits)
recovery on: 2 flights (+1 rig failure), both unfinished at 400 s, 212-213 kg, 1898 / 2429 m out;
recovery off: 3 flights, all unfinished, 194-238 kg, 845 (after a 140 kJ hit) / 2886 / 2274 m out.
With 220 s left, reversing ~17 m/s on the main jets (0.175 m/s^2) takes ~100 s and the shuttle is by then
~1.5-2 km from the ship: finishing inside 400 s is not physically possible from such a state, with or without the
recovery logic. An unfinished flight has no score, so after a heavy hit there is no damage-limiting behaviour that
changes the outcome; the recovery logic used ~25 kg less and ended closer.
### F3 loop-rate sensitivity (m5f31: +50 ms per tick, m5f32: +150 ms per tick; RB_TICK_SLEEP, test only)
| added delay per tick | arm | scored | contacts | median | mean | median fuel | overshoot median / p90 | plan switches | frames/s 10-60 s |
|---|---|---|---|---|---|---|---|---|---|
| 0 (A, today) | off | 43 | 12 (28 %) | 42.0 | -39.8 | 45.0 kg | 8.8 / 24.1 deg | 39 | 2.70 |
| 0 (A, today) | r3 | 46 | 1 (2 %) | 48.0 | -5.6 | 40.5 kg | 5.5 / 26.7 deg | 2 | 2.79 |
| +50 ms | off | 7 | 1 (Debris208) | 37.8 | 27.5 | 50.1 kg | 14.6 / 31.0 deg | 59 | 2.31 |
| +50 ms | r3 | 8 | 0 | 49.1 | 46.8 | 40.5 kg | 2.5 / 19.9 deg | 2 | 2.58 |
| +150 ms | off | 7 | 1 (Debris208) | 27.9 | 14.6 | 57.8 kg | 30.7 / 48.2 deg | 59 | 1.61 |
| +150 ms | r3 | 7 | 1 (Debris406 knocked, 6.1 m/s, 196 kJ) | 32.6 | -9.5 | 53.5 kg | 4.0 / 4.5 deg | 9 | 1.68 |
(m5f311 / m5f321 were single-lane batches while gpt-oss was loaded.) The frozen-equivalent controller degrades
steadily with loop delay: pitch-over overshoot doubles per step, the launch arrives late at Debris208, the planner
flip-flops (59 switches) and the median falls 42 -> 38 -> 28. Round 3 keeps the pitch-over (RB_BRAKE: overshoot
2.5-4 deg even at +150 ms) and the dodge logic (gate memory) intact and holds its score at +50 ms (49.1); at +150 ms
(frames/s 1.7) it also loses ~15 points. This supports F1: the PC's load acts through the launch.
Proposals (not implemented): measure dt and use it where 0.2 s is assumed (planner cadence, latency_s), shed load
when ticks run long (fewer turn candidates, skip a depth frame), run the planner on a time budget. RB_BRAKE_KEEP
must keep RB_BRAKE's overshoot protection under load before it can replace it (not tested under load).

## State at final stop (owner ended the robust work, ~23:10 22 Nov)
All chains, runners, evaluate/run_sim pythons and sims stopped; no queue lock; the 7 frozen submission/*.py copied into
the rig's SOLUTION_REPO (%LOCALAPPDATA%/SimDynamX/STPro/Assets/Repos/STU4-Solution) and verified with cmp;
SpawnDebris.py seed = 8193. Note: at 22:07 an `ollama stop` from chain_m5.sh restarted the Ollama server (it was
not running) and an `ollama ps | grep` pipe then hung the chain; chain_m5.sh now writes `ollama ps` to a file with a
timeout. The Ollama server was left running (no model loaded).
Flown since the first stop (sections above): G3 kickY (recovery on 3/3 clean kicks finished, mean -141.7, 169 kg;
off 3/5, mean -173.3, 193 kg), G4 heavy kick (no finish possible either way), F3 (+50 / +150 ms per tick).
D4 RB_BRAKE_KEEP re-run (m5d41-m5d44): bkeep 14 scored, 0 contacts, 0 lost, median 47.0, mean 40.3, worst 12.5,
fuel 42.9 kg; r3 12 scored, 0 contacts, median 42.9, mean 40.3, worst 16.7, fuel 45.9 kg. Passes the acceptance rule
on the re-run (0 vs 0 contacts, 12+ per arm); with the load-compromised run 1, 1/25 vs 0/24. Launch waste back to
baseline. Not tested under a slow loop (F3 shows RB_BRAKE's overshoot protection matters there).
D5 blook2 (RB_BIG_LOOK=35 RB_BIG_LOOK_EDGE=0.6): only 2 + 2 flown: blook2 47.5, 49.2; r3 51.6, -163.1 (contact).
Unflown: D5 to 12 per arm, F2 (submission/ vs robust/ off; code diff shows switch-guarded differences only), the stack
vs frozen baseline (20+ per arm), E (seeds 48386 51806 49223 64895 67041 15495 62891 10657 62013 60938).
Recommended env: round 3 (RB_WATCH=1 RB_BRAKE=1 RB_NEAR=1 RB_GATE_HOLD=10 RB_GATE_MODE=excess RB_RECOVER=1
RB_RECOVER_DEG=60 RB_RECOVER_BURN_DEG=30 RB_RECOVER_PLAN=1 RB_RECOVER_DVBURN=1 RB_RECOVER_SCORE=1); candidate to add
after a stack measurement and a slow-loop check: RB_BRAKE_KEEP=1. robust/*.py = robust/dev/*.py.
