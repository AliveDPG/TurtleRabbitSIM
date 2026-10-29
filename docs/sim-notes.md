# Sim notes: Escape from Phobos (Space Teams PRO 0.55.0)

Written before any framework change. Sections "Time-scale findings" and
"Headless findings" are filled in as the work proceeds (see the end of the file).

## 1. What runs where

| Piece | Location |
|---|---|
| Engine root ("Platform") | `C:\Users\Public\SimDynamX\STPRO\Apps\Main` |
| Unreal client (GUI, Chaos physics, cameras, collisions) | `...\Main\SpaceCRAFT\SpaceCRAFT.exe` → `SpaceCRAFT\SpaceCRAFT\Binaries\Win64\SpaceCRAFT-Win64-Shipping.exe` |
| Compute server (stand-alone, no GUI) | `...\Main\Compute_Server\run_Server.exe` |
| Bundled Python 3.12 venv with the `spaceteams` module | `%LOCALAPPDATA%\SimDynamX\STPro\Versions\v0.55.0\py\Scripts\python.exe` |
| Asset root (`Local/` in sim paths) | `%LOCALAPPDATA%\SimDynamX\STPro\Assets` |
| Competition sim repo (organisers') | `Assets\Repos\STU4-Sim` (git clone of SimDynamX/STU4-Sim, clean) |
| Our controller as executed by the sim | `Assets\Repos\STU4-Solution\{Controller,nav,vision}.py` (byte-identical copies of this repo; `tools/compare.py` is what copies them) |
| Per-run logs | `%LOCALAPPDATA%\SimDynamX\STPro\Logs\<yymmdd_HHMMSS>_COMPETITION_SIM__<uuid3>\` |

A run is one Unreal process ("Main", server mode `Main_Interactive`) that
spawns one Python process per Python system (`ServerAsLocalClient`):

```
python.exe -u <Source.py> --sim=Local/Repos/STU4-Sim/Sims/COMPETITION_SIM.simconf --name=Controller --port=4119x --netid=N --serv_netid=1 --run_uuid=<uuid>
```

Systems in `COMPETITION_SIM.simconf` (all Python, all `Active`):

| System | Source | LoopFreqHz | Role |
|---|---|---|---|
| ApplyGravity | Core/Propagator/ApplyGravity.py | 1 | gravity on the shuttle |
| JetSelectionActuator | STU4-Sim/Systems/JetSelectionActuator.py | 60 | reads `JetSelection/CommandedAccel`, LP jet selection, applies force/torque, **accumulates `Stats/FuelUsed`** |
| Controller | STU4-Solution/Controller.py | 60 | ours |
| Radar | STU4-Sim/Systems/Radar.py | 120 | one debris per tick, noisy, re-sorted every NumDebris/10 ticks |
| Scoring | STU4-Sim/Systems/Scoring.py | 10 | win/loss check, score |
| SpawnDebris | STU4-Sim/Systems/SpawnDebris.py | 60 | gravity on one debris per tick + random initial impulses |

Physics: shuttle and all 500 debris use `Dynamics/Propagator: "Unreal_Chaos"`,
i.e. they are integrated by Unreal's Chaos solver **inside the GUI client**, at
the client's frame rate. Collisions and the `Stats/Damage` joules come from
Chaos contacts in the client. Phobos/Mars are SPICE. Phobos gravity field is
sampled by the Python systems above and pushed back as `PhysEffect`s.

## 2. How the sim is launched today

GUI only: SpaceCRAFT → log in → *Sim* → tick *User-Made Sims* →
`Repos/STU4-Sim/Sims/COMPETITION_SIM.simconf` → *Confirm* → role screen
(*Orbit Pawn*) → *Start Simulation*. `tools/autorun.py` automates exactly those
clicks with pixel checks and reads the score from the run log.

Direct launch (documented in the STP wiki for 0.28, verified here in 0.55, now
what `run_sim.py` does):

```
cd C:\Users\Public\SimDynamX\STPRO\Apps\Main\SpaceCRAFT
SpaceCRAFT.exe --sim=Local/Repos/STU4-Sim/Sims/COMPETITION_SIM.simconf -firstRole
```

* No login screen, no menu: Loading → Lobby → Sync → Init → Runtime in ~3 s
  (log folder `261027_094913_COMPETITION_SIM__c9c`). `-firstRole` takes the
  first role in the config, which is *Pilot* (the GUI path used *Orbit Pawn*;
  both are camera pawns attached to the shuttle, no physics role).
* The client spawns `run_Server.exe` as the compute server ("Headless
  run_Server process created") and the server spawns the Python systems.
* The sim does **not** stop itself after the score (Scoring keeps looping), so
  the launcher has to stop it. `run_sim.py` adds a Harness Python system
  (`harness/Harness.py`) that watches the same end conditions as Scoring and
  calls `st.stop_sim()` 3 sim-seconds after them; the client then exits.
* `IncludeSims` cannot be used to add a system next to the competition config:
  each Python system process re-reads the simconf itself and only sees systems
  defined in the top-level file (others die with "no matching System ... found
  in sim config"). The wrapper written per run is therefore a verbatim copy of
  the scenario file with the Harness system appended, under
  `Assets\Repos\STU4-Eval\Sims\RUN_<id>.simconf`.
* Other launch modes probed (details in §8): `-RenderOffScreen` works;
  `-nullrhi` loads but never leaves the Lobby (the "Ready for Sync" step is UI
  driven); `run_Server.exe -Main_Headless` runs but nothing with the
  `Unreal_Chaos` propagator moves. `-det` (deterministic scheduler) only
  affects C++ systems; this sim has none.
* Only one sim at a time (fixed ports 41190+). `run_sim.py` refuses to start
  over a running sim and only ever kills processes it started itself.

## 3. Time

* Sim clock: `SimEntity/Time/{StartDateTAI, Timescale, UseCurrentTime}` in the
  simconf; runtime API `st.SimGlobals.SimClock.{GetTimescale,SetTimescale,Freeze,UnFreeze}`.
  Every process logs `Initial Timescale: X` when Runtime starts.
* What the timescale changes: sim time = wall time × timescale; Chaos physics
  in the client advances by sim time; C++ systems with `FreqType: Sim` scale
  their update rate. **Python systems do not**: every system above paces
  itself with `time.sleep(1/LoopFreqHz)` in wall-clock time, so at timescale k
  the controller, radar, actuator, scoring and gravity loops all run k× fewer
  iterations per sim second.
* Scoring timers: `Scoring.py` accumulates `sim_elapsed_time` from SimClock
  deltas (sim time, correct at any timescale). Fuel, however, is accumulated by
  the actuator **per actuator tick, independent of dt**
  (`GetEffectiveForceAndTorque_VariableOnTime` ignores `dt_tick`), so fuel used
  for the same manoeuvre scales with wall-clock ticks, i.e. ~1/timescale.
* Organisers' rule (Rules & Info #218): "Do not change timescale of the
  simulation. This subtly alters the physics behavior." Scored runs must be 1×.
* Render rate: client FrameRateLimit=0, vsync off (GameUserSettings.ini).

## 4. Scoring

`Scoring.py` (10 Hz wall clock), parameters from its `Inst_Parameters`:
`NominalTime_s=120`, `NominalFuel_kg=1.15`, `DamageThreshold_J=209200`,
`KeepOutRadius_m=50`.

* Win: `|shuttle − target| ≤ 50 m` →
  `raw = 120/t − 2·damage/209200 − max(fuel/1.15 − 1, 0)`
* Loss: `damage > 209200 J` →
  `raw = −dist/dist0 − 2·damage/209200 − max(fuel/1.15 − 1, 0)`
* `score = (raw + 0.5) × 100` (nominal run = 50, uncapped both ways)

Emitted once via `st.OnScreenAlert`, which lands in `COMPETITION_SIM.log` as
`Successfully escaped Phobos with a score of X` or
`Crashed into some debris and finished with a score of X`, followed by
`Sim elapsed time = …`, `Distance to ship = …`, `Total damage taken = …`,
`Total fuel used = …`. `tools/autorun.py` already parses these.

## 5. Scenario parameters and what the organisers can vary

Everything below is a plain value in `COMPETITION_SIM.simconf` or a constant in
the organisers' system scripts, so any of it can change in the scored sim.

| Where | Parameter | Current | Notes |
|---|---|---|---|
| SpawnDebris | `NumDebris` | 500 | spawned once, then **baked into the simconf** (`DoSpawning=false`) |
| SpawnDebris | `Spread_m` | (300, 300, 500) | half-widths of the box, Phobos-frame axes |
| SpawnDebris | `VerticalOffset_m` | 500 | box centre = shuttle + 500 m radially up |
| SpawnDebris | `DebrisWeights` | 0.45/0.35/0.15/0.05 | Rock1/Rock2/Rock3/ExplodedTank |
| Entities | `ScaleMin/Max`, `RadarDiameter_m`, mass | Rock1 10–400 ×0.08 m, Rock2 20–800 ×0.05 m, Rock3 40–1000 ×0.01 m, Tank 17 m | diameter = base × scale, mass = base × scale³ |
| SpawnDebris (script) | speed | 0.05–0.10 × √(μ/R) = 0.40–0.80 m/s tangential | `seed = 100` hard-coded; random impulse σ 0.02 m/s at start |
| SpawnDebris (script) | `doTrajectoryAdjustment` | False | "hunting" impulses toward the shuttle; a one-line switch |
| Radar | `NumDebris` | 200 | radar reports only Debris0..199 |
| Radar | `Position_1sigma_m`, `Velocity_1sigma_m_s`, `Diameter_1sigma_fraction` | 2.0, 0.02, 0.05 | rng seed 100 hard-coded |
| Radar | `LoopFreqHz` | 120 | one debris updated per tick |
| Scoring | nominal time/fuel, damage threshold, keep-out | 120 s, 1.15 kg, 209.2 kJ, 50 m | |
| SpaceBuggy | `Thrusters/MaxAccel_m_s2`, `MaxAngAccel_rad_s2` | 0.5, 0.05 | actuator clamps commands |
| SpaceBuggy | mass, start location | 3388.7 kg, (−651.0, 95.2, −8239.7) | |
| AsteroidCycler | target location | (−850, 120, −9530); 1306 m away | |
| Controller | `LoopFreqHz` | 60 | |

Debris field reproduction: the spawn maths (`np.random.seed(100)` draws in
the order loc, vel-mag, azimuth, angle, axis, angvel, size; `random.seed(100)`
for the type choice; NWU frame from the location) was re-implemented in numpy
and reproduces the baked `Debris0..4` locations, types and scales exactly.
This is what `scripts/make_config.py` uses to bake new fields for other seeds
and for the hard config, so the organisers' `SpawnDebris.py` stays untouched.

## 6. Determinism and "seeds"

There is no seed to set in the competition sim: the field is baked, the two
RNGs are hard-coded to 100. Yet identical code does **not** reproduce: the
Python systems are asynchronous wall-clock loops and the client frame rate
varies, so radar sample timing, actuator tick timing and Chaos contacts differ
run to run. From `results/runs.csv` (62 GUI runs of successive controller
versions, 1×): wins score mean 93 with s.d. 63, 11/62 runs crashed. The
noise floor for *any* "A matches B" claim is therefore large and every
comparison in this framework is done on batches, not single runs.

One sim-level failure mode seen in 7 of 55 GUI runs of the session (6 timed
out, 1 drifted into the keep-out: `261027_234027_hard_s5_ts1_gui`) and never
in 20 off-screen runs (e.g. `261027_214547_default_s10_ts1_gui_parityA`,
`…_s12_ts1_gui_parityA`, `261027_231237_default_s9_ts1_gui`): from t ≈ 3–10 s
the shuttle's velocity stays exactly constant (3.92, ~1, 1.94 m/s) for the
rest of the run although the controller commands 0.5 m/s² the whole time and
the actuator keeps charging fuel (16–19 kg by t = 400 s). Constant velocity
under a constant commanded force is what the client does when it falls back
from the `Unreal_Chaos` propagator to `Custom` (the client log strings
"…Unreal_Chaos propagator has invalid Actor root component. Setting
propagator to Custom instead" exist in the executable), i.e. most likely a
start-up race in the client, not anything the controller does. The harness
ends such runs at `MaxSimTime_s`, `evaluate.py` counts them as failures and
can re-run them (`--retries 1`). It became more frequent late in the session
(5 of the last 25 GUI launches); clearing the ~70 accumulated per-run wrapper
simconfs from `STU4-Eval\Sims` (which the client scans at start-up) did not
stop it, so the cause is not confirmed. It never happened off-screen, which is
one more reason to re-test that mode. A separate one-off (1 in 55 launches):
the client stayed in the Lobby and never started
(`261027_230043_default_s3_ts1_gui`); same handling.

Follow-up (28–29 Oct, 20-seed `hard-debris` batch): the stall hit 6 of 20
first attempts plus 2 partial stalls that recovered mid-flight (velocity
frozen for 42 % of the samples), i.e. it is now the dominant cost of a batch.
Two mitigations are in the framework: the Harness aborts a run as soon as the
velocity has been bit-identical for 4 s while thrust is commanded
(`StallAbort_s`, result error "sim thrust stall"), so a stalled attempt costs
~40 s instead of 10 min, and `run_sim.py` waits `--cooldown` 20 s before each
launch. The cool-down did not eliminate it (seed 14 stalled with it), so
`evaluate.py --retries 1` remains necessary; `report_batch.py` flags any
scored run whose velocity was frozen > 25 % of the time as "stall-affected"
and keeps it out of the controller statistics.

In this framework "seed" means the debris-field seed used by
`scripts/make_config.py`; `--seed 100` on the default config reproduces the
competition field exactly.

## 7. Time-scale findings

How it is exposed: `run_sim.py --timescale k` (also `scripts/evaluate.py`,
`scripts/parity_check.py --b-timescale k`). The Harness system calls
`st.SimGlobals.SimClock.SetTimescale(k)` at Runtime start and logs
`Harness: timescale requested k, clock reports k`; every run's `result.json`
carries `harness_timescale`, and the trajectory CSV has a `timescale` column
sampled every 0.1 s, so a run that silently ran at the wrong speed is visible.
(Setting `SimEntity/Time/Timescale` in the simconf was not used: the harness
route needs no change to the scenario file.)

Single-run probe, competition field, GUI (`results/runs/261027_210017_default_s100_ts4_gui_tsprobe`):

| | 1× (baseline run 2) | 4× |
|---|---|---|
| sim seconds per wall second (trajectory CSV) | 1.00 | 4.00 |
| wall time to score | 102 s | 32 s |
| flight time | 102.0 s | 123.3 s |
| fuel | 1.51 kg | 0.79 kg |
| distance left at t = 60 s | 665 m | 947 m |
| score | 136.2 | 147.3 |

So the sim does run 4× faster, but it is a different run: with every Python
system paced in wall-clock time the controller gets 15 control ticks per sim
second instead of 60, the radar refreshes 4× less often, and the actuator's
per-tick fuel accounting charges a quarter of the fuel for the same burn (the
organisers' "subtly alters the physics behavior"). The score moves *up*
because of the fuel artefact, which is exactly the wrong direction for a
development loop. Batch evidence (10 seeds at 2× and 1.5× against the 1×
reference set, and the 1× noise floor) is appended in §9.

## 8. Headless findings

Candidates and what happened (competition field, seed 100, one run each unless stated):

| Mode | Launch | Result |
|---|---|---|
| `server` | `run_Server.exe --sim=… -Main_Headless` | sim reaches Runtime, all Python systems run, but the shuttle never moves (position constant for 220 sim-s) while `Stats/FuelUsed` climbs to 7.96 kg: `Unreal_Chaos` propagation, collisions and cameras live in the client. Unusable; swapping the propagator to `ST_Rigid_Body` would be a different physics (no contacts → no damage) so it was not pursued. |
| `nullrhi` | `SpaceCRAFT.exe --sim=… -nullrhi -firstRole` | client loads the sim and spawns the systems but stays in Lobby forever (the "Ready for Sync" step is driven by the UI, which does not tick without an RHI). Unusable. |
| `offscreen` | `SpaceCRAFT.exe --sim=… -RenderOffScreen -firstRole` | full client, Chaos physics, collisions and camera captures (`frames=62`, no capture errors), no window on the desktop. Probe: win, 109.3, 101.8 s, 1.82 kg. This is the headless candidate; the 20-seed parity batch against the GUI is in §9. |

Note on wall time: the client itself is the physics engine, so none of the
modes is faster than real time at 1×; "headless" here means *no window and no
focus steal*, not *faster*. Speed can only come from the time scale, with the
caveats of §7.

## 9. Batch evidence

All runs: current controller (`bc5ed3b` + working tree, byte-identical in the
sim's copy), default scenario, field seeds 1–20 baked by `make_config.py`,
one sim at a time, 27 Oct 2026. Raw rows: `results/parity/parity_offscreen_20/runs_{A,B}.jsonl`,
`results/eval/<batch>/runs.jsonl`; per-run folders under `results/runs/`.

### 9.1 Headless parity: GUI 1× (A) vs off-screen 1× (B), 20 seeds

```
python scripts/parity_check.py --config default --seeds 1-20 --b-mode offscreen --batch parity_offscreen_20
```

| seed | A (gui) | B (offscreen) | B−A | traj max dev (m) | t A / B (s) | fuel A / B (kg) |
|---|---|---|---|---|---|---|
| 1 | 146.2 | 135.6 | −10.6 | 38 | 103.2 / 104.9 | 1.38 / 1.48 |
| 2 | 87.9 | 103.4 | +15.5 | 60 | 108.4 / 108.7 | 1.99 / 1.80 |
| 3 | 135.2 | 134.7 | −0.5 | 30 | 103.5 / 105.2 | 1.50 / 1.49 |
| 4 | 141.7 | 129.4 | −12.3 | 49 | 101.3 / 102.3 | 1.46 / 1.59 |
| 5 | −131.5 (hit) | 106.0 | +237.5 | 127 | 106.2 / 104.9 | 2.94 / 1.82 |
| 6 | 149.2 | 126.0 | −23.2 | 62 | 102.8 / 101.4 | 1.35 / 1.64 |
| 7 | −581.5 (loss) | 17.0 | +598.5 | 175 | 65.1 / 116.9 | 1.68 / 2.71 |
| 8 | 118.6 | 92.4 | −26.2 | 35 | 106.8 / 107.9 | 1.65 / 1.94 |
| 9 | −502.7 (loss) | 130.2 | +632.9 | 88 | 84.0 / 107.9 | 2.37 / 1.51 |
| 10 | no score (thrust stall) | −463.9 (loss) | – | – | – / 48.6 | – / 1.47 |
| 11 | 153.5 | 65.1 | −88.4 | 163 | 104.1 / 112.5 | 1.29 / 2.20 |
| 12 | no score (thrust stall) | 102.7 | – | – | – / 106.6 | – / 1.84 |
| 13 | 27.9 | 58.3 | +30.4 | 146 | 120.3 / 114.7 | 2.55 / 2.26 |
| 14 | 118.1 | −184.1 (hit) | −302.2 | 357 | 105.3 / 129.6 | 1.68 / 3.73 |
| 15 | 97.2 | 62.6 | −34.6 | 105 | 110.3 / 112.6 | 1.86 / 2.23 |
| 16 | 83.8 | 58.5 | −25.3 | 77 | 110.5 / 109.2 | 2.01 / 2.32 |
| 17 | 154.2 | −1106.6 (loss) | −1260.8 | 101 | 103.6 / 29.9 | 1.28 / 1.07 |
| 18 | −586.4 (loss) | 131.7 | +718.1 | 41 | 63.0 / 108.1 | 1.91 / 1.49 |
| 19 | −175.4 (hit) | 145.3 | +320.7 | 190 | 115.8 / 103.8 | 2.84 / 1.38 |
| 20 | 96.1 | 152.8 | +56.7 | 39 | 104.0 / 104.1 | 1.95 / 1.29 |

Pass rate at |Δ| ≤ 5 points: **1/20** (≤ 20 pts: 4/20, ≤ 40 pts: 9/20).
**Verdict: FAIL against the per-seed bar.** Shuttle trajectories diverge by
30–60 m even on seeds where both scores are within 15 points, i.e. the
controller takes a different avoidance each time.

Distributions, however, are the same within what 20 runs can resolve:

| | GUI (A) | off-screen (B) |
|---|---|---|
| scored / wins / losses / no-score | 18 / 15 / 3 / 2 | 20 / 18 / 2 / 0 |
| runs with any damage | 5 | 3 |
| score mean ± sd (scored runs) | −26 ± 254 | 0 ± 290 |
| score median | 97 | 103 |
| time to ship, wins (s) | 107.1 | 109.0 |
| fuel (kg) | 1.87 | 1.86 |

Interpretation: the off-screen client is the same physics, scoring and camera
path (it is the same executable with rendering to no window), and nothing in
the batch separates it from the GUI; what separates *every* pair is the sim's
own run-to-run randomness (§6, §9.3). Per the brief the per-seed bar is not
met, so off-screen is **not shipped as a validated mode**: `run_sim.py --mode
offscreen` refuses to run unless `--unvalidated` is given, and
`scripts/parity_check.py` (which passes that flag itself) is the way to re-test
it after a sim or controller change.

### 9.2 Time scale: GUI 2× vs the GUI 1× reference (seeds 1–10 of 9.1)

```
python scripts/evaluate.py --config default --seeds 1-10 --timescale 2 --batch default_ts2_gui
python scripts/parity_check.py --from results/eval/ref_gui_ts1_seeds1-10.jsonl results/eval/default_ts2_gui/runs.jsonl
```

| seed | 1× | 2× | Δ | fuel 1× / 2× (kg) | time 1× / 2× (s) |
|---|---|---|---|---|---|
| 1 | 146.2 | −332.3 (loss) | −478.5 | 1.38 / 0.88 | 103.2 / 55.3 |
| 2 | 87.9 | 113.5 | +25.6 | 1.99 / 1.64 | 108.4 / 112.8 |
| 3 | 135.2 | 100.4 | −34.8 | 1.50 / 1.69 | 103.5 / 123.5 |
| 4 | 141.7 | 163.9 | +22.2 | 1.46 / 0.74 | 101.3 / 105.4 |
| 5 | −131.5 | 164.0 | +295.5 | 2.94 / 1.11 | 106.2 / 105.3 |
| 6 | 149.2 | 164.1 | +14.9 | 1.35 / 0.87 | 102.8 / 105.1 |
| 7 | −581.5 | 160.1 | +741.6 | 1.68 / 0.80 | 65.1 / 109.0 |
| 8 | 118.6 | −425.2 (loss) | −543.8 | 1.65 / 0.67 | 106.8 / 40.9 |
| 9 | −502.7 | 69.0 | +571.7 | 2.37 / 1.44 | 84.0 / 122.4 |
| 10 | stall | 23.7 | – | – / 1.51 | – / 125.4 |

Pass rate at |Δ| ≤ 5: 0/10. Batch view: fuel **1.13 kg at 2× vs 1.81 kg at 1×**
(−38 %, the per-tick fuel accounting of the actuator), winning flight time
113.6 s vs 107.1 s (half the control ticks per sim second), mean score 20 vs −48
with 2 losses in 10 at both. Wall time per run 67 s instead of 120 s. The 4×
probe (§7) showed the same artefacts at twice the size. A 1.5× set follows
in 9.4 for the gradient; the per-seed bar is already failed at 2×.

### 9.3 Noise floor: GUI 1× vs GUI 1×, same fields (seeds 1–10), nothing changed

```
python scripts/evaluate.py --config default --seeds 1-10 --timescale 1 --batch default_ts1_gui_repeat
python scripts/parity_check.py --from results/eval/ref_gui_ts1_seeds1-10.jsonl results/eval/default_ts1_gui_repeat/runs.jsonl
```

| seed | run 1 | run 2 | Δ | traj max dev (m) |
|---|---|---|---|---|
| 1 | 146.2 | −692.8 (loss) | −839.0 | 69 |
| 2 | 87.9 | 120.4 | +32.5 | 18 |
| 3 | 135.2 | lobby hang | – | – |
| 4 | 141.7 | 144.5 | +2.8 | 17 |
| 5 | −131.5 | −463.8 (loss) | −332.3 | 175 |
| 6 | 149.2 | 164.5 | +15.3 | 28 |
| 7 | −581.5 (loss) | 20.1 | +601.6 | 85 |
| 8 | 118.6 | 14.6 | −104.0 | 73 |
| 9 | −502.7 (loss) | thrust stall | – | – |
| 10 | thrust stall | −5.1 | – | – |

Same executable, same window, same field, same controller: pass rate at
|Δ| ≤ 5 is **1/10**, mean |Δ| 275 points, and 4 of the 7 comparable seeds
flipped between "clean" and "hit/loss". The GUI-vs-offscreen comparison of
9.1 (1/20 within 5 points, mean |Δ| 244) is therefore indistinguishable from
the sim's own noise floor. **The per-seed criterion of the brief cannot be met
by any launch mode, including the real one against itself.** The usable
comparison unit is a batch mean over ≥ 20 fields (standard error ≈ 60
points at this controller's spread), which is what `scripts/evaluate.py`
reports.

### 9.4 Time scale 1.5× (seeds 1–10, GUI) and the gradient

```
python scripts/evaluate.py --config default --seeds 1-10 --timescale 1.5 --batch default_ts1.5_gui
```

| seed | 1× | 1.5× | fuel 1× / 1.5× (kg) |
|---|---|---|---|
| 1 | 146.2 | 166.2 | 1.38 / 1.00 |
| 2 | 87.9 | −253.5 (loss) | 1.99 / 1.05 |
| 3 | 135.2 | 118.7 | 1.50 / 1.66 |
| 4 | 141.7 | 165.3 | 1.46 / 1.04 |
| 5 | −131.5 | 96.5 | 2.94 / 1.74 |
| 6 | 149.2 | 153.7 | 1.35 / 1.28 |
| 7 | −581.5 | 112.5 | 1.68 / 1.64 |
| 8 | 118.6 | stall | 1.65 / – |
| 9 | −502.7 | stall | 2.37 / – |
| 10 | stall | −67.5 (hit) | – / 1.78 |

Pass at |Δ| ≤ 5: 1/10. Batch view across all default-config GUI runs of the session:

| time scale | runs scored | fuel mean (kg) | win time (s) | losses | median score | wall / run |
|---|---|---|---|---|---|---|
| 1× | 26 | 1.92 | 106.7 | 5 | 92 | 120 s |
| 1.5× | 8 | 1.40 | 109.0 | 1 | 116 | 86 s |
| 2× | 10 | 1.14 | 113.6 | 2 | 107 | 67 s |
| 4× (1 run) | 1 | 0.79 | 123.3 | 0 | 147 | 47 s |

Fuel falls monotonically with the time scale (≈ k^−0.6) and flight time
rises, both as predicted from the actuator and controller loop code, and the
score drifts *upward*. There is no time scale above 1× that reproduces 1×; the
effect is already visible at 1.5×.

## 10. Conclusions for the development loop

* **Max stable time scale: 1×.** Mechanically every time scale works
  (verified at 1.5×, 2×, 4×: the clock reports the requested value, wall time
  shrinks accordingly, the run completes and is scored). But none reproduces
  1× even on average: fuel scales ~1/k because the actuator charges per
  wall-clock tick, the controller gets 60/k ticks per sim second, and the
  organisers forbid it for scored runs. `--timescale` stays available for
  smoke tests (does the controller still fly?) and is reported in every
  result, never for scores that are meant to be compared with the real thing.
* **Headless: not shipped.** Off-screen rendering is the real client with no
  window and behaves like the GUI in every batch statistic, but the brief's
  per-seed criterion cannot be met by *any* mode because the GUI does not
  reproduce itself (9.3). `--mode offscreen` is gated behind `--unvalidated`;
  `scripts/parity_check.py` re-tests it against the GUI whenever wanted.
  Compute-server-only and `-nullrhi` modes are dead ends (§8).
* **What a trustworthy comparison looks like:** same config, ≥ 20 field
  seeds, GUI 1×, compare batch means / medians / loss counts from
  `scripts/evaluate.py`; expect ±60 points of standard error on the mean at
  this controller's spread, and re-run seeds that end in a thrust stall
  (`--retries 1`). One run tells you almost nothing.
* **Wall-clock cost:** ~2.0–2.3 min per run at 1× (100–110 s of flight,
  ~20 s start-up/teardown). 20 seeds ≈ 45 min. The sim takes the screen while
  it runs (fullscreen client).
