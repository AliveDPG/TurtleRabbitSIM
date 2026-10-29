# Hard config (`configs/hard.yaml`)

A deliberately adverse version of the competition scenario, built only from the
parameters the organisers can actually change (see `docs/sim-notes.md` §5).
`configs/default.yaml` and the organisers' `COMPETITION_SIM.simconf` are
untouched; the hard scenario is baked on demand by `scripts/make_config.py`
into `%LOCALAPPDATA%\SimDynamX\STPro\Assets\Repos\STU4-Eval\Sims\`, using the
same spawn maths as the organisers' `SpawnDebris.py` (verified bit-exact on the
competition field) and the organisers' own Radar/Scoring/Actuator scripts with
different instance parameters.

## What changed and why

| Parameter | Default | Hard | Why it is the adverse direction |
|---|---|---|---|
| debris count | 500 | 700 | more rocks on and near the corridor |
| spawn box half-widths (m) | 300, 300, 500 | 220, 220, 450 | with the count change, number density ×2.9; the corridor from the pad to the ship runs through the box |
| type weights (Rock1/2/3/Tank) | .45/.35/.15/.05 | .35/.35/.15/.15 | 3× more 17 m exploded tanks, the biggest objects |
| rock scale multiplier | 1.0 | 1.3 | diameters +30 % (cross-section ×1.7, mass ×2.2 → more damage per hit) |
| debris speed | 0.40–0.80 m/s | 0.80–1.60 m/s | twice the closing-rate error from the same radar noise; constant-velocity forecasts degrade faster |
| radar: rocks reported | 200 of 500 | 150 of 700 | 79 % of the field is invisible to the radar (default 60 %) |
| radar σ (pos / range-rate / diam) | 2 m / 0.02 m/s / 5 % | 3 m / 0.04 m/s / 10 % | noisier tracks |
| radar loop rate | 120 Hz | 60 Hz | each reported rock revisited every 2.5 s instead of 1.67 s |
| damage threshold | 209.2 kJ | 150 kJ | run is lost after less damage; damage penalty per joule ×1.4 |
| nominal fuel | 1.15 kg | 1.0 kg | fuel penalty starts 0.15 kg earlier |
| nominal time | 120 s | 110 s | same flight time scores 8 % lower |
| keep-out radius | 50 m | 40 m | must get 10 m closer to the ship (through its surroundings) |
| max linear / angular accel | 0.5 / 0.05 | 0.4 / 0.04 | 20 % less control authority for the same avoidance |

Not covered (needs a change to the organisers' scripts, so left out):
the "hunting" impulses in `SpawnDebris.py` (`doTrajectoryAdjustment`), the
radar/impulse RNG seeds (fixed at 100 inside the scripts), and the start /
target positions (would move the whole scenario rather than make it harder).

## Baseline on the hard config

Current controller, GUI, 1×, field seeds 1–5 (`results/eval/hard_ts1_gui/runs.jsonl`):

```
python scripts/evaluate.py --config hard --seeds 1-5 --batch hard_ts1_gui
```

| seed | outcome | score | time (s) | fuel (kg) | damage (J) | note |
|---|---|---|---|---|---|---|
| 1 | win | 21.2 | 114.5 | 2.25 | 0 | clean but slow and fuel-heavy |
| 2 | no score | – | – | 12.7 by t=400 | 0 | sim thrust-stall (see sim-notes §6), not a controller result |
| 3 | loss | −4083.2 | 25.2 | 0.72 | 3 028 464 | head-on with a large rock 25 s into the main burn |
| 4 | win | −23.1 | 117.6 | 2.35 | 23 533 | arrived with one hit |
| 5 | "win" | −786.5 | 313.9 | 9.72 | 0 | sim thrust-stall that drifted into the keep-out; not a controller result |

Controller-attributable results: 3 of 5 (21.2, −4083.2, −23.1), mean −1362,
one loss in three. Two runs were lost to the client-side thrust stall that
hit 7 of 55 GUI runs in the same session across all configs (its onset is in
the first 10 s, before the field matters, so it is not a hard-config effect).

Comparison with the default scenario, same controller, same session (GUI 1×,
20 + 10 runs, `docs/sim-notes.md` §9.1/9.3):

| | default (30 GUI runs) | hard (5 runs) |
|---|---|---|
| clean arrivals (no damage) | 18 / 26 scored | 1 / 3 scored |
| runs with damage | 8 / 26 | 2 / 3 |
| losses (damage threshold) | 5 / 26 | 1 / 3 |
| median score of scored runs | 92 | −23 |
| fuel, clean wins (kg) | 1.2–2.8 (median 1.6) | 2.25 |
| time, clean wins (s) | 101–120 (median 104) | 114.5 |

Five runs is a small sample, but the direction is unambiguous: on the hard
field the current controller hits something in two runs out of three, and even
its clean run is 15 % slower and uses 1 kg more fuel than a typical default
run (the 0.4 m/s² cap and 40 m keep-out alone cost ~10 s). A planner that
scores well here has margin against anything the organisers can plausibly do
with the parameters in the table above. Use ≥ 10 seeds (`--seeds 1-10
--retries 1`) for any comparison on this config; the per-run spread is huge.

## Debris-only variant: `configs/hard-debris.yaml`

Same rocks as `hard` (700, tighter box, 3× tanks, 0.8–1.6 m/s, ×1.3 size)
but radar, scoring and shuttle are left out of the file, so they take the
competition values. Baked and checked: every system block except
`SpawnDebris` (`NumDebris`, `Spread_m`, `DebrisWeights`), every non-debris
entity and the SimEntity settings are byte-identical to
`COMPETITION_SIM.simconf`. Use this one when the point is transfer to the
real competition: the controller sees exactly the sensors, thrust cap and
scoring it will be judged with, only the traffic is worse.

```
python scripts/evaluate.py --config hard-debris --seeds 1-10 --retries 1
```

## Reproducing

```
python scripts/make_config.py --config hard --seed 3     # bakes the field (cached)
python run_sim.py --config hard --seed 3                 # one run
python scripts/evaluate.py --config hard --seeds 1-10 --retries 1
```
