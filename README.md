# TurtleRabbitSIM

Test rig for the **Escape from Phobos** competition (Space Teams PRO, STU4).
It wraps the organisers' `COMPETITION_SIM` so you can run your shuttle
controller from one command, run it on **harder debris fields** than the
competition one, run batches over many fields, and get a report on how every
run ended (which rock, radar-visible or hidden, what the controller was doing).

Nothing in the organisers' sim is modified. Scenario files are generated into a
separate folder (`Assets\Repos\STU4-Eval`), and the organisers' Radar, Scoring,
Actuator and physics are used as-is.

| Links | |
|---|---|
| Rules & Info | https://github.com/SimDynamX/SpaceTeamsPRO/discussions/218 |
| Starter code + wiki (API, scoring, vehicle) | https://github.com/SimDynamX/STU4-Solution |
| Forum | https://github.com/SimDynamX/SpaceTeamsPRO/discussions/categories/2026-escape-from-phobos |

---

## 1. One-time setup (per machine)

1. **Windows + Space Teams PRO installed** (tested on 0.55.0), trial activated, you can log in.
2. **Clone the two competition repos inside Space Teams PRO** (the app's own
   repo-clone feature, see the "Asset Git Repos" page of the Space Teams PRO wiki):
   - `https://github.com/SimDynamX/STU4-Sim.git`
   - `https://github.com/SimDynamX/STU4-Solution.git`

   They end up in `%LOCALAPPDATA%\SimDynamX\STPro\Assets\Repos\STU4-Sim` and
   `...\Repos\STU4-Solution`. Start `COMPETITION_SIM` once from the GUI
   (Sim, tick *User-Made Sims*, pick `Repos/STU4-Sim/Sims/COMPETITION_SIM.simconf`)
   to confirm the sim itself works.
3. **Clone this repo** anywhere outside the sim's folders, e.g. `Documents\SpaceTeams`:

   ```bash
   git clone https://github.com/AliveDPG/TurtleRabbitSIM.git
   ```

4. **Python packages** for the launcher scripts (your normal Python 3.9 or newer,
   *not* the one bundled with Space Teams PRO):

   ```bash
   pip install -r requirements.txt
   ```

5. **Check** (resolves all sim paths, launches nothing):

   ```bash
   python run_sim.py --dry-run
   ```

   It should print `would run: C:\Users\Public\SimDynamX\STPRO\Apps\Main\SpaceCRAFT\SpaceCRAFT.exe --sim=...`.
   If Space Teams PRO is installed somewhere else, set the `STP_PLATFORM`
   environment variable to its `Apps\Main` folder.

---

## 2. Running your shuttle code

**Your shuttle code is `Controller.py` in the root of this repo**, plus any helper
modules you put next to it (right now `nav.py` and `vision.py`). Edit only below
the `DON'T CHANGE ANY OF THE ABOVE` marker in `Controller.py`; the boilerplate
at the top is what joins the sim.

The sim does **not** execute this folder. It executes
`%LOCALAPPDATA%\SimDynamX\STPro\Assets\Repos\STU4-Solution\Controller.py`.
`run_sim.py --install-controller` copies `Controller.py` and every other
top-level `.py` here (except the framework's own `run_sim.py`, `sim_offline.py`,
`BasicController.py`, `UserAPI.py`) into that folder before launching. Without
the flag the launcher **refuses to start if the copies differ**, so you can never
accidentally test stale code.

### Step by step

1. Put your controller in place:
   - either edit `Controller.py` / `nav.py` / `vision.py` here, or
   - drop in your own `Controller.py` and helper modules. If you do not use
     `nav.py` / `vision.py`, delete them so they are not copied.
2. One run on the **competition field** (GUI, real time, about 2 minutes):

   ```bash
   python run_sim.py --install-controller
   ```

   The sim window opens, flies, closes itself, and you get one line like
   `RESULT win score=136.2 time=102.0s fuel=1.51kg damage=0J ... -> results/runs/<run id>`.
3. One run on the **hard debris field** (same sensors and scoring as the competition, worse rocks):

   ```bash
   python run_sim.py --config hard-debris --seed 1 --install-controller
   ```

4. A **batch** over many fields (this is the number that means something; one run is mostly noise):

   ```bash
   python scripts/evaluate.py --config hard-debris --seeds 1-10 --retries 1 --run-args "--install-controller"
   ```

   Prints a per-seed table plus mean / std / wins / losses, and writes `results/eval/<batch>/runs.jsonl`.
5. The **report** for a batch (self-contained HTML: score per seed, every collision with
   the rock that caused it, distance/speed/fuel curves, corridor view):

   ```bash
   python scripts/report_batch.py results/eval/<batch>
   ```

Every run leaves `results/runs/<run id>/` with `result.json`, `trajectory.csv`
(10 Hz), `summary.json`, `collisions.json` and copies of the sim and controller
logs. `results/` is git-ignored.

### Good to know

- **One sim at a time**, and it takes the screen while it flies (fullscreen client).
  `run_sim.py` refuses to start over a running sim; add `--kill-existing` to replace it.
- **Keep the time scale at 1x** for anything you compare or submit. `--timescale 2`
  works mechanically but changes fuel and control-loop behaviour (details in `docs/sim-notes.md`, section 7).
- **The GUI path still works.** After `--install-controller` the sim's copy is your
  code, so starting `COMPETITION_SIM` from the Space Teams PRO menu runs it too.
  `python run_sim.py --no-harness` launches the organisers' sim file exactly as is.
- **Sim thrust stall.** In some runs the client freezes the shuttle's velocity a few
  seconds in while thrust is still commanded. That is the sim, not your code. The
  harness aborts such runs after 4 s, `evaluate.py --retries 1` re-runs them, and the
  report marks stall-affected runs and keeps them out of the statistics.
- **Offline rehearsal without the sim**: `python sim_offline.py 8` is a point-mass
  model with the competition numbers, for quick logic checks of `nav.py` only.
- **Submission** is still what the organisers ask for (screen recording, log
  folders from `%LOCALAPPDATA%\SimDynamX\STPro\Logs`, your source). This rig
  does not change that.

---

## 3. The hard sim

Scenarios are YAML files in `configs/`. Any seed other than 100, or any changed
value, makes `scripts/make_config.py` bake a new debris field with the organisers'
own spawn maths (verified bit-exact against the competition field) into
`Assets\Repos\STU4-Eval\Sims`. The competition field is `--config default --seed 100`.

| Config | What changes | Use it for |
|---|---|---|
| `default` | nothing; seed 100 is the competition field, other seeds are fields the organisers' code would have produced | the real thing, and noise-floor checks |
| `hard-debris` | 700 rocks (was 500) in a tighter box, 3x more 17 m tanks, rocks 30 % bigger, twice as fast. Radar, scoring and thrust are the competition values | **transfer to the competition**: same sensors and scoring, worse traffic |
| `hard` | `hard-debris` plus noisier and slower radar reporting fewer rocks, lower damage limit, less fuel/time allowance, smaller keep-out, 20 % less thrust | stress test; margin against anything the organisers could change |

Reasoning, the parameter table and a baseline: `docs/hard-config.md`.

---

## 4. Command cheat sheet

| Command | What it does |
|---|---|
| `python run_sim.py --install-controller` | one run, competition field, GUI, 1x |
| `python run_sim.py --config hard-debris --seed 3 --install-controller` | one run on a hard field |
| `python run_sim.py --dry-run` | resolve paths and config, launch nothing |
| `python run_sim.py --no-harness` | the organisers' sim exactly as is (no observer, no auto-stop) |
| `python scripts/evaluate.py --config hard-debris --seeds 1-10 --retries 1 --run-args "--install-controller"` | batch over seeds, summary table, `results/eval/<batch>/runs.jsonl` |
| `python scripts/evaluate.py --summary results/eval/<batch>/runs.jsonl` | re-print a batch table |
| `python scripts/report_batch.py results/eval/<batch> [--compare results/eval/<other>]` | HTML report for a batch, optionally against another |
| `python scripts/parity_check.py --seeds 1-10 --noise-floor` | same field twice, how much the sim alone varies |
| `python scripts/make_config.py --config hard --seed 3` | bake a field without running it (`--check` on `default --seed 100` verifies the competition field) |
| `python tools/autorun.py --runs 3` | older GUI-clicking path, still works |

---

## 5. Working as a team

- Commit your controller on a branch (`git checkout -b <name>/<idea>`), open a pull
  request into `main` when a batch on `hard-debris` beats what `main` does.
- Quote batches, not single runs: the run-to-run spread of the sim itself is
  hundreds of points, so compare means and loss counts over 10 to 20 seeds
  (`docs/sim-notes.md`, section 9.3, has the measured noise floor).
- `results/` is ignored. If a report is worth keeping, paste the numbers into the PR
  or commit the `report.html` deliberately.

---

## 6. What is in here

| Path | What |
|---|---|
| `Controller.py`, `nav.py`, `vision.py` | the current shuttle code (design notes: `docs/controller.md`) |
| `run_sim.py` | launcher: bakes the scenario, adds the harness, starts the sim, waits for the score, collects logs |
| `harness/Harness.py` | passive observer Python system injected next to the sim: trajectory CSV, collision forensics, end detection, stops the sim |
| `harness/simlib.py` | paths, config baking, process control, log parsing |
| `scripts/evaluate.py` | batch runner |
| `scripts/report_batch.py` | HTML report for a batch |
| `scripts/parity_check.py` | A vs B comparisons (modes, time scales, noise floor) |
| `scripts/make_config.py` | bakes `configs/*.yaml` + seed into a `.simconf` using the organisers' spawn maths |
| `configs/` | `default.yaml`, `hard-debris.yaml`, `hard.yaml` |
| `docs/sim-notes.md` | how the sim runs, launch modes, time-scale and headless findings, measured noise floor |
| `docs/hard-config.md` | the hard scenarios and a baseline |
| `sim_offline.py` | point-mass rehearsal model, no sim needed |
| `tools/` | older GUI automation (`autorun.py`), A/B runner (`compare.py`), radar-dump analysis |
| `variants/` | reference controllers (`baseline`: straight line, no avoidance; `diag`: radar-dump instrumentation) |
| `BasicController.py`, `UserAPI.py` | the organisers' starter files, for reference |
