"""
run_sim.py - run the competition sim end to end from one command, no GUI clicking.

    python run_sim.py                                      # default config, seed 100, 1x, GUI
    python run_sim.py --config hard --seed 3 --timescale 2
    python run_sim.py --mode offscreen                     # Unreal client without a window
    python run_sim.py --no-harness                         # the untouched competition sim, nothing added

What happens:
  1. the scenario simconf is resolved (seed 100 + default config = the organisers'
     file as-is; anything else is baked by scripts/make_config.py into
     Assets/Repos/STU4-Eval/Sims, cached);
  2. a tiny wrapper simconf is written that IncludeSims the scenario and adds the
     Harness system (harness/Harness.py: timescale, 10 Hz trajectory CSV, stop
     after the score).  --no-harness launches the scenario file directly;
  3. SpaceCRAFT.exe --sim=<wrapper> -firstRole (or run_Server.exe -Main_Headless
     for --mode server) is started, the new run-log folder is found, the score
     line is awaited, the sim is stopped and every sim process is killed;
  4. results/runs/<run_id>/ gets result.json, trajectory.csv, summary.json and
     copies of the main + controller logs.  The result is printed as one JSON
     line.  Exit code 0 = score obtained, 2 = no score (timeout/crash), 3 = setup.

Only one sim can run at a time on this machine (ports); the script refuses to
start if sim processes are already alive unless --kill-existing is given.
"""
from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from harness import simlib  # noqa: E402


def run(args) -> dict:
    t_start = time.time()
    cfg = simlib.load_config(args.config)
    run_id = f"{simlib.now_id()}_{cfg['name']}_s{args.seed}_ts{args.timescale:g}_{args.mode}" + (f"_{args.tag}" if args.tag else "")
    out_dir = (Path(args.out) / run_id).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    result = {"run_id": run_id, "config": cfg["name"], "seed": args.seed, "timescale": args.timescale,
              "mode": args.mode, "harness": not args.no_harness, "out_dir": str(out_dir), "ok": False}

    stale = simlib.check_controller_installed()
    if stale:
        if args.install_controller:
            simlib.install_controller()
            print(f"  installed {', '.join(stale)} into {simlib.SOLUTION_REPO}")
        elif not args.allow_stale:
            raise SystemExit(f"setup: {', '.join(stale)} differ between this repo and {simlib.SOLUTION_REPO} "
                             f"(the sim executes the latter). Re-run with --install-controller or --allow-stale.")
        else:
            result["controller_stale"] = stale

    if args.mode != "gui" and not args.unvalidated:
        raise SystemExit(f"setup: --mode {args.mode} is not validated against the GUI sim (docs/sim-notes.md §8-9: "
                         f"headless parity FAILED the per-seed bar). Pass --unvalidated to run it anyway, e.g. for parity_check.")
    alive = simlib.sim_processes()
    if alive:
        if args.kill_existing:
            simlib.kill_sim_processes()
            time.sleep(2.0)
        else:
            raise SystemExit(f"setup: sim processes already running {alive}; use --kill-existing")

    scenario_abs, scenario_local = simlib.bake_scenario(cfg, args.seed)
    result["scenario_simconf"] = str(scenario_abs)
    if args.no_harness:
        if abs(args.timescale - 1.0) > 1e-9:
            raise SystemExit("setup: --timescale needs the harness (it is what sets the clock); drop --no-harness")
        sim_local = scenario_local
    else:
        _, sim_local = simlib.write_wrapper(scenario_abs, scenario_local, run_id, out_dir, args.timescale, cfg,
                                            max_sim_time=args.max_sim_time, sample_hz=args.sample_hz,
                                            stop_after=args.stop_after)
    result["launched_simconf"] = sim_local

    if args.dry_run:
        exe, extra, cwd, _ = simlib.MODES[args.mode]
        print(f"would run: {exe} --sim={sim_local} {' '.join(extra)} -firstRole  (cwd {cwd})")
        if not args.no_harness:
            (simlib.EVAL_REPO / "Sims" / f"RUN_{run_id}.simconf").unlink(missing_ok=True)
        shutil.rmtree(out_dir, ignore_errors=True)
        result["dry_run"] = True
        return result

    if args.cooldown > 0:
        # let the previous client finish tearing down (GPU/driver, sockets) before the next one loads;
        # back-to-back launches showed a thrust-stall race in the client (docs/sim-notes.md §6)
        time.sleep(args.cooldown)
    before = simlib.log_folders()
    t_launch = time.time()
    proc = simlib.launch(args.mode, sim_local)
    print(f"  launched {args.mode}: --sim={sim_local}  (pid {proc.pid})")

    folder = simlib.wait_for(lambda: next(iter(simlib.log_folders() - before), None), 60.0, 1.0)
    if folder is None:
        simlib.kill_sim_processes(since=t_launch)
        result["error"] = "no run-log folder appeared within 60 s"
        return result
    result["log_dir"] = str(folder)
    print(f"  run log: {folder}")

    started = simlib.wait_for(lambda: simlib.log_has(folder, "SimState: Runtime"), 120.0, 1.0)
    if not started:
        simlib.kill_sim_processes(since=t_launch)
        result["error"] = "sim never reached Runtime (lobby/login?)"
        return result
    t_runtime = time.time()

    def finished():
        sc = simlib.parse_score(folder)
        if sc and all(sc.get(k) is not None for k in ("elapsed_s", "fuel_kg")):
            return sc
        if proc.poll() is not None and sc:
            return sc
        if proc.poll() is not None and not simlib.sim_processes(since=t_launch):
            return {"outcome": "crash", "score": None}
        if (out_dir / "summary.json").exists():
            try:
                sm = json.loads((out_dir / "summary.json").read_text())
                if sm.get("end_kind") in ("stall", "timeout"):
                    return {"outcome": sm["end_kind"], "score": None, "harness_end": sm}
            except Exception:
                pass
        return None

    score = simlib.wait_for(finished, args.timeout, 2.0)
    result["wall_to_score_s"] = round(time.time() - t_runtime, 1)
    if score is None or score.get("score") is None:
        if score and score.get("outcome") == "stall":
            result["error"] = f"sim thrust stall (harness aborted at t={score['harness_end'].get('t_end', 0):.0f}s)"
        elif score and score.get("outcome") == "timeout":
            result["error"] = "harness MaxSimTime reached without a score"
        else:
            result["error"] = "no score line within timeout" if score is None else "sim exited without a score"
    else:
        result.update(score)
        result["ok"] = True
    # let the harness write its summary and stop the sim, then make sure nothing is left
    if not args.no_harness:
        simlib.wait_for(lambda: (out_dir / "summary.json").exists(), 20.0 if result["ok"] else 2.0, 0.5)
    if not args.keep_open:
        simlib.wait_for(lambda: proc.poll() is not None, 8.0, 0.5)
        simlib.kill_sim_processes(since=t_launch, quiet=True)
    # collect
    for p in folder.glob("*.log"):
        if "_PySys_" not in p.name or "Controller" in p.name or "Harness" in p.name:
            shutil.copy2(p, out_dir / p.name)
    if not args.no_harness and not args.keep_wrapper:
        # the wrapper is reproducible (scenario + Harness block); leaving dozens of 0.9 MB simconfs
        # in STU4-Eval/Sims slows every later client start-up (the client scans the asset repos)
        try:
            (simlib.EVAL_REPO / "Sims" / f"RUN_{run_id}.simconf").unlink()
        except OSError:
            pass
    if (out_dir / "summary.json").exists():
        result["harness"] = json.loads((out_dir / "summary.json").read_text())
    if (out_dir / "trajectory.csv").exists():
        result["trajectory_csv"] = str(out_dir / "trajectory.csv")
    result["wall_total_s"] = round(time.time() - t_start, 1)
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default="default", help="configs/<name>.yaml or a path")
    ap.add_argument("--seed", type=int, default=100, help="debris-field seed (100 = the competition field)")
    ap.add_argument("--timescale", type=float, default=1.0)
    ap.add_argument("--mode", choices=list(simlib.MODES), default="gui")
    ap.add_argument("--no-harness", action="store_true", help="launch the scenario file itself, no observer system")
    ap.add_argument("--tag", default="", help="suffix for the run id")
    ap.add_argument("--out", default=str(simlib.REPO / "results" / "runs"))
    ap.add_argument("--timeout", type=float, default=600.0, help="wall seconds to wait for the score after Runtime")
    ap.add_argument("--max-sim-time", type=float, default=400.0, help="harness stops the sim after this many sim seconds")
    ap.add_argument("--sample-hz", type=float, default=10.0)
    ap.add_argument("--stop-after", type=float, default=3.0, help="sim seconds the harness waits after the end condition")
    ap.add_argument("--install-controller", action="store_true", help="copy Controller.py and its helper modules (every top-level .py here except the framework's) into the sim's STU4-Solution first")
    ap.add_argument("--allow-stale", action="store_true")
    ap.add_argument("--kill-existing", action="store_true")
    ap.add_argument("--unvalidated", action="store_true", help="allow a non-GUI mode although it failed the parity bar")
    ap.add_argument("--keep-open", action="store_true", help="do not kill the sim after the score")
    ap.add_argument("--keep-wrapper", action="store_true", help="keep the generated RUN_<id>.simconf in STU4-Eval/Sims")
    ap.add_argument("--cooldown", type=float, default=20.0, help="seconds to wait before launching the client")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()
    try:
        result = run(args)
    except SystemExit as ex:
        print(str(ex), file=sys.stderr)
        sys.exit(3)
    if result.get("dry_run"):
        sys.exit(0)
    (Path(result["out_dir"]) / "result.json").write_text(json.dumps(result, indent=1))
    if result.get("ok"):
        print(f"RESULT {result['outcome']} score={result['score']:.1f} time={result['elapsed_s']}s fuel={result['fuel_kg']}kg "
              f"damage={result['damage_J']}J dist={result['distance_m']}m wall={result['wall_to_score_s']}s "
              f"timescale={result.get('harness_timescale') or result.get('initial_timescale')} -> {result['out_dir']}")
    else:
        print(f"FAILED {result.get('error')} -> {result['out_dir']}")
    print(json.dumps(result))
    sys.exit(0 if result.get("ok") else 2)


if __name__ == "__main__":
    main()
