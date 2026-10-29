"""
evaluate.py - run the controller across N seeds on one config and summarise.

    python scripts/evaluate.py --config default --seeds 1-10
    python scripts/evaluate.py --config hard --seeds 1,2,3 --timescale 2 --mode offscreen
    python scripts/evaluate.py --config default --seeds 100 --repeat 5          # same field 5x (noise floor)
    python scripts/evaluate.py --summary results/eval/<batch>/runs.jsonl        # re-print a table

Each run is `python run_sim.py ...` in its own process (one sim at a time).
Per-run JSON lines are appended to results/eval/<batch_id>/runs.jsonl as they
finish, so a batch can be interrupted and re-summarised.  A run that produces
no score counts as a failure and is excluded from the score statistics but
listed in the table.
"""
from __future__ import annotations

import argparse
import json
import statistics
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
from harness import simlib  # noqa: E402


def parse_seeds(spec: str) -> list[int]:
    seeds = []
    for part in spec.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            a, b = part.split("-")
            seeds.extend(range(int(a), int(b) + 1))
        else:
            seeds.append(int(part))
    return seeds


def run_one(config: str, seed: int, timescale: float, mode: str, tag: str, extra: list[str]) -> dict:
    cmd = [sys.executable, str(REPO / "run_sim.py"), "--config", config, "--seed", str(seed),
           "--timescale", str(timescale), "--mode", mode, "--kill-existing"] + (["--tag", tag] if tag else []) + extra
    out = subprocess.run(cmd, capture_output=True, text=True)
    last = [l for l in out.stdout.splitlines() if l.startswith("{")]
    if last:
        res = json.loads(last[-1])
    else:
        res = {"ok": False, "error": f"run_sim exit {out.returncode}: {out.stderr.strip()[-300:]}", "seed": seed}
    res["exit_code"] = out.returncode
    return res


def summarise(rows: list[dict]) -> dict:
    ok = [r for r in rows if r.get("ok")]
    scores = [r["score"] for r in ok]
    wins = [r for r in ok if r.get("outcome") == "win"]
    def st(vals):
        if not vals:
            return (float("nan"),) * 4
        return (statistics.mean(vals), statistics.pstdev(vals) if len(vals) > 1 else 0.0, min(vals), max(vals))
    s = dict(n=len(rows), scored=len(ok), failures=len(rows) - len(ok), wins=len(wins), losses=len(ok) - len(wins))
    s["score_mean"], s["score_std"], s["score_min"], s["score_max"] = st(scores)
    s["time_mean"], s["time_std"], _, _ = st([r["elapsed_s"] for r in wins if r.get("elapsed_s") is not None])
    s["fuel_mean"], s["fuel_std"], _, _ = st([r["fuel_kg"] for r in ok if r.get("fuel_kg") is not None])
    s["damage_mean"], _, _, s["damage_max"] = st([r["damage_J"] for r in ok if r.get("damage_J") is not None])
    s["wall_mean"], _, _, _ = st([r["wall_total_s"] for r in ok if r.get("wall_total_s") is not None])
    s["wall_total_s"] = sum(r.get("wall_total_s", 0) or 0 for r in rows)
    return s


def print_table(rows: list[dict], title: str = "") -> None:
    if title:
        print(f"\n{title}")
    print(f"{'seed':>5} {'ts':>4} {'outcome':8} {'score':>8} {'time s':>7} {'fuel kg':>7} {'dmg J':>9} {'wall s':>7}  run")
    for r in rows:
        if r.get("ok"):
            print(f"{r['seed']:>5} {r['timescale']:>4g} {r['outcome']:8} {r['score']:>8.1f} {r['elapsed_s']:>7.1f} "
                  f"{r['fuel_kg']:>7.2f} {r['damage_J']:>9.0f} {r.get('wall_total_s', 0):>7.0f}  {r.get('run_id', '')}")
        else:
            print(f"{r.get('seed', '?'):>5} {r.get('timescale', ''):>4} {'FAIL':8} {'-':>8} {'-':>7} {'-':>7} {'-':>9} "
                  f"{r.get('wall_total_s', 0):>7.0f}  {r.get('error', '')[:70]}")
    s = summarise(rows)
    print(f"\nscore  mean {s['score_mean']:.1f}  std {s['score_std']:.1f}  min {s['score_min']:.1f}  max {s['score_max']:.1f}   "
          f"({s['scored']}/{s['n']} scored, {s['wins']} wins, {s['losses']} losses, {s['failures']} failures)")
    print(f"time   mean {s['time_mean']:.1f} s (wins)   fuel mean {s['fuel_mean']:.2f} kg   damage mean {s['damage_mean']:.0f} J max {s['damage_max']:.0f} J")
    print(f"wall   {s['wall_total_s']:.0f} s total, {s['wall_mean']:.0f} s per scored run")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default="default")
    ap.add_argument("--seeds", default="1-5", help="e.g. 1-20 or 3,7,100")
    ap.add_argument("--repeat", type=int, default=1, help="runs per seed")
    ap.add_argument("--timescale", type=float, default=1.0)
    ap.add_argument("--mode", choices=list(simlib.MODES), default="gui")
    ap.add_argument("--tag", default="")
    ap.add_argument("--batch", default="", help="batch id (default: timestamp_config_ts_mode)")
    ap.add_argument("--summary", help="only print the table for an existing runs.jsonl")
    ap.add_argument("--run-args", default="", help="extra args passed to run_sim.py, e.g. \"--install-controller\"")
    ap.add_argument("--retries", type=int, default=0, help="re-run a seed this many times when it produces no score (sim stall/crash)")
    args = ap.parse_args()

    if args.summary:
        rows = [json.loads(l) for l in Path(args.summary).read_text().splitlines() if l.strip()]
        print_table(rows, f"{args.summary}")
        return

    batch = args.batch or f"{simlib.now_id()}_{args.config}_ts{args.timescale:g}_{args.mode}" + (f"_{args.tag}" if args.tag else "")
    bdir = REPO / "results" / "eval" / batch
    bdir.mkdir(parents=True, exist_ok=True)
    jsonl = bdir / "runs.jsonl"
    seeds = parse_seeds(args.seeds)
    extra = args.run_args.split() if args.run_args else []
    rows = []
    t0 = time.time()
    print(f"batch {batch}: config={args.config} seeds={seeds} x{args.repeat} timescale={args.timescale} mode={args.mode}")
    for rep in range(args.repeat):
        for seed in seeds:
            print(f"\n--- seed {seed} (repeat {rep + 1}/{args.repeat}) ---")
            r = run_one(args.config, seed, args.timescale, args.mode, args.tag, extra)
            for attempt in range(args.retries):
                if r.get("ok"):
                    break
                print(f"  no score ({r.get('error')}); retry {attempt + 1}/{args.retries}")
                r = run_one(args.config, seed, args.timescale, args.mode, args.tag, extra)
                r["retries"] = attempt + 1
            rows.append(r)
            with jsonl.open("a") as f:
                f.write(json.dumps(r) + "\n")
            if r.get("ok"):
                print(f"  {r['outcome']} score={r['score']:.1f} time={r['elapsed_s']} fuel={r['fuel_kg']} damage={r['damage_J']} wall={r.get('wall_total_s')}s")
            else:
                print(f"  FAILED: {r.get('error')}")
    print_table(rows, f"batch {batch}  ({time.time() - t0:.0f} s wall)")
    (bdir / "summary.json").write_text(json.dumps(summarise(rows), indent=1))
    print(f"-> {jsonl}")


if __name__ == "__main__":
    main()
