"""
parity_check.py - does mode/timescale B reproduce reference A?  Per-seed score
delta, trajectory deviation and overall pass rate.

    python scripts/parity_check.py --seeds 1-20 --b-mode offscreen            # headless candidate vs GUI
    python scripts/parity_check.py --seeds 1-10 --b-timescale 2               # 2x vs 1x, both GUI
    python scripts/parity_check.py --seeds 1-10 --noise-floor                 # GUI 1x vs GUI 1x (same seed twice)
    python scripts/parity_check.py --from results/eval/A/runs.jsonl results/eval/B/runs.jsonl   # compare two evaluate.py batches

A and B are run back to back per seed (one sim at a time).  Trajectories come
from the harness CSV (10 Hz sim time): both runs are resampled on the common
sim-time grid and the shuttle position difference is reported as max / rms
over the overlapping interval.  A seed passes when |score_B - score_A| <= tol
(--score-tol, default 5 points) and both runs produced a score.  The batch
passes when the pass rate >= --pass-rate (default 0.9).
Results: results/parity/<batch>/{runs_A.jsonl, runs_B.jsonl, parity.json}.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
from harness import simlib  # noqa: E402
from scripts.evaluate import parse_seeds, run_one  # noqa: E402


def load_traj(path: str | None):
    if not path or not Path(path).exists():
        return None
    rows = list(csv.DictReader(open(path)))
    if len(rows) < 5:
        return None
    t = np.array([float(r["t_sim"]) for r in rows])
    p = np.array([[float(r["x"]), float(r["y"]), float(r["z"])] for r in rows])
    return t, p


def traj_deviation(a, b) -> dict:
    """max / rms position difference on the common sim-time interval, plus end-time difference."""
    if a is None or b is None:
        return {"max_m": None, "rms_m": None, "overlap_s": None}
    ta, pa = a
    tb, pb = b
    lo, hi = max(ta[0], tb[0]), min(ta[-1], tb[-1])
    if hi - lo < 1.0:
        return {"max_m": None, "rms_m": None, "overlap_s": round(hi - lo, 2)}
    grid = np.arange(lo, hi, 0.1)
    qa = np.column_stack([np.interp(grid, ta, pa[:, i]) for i in range(3)])
    qb = np.column_stack([np.interp(grid, tb, pb[:, i]) for i in range(3)])
    d = np.linalg.norm(qa - qb, axis=1)
    return {"max_m": round(float(d.max()), 2), "rms_m": round(float(np.sqrt((d ** 2).mean())), 2),
            "overlap_s": round(hi - lo, 1), "t_end_a": round(float(ta[-1]), 1), "t_end_b": round(float(tb[-1]), 1)}


def compare(rows_a: list[dict], rows_b: list[dict], score_tol: float, traj_tol: float) -> list[dict]:
    by_seed_b = {}
    for r in rows_b:
        by_seed_b.setdefault(r.get("seed"), []).append(r)
    out = []
    for ra in rows_a:
        cands = by_seed_b.get(ra.get("seed"), [])
        rb = cands.pop(0) if cands else None
        row = {"seed": ra.get("seed"), "a_ok": bool(ra.get("ok")), "b_ok": bool(rb and rb.get("ok")),
               "a_outcome": ra.get("outcome"), "b_outcome": rb.get("outcome") if rb else None,
               "a_score": ra.get("score"), "b_score": rb.get("score") if rb else None,
               "a_time": ra.get("elapsed_s"), "b_time": rb.get("elapsed_s") if rb else None,
               "a_fuel": ra.get("fuel_kg"), "b_fuel": rb.get("fuel_kg") if rb else None,
               "a_damage": ra.get("damage_J"), "b_damage": rb.get("damage_J") if rb else None}
        if row["a_ok"] and row["b_ok"]:
            row["score_delta"] = round(row["b_score"] - row["a_score"], 2)
            row.update({f"traj_{k}": v for k, v in traj_deviation(load_traj(ra.get("trajectory_csv")),
                                                                  load_traj(rb.get("trajectory_csv"))).items()})
            row["pass_score"] = abs(row["score_delta"]) <= score_tol
            row["pass_traj"] = row.get("traj_max_m") is None or row["traj_max_m"] <= traj_tol
            row["pass"] = row["pass_score"] and row["pass_traj"]
        else:
            row["score_delta"] = None
            row["pass"] = False
        out.append(row)
    return out


def report(rows: list[dict], score_tol: float, traj_tol: float, pass_rate: float, label_a: str, label_b: str) -> dict:
    print(f"\n{'seed':>5} {'A':>8} {'B':>8} {'delta':>7} {'trajMax':>8} {'trajRMS':>8} {'tA':>6} {'tB':>6} {'fuelA':>6} {'fuelB':>6}  pass")
    for r in rows:
        f = lambda v, w=8, p=1: (f"{v:>{w}.{p}f}" if isinstance(v, (int, float)) and v is not None else f"{'-':>{w}}")
        print(f"{r['seed']:>5} {f(r['a_score'])} {f(r['b_score'])} {f(r['score_delta'], 7)} {f(r.get('traj_max_m'))} "
              f"{f(r.get('traj_rms_m'))} {f(r['a_time'], 6)} {f(r['b_time'], 6)} {f(r['a_fuel'], 6, 2)} {f(r['b_fuel'], 6, 2)}  "
              f"{'PASS' if r['pass'] else 'FAIL'}{'' if r['a_ok'] and r['b_ok'] else ' (missing score)'}")
    n = len(rows)
    passed = sum(1 for r in rows if r["pass"])
    deltas = [r["score_delta"] for r in rows if r["score_delta"] is not None]
    summary = {"n": n, "passed": passed, "pass_rate": passed / n if n else 0.0, "score_tol": score_tol, "traj_tol_m": traj_tol,
               "required_pass_rate": pass_rate, "verdict": "PASS" if n and passed / n >= pass_rate else "FAIL",
               "delta_mean": float(np.mean(deltas)) if deltas else None, "delta_abs_mean": float(np.mean(np.abs(deltas))) if deltas else None,
               "delta_max_abs": float(np.max(np.abs(deltas))) if deltas else None,
               "a_mean": float(np.mean([r["a_score"] for r in rows if r["a_ok"]])) if any(r["a_ok"] for r in rows) else None,
               "b_mean": float(np.mean([r["b_score"] for r in rows if r["b_ok"]])) if any(r["b_ok"] for r in rows) else None,
               "a_fuel_mean": float(np.mean([r["a_fuel"] for r in rows if r["a_ok"] and r["a_fuel"] is not None])) if any(r["a_ok"] for r in rows) else None,
               "b_fuel_mean": float(np.mean([r["b_fuel"] for r in rows if r["b_ok"] and r["b_fuel"] is not None])) if any(r["b_ok"] for r in rows) else None}
    print(f"\nA = {label_a}   B = {label_b}")
    print(f"pass {passed}/{n} = {summary['pass_rate']:.0%} (need {pass_rate:.0%}, |delta| <= {score_tol} pts, traj max <= {traj_tol} m)  -> {summary['verdict']}")
    if deltas:
        print(f"score delta B-A: mean {summary['delta_mean']:+.1f}, mean |delta| {summary['delta_abs_mean']:.1f}, max |delta| {summary['delta_max_abs']:.1f};  "
              f"A mean {summary['a_mean']:.1f}  B mean {summary['b_mean']:.1f};  fuel A {summary['a_fuel_mean']:.2f} kg  B {summary['b_fuel_mean']:.2f} kg")
    for tol in (2, 5, 10, 20, 40):
        k = sum(1 for d in deltas if abs(d) <= tol)
        print(f"  |delta| <= {tol:>2} pts: {k}/{n}")
    return summary


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default="default")
    ap.add_argument("--seeds", default="1-20")
    ap.add_argument("--a-mode", default="gui")
    ap.add_argument("--a-timescale", type=float, default=1.0)
    ap.add_argument("--b-mode", default="gui")
    ap.add_argument("--b-timescale", type=float, default=1.0)
    ap.add_argument("--noise-floor", action="store_true", help="B = A (same mode and timescale, run twice)")
    ap.add_argument("--score-tol", type=float, default=5.0)
    ap.add_argument("--traj-tol", type=float, default=10.0, help="max shuttle position deviation in m")
    ap.add_argument("--pass-rate", type=float, default=0.9)
    ap.add_argument("--from", dest="from_files", nargs=2, metavar=("A.jsonl", "B.jsonl"), help="compare existing batches")
    ap.add_argument("--batch", default="")
    ap.add_argument("--run-args", default="")
    args = ap.parse_args()

    if args.noise_floor:
        args.b_mode, args.b_timescale = args.a_mode, args.a_timescale
    label_a = f"{args.a_mode} ts{args.a_timescale:g}"
    label_b = f"{args.b_mode} ts{args.b_timescale:g}"

    if args.from_files:
        rows_a = [json.loads(l) for l in Path(args.from_files[0]).read_text().splitlines() if l.strip()]
        rows_b = [json.loads(l) for l in Path(args.from_files[1]).read_text().splitlines() if l.strip()]
        rows = compare(rows_a, rows_b, args.score_tol, args.traj_tol)
        report(rows, args.score_tol, args.traj_tol, args.pass_rate, args.from_files[0], args.from_files[1])
        return

    batch = args.batch or f"{simlib.now_id()}_{args.config}_{args.a_mode}{args.a_timescale:g}_vs_{args.b_mode}{args.b_timescale:g}"
    bdir = REPO / "results" / "parity" / batch
    bdir.mkdir(parents=True, exist_ok=True)
    extra = (args.run_args.split() if args.run_args else []) + ["--unvalidated"]   # parity is where non-GUI modes get tested
    seeds = parse_seeds(args.seeds)
    rows_a, rows_b = [], []
    t0 = time.time()
    print(f"parity batch {batch}: config={args.config} seeds={seeds}  A={label_a}  B={label_b}")
    for seed in seeds:
        for side, mode, ts, rows, fn in (("A", args.a_mode, args.a_timescale, rows_a, "runs_A.jsonl"),
                                         ("B", args.b_mode, args.b_timescale, rows_b, "runs_B.jsonl")):
            print(f"\n--- seed {seed} {side} ({mode} ts{ts:g}) ---")
            r = run_one(args.config, seed, ts, mode, f"parity{side}", extra)
            rows.append(r)
            with (bdir / fn).open("a") as f:
                f.write(json.dumps(r) + "\n")
            print(f"  {r.get('outcome', 'FAIL')} score={r.get('score')} time={r.get('elapsed_s')} fuel={r.get('fuel_kg')} "
                  f"damage={r.get('damage_J')} ts={r.get('harness_timescale')} wall={r.get('wall_total_s')}s"
                  + (f"  error={r.get('error')}" if not r.get("ok") else ""))
        rows = compare(rows_a, rows_b, args.score_tol, args.traj_tol)
        (bdir / "parity.json").write_text(json.dumps({"rows": rows}, indent=1))
    rows = compare(rows_a, rows_b, args.score_tol, args.traj_tol)
    summary = report(rows, args.score_tol, args.traj_tol, args.pass_rate, label_a, label_b)
    summary["wall_s"] = round(time.time() - t0)
    (bdir / "parity.json").write_text(json.dumps({"summary": summary, "rows": rows}, indent=1))
    print(f"-> {bdir / 'parity.json'}")


if __name__ == "__main__":
    main()
