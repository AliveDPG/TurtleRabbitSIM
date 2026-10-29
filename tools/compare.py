"""
compare.py - A/B/C test of controller variants in the real simulator, with a chart.

Variants (name -> where its Controller.py / nav.py / vision.py come from):
    baseline    variants/baseline/Controller.py          one burn straight at the ship, coast, no avoidance
    current     git ref `main`                           the code in this repo
    fuel-aware  git ref `keep-28815b2`                   fuel-budget planner (cruise forced to the same speed)
    lag-fix     git ref `wip-lag-compensation`           fuel-aware + radar-lag-compensated tracker

For every run the variant's files are copied into the folder the sim actually executes, the
run is driven hands-off by tools/autorun.py, and the result is appended to results/compare.csv.
Variants are interleaved (round-robin) so none of them gets a "lucky" stretch of the session.
At the end (or with --chart-only) results/compare.html is written: scores per run, means, and
time / fuel / damage per variant.

    python tools/compare.py --runs 5
    python tools/compare.py --chart-only
"""
from __future__ import annotations

import argparse
import csv
import html
import os
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "tools"))
import autorun  # noqa: E402

SIM_DIR = Path(os.environ.get("LOCALAPPDATA", "")) / "SimDynamX" / "STPro" / "Assets" / "Repos" / "STU4-Solution"
OUT_CSV = REPO / "results" / "compare.csv"
OUT_HTML = REPO / "results" / "compare.html"
FILES = ["Controller.py", "nav.py", "vision.py"]
CRUISE = 15.0

VARIANTS = {
    "baseline": {"dir": REPO / "variants" / "baseline"},
    "current": {"git": "main"},
    "fuel-aware": {"git": "keep-28815b2"},
    "lag-fix": {"git": "wip-lag-compensation"},   # fuel-aware + radar-lag-compensated tracker
    "dyn-track": {"git": "37a92b1"},              # current + tracker retuned for moving debris
    "short-hz": {"git": "58e16b0"},               # dyn-track + 20 s horizon, tau^2 clearance
    "v7": {"git": "bc5ed3b"},                      # short-hz + 15 m uncertainty cap + goal-biased trim direction
    "v7-15": {"git": "bc5ed3b", "cruise": 15.0},
    "v7-18": {"git": "bc5ed3b", "cruise": 18.0},
    "v7-12": {"git": "bc5ed3b", "cruise": 12.0},
    "v7-9": {"git": "bc5ed3b", "cruise": 9.0},
}
COLORS = {"baseline": "#8c8c8c", "current": "#3b82f6", "fuel-aware": "#f59e0b", "lag-fix": "#10b981", "dyn-track": "#8b5cf6", "short-hz": "#ef4444", "v7": "#0ea5e9", "v7-15": "#0ea5e9", "v7-18": "#dc2626", "v7-12": "#16a34a", "v7-9": "#7c3aed"}


# ---------------------------------------------------------------- variant materialisation
def materialize(name: str) -> Path:
    spec = VARIANTS[name]
    dst = REPO / "results" / "variants" / name
    dst.mkdir(parents=True, exist_ok=True)
    for f in FILES:
        if "dir" in spec:
            src = spec["dir"] / f
            if src.exists():
                shutil.copy2(src, dst / f)
            elif (dst / f).exists():
                (dst / f).unlink()
        else:
            out = subprocess.run(["git", "show", f"{spec['git']}:{f}"], cwd=REPO, capture_output=True, text=True)
            if out.returncode == 0:
                (dst / f).write_text(out.stdout, encoding="utf-8")
            elif (dst / f).exists():
                (dst / f).unlink()
    ctrl = dst / "Controller.py"
    txt = ctrl.read_text(encoding="utf-8")
    # same cruise speed for every variant so only the guidance logic differs
    cruise = spec.get("cruise", CRUISE)   # one cruise speed per variant
    txt = re.sub(r"(\n\s*v_cruise=)[\d.]+", rf"\g<1>{cruise}", txt)
    txt = re.sub(r"(\n\s*v_arrive=)[\d.]+", rf"\g<1>{cruise}", txt)
    txt = re.sub(r"(\nV_CRUISE = )[\d.]+", rf"\g<1>{cruise}", txt)
    ctrl.write_text(txt, encoding="utf-8")
    return dst


def install(name: str) -> None:
    src = materialize(name)
    for f in FILES:
        if (src / f).exists():
            shutil.copy2(src / f, SIM_DIR / f)
        elif (SIM_DIR / f).exists():
            (SIM_DIR / f).unlink()           # baseline has no nav.py / vision.py
    for junk in ("__pycache__", "run_overrides.json"):
        p = SIM_DIR / junk
        if p.is_dir():
            shutil.rmtree(p, ignore_errors=True)
        elif p.exists():
            p.unlink()


def restore_current() -> None:
    install("current")


# ---------------------------------------------------------------- results
def load_rows():
    if not OUT_CSV.exists():
        return []
    with OUT_CSV.open() as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        for k in ("score", "elapsed_s", "distance_m", "damage_J", "fuel_kg"):
            try:
                r[k] = float(r[k])
            except (ValueError, KeyError):
                r[k] = None
    return rows


def append_row(variant, r):
    OUT_CSV.parent.mkdir(parents=True, exist_ok=True)
    new = not OUT_CSV.exists()
    with OUT_CSV.open("a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["variant", "stamp", "outcome", "score", "elapsed_s", "distance_m", "damage_J", "fuel_kg"])
        if new:
            w.writeheader()
        w.writerow({"variant": variant, **{k: r[k] for k in ["stamp", "outcome", "score", "elapsed_s", "distance_m", "damage_J", "fuel_kg"]}})


def summary(rows):
    out = {}
    for name in VARIANTS:
        rs = [r for r in rows if r["variant"] == name and r["score"] is not None]
        if not rs:
            continue
        sc = [r["score"] for r in rs]
        out[name] = dict(n=len(rs), mean=sum(sc) / len(sc), best=max(sc), worst=min(sc),
                         time=sum(r["elapsed_s"] or 0 for r in rs) / len(rs),
                         fuel=sum(r["fuel_kg"] or 0 for r in rs) / len(rs),
                         damage=sum(r["damage_J"] or 0 for r in rs) / len(rs),
                         wins=sum(1 for r in rs if r["outcome"] == "win"), runs=rs)
    return out


# ---------------------------------------------------------------- chart (self-contained SVG in HTML)
def chart(rows) -> str:
    S = summary(rows)
    names = [n for n in VARIANTS if n in S]
    if not names:
        return "<p>no results yet</p>"
    W, H, L, R, T, B = 900, 420, 70, 30, 40, 70
    allscores = [r["score"] for n in names for r in S[n]["runs"]]
    lo, hi = min(min(allscores), 0.0), max(max(allscores), 50.0)
    lo, hi = lo - 10, hi + 15
    def y(val):
        return T + (hi - val) / (hi - lo) * (H - T - B)
    gw = (W - L - R) / len(names)
    parts = [f'<svg viewBox="0 0 {W} {H}" width="{W}" height="{H}" font-family="Segoe UI, sans-serif" font-size="13">',
             f'<rect width="{W}" height="{H}" fill="white"/>']
    # gridlines
    step = 50 if hi - lo > 200 else 25
    g = int(lo // step) * step
    while g <= hi:
        parts.append(f'<line x1="{L}" x2="{W-R}" y1="{y(g):.1f}" y2="{y(g):.1f}" stroke="#e5e7eb"/>'
                     f'<text x="{L-8}" y="{y(g)+4:.1f}" text-anchor="end" fill="#6b7280">{g:g}</text>')
        g += step
    # nominal line
    parts.append(f'<line x1="{L}" x2="{W-R}" y1="{y(50):.1f}" y2="{y(50):.1f}" stroke="#9ca3af" stroke-dasharray="4 4"/>'
                 f'<text x="{W-R}" y="{y(50)-4:.1f}" text-anchor="end" fill="#6b7280">nominal run = 50</text>')
    for i, n in enumerate(names):
        s = S[n]
        cx = L + gw * (i + 0.5)
        bw = gw * 0.42
        col = COLORS.get(n, "#999")
        # mean bar
        top, base = y(max(s["mean"], 0)), y(min(s["mean"], 0))
        parts.append(f'<rect x="{cx-bw/2:.1f}" y="{top:.1f}" width="{bw:.1f}" height="{max(base-top,1):.1f}" fill="{col}" opacity="0.35"/>')
        parts.append(f'<line x1="{cx-bw/2:.1f}" x2="{cx+bw/2:.1f}" y1="{y(s["mean"]):.1f}" y2="{y(s["mean"]):.1f}" stroke="{col}" stroke-width="3"/>')
        # individual runs
        k = len(s["runs"])
        for j, r in enumerate(s["runs"]):
            px = cx - bw * 0.35 + (bw * 0.7) * (j / max(k - 1, 1) if k > 1 else 0.5)
            fill = col if r["outcome"] == "win" else "#dc2626"
            parts.append(f'<circle cx="{px:.1f}" cy="{y(r["score"]):.1f}" r="5" fill="{fill}" stroke="white" stroke-width="1.5">'
                         f'<title>{html.escape(r["stamp"])}: {r["score"]:.1f} ({r["elapsed_s"]:.1f}s, {r["fuel_kg"]:.2f}kg, {r["damage_J"]:.0f}J)</title></circle>')
        parts.append(f'<text x="{cx:.1f}" y="{H-B+22}" text-anchor="middle" font-weight="600">{html.escape(n)}</text>')
        parts.append(f'<text x="{cx:.1f}" y="{H-B+40}" text-anchor="middle" fill="#374151">mean {s["mean"]:.1f}  ·  {s["time"]:.0f} s  ·  {s["fuel"]:.2f} kg</text>')
        parts.append(f'<text x="{cx:.1f}" y="{H-B+56}" text-anchor="middle" fill="#6b7280">{s["wins"]}/{s["n"]} reached the ship · dmg {s["damage"]/1000:.0f} kJ</text>')
    parts.append(f'<text x="{L}" y="{T-14}" font-weight="600" font-size="15">Score per run (dots) and mean (bar) — COMPETITION_SIM</text>')
    parts.append('</svg>')
    return "\n".join(parts)


def write_html(rows):
    S = summary(rows)
    tbl = ["<table><tr><th>variant</th><th>runs</th><th>mean</th><th>best</th><th>worst</th><th>time s</th><th>fuel kg</th><th>damage J</th><th>reached</th></tr>"]
    for n, s in S.items():
        tbl.append(f"<tr><td>{n}</td><td>{s['n']}</td><td><b>{s['mean']:.1f}</b></td><td>{s['best']:.1f}</td><td>{s['worst']:.1f}</td>"
                   f"<td>{s['time']:.1f}</td><td>{s['fuel']:.2f}</td><td>{s['damage']:.0f}</td><td>{s['wins']}/{s['n']}</td></tr>")
    tbl.append("</table>")
    runs = ["<table><tr><th>variant</th><th>run</th><th>outcome</th><th>score</th><th>time s</th><th>fuel kg</th><th>damage J</th><th>dist m</th></tr>"]
    for r in rows:
        runs.append(f"<tr><td>{r['variant']}</td><td>{r['stamp']}</td><td>{r['outcome']}</td><td>{r['score']:.1f}</td>"
                    f"<td>{r['elapsed_s']:.1f}</td><td>{r['fuel_kg']:.2f}</td><td>{r['damage_J']:.0f}</td><td>{r['distance_m']:.1f}</td></tr>")
    runs.append("</table>")
    doc = f"""<!doctype html><html><head><meta charset="utf-8"><title>Controller comparison</title>
<style>body{{font-family:Segoe UI,sans-serif;margin:24px;color:#111}}table{{border-collapse:collapse;margin:16px 0}}
td,th{{border:1px solid #d1d5db;padding:4px 10px;text-align:right}}th{{background:#f3f4f6}}td:first-child,th:first-child{{text-align:left}}</style></head>
<body><h2>Escape from Phobos — controller comparison</h2>
<p>Score = (120 / time − 2·damage / 209 kJ − max(fuel / 1.15 kg − 1, 0) + 0.5) × 100. Red dots are runs that did not reach the ship.</p>
{chart(rows)}
<h3>Summary</h3>{''.join(tbl)}<h3>All runs</h3>{''.join(runs)}
<p style="color:#6b7280">generated {time.strftime('%Y-%m-%d %H:%M')}</p></body></html>"""
    OUT_HTML.parent.mkdir(parents=True, exist_ok=True)
    OUT_HTML.write_text(doc, encoding="utf-8")


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--runs", type=int, default=5, help="runs per variant")
    ap.add_argument("--variants", default=",".join(VARIANTS), help="comma-separated subset")
    ap.add_argument("--chart-only", action="store_true")
    ap.add_argument("--fresh", action="store_true", help="start a new compare.csv")
    args = ap.parse_args()

    if args.chart_only:
        write_html(load_rows())
        print(f"chart -> {OUT_HTML}")
        return
    if args.fresh and OUT_CSV.exists():
        OUT_CSV.unlink()

    names = [n.strip() for n in args.variants.split(",") if n.strip() in VARIANTS]
    h = autorun.find_window()
    if not h:
        sys.exit("SpaceCRAFT window not found")
    try:
        for i in range(args.runs):
            for name in names:
                done = sum(1 for r in load_rows() if r["variant"] == name)
                if done >= args.runs:
                    continue
                print(f"\n=== {name}: run {done + 1}/{args.runs} ===")
                install(name)
                autorun.focus(h)
                r = autorun.one_run(h)
                append_row(name, r)
                write_html(load_rows())
    finally:
        restore_current()
        write_html(load_rows())
    S = summary(load_rows())
    print("\n" + f"{'variant':12} {'n':>2} {'mean':>7} {'best':>7} {'worst':>7} {'time':>6} {'fuel':>5} {'dmg kJ':>6} reached")
    for n, s in S.items():
        print(f"{n:12} {s['n']:>2} {s['mean']:>7.1f} {s['best']:>7.1f} {s['worst']:>7.1f} {s['time']:>6.1f} {s['fuel']:>5.2f} {s['damage']/1000:>6.0f} {s['wins']}/{s['n']}")
    print(f"\nchart -> {OUT_HTML}")


if __name__ == "__main__":
    main()
