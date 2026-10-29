"""
report_batch.py - one self-contained HTML report for an evaluate.py batch.

    python scripts/report_batch.py results/eval/hard-debris_newctl
    python scripts/report_batch.py results/eval/hard-debris_newctl --compare results/eval/hard-debris_oldctl

Reads <batch>/runs.jsonl and every run's trajectory.csv, collisions.json,
summary.json and controller log, and writes <batch>/report.html with:
  * headline tiles (scored / clean / hit / lost / stalled, mean & median score, time, fuel)
  * score per seed, outcome coloured
  * distance-to-ship, speed and fuel against sim time for every run, crashes marked
  * corridor view of every trajectory (along-track vs the two cross-track axes)
  * one row per collision: when, where, which rock (size, radar-visible or hidden,
    relative speed), what the controller was doing (mode, tracks, conflicts, its
    nearest predicted conflict) at that instant
  * the per-seed table
No dependencies beyond the standard library and numpy (inline SVG).
"""
from __future__ import annotations

import argparse
import csv
import html
import json
import math
import re
import statistics
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
RE_STATUS = re.compile(r"OnScreenLogMessage: t= *([\d.]+)s (\S+) +(.*?)rem= *([\d.-]+)m v= *([\d.-]+)m/s \|v_lat\|= *([\d.]+) tracks=(\d+)\(\+(\d+)cam\) tube=(\d+) conf=(\d+) dv= *([\d.]+)")
RE_THREAT = re.compile(r"nearest conflict: id(\d+) tau= *([\d.]+)s miss= *([\d.]+)m need= *([\d.]+)m diam= *([\d.]+)m")

OUTCOME_COLOR = {"clean": "var(--c-clean)", "hit": "var(--c-hit)", "loss": "var(--c-loss)", "stall": "var(--c-fail)", "fail": "var(--c-fail)"}
OUTCOME_LABEL = {"clean": "arrived, no damage", "hit": "arrived with damage", "loss": "lost (damage limit)",
                 "stall": "scored, but sim thrust-stall affected (not a controller result)", "fail": "no score (sim stall / hang)"}
STALL_FRACTION = 0.25   # share of 10 Hz samples with unchanged velocity while thrust is commanded


# ----------------------------------------------------------------------------- loading
def classify(r: dict) -> str:
    if not r.get("ok"):
        return "fail"
    if r.get("outcome") == "loss":
        return "loss"
    return "hit" if (r.get("damage_J") or 0) > 0 else "clean"


def load_traj(p: Path):
    if not p.exists():
        return None
    rows = list(csv.DictReader(p.open()))
    if len(rows) < 3:
        return None
    cols = {k: np.array([float(r[k]) for r in rows]) for k in ("t_sim", "x", "y", "z", "vx", "vy", "vz", "dist", "damage_J", "fuel_kg", "cmd_ax", "cmd_ay", "cmd_az")}
    cols["speed"] = np.sqrt(cols["vx"] ** 2 + cols["vy"] ** 2 + cols["vz"] ** 2)
    v = np.column_stack([cols["vx"], cols["vy"], cols["vz"]])
    cmd = np.sqrt(cols["cmd_ax"] ** 2 + cols["cmd_ay"] ** 2 + cols["cmd_az"] ** 2)
    frozen = (np.linalg.norm(np.diff(v, axis=0), axis=1) < 1e-9) & (cmd[1:] > 0.05)
    cols["frozen_fraction"] = float(frozen.mean()) if len(frozen) else 0.0
    return cols


def load_controller_log(out_dir: Path):
    logs = list(out_dir.glob("*_PySys_Controller.log"))
    if not logs:
        return [], []
    status, threats = [], []
    for line in logs[0].read_text(errors="ignore").splitlines():
        m = RE_STATUS.search(line)
        if m:
            status.append(dict(t=float(m.group(1)), mode=m.group(2), plan=m.group(3).strip(), rem=float(m.group(4)),
                               v_along=float(m.group(5)), v_lat=float(m.group(6)), tracks=int(m.group(7)),
                               cam=int(m.group(8)), tube=int(m.group(9)), conf=int(m.group(10)), dv=float(m.group(11)), threat=None))
            continue
        m = RE_THREAT.search(line)
        if m and status:
            status[-1]["threat"] = dict(id=int(m.group(1)), tau=float(m.group(2)), miss=float(m.group(3)),
                                        need=float(m.group(4)), diam=float(m.group(5)))
    return status, threats


def controller_state_at(status: list, t_ctrl: float):
    best = None
    for s in status:
        if s["t"] <= t_ctrl + 0.05:
            best = s
        else:
            break
    return best


def load_batch(batch_dir: Path) -> list[dict]:
    runs = []
    for line in (batch_dir / "runs.jsonl").read_text().splitlines():
        if not line.strip():
            continue
        r = json.loads(line)
        r["kind"] = classify(r)
        out = Path(r.get("out_dir", ""))
        r["traj"] = load_traj(out / "trajectory.csv") if out.exists() else None
        r["frozen_fraction"] = r["traj"]["frozen_fraction"] if r["traj"] else None
        if r["kind"] != "fail" and (r["frozen_fraction"] or 0) > STALL_FRACTION:
            r["kind"] = "stall"
        r["summary"] = json.loads((out / "summary.json").read_text()) if (out / "summary.json").exists() else {}
        r["collisions"] = json.loads((out / "collisions.json").read_text()) if (out / "collisions.json").exists() else []
        status, _ = load_controller_log(out) if out.exists() else ([], [])
        off = r["summary"].get("controller_t_offset") or 0.0
        for c in r["collisions"]:
            c["ctrl"] = controller_state_at(status, c["t"] + off)
        r["status"] = status
        runs.append(r)
    return runs


# ----------------------------------------------------------------------------- svg helpers
def nice_ticks(lo, hi, n=5):
    if hi <= lo:
        hi = lo + 1
    raw = (hi - lo) / n
    mag = 10 ** math.floor(math.log10(raw))
    step = min([1, 2, 2.5, 5, 10], key=lambda s: abs(s * mag - raw)) * mag
    start = math.floor(lo / step) * step
    return [start + i * step for i in range(int((hi - start) / step) + 2) if start + i * step <= hi + 1e-9]


class Plot:
    def __init__(self, w=900, h=320, left=60, right=20, top=28, bottom=44, xlabel="", ylabel="", title=""):
        self.w, self.h, self.l, self.r, self.t, self.b = w, h, left, right, top, bottom
        self.xlabel, self.ylabel, self.title = xlabel, ylabel, title
        self.parts = []
        self.xlo = self.xhi = self.ylo = self.yhi = None

    def scale(self, xlo, xhi, ylo, yhi):
        self.xlo, self.xhi, self.ylo, self.yhi = xlo, xhi, ylo, yhi

    def x(self, v):
        return self.l + (v - self.xlo) / (self.xhi - self.xlo) * (self.w - self.l - self.r)

    def y(self, v):
        return self.t + (self.yhi - v) / (self.yhi - self.ylo) * (self.h - self.t - self.b)

    def axes(self, xticks=None, yticks=None, xfmt="{:g}", yfmt="{:g}"):
        xt = xticks or nice_ticks(self.xlo, self.xhi)
        yt = yticks or nice_ticks(self.ylo, self.yhi)
        for v in yt:
            if self.ylo <= v <= self.yhi:
                self.parts.append(f'<line x1="{self.l}" x2="{self.w - self.r}" y1="{self.y(v):.1f}" y2="{self.y(v):.1f}" class="grid"/>'
                                  f'<text x="{self.l - 8}" y="{self.y(v) + 4:.1f}" text-anchor="end" class="tick">{yfmt.format(v)}</text>')
        for v in xt:
            if self.xlo <= v <= self.xhi:
                self.parts.append(f'<text x="{self.x(v):.1f}" y="{self.h - self.b + 18}" text-anchor="middle" class="tick">{xfmt.format(v)}</text>')
        self.parts.append(f'<line x1="{self.l}" x2="{self.w - self.r}" y1="{self.h - self.b}" y2="{self.h - self.b}" class="axis"/>')
        if self.xlabel:
            self.parts.append(f'<text x="{(self.l + self.w - self.r) / 2:.0f}" y="{self.h - 6}" text-anchor="middle" class="label">{html.escape(self.xlabel)}</text>')
        if self.ylabel:
            self.parts.append(f'<text transform="translate(14,{(self.t + self.h - self.b) / 2:.0f}) rotate(-90)" text-anchor="middle" class="label">{html.escape(self.ylabel)}</text>')
        if self.title:
            self.parts.append(f'<text x="{self.l}" y="{self.t - 10}" class="title">{html.escape(self.title)}</text>')

    def line(self, xs, ys, color, width=1.5, opacity=0.85, tooltip=""):
        pts = " ".join(f"{self.x(a):.1f},{self.y(b):.1f}" for a, b in zip(xs, ys) if self.xlo <= a <= self.xhi)
        if pts:
            self.parts.append(f'<polyline points="{pts}" fill="none" stroke="{color}" stroke-width="{width}" opacity="{opacity}" stroke-linejoin="round">'
                              f'<title>{html.escape(tooltip)}</title></polyline>')

    def hline(self, v, color="var(--ink-3)", dash="4 4", label=""):
        self.parts.append(f'<line x1="{self.l}" x2="{self.w - self.r}" y1="{self.y(v):.1f}" y2="{self.y(v):.1f}" stroke="{color}" stroke-dasharray="{dash}"/>')
        if label:
            self.parts.append(f'<text x="{self.w - self.r}" y="{self.y(v) - 4:.1f}" text-anchor="end" class="tick">{html.escape(label)}</text>')

    def dot(self, x, y, color, r=5, tooltip="", stroke="var(--surface)"):
        self.parts.append(f'<circle cx="{self.x(x):.1f}" cy="{self.y(y):.1f}" r="{r}" fill="{color}" stroke="{stroke}" stroke-width="1.5"><title>{html.escape(tooltip)}</title></circle>')

    def bar(self, x0, x1, y0, y1, color, tooltip=""):
        X0, X1, Y0, Y1 = self.x(x0), self.x(x1), self.y(max(y0, y1)), self.y(min(y0, y1))
        self.parts.append(f'<rect x="{X0:.1f}" y="{Y0:.1f}" width="{max(X1 - X0, 1):.1f}" height="{max(Y1 - Y0, 1):.1f}" fill="{color}" rx="2"><title>{html.escape(tooltip)}</title></rect>')

    def text(self, x, y, s, cls="tick", anchor="middle"):
        self.parts.append(f'<text x="{self.x(x):.1f}" y="{self.y(y):.1f}" text-anchor="{anchor}" class="{cls}">{html.escape(s)}</text>')

    def svg(self):
        return f'<svg viewBox="0 0 {self.w} {self.h}" class="chart" role="img">' + "".join(self.parts) + "</svg>"


# ----------------------------------------------------------------------------- charts
def chart_scores(runs, compare=None):
    n = len(runs)
    p = Plot(h=340, title="Score per seed", xlabel="field seed", ylabel="score")
    scores = [r["score"] for r in runs if r["kind"] != "fail"] + ([r["score"] for r in compare if r["kind"] != "fail"] if compare else []) or [0.0]
    lo, hi = min(min(scores), 0) - 20, max(max(scores), 60) + 20
    p.scale(0, n, lo, hi)
    p.axes(xticks=[i + 0.5 for i in range(n)], xfmt="")
    for i, r in enumerate(runs):
        p.text(i + 0.5, lo - (hi - lo) * 0.045, str(r["seed"]))
    p.hline(50, label="nominal run = 50")
    p.hline(0, color="var(--ink-2)", dash="")
    for i, r in enumerate(runs):
        if r["kind"] == "stall":
            p.bar(i + 0.15, i + 0.85, 0, r["score"], "url(#hatch)", f"seed {r['seed']}: {r['score']:.1f} but velocity frozen {r['frozen_fraction']:.0%} of the time under thrust (sim stall)")
            continue
        if r["kind"] == "fail":
            p.parts.append(f'<rect x="{p.x(i + 0.15):.1f}" y="{p.y(hi):.1f}" width="{p.x(i + 0.85) - p.x(i + 0.15):.1f}" height="{p.y(lo) - p.y(hi):.1f}" fill="url(#hatch)" opacity="0.5"><title>seed {r["seed"]}: {r.get("error", "no score")}</title></rect>')
            continue
        tip = f"seed {r['seed']}: {r['score']:.1f} ({OUTCOME_LABEL[r['kind']]}), {r['elapsed_s']} s, {r['fuel_kg']} kg, {r['damage_J']:.0f} J"
        if compare:
            cmp_r = next((c for c in compare if c["seed"] == r["seed"] and c["kind"] != "fail"), None)
            p.bar(i + 0.1, i + 0.48, 0, r["score"], OUTCOME_COLOR[r["kind"]], tip)
            if cmp_r:
                p.bar(i + 0.52, i + 0.9, 0, cmp_r["score"], "var(--c-compare)", f"reference seed {cmp_r['seed']}: {cmp_r['score']:.1f} ({OUTCOME_LABEL[cmp_r['kind']]})")
        else:
            p.bar(i + 0.15, i + 0.85, 0, r["score"], OUTCOME_COLOR[r["kind"]], tip)
    ok = [r["score"] for r in runs if r["kind"] not in ("fail", "stall")]
    if ok:
        p.hline(statistics.mean(ok), color="var(--ink-1)", dash="2 3", label=f"mean {statistics.mean(ok):.0f} (controller results)")
    return p.svg()


def chart_timeseries(runs, key, ylabel, title, ylim=None, mark_crashes=True):
    p = Plot(h=300, title=title, xlabel="sim time (s)", ylabel=ylabel)
    tmax = max((float(r["traj"]["t_sim"][-1]) for r in runs if r["traj"]), default=120)
    ymax = ylim or max((float(r["traj"][key].max()) for r in runs if r["traj"]), default=1)
    p.scale(0, min(tmax, 200), 0, ymax * 1.05)
    p.axes()
    for r in runs:
        tr = r["traj"]
        if tr is None:
            continue
        p.line(tr["t_sim"], tr[key], OUTCOME_COLOR[r["kind"]], width=1.2, opacity=0.7,
               tooltip=f"seed {r['seed']} ({OUTCOME_LABEL[r['kind']]})")
        if mark_crashes:
            for c in r["collisions"]:
                if c["t"] <= p.xhi:
                    yv = float(np.interp(c["t"], tr["t_sim"], tr[key]))
                    d = c.get("debris") or {}
                    p.dot(c["t"], yv, "var(--c-loss)", r=4,
                          tooltip=f"seed {r['seed']} crash t={c['t']:.1f}s +{c['damage_J']:.0f} J, {d.get('name', '?')} {d.get('diameter_m', 0):.1f} m")
    return p.svg()


def corridor_frames(runs):
    tr = next((r["traj"] for r in runs if r["traj"]), None)
    if tr is None:
        return None
    p0 = np.array([tr["x"][0], tr["y"][0], tr["z"][0]])
    goal = np.array([-850.0, 120.0, -9530.0])
    e1 = goal - p0
    L = np.linalg.norm(e1)
    e1 /= L
    helper = np.array([0, 0, 1.0]) if abs(e1[2]) < 0.9 else np.array([1.0, 0, 0])
    e2 = np.cross(e1, helper)
    e2 /= np.linalg.norm(e2)
    e3 = np.cross(e1, e2)
    return p0, e1, e2, e3, L


def chart_corridor(runs, axis_idx, label):
    fr = corridor_frames(runs)
    if fr is None:
        return ""
    p0, e1, e2, e3, L = fr
    eN = (e2, e3)[axis_idx]
    p = Plot(h=300, title=f"Corridor view: along-track vs cross-track {label}", xlabel="along-track from pad (m)", ylabel=f"cross-track {label} (m)")
    cross_max = 0.0
    projs = []
    for r in runs:
        tr = r["traj"]
        if tr is None:
            projs.append(None)
            continue
        P = np.column_stack([tr["x"], tr["y"], tr["z"]]) - p0
        a, c = P @ e1, P @ eN
        projs.append((a, c))
        cross_max = max(cross_max, float(np.abs(c).max()))
    cm = max(cross_max * 1.1, 60)
    p.scale(-50, L + 100, -cm, cm)
    p.axes()
    p.hline(0, color="var(--ink-3)", dash="2 4")
    p.dot(L, 0, "var(--ink-1)", r=6, tooltip="interplanetary ship")
    for r, pr in zip(runs, projs):
        if pr is None:
            continue
        p.line(pr[0], pr[1], OUTCOME_COLOR[r["kind"]], width=1.2, opacity=0.7, tooltip=f"seed {r['seed']} ({OUTCOME_LABEL[r['kind']]})")
        for c in r["collisions"]:
            sp = np.array(c["shuttle_pos"]) - p0
            d = c.get("debris") or {}
            p.dot(float(sp @ e1), float(sp @ eN), "var(--c-loss)", r=4 + min(c["damage_J"] / 150000, 6),
                  tooltip=f"seed {r['seed']} crash t={c['t']:.1f}s +{c['damage_J']:.0f} J with {d.get('name', '?')} ({d.get('diameter_m', 0):.1f} m, {'radar' if d.get('radar_visible') else 'hidden'})")
    return p.svg()


def chart_time_fuel(runs):
    p = Plot(h=300, title="Flight time vs fuel (arrived runs)", xlabel="time to ship (s)", ylabel="fuel used (kg)")
    ok = [r for r in runs if r["kind"] in ("clean", "hit")]
    if not ok:
        return "<p>No arrived controller runs yet.</p>"
    p.scale(min(r["elapsed_s"] for r in ok) - 5, max(r["elapsed_s"] for r in ok) + 5, 0, max(r["fuel_kg"] for r in ok) * 1.15)
    p.axes()
    p.hline(1.15, label="nominal fuel 1.15 kg")
    for r in ok:
        p.dot(r["elapsed_s"], r["fuel_kg"], OUTCOME_COLOR[r["kind"]], r=6,
              tooltip=f"seed {r['seed']}: {r['elapsed_s']} s, {r['fuel_kg']} kg, score {r['score']:.1f}, damage {r['damage_J']:.0f} J")
        p.text(r["elapsed_s"], r["fuel_kg"] + p.yhi * 0.03, str(r["seed"]))
    return p.svg()


# ----------------------------------------------------------------------------- tables
def fmt(v, nd=1):
    return "–" if v is None else (f"{v:.{nd}f}" if isinstance(v, (int, float)) else str(v))


def crash_rows(runs):
    rows = []
    for r in runs:
        for c in r["collisions"]:
            d = c.get("debris") or {}
            s = c.get("ctrl") or {}
            th = s.get("threat") or {}
            hist = [h for h in c.get("history_5s", []) if h["t"] < c["t"] - 0.3]
            pre = hist[-1] if hist else None          # ~0.5 s before the damage registered (the recorded state is post-bounce)
            vsh_pre = pre["speed"] if pre else c.get("shuttle_speed_m_s")
            vrel_pre = (float(np.linalg.norm(np.array(pre["vel"]) - np.array(d["debris_vel"]))) if pre and "vel" in pre and d.get("debris_vel")
                        else d.get("rel_speed_m_s"))
            rows.append(dict(
                seed=r["seed"], t=c["t"], progress=c["progress"] * 100, damage=c["damage_J"], hits=c["hits"],
                before=c.get("before_win", True), rock=d.get("name", "no rock within 10 m (ship/structure)"), diam=d.get("diameter_m"), mass=d.get("mass_kg"),
                visible=d.get("radar_visible"), vrel=vrel_pre, closing=d.get("closing_m_s"), vsh=vsh_pre,
                mode=s.get("mode"), tracks=s.get("tracks"), conf=s.get("conf"), tube=s.get("tube"), v_lat=s.get("v_lat"),
                th_tau=th.get("tau"), th_miss=th.get("miss"), th_need=th.get("need"), th_diam=th.get("diam"),
                run_id=r.get("run_id", "")))
    return rows


def crash_table(rows):
    if not rows:
        return "<p>No collisions in this batch.</p>"
    head = ("<tr><th>seed</th><th>t (s)</th><th>progress</th><th>damage (J)</th><th>rock</th><th>diam (m)</th><th>radar</th>"
            "<th>rel. speed (m/s)</th><th>shuttle speed before (m/s)</th><th>controller mode</th><th>tracks</th><th>conflicts</th>"
            "<th>controller's nearest predicted conflict</th></tr>")
    body = []
    for c in rows:
        th = (f"id{int(c['th_tau'] or 0) and ''}{''}τ={fmt(c['th_tau'])}s miss={fmt(c['th_miss'])}m need={fmt(c['th_need'])}m diam={fmt(c['th_diam'])}m"
              if c["th_tau"] is not None else ("none" if c["mode"] else "–"))
        after = "" if c["before"] else " <span class=muted>(after arrival)</span>"
        body.append(f"<tr><td>{c['seed']}</td><td>{fmt(c['t'])}</td><td>{fmt(c['progress'], 0)} %</td><td>{fmt(c['damage'], 0)}{after}</td>"
                    f"<td>{html.escape(str(c['rock']))}</td><td>{fmt(c['diam'])}</td><td>{'yes' if c['visible'] else ('<b>hidden</b>' if c['visible'] is not None else '–')}</td>"
                    f"<td>{fmt(c['vrel'])}</td><td>{fmt(c['vsh'])}</td><td>{html.escape(str(c['mode'] or '–'))}</td><td>{fmt(c['tracks'], 0)}</td><td>{fmt(c['conf'], 0)}</td><td>{th}</td></tr>")
    return "<table>" + head + "".join(body) + "</table>"


def seed_table(runs):
    head = "<tr><th>seed</th><th>outcome</th><th>score</th><th>time (s)</th><th>fuel (kg)</th><th>damage (J)</th><th>collisions</th><th>min dist to ship (m)</th><th>frozen under thrust</th><th>wall (s)</th><th>run</th></tr>"
    body = []
    for r in runs:
        body.append(f"<tr><td>{r['seed']}</td><td><span class=dot style=\"background:{OUTCOME_COLOR[r['kind']]}\"></span>{OUTCOME_LABEL[r['kind']]}</td>"
                    f"<td>{fmt(r.get('score'))}</td><td>{fmt(r.get('elapsed_s'))}</td><td>{fmt(r.get('fuel_kg'), 2)}</td><td>{fmt(r.get('damage_J'), 0)}</td>"
                    f"<td>{len(r['collisions'])}</td><td>{fmt(r['summary'].get('min_dist'))}</td><td>{fmt((r.get('frozen_fraction') or 0) * 100, 0)} %</td><td>{fmt(r.get('wall_total_s'), 0)}</td><td class=mono>{html.escape(r.get('run_id', ''))}</td></tr>")
    return "<table>" + head + "".join(body) + "</table>"


def crash_summary(runs, rows):
    before = [c for c in rows if c["before"]]
    if not before:
        return "<p>No collisions before arrival.</p>"
    before = [c for c in before if c["visible"] is not None]   # rock impacts only; ship/structure contacts are listed in the table
    if not before:
        return "<p>No rock collisions before arrival.</p>"
    hidden = sum(1 for c in before if c["visible"] is False)
    by_mode = {}
    for c in before:
        by_mode[c["mode"] or "?"] = by_mode.get(c["mode"] or "?", 0) + 1
    early = sum(1 for c in before if c["progress"] < 33)
    mid = sum(1 for c in before if 33 <= c["progress"] < 66)
    late = len(before) - early - mid
    predicted = sum(1 for c in before if c["conf"])
    big = sum(1 for c in before if (c["diam"] or 0) >= 20)
    vrel = [c["vrel"] for c in before if c["vrel"] is not None]
    items = [
        f"{len(before)} collision(s) before arrival in {sum(1 for r in runs if any(c['before_win'] for c in r['collisions']))} of {len(runs)} runs",
        f"{hidden} with rocks the radar never reports (index ≥ radar count), {len(before) - hidden} with radar-visible rocks",
        f"{predicted} happened while the controller had ≥ 1 active conflict (it saw something coming), {len(before) - predicted} with no conflict flagged",
        f"by controller mode at impact: " + ", ".join(f"{k} {v}" for k, v in sorted(by_mode.items(), key=lambda kv: -kv[1])),
        f"by progress along the corridor: {early} early (&lt;33 %), {mid} mid, {late} late (&gt;66 %)",
        f"{big} with rocks ≥ 20 m diameter; relative speed at impact {min(vrel):.1f}–{max(vrel):.1f} m/s (median {statistics.median(vrel):.1f})" if vrel else "",
    ]
    return "<ul>" + "".join(f"<li>{s}</li>" for s in items if s) + "</ul>"


# ----------------------------------------------------------------------------- page
CSS = """
:root{--surface:#fcfcfb;--ink-1:#0b0b0b;--ink-2:#52514e;--ink-3:#b9b8b2;--grid:#ebeae6;
--c-clean:#2a78d6;--c-hit:#eda100;--c-loss:#eb6834;--c-fail:#9a9a94;--c-compare:#b9b8b2;--tile:#f3f2ee}
@media (prefers-color-scheme: dark){:root:not([data-theme="light"]){--surface:#1a1a19;--ink-1:#f0efec;--ink-2:#c3c2b7;--ink-3:#5b5a55;--grid:#2a2a28;
--c-clean:#3987e5;--c-hit:#c98500;--c-loss:#d95926;--c-fail:#777670;--c-compare:#5b5a55;--tile:#242422}}
:root[data-theme="dark"]{--surface:#1a1a19;--ink-1:#f0efec;--ink-2:#c3c2b7;--ink-3:#5b5a55;--grid:#2a2a28;
--c-clean:#3987e5;--c-hit:#c98500;--c-loss:#d95926;--c-fail:#777670;--c-compare:#5b5a55;--tile:#242422}
body{background:var(--surface);color:var(--ink-1);font:14px/1.45 Segoe UI,system-ui,sans-serif;margin:0;padding:24px 16px;max-width:1000px;margin:auto}
h1{font-size:22px;margin:0 0 4px}h2{font-size:17px;margin:28px 0 8px}p.sub{color:var(--ink-2);margin:0 0 16px}
.tiles{display:grid;grid-template-columns:repeat(auto-fit,minmax(130px,1fr));gap:10px;margin:14px 0}
.tile{background:var(--tile);border-radius:8px;padding:10px 12px}.tile b{display:block;font-size:22px}.tile span{color:var(--ink-2);font-size:12px}
.chart{width:100%;height:auto;display:block;margin:6px 0 14px}.grid{stroke:var(--grid)}.axis{stroke:var(--ink-3)}
.tick{fill:var(--ink-2);font-size:11px}.label{fill:var(--ink-2);font-size:12px}.title{fill:var(--ink-1);font-size:13px;font-weight:600}
.legend{display:flex;flex-wrap:wrap;gap:14px;color:var(--ink-2);font-size:12px;margin:0 0 6px}.dot{display:inline-block;width:10px;height:10px;border-radius:50%;margin-right:6px;vertical-align:middle}
table{border-collapse:collapse;width:100%;font-size:12px;margin:8px 0 16px;display:block;overflow-x:auto}th,td{border-bottom:1px solid var(--grid);padding:5px 8px;text-align:right;white-space:nowrap}
th{color:var(--ink-2);font-weight:600}td:first-child,th:first-child,td:nth-child(2),th:nth-child(2){text-align:left}.mono{font-family:Consolas,monospace;font-size:11px}.muted{color:var(--ink-2)}
"""


def legend():
    return '<div class="legend">' + "".join(f'<span><span class="dot" style="background:{OUTCOME_COLOR[k]}"></span>{OUTCOME_LABEL[k]}</span>' for k in ("clean", "hit", "loss", "stall", "fail")) + '<span><span class="dot" style="background:var(--c-loss);width:8px;height:8px"></span>collision marker</span></div>'


def tiles(runs):
    ok = [r for r in runs if r["kind"] not in ("fail", "stall")]
    kinds = {k: sum(1 for r in runs if r["kind"] == k) for k in OUTCOME_COLOR}
    sc = [r["score"] for r in ok]
    arrived = [r for r in ok if r["kind"] != "loss"]
    t = [
        (f"{len(ok)} / {len(runs)}", "controller results"),
        (f"{kinds['clean']}", "arrived clean"), (f"{kinds['hit']}", "arrived with damage"), (f"{kinds['loss']}", "lost"), (f"{kinds['fail'] + kinds['stall']}", "sim stall (excluded)"),
        (f"{statistics.mean(sc):.0f}" if sc else "–", "mean score"), (f"{statistics.median(sc):.0f}" if sc else "–", "median score"),
        (f"{statistics.pstdev(sc):.0f}" if len(sc) > 1 else "–", "score std"),
        (f"{statistics.mean(r['elapsed_s'] for r in arrived):.0f} s" if arrived else "–", "time to ship (arrived)"),
        (f"{statistics.mean(r['fuel_kg'] for r in ok):.2f} kg" if ok else "–", "fuel (scored runs)"),
        (f"{sum(len([c for c in r['collisions'] if c.get('before_win', True)]) for r in runs)}", "collisions before arrival"),
        (f"{sum(r.get('wall_total_s') or 0 for r in runs) / 60:.0f} min", "wall time"),
    ]
    return '<div class="tiles">' + "".join(f'<div class="tile"><b>{a}</b><span>{b}</span></div>' for a, b in t) + "</div>"


def build(batch_dir: Path, compare_dir: Path | None) -> Path:
    runs = load_batch(batch_dir)
    cmp_runs = load_batch(compare_dir) if compare_dir else None
    meta = runs[0] if runs else {}
    rows = crash_rows(runs)
    cmp_block = ""
    if cmp_runs:
        cs = [r["score"] for r in cmp_runs if r["kind"] != "fail"]
        cmp_block = (f'<p class="sub">Grey bars: reference batch <code>{html.escape(compare_dir.name)}</code> '
                     f'(mean {statistics.mean(cs):.0f}, median {statistics.median(cs):.0f}, {sum(1 for r in cmp_runs if r["kind"] == "loss")} lost, '
                     f'{sum(1 for r in cmp_runs if r["kind"] == "fail")} no score)</p>' if cs else "")
    doc = f"""<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Batch report {html.escape(batch_dir.name)}</title><style>{CSS}</style></head><body>
<svg width="0" height="0" style="position:absolute"><defs><pattern id="hatch" width="6" height="6" patternUnits="userSpaceOnUse" patternTransform="rotate(45)"><line x1="0" y1="0" x2="0" y2="6" stroke="var(--c-fail)" stroke-width="2"/></pattern></defs></svg>
<h1>Controller evaluation: {html.escape(batch_dir.name)}</h1>
<p class="sub">config <b>{html.escape(str(meta.get('config')))}</b>, {len(runs)} field seeds, time scale {meta.get('timescale')}, mode {html.escape(str(meta.get('mode')))}. Score = (120/t − 2·damage/threshold − max(fuel/nominal − 1, 0) + 0.5) × 100; nominal run = 50.</p>
{tiles(runs)}
<h2>Score per seed</h2>{legend()}{cmp_block}{chart_scores(runs, cmp_runs)}
<h2>How the crashes happened</h2>{crash_summary(runs, rows)}{crash_table(rows)}
<p class="sub">"radar" = the rock is among the first N the radar reports (the controller could have tracked it); "hidden" = the radar never reports it. "controller's nearest predicted conflict" is the last threat line the controller logged before the impact (τ = time to closest approach, miss = predicted miss distance, need = required clearance).</p>
<h2>Distance to the ship</h2>{chart_timeseries(runs, "dist", "distance to ship (m)", "Distance to ship vs sim time (dots = collisions)")}
<h2>Speed</h2>{chart_timeseries(runs, "speed", "speed (m/s)", "Shuttle speed vs sim time")}
<h2>Fuel</h2>{chart_timeseries(runs, "fuel_kg", "fuel used (kg)", "Fuel used vs sim time", mark_crashes=False)}
<h2>Flight time vs fuel</h2>{chart_time_fuel(runs)}
<h2>Where in the corridor</h2>{chart_corridor(runs, 0, "axis 1")}{chart_corridor(runs, 1, "axis 2")}
<p class="sub">Along-track = distance from the launch pad toward the ship along the straight line; cross-track = sideways offset from that line on two perpendicular axes. The black dot is the ship.</p>
<h2>All runs</h2>{seed_table(runs)}
<p class="sub">Raw data: <code>{html.escape(str(batch_dir / 'runs.jsonl'))}</code>; per run: trajectory.csv (10 Hz), collisions.json, summary.json, controller log.</p>
</body></html>"""
    out = batch_dir / "report.html"
    out.write_text(doc, encoding="utf-8")
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("batch", help="results/eval/<batch> directory")
    ap.add_argument("--compare", help="another batch directory to show as grey reference bars")
    args = ap.parse_args()
    out = build(Path(args.batch), Path(args.compare) if args.compare else None)
    print(out)


if __name__ == "__main__":
    main()
