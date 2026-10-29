"""
simlib.py - shared helpers for run_sim.py, scripts/evaluate.py and scripts/parity_check.py.

Paths, config baking, sim launch/kill, run-log discovery and score parsing.
Pure stdlib + numpy + PyYAML (both already installed).
"""
from __future__ import annotations

import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
PLATFORM = Path(os.environ.get("STP_PLATFORM", r"C:\Users\Public\SimDynamX\STPRO\Apps\Main"))
SPACECRAFT_DIR = PLATFORM / "SpaceCRAFT"
SPACECRAFT_EXE = SPACECRAFT_DIR / "SpaceCRAFT.exe"
SPACECRAFT_SHIPPING = SPACECRAFT_DIR / "SpaceCRAFT" / "Binaries" / "Win64" / "SpaceCRAFT-Win64-Shipping.exe"
SERVER_DIR = PLATFORM / "Compute_Server"
SERVER_EXE = SERVER_DIR / "run_Server.exe"
LOCALAPP = Path(os.environ.get("LOCALAPPDATA", ""))
STPRO = LOCALAPP / "SimDynamX" / "STPro"
ASSETS = STPRO / "Assets"
LOGS = STPRO / "Logs"
SIM_REPO = ASSETS / "Repos" / "STU4-Sim"                 # organisers' sim (never written)
SOLUTION_REPO = ASSETS / "Repos" / "STU4-Solution"       # the controller copy the sim executes
EVAL_REPO = ASSETS / "Repos" / "STU4-Eval"               # everything this framework generates
COMPETITION_SIMCONF = SIM_REPO / "Sims" / "COMPETITION_SIM.simconf"
COMPETITION_SIMCONF_LOCAL = "Local/Repos/STU4-Sim/Sims/COMPETITION_SIM.simconf"
# top-level .py files that belong to this framework / the organisers' starter code, never copied into the sim
FRAMEWORK_FILES = {"run_sim.py", "sim_offline.py", "BasicController.py", "UserAPI.py"}


def controller_files() -> list[str]:
    """The shuttle code: Controller.py plus every other top-level .py in this repo (its helper modules)."""
    files = ["Controller.py"] if (REPO / "Controller.py").exists() else []
    files += sorted(p.name for p in REPO.glob("*.py") if p.name != "Controller.py" and p.name not in FRAMEWORK_FILES)
    return files


CONTROLLER_FILES = ("Controller.py", "nav.py", "vision.py")   # kept for older callers; the sim copy uses controller_files()

RE_SCORE = re.compile(r"(Successfully escaped Phobos with a score of|Crashed into some debris and finished with a score of)\s*(-?[\d.]+)")
RE_NUM = {k: re.compile(p) for k, p in {
    "elapsed_s": r"Sim elapsed time\s*=\s*([\d.]+)", "distance_m": r"Distance to ship\s*=\s*([\d.]+)",
    "damage_J": r"Total damage taken\s*=\s*([\d.]+)", "fuel_kg": r"Total fuel used\s*=\s*([\d.]+)"}.items()}
RE_TIMESCALE = re.compile(r"Initial Timescale:\s*([\d.]+)")
RE_HARNESS_TS = re.compile(r"Harness: timescale requested ([\d.]+), clock reports ([\d.]+)")

MODES = {
    # mode -> (executable, extra args, cwd, needs_unreal)
    "gui": (SPACECRAFT_EXE, [], SPACECRAFT_DIR, True),
    "offscreen": (SPACECRAFT_EXE, ["-RenderOffScreen"], SPACECRAFT_DIR, True),
    "nullrhi": (SPACECRAFT_EXE, ["-nullrhi"], SPACECRAFT_DIR, True),
    "server": (SERVER_EXE, ["-Main_Headless"], SERVER_DIR, False),
}


# ----------------------------------------------------------------------------- configs
def load_config(name_or_path: str) -> dict:
    """configs/<name>.yaml (or a path).  Returns the dict with 'name' filled in."""
    import yaml
    p = Path(name_or_path)
    if not p.exists():
        p = REPO / "configs" / f"{name_or_path}.yaml"
    if not p.exists():
        raise FileNotFoundError(f"config not found: {name_or_path} (looked for {p})")
    cfg = yaml.safe_load(p.read_text(encoding="utf-8")) or {}
    cfg.setdefault("name", p.stem)
    cfg["_path"] = str(p)
    return cfg


def config_fingerprint(cfg: dict, seed: int) -> str:
    body = {k: v for k, v in cfg.items() if not k.startswith("_")}
    return hashlib.sha1(json.dumps(body, sort_keys=True).encode() + f":{seed}".encode()).hexdigest()[:10]


def ensure_eval_repo() -> None:
    (EVAL_REPO / "Sims").mkdir(parents=True, exist_ok=True)
    (EVAL_REPO / "Systems").mkdir(parents=True, exist_ok=True)
    src = REPO / "harness" / "Harness.py"
    dst = EVAL_REPO / "Systems" / "Harness.py"
    if not dst.exists() or dst.read_bytes() != src.read_bytes():
        shutil.copy2(src, dst)


def check_controller_installed() -> list[str]:
    """Files that differ between this repo and the copy the sim executes."""
    diff = []

    def body(p: Path) -> bytes:
        return p.read_bytes().replace(bytes([13, 10]), bytes([10]))   # git may check out CRLF or LF; only content matters

    for f in controller_files():
        a, b = REPO / f, SOLUTION_REPO / f
        if a.exists() and (not b.exists() or body(a) != body(b)):
            diff.append(f)
    return diff


def install_controller() -> None:
    SOLUTION_REPO.mkdir(parents=True, exist_ok=True)
    for f in controller_files():
        if (REPO / f).exists():
            shutil.copy2(REPO / f, SOLUTION_REPO / f)
    shutil.rmtree(SOLUTION_REPO / "__pycache__", ignore_errors=True)


def bake_scenario(cfg: dict, seed: int) -> tuple[Path, str]:
    """
    Returns (absolute simconf path, 'Local/...' path) of the scenario simconf for
    this config+seed.  The untouched competition file is used when the config
    is the default scenario with the competition seed; otherwise
    scripts/make_config.py bakes a new field into STU4-Eval/Sims (cached).
    """
    from scripts import make_config  # noqa: WPS433 (lazy: numpy heavy)
    ensure_eval_repo()
    if make_config.is_competition_scenario(cfg, seed):
        return COMPETITION_SIMCONF, COMPETITION_SIMCONF_LOCAL
    fname = f"{cfg['name']}_s{seed}_{config_fingerprint(cfg, seed)}.simconf"
    out = EVAL_REPO / "Sims" / fname
    if not out.exists():
        make_config.bake(cfg, seed, out)
    return out, f"Local/Repos/STU4-Eval/Sims/{fname}"


def write_wrapper(scenario_abs: Path, scenario_local: str, run_id: str, out_dir: Path, timescale: float, cfg: dict,
                  harness: bool = True, max_sim_time: float = 400.0, sample_hz: float = 10.0,
                  stop_after: float = 3.0) -> tuple[Path, str]:
    """
    Wrapper simconf = a verbatim copy of the scenario simconf + the Harness system
    appended to SystemManager/Systems.  (IncludeSims cannot be used for this: the
    Python-side config loader of each system process does not see systems that
    come from an included file, so every Python system except the one defined in
    the top-level file dies with "no matching System ... found in sim config".)
    Nothing in the scenario itself is modified; the copy lives in STU4-Eval/Sims.
    """
    ensure_eval_repo()
    doc = json.loads(scenario_abs.read_text(encoding="utf-8"))
    scoring = (cfg.get("scoring") or {})
    debris_n = int((cfg.get("debris") or {}).get("num", 500))
    radar_n = min(int((cfg.get("radar") or {}).get("num", 200)), debris_n)
    if harness:
        doc["SystemManager"]["Systems"].append({
            "Active": True, "Type": "Python", "Nametag": "Harness",
            "Source": "Local/Repos/STU4-Eval/Systems/Harness.py",
            "Inst_Parameters": {
                "Shuttle": ["EntityRef", "SpaceBuggy"], "Phobos": ["EntityRef", "Phobos"],
                "Target": ["EntityRef", "AsteroidCycler"],
                "LoopFreqHz": 60.0, "SampleHz": float(sample_hz), "Timescale": float(timescale),
                "OutputDir": str(out_dir.resolve()),
                "KeepOutRadius_m": float(scoring.get("KeepOutRadius_m", 50.0)),
                "DamageThreshold_J": float(scoring.get("DamageThreshold_J", 209200.0)),
                "StopAfterEnd_s": float(stop_after), "MaxSimTime_s": float(max_sim_time),
                "NumDebris": ["int32", debris_n], "RadarNum": ["int32", radar_n],
            }})
    doc.setdefault("meta", {})["wrapper_of"] = scenario_local
    name = f"RUN_{run_id}"
    p = EVAL_REPO / "Sims" / f"{name}.simconf"
    p.write_text(json.dumps(doc, indent="	"), encoding="utf-8")
    return p, f"Local/Repos/STU4-Eval/Sims/{name}.simconf"


# ----------------------------------------------------------------------------- processes
def sim_processes(since: float | None = None) -> list[tuple[int, str, float]]:
    """(pid, name, created_epoch) of SpaceCRAFT / run_Server / sim-venv python processes,
    optionally only those created after `since` (time.time() seconds)."""
    ps = ("Get-CimInstance Win32_Process | Where-Object { $_.Name -match '^(SpaceCRAFT|run_Server|python)' } | "
          "ForEach-Object { $_.ProcessId.ToString() + '|' + $_.Name + '|' + ([DateTimeOffset]$_.CreationDate).ToUnixTimeSeconds().ToString() "
          "+ '|' + [string]$_.ExecutablePath + '|' + [string]$_.CommandLine }")
    out = subprocess.run(["powershell", "-NoProfile", "-Command", ps], capture_output=True, text=True).stdout
    procs = []
    for line in out.splitlines():
        parts = line.strip().split("|", 4)
        if len(parts) < 4:
            continue
        pid, name, created, path = parts[0], parts[1], float(parts[2]), parts[3]
        cmd = parts[4] if len(parts) > 4 else ""
        if name.lower().startswith("python") and "STPro\Versions" not in path and "STPro\Versions" not in cmd:
            continue          # some other python (e.g. this script)
        if since is not None and created < since - 2.0:
            continue
        procs.append((int(pid), name, created))
    return procs


def kill_sim_processes(since: float | None = None, quiet: bool = False) -> int:
    """Kill sim processes; with `since`, only those started after that time (i.e. ours)."""
    n = 0
    for pid, name, _ in sim_processes(since):
        subprocess.run(["taskkill", "/PID", str(pid), "/T", "/F"], capture_output=True)
        n += 1
    if n and not quiet:
        print(f"  killed {n} sim process(es)")
    return n


def launch(mode: str, sim_local_path: str, extra: list[str] | None = None) -> subprocess.Popen:
    exe, args, cwd, _ = MODES[mode]
    if not exe.exists():
        raise FileNotFoundError(exe)
    cmd = [str(exe), f"--sim={sim_local_path}"] + args + (extra or [])
    if mode != "server":
        cmd.append("-firstRole")
    return subprocess.Popen(cmd, cwd=str(cwd), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                            creationflags=getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0))


# ----------------------------------------------------------------------------- run logs
def log_folders() -> set[Path]:
    return {p for p in LOGS.glob("*__*") if p.is_dir()}


def main_log(folder: Path) -> Path | None:
    logs = [p for p in folder.glob("*.log") if "_PySys_" not in p.name]
    return logs[0] if logs else None


def parse_score(folder: Path) -> dict | None:
    log = main_log(folder)
    if log is None or not log.exists():
        return None
    text = log.read_text(errors="ignore")
    m = RE_SCORE.search(text)
    if not m:
        return None
    row = {"outcome": "win" if m.group(1).startswith("Successfully") else "loss", "score": float(m.group(2))}
    for k, rx in RE_NUM.items():
        mm = rx.search(text)
        row[k] = float(mm.group(1)) if mm else None
    mt = RE_TIMESCALE.search(text)
    row["initial_timescale"] = float(mt.group(1)) if mt else None
    mh = RE_HARNESS_TS.search(text)
    row["harness_timescale"] = float(mh.group(2)) if mh else None
    return row


def log_has(folder: Path, needle: str) -> bool:
    log = main_log(folder)
    return log is not None and needle in log.read_text(errors="ignore")


def wait_for(pred, timeout: float, poll: float = 1.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        r = pred()
        if r:
            return r
        time.sleep(poll)
    return None


def controller_log(folder: Path) -> Path | None:
    hits = list(folder.glob("*_PySys_Controller.log"))
    return hits[0] if hits else None


def now_id() -> str:
    return time.strftime("%y%m%d_%H%M%S")
