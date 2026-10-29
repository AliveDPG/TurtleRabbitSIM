"""
autorun.py - Run COMPETITION_SIM in Space Teams PRO hands-off, N times in a row.

Flow it drives (SpaceCRAFT fullscreen, already logged in):
    main menu --Sim--> sim-config page --Confirm--> role screen --Orbit Pawn, Start Simulation-->
    running sim (waits for the Scoring system's score line in the run log) --Exit Sim--> main menu ...

Each step is verified before clicking (pixel colours of the green Confirm / blue Sim tile, and the
run-log folder + score line as ground truth).  If the screen is not what it expects it saves a
screenshot next to this file and stops instead of clicking blindly.  Move the mouse into a screen
corner to abort at any time.

    python tools/autorun.py --dry        # report which screen is detected and where it would click
    python tools/autorun.py --runs 3     # run three times, print each score, write results/runs.csv

Pure ctypes, no extra packages.  Click positions are fractions of the window, measured on
1920x1080 screenshots; run --dry once to confirm the detection on your screen.
"""
from __future__ import annotations

import argparse
import csv
import ctypes
import ctypes.wintypes as wt
import os
import re
import subprocess
import sys
import time
from pathlib import Path

user32, gdi32 = ctypes.windll.user32, ctypes.windll.gdi32
user32.SetProcessDPIAware()

REPO = Path(__file__).resolve().parents[1]
LOGS = Path(os.environ.get("LOCALAPPDATA", "")) / "SimDynamX" / "STPro" / "Logs"
RESULTS = REPO / "results" / "runs.csv"
WINDOW_TITLE = "Space Teams Platform"

# ---- click targets, as fractions of the client area -----------------------------------------
SIM_TILE = (0.055, 0.165)          # left panel "Sim" (main menu and sim-config page)
EXIT_DESKTOP = (0.055, 0.965)      # left panel "Exit to Desktop" (second menu probe)
CONFIRM = (0.950, 0.852)           # green Confirm (sim-config page) - click point
CONFIRM_PROBE = (0.915, 0.852)     # left part of the button, away from the white label text
CORE_SIMS_BOX = (0.124, 0.231)     # "Core Sims" filter checkbox
USER_SIMS_BOX = (0.124, 0.276)     # "User-Made Sims" filter checkbox
FIRST_ROW = (0.450, 0.226)         # first row of the list = COMPETITION_SIM with the User-Made filter
ROLE_ORBIT_PAWN = (0.235, 0.331)   # "Orbit Pawn" row on the Select Role screen (lobby)
START_SIM = (0.481, 0.731)         # "Start Simulation" button
EXIT_TO_MENU = (0.4985, 0.214)     # "Exit to Main Menu" on the lobby screen
EXIT_SIM = (0.980, 0.0145)         # "Exit Sim" at the top right of the in-sim HUD

RE_SCORE = re.compile(r"(Successfully escaped Phobos with a score of|Crashed into some debris and finished with a score of)\s*(-?[\d.]+)")
RE_NUM = {k: re.compile(p) for k, p in {
    "elapsed_s": r"Sim elapsed time\s*=\s*([\d.]+)", "distance_m": r"Distance to ship\s*=\s*([\d.]+)",
    "damage_J": r"Total damage taken\s*=\s*([\d.]+)", "fuel_kg": r"Total fuel used\s*=\s*([\d.]+)"}.items()}


# ---- win32 helpers -----------------------------------------------------------------------
def find_window():
    found = []

    @ctypes.WINFUNCTYPE(ctypes.c_bool, wt.HWND, wt.LPARAM)
    def cb(hwnd, _):
        if user32.IsWindowVisible(hwnd):
            n = user32.GetWindowTextLengthW(hwnd)
            if n:
                buf = ctypes.create_unicode_buffer(n + 1)
                user32.GetWindowTextW(hwnd, buf, n + 1)
                if WINDOW_TITLE.lower() in buf.value.lower():
                    found.append(hwnd)
        return True
    user32.EnumWindows(cb, 0)
    return found[0] if found else None


def client_rect(h):
    r = wt.RECT()
    user32.GetClientRect(h, ctypes.byref(r))
    pt = wt.POINT(0, 0)
    user32.ClientToScreen(h, ctypes.byref(pt))
    return pt.x, pt.y, r.right, r.bottom


def to_screen(h, frac):
    x0, y0, w, hgt = client_rect(h)
    return int(x0 + frac[0] * w), int(y0 + frac[1] * hgt)


def pixel(x, y):
    dc = user32.GetDC(0)
    c = gdi32.GetPixel(dc, x, y)
    user32.ReleaseDC(0, dc)
    return c & 0xFF, (c >> 8) & 0xFF, (c >> 16) & 0xFF


def focus(h):
    user32.ShowWindow(h, 9)
    user32.SetForegroundWindow(h)
    time.sleep(0.4)


def click(x, y):
    user32.SetCursorPos(x, y)
    time.sleep(0.15)
    user32.mouse_event(2, 0, 0, 0, 0)
    time.sleep(0.08)
    user32.mouse_event(4, 0, 0, 0, 0)
    time.sleep(0.35)


def mouse_in_corner():
    pt = wt.POINT()
    user32.GetCursorPos(ctypes.byref(pt))
    w, hgt = user32.GetSystemMetrics(0), user32.GetSystemMetrics(1)
    return (pt.x <= 1 or pt.x >= w - 2) and (pt.y <= 1 or pt.y >= hgt - 2)


def screenshot(path: Path):
    ps = ("Add-Type -AssemblyName System.Windows.Forms,System.Drawing;"
          "$b=[System.Windows.Forms.Screen]::PrimaryScreen.Bounds;"
          "$bmp=New-Object System.Drawing.Bitmap $b.Width,$b.Height;"
          "$g=[System.Drawing.Graphics]::FromImage($bmp);$g.CopyFromScreen($b.Location,[System.Drawing.Point]::Empty,$b.Size);"
          f"$bmp.Save('{path}',[System.Drawing.Imaging.ImageFormat]::Png)")
    subprocess.run(["powershell", "-NoProfile", "-Command", ps], capture_output=True)


def fail(msg):
    shot = REPO / "tools" / "autorun_fail.png"
    screenshot(shot)
    sys.exit(f"ABORT: {msg}\n(screenshot saved to {shot})")


# ---- screen detection --------------------------------------------------------------------
def is_green(rgb):
    r, g, b = rgb
    return g > 110 and g > r + 40 and g > b + 40


def is_panel_grey(rgb):
    """The left menu panel's tiles are neutral dark grey (highlighted tile is blue-grey)."""
    r, g, b = rgb
    return 22 <= r <= 120 and 22 <= g <= 120 and 22 <= b <= 170 and abs(r - g) < 14


def screen(h) -> str:
    """'simpage' (green Confirm), 'mainmenu' (left panel visible, no Confirm), or 'other' (role screen / in sim)."""
    if is_green(pixel(*to_screen(h, CONFIRM_PROBE))):
        return "simpage"
    if is_panel_grey(pixel(*to_screen(h, SIM_TILE))) and is_panel_grey(pixel(*to_screen(h, EXIT_DESKTOP))):
        return "mainmenu"
    return "other"


# ---- run-log helpers ---------------------------------------------------------------------
def run_folders():
    return {p.name for p in LOGS.glob("*_COMPETITION_SIM__*")}


def parse_run(folder: Path):
    log = folder / "COMPETITION_SIM.log"
    if not log.exists():
        return None
    text = log.read_text(errors="ignore")
    m = RE_SCORE.search(text)
    if not m:
        return None
    row = {"stamp": folder.name.split("_COMPETITION_SIM")[0],
           "outcome": "win" if m.group(1).startswith("Successfully") else "loss", "score": float(m.group(2))}
    for k, rx in RE_NUM.items():
        mm = rx.search(text)
        row[k] = float(mm.group(1)) if mm else ""
    return row


def run_in_progress() -> bool:
    folders = sorted(LOGS.glob("*_COMPETITION_SIM__*"))
    if not folders:
        return False
    log = folders[-1] / "COMPETITION_SIM.log"
    return log.exists() and time.time() - log.stat().st_mtime < 15.0 and parse_run(folders[-1]) is None


def append_result(row):
    RESULTS.parent.mkdir(parents=True, exist_ok=True)
    new = not RESULTS.exists()
    with RESULTS.open("a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["stamp", "outcome", "score", "elapsed_s", "distance_m", "damage_J", "fuel_kg"])
        if new:
            w.writeheader()
        w.writerow(row)


def wait_for(pred, timeout, what, poll=1.5):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if mouse_in_corner():
            sys.exit("aborted: mouse in screen corner")
        if pred():
            return True
        time.sleep(poll)
    fail(f"timeout waiting for {what}")


# ---- the flow -----------------------------------------------------------------------------
def box_checked(h, frac) -> bool:
    """Filter checkbox: checked boxes are light (white with a dark tick), unchecked are dark grey."""
    x, y = to_screen(h, frac)
    return max(max(pixel(x - 6, y)), max(pixel(x + 6, y)), max(pixel(x, y - 6))) > 150


def select_competition_sim(h):
    """On the sim-config page: User-Made filter on, Core off, first row (COMPETITION_SIM) selected."""
    if box_checked(h, CORE_SIMS_BOX):
        click(*to_screen(h, CORE_SIMS_BOX))
        time.sleep(0.8)
    if not box_checked(h, USER_SIMS_BOX):
        click(*to_screen(h, USER_SIMS_BOX))
        time.sleep(0.8)
    if box_checked(h, CORE_SIMS_BOX) or not box_checked(h, USER_SIMS_BOX):
        fail("could not set the filters to User-Made Sims only")
    click(*to_screen(h, FIRST_ROW))          # Repos/STU4-Sim/Sims/COMPETITION_SIM.simconf
    time.sleep(0.6)


def to_sim_page(h):
    s = screen(h)
    if s == "other":
        # most likely the lobby (role screen) left over from an earlier attempt: back out of it
        print("  not at a menu; trying 'Exit to Main Menu' (lobby)")
        click(*to_screen(h, EXIT_TO_MENU))
        wait_for(lambda: screen(h) in ("mainmenu", "simpage"), 30, "the main menu after leaving the lobby")
        s = screen(h)
    if s == "mainmenu":
        click(*to_screen(h, SIM_TILE))
        wait_for(lambda: screen(h) == "simpage", 60, "the sim-config page")
    if screen(h) != "simpage":
        fail("expected the sim-config page")
    select_competition_sim(h)


def one_run(h):
    if run_in_progress():
        fail("a run is already in progress")
    focus(h)
    before = run_folders()
    to_sim_page(h)
    time.sleep(0.5)
    if not is_green(pixel(*to_screen(h, CONFIRM_PROBE))):
        fail("Confirm button not green")
    click(*to_screen(h, CONFIRM))
    # role screen (lobby): pick Orbit Pawn and start.  The lobby can take a while to appear and
    # the run folder is created while still in the lobby, so: wait for the page to change, give
    # it time, click, and if the controller does not start logging, click again.
    wait_for(lambda: screen(h) != "simpage", 30, "the role screen")
    time.sleep(5.0)
    new = {}
    folder = None
    for attempt in range(4):
        focus(h)
        click(*to_screen(h, ROLE_ORBIT_PAWN))
        time.sleep(0.8)
        click(*to_screen(h, START_SIM))
        t0 = time.time()
        while time.time() - t0 < 25.0:
            time.sleep(2.0)
            new["f"] = run_folders() - before
            if new["f"]:
                folder = LOGS / sorted(new["f"])[-1]
                ctrl = folder / "COMPETITION_SIM_PySys_Controller.log"
                if ctrl.exists() and "OnScreenLogMessage: t=" in ctrl.read_text(errors="ignore"):
                    break
        else:
            print(f"  controller not running yet (attempt {attempt + 1}); clicking Start again")
            continue
        break
    if folder is None:
        fail("the sim never started")
    print(f"  run {folder.name} started")
    res = {}
    wait_for(lambda: bool(res.update({"r": parse_run(folder)}) or res["r"]), 480, "the score", poll=3.0)
    r = res["r"]
    print(f"  {r['outcome']}: score={r['score']}  time={r['elapsed_s']}s  fuel={r['fuel_kg']}kg  damage={r['damage_J']}J  dist={r['distance_m']}m")
    append_result(r)
    # leave the sim and get back to a menu
    time.sleep(3.0)
    focus(h)
    click(*to_screen(h, EXIT_SIM))
    wait_for(lambda: screen(h) in ("mainmenu", "simpage"), 60, "the menu after Exit Sim", poll=2.0)
    time.sleep(2.0)
    return r


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--runs", type=int, default=1)
    ap.add_argument("--dry", action="store_true", help="only report the detected screen and click targets")
    ap.add_argument("--shot", action="store_true", help="bring the sim to the front and save tools/probe.png")
    ap.add_argument("--to-role", action="store_true", help="go to the sim page, set the filter/row, screenshot, stop (no Confirm)")
    args = ap.parse_args()
    h = find_window()
    if not h:
        sys.exit(f"window '{WINDOW_TITLE}' not found - is SpaceCRAFT open?")
    focus(h)                       # pixel reads must see the sim, not whatever window is on top
    time.sleep(0.8)
    print(f"window {h:#x} client {client_rect(h)}  screen={screen(h)}  run_in_progress={run_in_progress()}")
    if args.shot:
        shot = REPO / "tools" / "probe.png"
        screenshot(shot)
        print(f"saved {shot}")
    if args.to_role:
        if run_in_progress():
            fail("a run is already in progress")
        to_sim_page(h)
        shot = REPO / "tools" / "probe_simpage.png"
        screenshot(shot)
        print(f"sim page prepared (User-Made filter, COMPETITION_SIM row); saved {shot}")
        return
    if args.dry or args.shot:
        for name, frac in [("Sim tile", SIM_TILE), ("Confirm", CONFIRM), ("Orbit Pawn", ROLE_ORBIT_PAWN),
                           ("Start Simulation", START_SIM), ("Exit Sim", EXIT_SIM)]:
            x, y = to_screen(h, frac)
            print(f"  {name:17} -> ({x},{y}) pixel={pixel(x, y)}")
        return
    scores = []
    for i in range(args.runs):
        print(f"\n=== run {i + 1}/{args.runs} ===")
        scores.append(one_run(h)["score"])
    print(f"\nscores: {scores}  mean={sum(scores) / len(scores):.1f}  -> {RESULTS}")


if __name__ == "__main__":
    main()
