"""
make_config.py - bake a scenario (configs/<name>.yaml + seed) into a .simconf.

Re-implements the organisers' SpawnDebris.py spawn maths (same RNG draw order,
same seed semantics: np.random.seed(seed) for the numbers, random.seed(seed)
for the rock-type choice) on top of the untouched COMPETITION_SIM.simconf, so

    python scripts/make_config.py --config default --seed 100 --check

reproduces the competition field bit-for-bit, and any other seed / config gives
a field the real sim would have produced with those settings.  Nothing in
Assets/Repos/STU4-Sim is written; output goes to Assets/Repos/STU4-Eval/Sims
(or --out).

    python scripts/make_config.py --config hard --seed 7
    python scripts/make_config.py --config hard --seed 7 --out C:/tmp/hard7.simconf
"""
from __future__ import annotations

import argparse
import copy
import json
import math
import random
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from harness import simlib  # noqa: E402

MU_M3_S2 = 707.5e-6 * 1e9       # Phobos, from Phobos_Proc_GDAL.enconf (km^3/s^2 -> m^3/s^2)
RADIUS_M = 11100.0              # "#Planet/General/Radius_m" of the same entity
ROCK_TYPES = ["Rock1", "Rock2", "Rock3", "ExplodedTank"]
COMPETITION_SEED = 100

DEFAULTS = {
    "debris": {"num": 500, "spread_m": [300.0, 300.0, 500.0], "vertical_offset_m": 500.0,
               "weights": [0.45, 0.35, 0.15, 0.05], "speed_frac": [0.05, 0.10], "scale_mult": 1.0,
               "angvel_max_rad_s": 0.2},
    "radar": {"num": 200, "position_1sigma_m": 2.0, "velocity_1sigma_m_s": 0.02,
              "diameter_1sigma_fraction": 0.05, "loop_hz": 120.0},
    "scoring": {"DamageThreshold_J": 209200.0, "NominalFuel_kg": 1.15, "NominalTime_s": 120.0,
                "KeepOutRadius_m": 50.0},
    "shuttle": {"max_accel_m_s2": 0.5, "max_ang_accel_rad_s2": 0.05, "controller_loop_hz": 60.0},
}


def merged(cfg: dict) -> dict:
    out = copy.deepcopy(DEFAULTS)
    for sec, vals in cfg.items():
        if sec in out and isinstance(vals, dict):
            out[sec].update(vals)
    return out


def is_competition_scenario(cfg: dict, seed: int) -> bool:
    if seed != COMPETITION_SEED:
        return False
    m = merged(cfg)
    return json.dumps(m, sort_keys=True) == json.dumps(DEFAULTS, sort_keys=True)


# ----------------------------------------------------------------------------- spawn maths
def clamp(v, lo, hi):
    return min(max(v, lo), hi)


def remap(v, in_min, in_max, out_min, out_max):
    return clamp(out_min + (out_max - out_min) * (v - in_min) / (in_max - in_min), out_min, out_max)


def remap_vec(v, in_min, in_max, out_min, out_max):
    return np.minimum(np.maximum(out_min + (out_max - out_min) * (v - in_min) / (in_max - in_min), out_min), out_max)


def nwu(loc: np.ndarray):
    """North/West/Up unit vectors at loc in the body-fixed frame (pole = +z)."""
    up = loc / np.linalg.norm(loc)
    z = np.array([0.0, 0.0, 1.0])
    north = z - np.dot(z, up) * up
    north /= np.linalg.norm(north)
    west = np.cross(up, north)
    return north, west, up


def load_rock_templates() -> dict:
    out = {}
    for name in ROCK_TYPES:
        p = simlib.SIM_REPO / "Entities" / f"{name}.enconf"
        out[name] = json.loads(p.read_text(encoding="utf-8"))["Entity"]
    return out


def spawn_debris(m: dict, seed: int, shuttle_loc: np.ndarray, templates: dict) -> list[dict]:
    d = m["debris"]
    n = int(d["num"])
    spread = np.asarray(d["spread_m"], float)
    _, _, up0 = nwu(shuttle_loc)
    centre = shuttle_loc + up0 * float(d["vertical_offset_m"])
    loc_min, loc_max = centre - spread, centre + spread
    weights = np.asarray(d["weights"], float)
    weights = list(weights / weights.sum())
    f_lo, f_hi = d["speed_frac"]
    vmax = math.sqrt(MU_M3_S2 / RADIUS_M)
    angvel_max = float(d["angvel_max_rad_s"])
    scale_mult = float(d["scale_mult"])

    random.seed(seed)
    np.random.seed(seed)
    ents = []
    for i in range(n):
        loc_r = np.random.random(3)
        vel_r = np.random.random()
        az_r = np.random.random()
        angle_r = np.random.random()
        axis_r = np.random.random(3)
        angvel_r = np.random.random(3)
        size_r = np.random.random()

        azimuth = remap(az_r, 0.0, 1.0, 0.0, 2.0 * math.pi)
        loc = remap_vec(loc_r, np.zeros(3), np.ones(3), loc_min, loc_max)
        north, west, _ = nwu(loc)
        vel_mag = remap(vel_r, 0.0, 1.0, f_lo * vmax, f_hi * vmax)
        vel = vel_mag * (math.cos(azimuth) * north - math.sin(azimuth) * west)

        angle = remap(angle_r, 0.0, 1.0, 0.0, 2.0 * math.pi)
        axis = remap_vec(axis_r, np.zeros(3), np.ones(3), -np.ones(3), np.ones(3))
        axis = axis / np.linalg.norm(axis)
        quat = np.array([math.sin(0.5 * angle) * axis[0], math.sin(0.5 * angle) * axis[1],
                         math.sin(0.5 * angle) * axis[2], math.cos(0.5 * angle)])
        quat = quat / np.linalg.norm(quat)
        angvel = remap_vec(angvel_r, np.zeros(3), np.ones(3), -np.ones(3), np.ones(3))
        angvel = angvel_max * angvel / np.linalg.norm(angvel)

        kind = random.choices(ROCK_TYPES, weights=weights, k=1)[0]
        tpl = templates[kind]
        smin, smax = float(tpl["ScaleMin"]) * scale_mult, float(tpl["ScaleMax"]) * scale_mult
        size = remap(size_r, 0.0, 1.0, smin, smax)
        base_diam = float(tpl["RadarDiameter_m"])
        base_mass = float(tpl["Dynamics"]["Mass"])

        ents.append({
            "#Object": {"Mesh_Path": tpl["#Object"]["Mesh_Path"]},
            "#Required": {"Name": f"Debris{i}", "Systems": [], "Type": "Object"},
            "Acceleration": ["doubleV3", 0.0, 0.0, 0.0],
            "AngAcceleration": ["doubleV3", 0.0, 0.0, 0.0],
            "AngVelocity": ["doubleV3", *map(float, angvel)],
            "Dynamics": {
                "Inertia_BodyFrame": tpl["Dynamics"]["Inertia_BodyFrame"],
                "Mass": base_mass * size ** 3,
                "PhysicsApplyFreq": tpl["Dynamics"].get("PhysicsApplyFreq", 1.0),
                "PhysicsSyncFreq": tpl["Dynamics"].get("PhysicsSyncFreq", 1.0),
                "Propagator": tpl["Dynamics"]["Propagator"],
                "ResidentFrame": ["EntityRef", "Phobos"]},
            "Graphics": {"RenderFar": tpl["Graphics"].get("RenderFar", False), "Scale": size,
                         "VisualRadius_m": 0.5 * base_diam * size},
            "Location": ["doubleV3", *map(float, loc)],
            "RadarDiameter_m": base_diam * size,
            "Rotation": ["doubleV4", *map(float, quat)],
            "ScaleMax": smax, "ScaleMin": smin,
            "Velocity": ["doubleV3", *map(float, vel)],
        })
    return ents


# ----------------------------------------------------------------------------- baking
def bake(cfg: dict, seed: int, out: Path) -> Path:
    m = merged(cfg)
    base = json.loads(simlib.COMPETITION_SIMCONF.read_text(encoding="utf-8"))
    doc = copy.deepcopy(base)
    ents = doc["DataManager"]["entities"]
    keep = [e for e in ents if not e["#Required"]["Name"].startswith("Debris")]
    shuttle = next(e for e in keep if e["#Required"]["Name"] == "SpaceBuggy")
    shuttle_loc = np.asarray(shuttle["Location"][1:], float)

    debris = spawn_debris(m, seed, shuttle_loc, load_rock_templates())
    doc["DataManager"]["entities"] = keep + debris
    doc["DataManager"]["SimEntity"]["Debris"] = {e["#Required"]["Name"]: ["EntityRef", e["#Required"]["Name"]] for e in debris}
    doc["DataManager"]["SimEntity"]["DoSpawning"] = False

    sh = m["shuttle"]
    shuttle["Thrusters"]["MaxAccel_m_s2"] = float(sh["max_accel_m_s2"])
    shuttle["Thrusters"]["MaxAngAccel_rad_s2"] = float(sh["max_ang_accel_rad_s2"])

    d, r, s = m["debris"], m["radar"], m["scoring"]
    for sysc in doc["SystemManager"]["Systems"]:
        ip = sysc["Inst_Parameters"]
        tag = sysc["Nametag"]
        if tag == "SpawnDebris":
            ip["NumDebris"] = ["int32", int(d["num"])]
            ip["Spread_m"] = ["doubleV3", *map(float, d["spread_m"])]
            ip["VerticalOffset_m"] = float(d["vertical_offset_m"])
            ip["DebrisWeights"] = ["array", "double", *map(float, d["weights"])]
        elif tag == "Radar":
            ip["NumDebris"] = ["int32", min(int(r["num"]), int(d["num"]))]
            ip["Position_1sigma_m"] = float(r["position_1sigma_m"])
            ip["Velocity_1sigma_m_s"] = float(r["velocity_1sigma_m_s"])
            ip["Diameter_1sigma_fraction"] = float(r["diameter_1sigma_fraction"])
            ip["LoopFreqHz"] = float(r["loop_hz"])
        elif tag == "Scoring":
            for k in ("DamageThreshold_J", "NominalFuel_kg", "NominalTime_s", "KeepOutRadius_m"):
                ip[k] = float(s[k])
        elif tag == "Controller":
            ip["LoopFreqHz"] = float(sh["controller_loop_hz"])
    doc.setdefault("meta", {})["baked_by"] = {"config": cfg.get("name"), "seed": seed, "params": m}
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(doc, indent="\t"), encoding="utf-8")
    return out


def check_against_competition(cfg: dict) -> int:
    """Rebuild the default field with seed 100 and compare every debris entity to the baked file."""
    m = merged(cfg)
    base = json.loads(simlib.COMPETITION_SIMCONF.read_text(encoding="utf-8"))
    byname = {e["#Required"]["Name"]: e for e in base["DataManager"]["entities"]}
    shuttle_loc = np.asarray(byname["SpaceBuggy"]["Location"][1:], float)
    mine = spawn_debris(m, COMPETITION_SEED, shuttle_loc, load_rock_templates())
    worst = 0.0
    bad = 0
    for e in mine:
        ref = byname.get(e["#Required"]["Name"])
        if ref is None:
            bad += 1
            continue
        for key in ("Location", "Velocity", "Rotation", "AngVelocity"):
            worst = max(worst, float(np.max(np.abs(np.asarray(e[key][1:]) - np.asarray(ref[key][1:])))))
        for key in ("RadarDiameter_m", "ScaleMin", "ScaleMax"):
            worst = max(worst, abs(e[key] - ref[key]))
        worst = max(worst, abs(e["Dynamics"]["Mass"] - ref["Dynamics"]["Mass"]) / max(ref["Dynamics"]["Mass"], 1))
        worst = max(worst, abs(e["Graphics"]["Scale"] - ref["Graphics"]["Scale"]))
        if e["#Object"]["Mesh_Path"] != ref["#Object"]["Mesh_Path"]:
            bad += 1
    print(f"check: {len(mine)} debris regenerated, {bad} mismatching type/name, max abs numeric deviation {worst:.3e}")
    return 0 if bad == 0 and worst < 1e-6 else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default="default")
    ap.add_argument("--seed", type=int, default=COMPETITION_SEED)
    ap.add_argument("--out", help="output .simconf (default: Assets/Repos/STU4-Eval/Sims/<config>_s<seed>_<hash>.simconf)")
    ap.add_argument("--check", action="store_true", help="verify the generator reproduces the competition field (seed 100)")
    args = ap.parse_args()
    cfg = simlib.load_config(args.config)
    if args.check:
        sys.exit(check_against_competition(cfg))
    if args.out:
        out = bake(cfg, args.seed, Path(args.out))
    else:
        out, _ = simlib.bake_scenario(cfg, args.seed)
    print(out)


if __name__ == "__main__":
    main()
