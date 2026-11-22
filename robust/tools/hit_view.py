"""Milestone 5: what the controller saw before a contact. usage: python robust/tools/hit_view.py RUN_DIR [SECONDS_BEFORE]"""
import json, glob, os, sys, re, numpy as np
d = sys.argv[1]; W = float(sys.argv[2]) if len(sys.argv) > 2 else 25
C = json.load(open(os.path.join(d, "collisions.json")))
c = C[0]; deb = c.get("debris") or {}; h = c.get("history_5s") or []
sv = np.array(h[-1]["vel"]) if h else np.array(c["shuttle_vel"]); dv = np.array(deb.get("debris_vel", [0, 0, 0]))
print(f"hit t={c['t']:.1f} {deb.get('name')} d={deb.get('diameter_m', 0):.1f} radar={deb.get('radar_visible')} rock speed {np.linalg.norm(dv):.1f} "
      f"pre-impact rel {np.linalg.norm(sv - dv):.1f} m/s (harness {deb.get('rel_speed_m_s', 0):.1f}) dmg {c['damage_total_J']/1e3:.0f} kJ dist_to_target {c['dist_to_target_m']:.0f}")
L = [x.split("OnScreenLogMessage: ", 1)[1].rstrip() for x in open(glob.glob(os.path.join(d, "*_PySys_Controller.log"))[0], errors="ignore") if "OnScreenLogMessage: " in x]
for x in L:
    m = re.match(r"t=\s*([\d.]+)", x)
    if m and c["t"] - W <= float(m.group(1)) <= c["t"] + 0.5 and ("THREATS" in x or "PLAN" in x or "pause" in x.lower()):
        print("  " + x[:230])
