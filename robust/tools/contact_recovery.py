"""After-hit behaviour of contact flights: spin, attitude, velocity error, fuel, plan switches, repeat hits.
usage: python robust/tools/contact_recovery.py BATCH [...]"""
import json, glob, os, re, sys
ST = re.compile(r"t=\s*([\d.]+) rem=\s*([\d.]+) \|v\|=\s*([\d.]+) \|vt\|=\s*([\d.]+) dv=([\d.]+) att=\s*([\d.]+) w=([\d.]+) mode=(\w+).*?cam=(\d+).*?worst=(\S+):P=([\d.]+).*?fuel=\s*([\d.]+) dmg=(\d+)")
PL = re.compile(r"t=\s*([\d.]+) PLAN (\w+): dv=([\d.]+)")
for b in sys.argv[1:]:
    for l in open(f"results/eval/{b}/runs.jsonl"):
        r = json.loads(l)
        if not (r.get("damage_J") or 0) > 0: continue
        cf = os.path.join(r["out_dir"], "collisions.json")
        C = json.load(open(cf)) if os.path.exists(cf) else []
        lg = glob.glob(os.path.join(r["out_dir"], "*_PySys_Controller.log"))
        if not lg or not C: continue
        tc = C[0]["t"]
        L = [x.split("OnScreenLogMessage: ", 1)[1] for x in open(lg[0], errors="ignore") if "OnScreenLogMessage: " in x]
        S = [m.groups() for m in map(ST.search, L) if m]
        S = [(float(a), float(b_), float(c), float(d), float(e), float(f), float(g), h, int(i), j, float(k), float(m_), int(n)) for a, b_, c, d, e, f, g, h, i, j, k, m_, n in S]
        pre = [x for x in S if x[0] < tc]; post = [x for x in S if x[0] >= tc]
        if not post: continue
        f0 = pre[-1][11] if pre else 0
        wmax = max(x[6] for x in post[:30])
        t_w = next((x[0] - tc for x in post if x[6] < 0.1), None)
        t_att = next((x[0] - tc for x in post if x[0] - tc > 2 and x[5] < 20 and x[6] < 0.1), None)
        t_dv = next((x[0] - tc for x in post if x[0] - tc > 2 and x[4] < 0.5), None)
        tum = sum(1 for x in post if x[7] == "tumble")
        f_at = lambda dt: next((x[11] for x in post if x[0] - tc >= dt), post[-1][11])
        nsw = sum(1 for m in map(PL.search, L) if m and tc <= float(m.group(1)) < tc + 60)
        cam_drop = min((x[8] for x in post[:20]), default=-1)
        hits = sum(c.get("hits", 1) for c in C)
        end = post[-1]
        fmt = lambda v: "never" if v is None else f"{v:.0f}s"
        print(f"{b[:12]:12s} score={r['score'] if r['score'] is None else round(r['score'])} t_hit={tc:.0f} dmg={r['damage_J']/1e3:.0f}kJ hits={hits} contacts_logged={len(C)} "
              f"v={C[0]['shuttle_speed_m_s']:.1f} wmax={wmax:.2f} tumble={tum}s w<0.1 after {fmt(t_w)} att<20 after {fmt(t_att)} dv<0.5 after {fmt(t_dv)} "
              f"fuel +30s={f_at(30)-f0:.0f} +60s={f_at(60)-f0:.0f} +120s={f_at(120)-f0:.0f} total_after={end[11]-f0:.0f}kg "
              f"plans60s={nsw} cam_min={cam_drop} end_rem={end[1]:.0f}")
