"""RB_KICK upset test: from 'KICK2 released' until recovered (rate < 0.1 rad/s, nose < 20 deg, velocity error
< 0.5 m/s), fuel from release to recovery and to the end, final score.  usage: python robust/tools/kick2_report.py BATCH [...]"""
import json, glob, os, re, sys
ST = re.compile(r"t=\s*([\d.]+) rem=\s*([\d.]+) .*?dv=([\d.]+) att=\s*([\d.]+) w=([\d.]+) mode=(\w+).*?fuel=\s*([\d.]+)")
KR = re.compile(r"KICK2 released at t=([\d.]+)s \|v\|=([\d.]+) dv=([\d.]+) w=([\d.]+)")
KS = re.compile(r"KICK2 start at t=([\d.]+)s")
for b in sys.argv[1:]:
    if not os.path.exists(f"results/eval/{b}/runs.jsonl"):
        continue
    for l in open(f"results/eval/{b}/runs.jsonl"):
        r = json.loads(l)
        lg = glob.glob(os.path.join(r.get("out_dir") or "x", "*_PySys_Controller.log"))
        if not lg:
            print(f"{b} rig failure: {r.get('error')}"); continue
        L = [x.split("OnScreenLogMessage: ", 1)[1] for x in open(lg[0], errors="ignore") if "OnScreenLogMessage: " in x]
        m = next((m for m in map(KR.search, L) if m), None)
        ks = next((float(k.group(1)) for k in map(KS.search, L) if k), None)
        S = [(float(a), float(b_), float(c), float(d), float(e), f, float(g)) for a, b_, c, d, e, f, g in (x.groups() for x in map(ST.search, L) if x)]
        if m is None or not S:
            print(f"{b} no release; score={r.get('score')} {r.get('error')}"); continue
        tr = float(m.group(1))
        f_ks = next(x[6] for x in S if x[0] >= ks)
        f_r = next(x[6] for x in S if x[0] >= tr)
        rec = next((x for x in S if x[0] > tr + 1 and x[4] < 0.1 and x[3] < 20 and x[2] < 0.5), None)
        nose = next((x for x in S if x[0] > tr + 1 and x[4] < 0.1 and x[3] < 20), None)
        print(f"{b} score={r.get('score')} fuel={r.get('fuel_kg')} dmg={r.get('damage_J')} | upset at release: dv={m.group(3)} m/s w={m.group(4)} rad/s "
              f"(kick fuel {f_r - f_ks:.1f} kg) | nose<20&w<0.1 after {('%.0f s' % (nose[0] - tr)) if nose else 'never'}; "
              f"recovered (also dv<0.5) after {('%.0f s, %.1f kg' % (rec[0] - tr, rec[6] - f_r)) if rec else 'never'}; "
              f"fuel release->end {S[-1][6] - f_r:.1f} kg")
