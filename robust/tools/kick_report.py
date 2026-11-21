"""Recovery after the RB_KICK_T test kick: time from 'KICK released' until the nose is within 10 deg (and 20 deg),
fuel used in that time, and the flight result.  usage: python robust/tools/kick_report.py BATCH [...]"""
import json, glob, os, re, sys
ST = re.compile(r"t=\s*([\d.]+) rem=.*?att=\s*([\d.]+) w=([\d.]+) mode=(\w+).*?fuel=\s*([\d.]+)")
KR = re.compile(r"KICK released at t=([\d.]+)s att=([\d.]+)")
KS = re.compile(r"KICK start at t=([\d.]+)s")
for b in sys.argv[1:]:
    for l in open(f"results/eval/{b}/runs.jsonl"):
        r = json.loads(l)
        lg = glob.glob(os.path.join(r.get("out_dir") or "x", "*_PySys_Controller.log"))
        if not lg:
            print(b, "no log", r.get("error")); continue
        L = [x.split("OnScreenLogMessage: ", 1)[1] for x in open(lg[0], errors="ignore") if "OnScreenLogMessage: " in x]
        ks = next((float(m.group(1)) for m in map(KS.search, L) if m), None)
        kr = next((float(m.group(1)) for m in map(KR.search, L) if m), None)
        S = [(float(a), float(b_), float(c), d, float(e)) for a, b_, c, d, e in (m.groups() for m in map(ST.search, L) if m)]
        if kr is None:
            print(b, "no release", r.get("score"), r.get("error")); continue
        f0 = next((x[4] for x in S if x[0] >= kr), None)
        fs = next((x[4] for x in S if x[0] >= ks), None)
        t20 = next((x[0] for x in S if x[0] > kr and x[1] < 20), None)
        t10 = next((x[0] for x in S if x[0] > kr and x[1] < 10), None)
        f10 = next((x[4] for x in S if t10 is not None and x[0] >= t10), None)
        print(f"{b} score={r.get('score')} dmg={r.get('damage_J')} kick {ks:.0f}->{kr:.0f}s (kick fuel {f0 - fs:.2f} kg) "
              f"att<20 after {('%.0f s' % (t20 - kr)) if t20 else 'never'}, att<10 after {('%.0f s' % (t10 - kr)) if t10 else 'never'}"
              f"{(', recovery fuel %.2f kg' % (f10 - f0)) if f10 else ''}")
