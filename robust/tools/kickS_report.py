"""Strong-kick test: finished?, time, fuel, score; time from release to velocity error < 1 m/s and its fuel.
usage: python robust/tools/kickS_report.py BATCH [...]"""
import json, glob, os, re, sys
ST = re.compile(r"t=\s*([\d.]+) rem=\s*([\d.]+) .*?dv=([\d.]+) att=\s*([\d.]+) w=([\d.]+) mode=(\w+).*?fuel=\s*([\d.]+)")
FT = re.compile(r"rscore (\d) dvburn (\d)")
for b in sys.argv[1:]:
    if not os.path.exists(f"results/eval/{b}/runs.jsonl"): continue
    for l in open(f"results/eval/{b}/runs.jsonl"):
        r = json.loads(l)
        lg = glob.glob(os.path.join(r.get("out_dir") or "x", "*_PySys_Controller.log"))
        if not lg:
            print(f"{b} rig failure: {r.get('error')}"); continue
        L = [x.split("OnScreenLogMessage: ", 1)[1] for x in open(lg[0], errors="ignore") if "OnScreenLogMessage: " in x]
        fx = next((m.groups() for m in map(FT.search, L) if m), ("?", "?"))
        rel = next((float(re.search(r"t=([\d.]+)s", x).group(1)) for x in L if "KICK2 released" in x), None)
        S = [(float(a), float(b_), float(c), float(d), float(e), f, float(g)) for a, b_, c, d, e, f, g in (m.groups() for m in map(ST.search, L) if m)]
        modes = {}
        if rel:
            f0 = next(x[6] for x in S if x[0] >= rel)
            ok = next((x for x in S if x[0] > rel + 2 and x[2] < 1.0 and x[4] < 0.1), None)
            for x in S:
                if rel < x[0] < (ok[0] if ok else 1e9): modes[x[5]] = modes.get(x[5], 0) + 1
            rec = f"error<1 m/s after {ok[0] - rel:.0f} s, {ok[6] - f0:.1f} kg" if ok else "error never < 1 m/s"
        else:
            rec = "no release"
        res = f"score={r['score']:.1f} t={r['elapsed_s']:.0f} fuel={r['fuel_kg']:.0f}" if r.get("score") is not None else f"UNFINISHED ({r.get('error')}) end_rem={S[-1][1]:.0f}"
        print(f"{b} fix(rscore,dvburn)={fx} {res} dmg={r.get('damage_J')} | released {rel} | {rec} | modes {modes}")
