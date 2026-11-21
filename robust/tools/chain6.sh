#!/bin/bash
# confirmation: candidate vs baseline (robust/, all switches off), two lanes swapped each batch, until each arm
# has NEED scored flights or 3 h; single-lane alternating fallback when free RAM < 5 GB.
cd /c/Users/darpan/Documents/SpaceTeams/TurtleRabbitSIM
CAND="$1"; NEED=${2:-12}; OFF="RB_NONE=1"
until grep -q "CHAIN5 DONE" results/eval/chain5.log 2>/dev/null; do sleep 20; done
sleep 10; until [ $(tasklist | grep -ci spacecraft) -eq 0 ]; do sleep 10; done
T0=$(date +%s); i=0
count() { python - "$@" <<'PY'
import json, sys, glob
arm = sys.argv[1]; n = 0
for f in glob.glob("results/eval/conf*_*/runs.jsonl"):
    b = f.split("\\")[-2] if "\\" in f else f.split("/")[-2]
    for l in open(f):
        r = json.loads(l)
        if r.get("score") is None: continue
        if json.loads(open(f.replace("runs.jsonl", "../" + b + ".arm")).read())["arm"] == arm: n += 1
print(n)
PY
}
while [ $(( $(date +%s) - T0 )) -lt 10800 ]; do
  nc=$(count cand); nb=$(count base)
  echo "$(date +%T) cand=$nc base=$nb" >> results/eval/conf.progress
  [ $nc -ge $NEED ] && [ $nb -ge $NEED ] && break
  i=$((i + 1)); mem=$(powershell -NoProfile -Command "[int]((Get-CimInstance Win32_OperatingSystem).FreePhysicalMemory/1024)")
  if [ "$mem" -ge 5000 ]; then
    if [ $((i % 2)) -eq 1 ]; then EA=$CAND; EB=$OFF; AA=cand; AB=base; else EA=$OFF; EB=$CAND; AA=base; AB=cand; fi
    echo "{\"arm\": \"$AA\"}" > results/eval/conf${i}_a.arm; echo "{\"arm\": \"$AB\"}" > results/eval/conf${i}_b.arm
    echo "$(date +%T) batch conf$i two lanes (a=$AA b=$AB) free ${mem} MB" >> results/eval/conf.progress
    bash robust/tools/run_pair.sh conf$i 3 "$EA" "$EB" > results/eval/conf$i.pair.log 2>&1
  else
    echo "{\"arm\": \"cand\"}" > results/eval/conf${i}_a.arm; echo "{\"arm\": \"base\"}" > results/eval/conf${i}_b.arm
    echo "$(date +%T) batch conf$i SINGLE lane (free ${mem} MB)" >> results/eval/conf.progress
    bash robust/tools/run_alt.sh conf$i 2 "$CAND" "$OFF" 3 robust robust > results/eval/conf$i.alt.log 2>&1
  fi
  sleep 10
done
echo "CHAIN6 DONE" >> results/eval/conf.progress
