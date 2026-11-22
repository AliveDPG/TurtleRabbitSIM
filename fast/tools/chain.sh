#!/bin/bash
# Milestone 5 head-to-head: arm X vs arm Y, two lanes, lanes swapped every batch of REP flights per lane,
# until each arm has NEED scored flights, CAP seconds, or the stop file results/eval/PREFIX.stop exists
# (checked between batches). Single-lane alternating fallback when free RAM < 5 GB.
# I0=<last batch index> continues numbering. Stops gpt-oss:20b if loaded (owner allowed, milestone 5).
# usage: bash fast/tools/chain.sh PREFIX "ENV_X" LABEL_X "ENV_Y" LABEL_Y NEED REP CAP
cd /c/Users/darpan/Documents/SpaceTeams/TurtleRabbitSIM
P=$1; EX="$2"; LX=$3; EY="$4"; LY=$5; NEED=${6:-40}; REP=${7:-6}; CAP=${8:-18000}
LOG=results/eval/$P.progress
until [ $(tasklist | grep -ci spacecraft) -eq 0 ]; do sleep 10; done
T0=$(date +%s); i=${I0:-0}
count() { python - "$P" "$1" <<'PY'
import json, sys, glob, os
p, arm = sys.argv[1], sys.argv[2]; n = 0
for f in glob.glob("results/eval/%s[0-9]*_*/runs.jsonl" % p):
    b = os.path.basename(os.path.dirname(f))
    a = "results/eval/%s.arm" % b
    if not os.path.exists(a): continue
    if json.load(open(a))["arm"] != arm: continue
    for l in open(f):
        r = json.loads(l)
        if r.get("score") is not None: n += 1
print(n)
PY
}
while [ $(( $(date +%s) - T0 )) -lt $CAP ]; do
  nx=$(count $LX); ny=$(count $LY)
  echo "$(date +%T) $LX=$nx $LY=$ny" >> $LOG
  [ $nx -ge $NEED ] && [ $ny -ge $NEED ] && break
  [ -e results/eval/$P.stop ] && echo "$(date +%T) stop file" >> $LOG && break
  if ollama ps 2>/dev/null | grep -q "gpt-oss:20b"; then ollama stop gpt-oss:20b >/dev/null 2>&1; echo "$(date +%T) stopped gpt-oss:20b (owner-allowed)" >> $LOG; sleep 20; fi
  i=$((i + 1)); mem=$(powershell -NoProfile -Command "[int]((Get-CimInstance Win32_OperatingSystem).FreePhysicalMemory/1024)")
  rob=$(tasklist | grep -i roblox | awk '{s+=$5} END {print NR" procs"}'); oll=$(ollama ps 2>/dev/null | tail -n +2 | awk '{print $1}' | tr '\n' ' ')
  if [ "$mem" -ge 5000 ]; then
    if [ $((i % 2)) -eq 1 ]; then EA=$EX; EB=$EY; AA=$LX; AB=$LY; else EA=$EY; EB=$EX; AA=$LY; AB=$LX; fi
    echo "{\"arm\": \"$AA\"}" > results/eval/$P${i}_a.arm; echo "{\"arm\": \"$AB\"}" > results/eval/$P${i}_b.arm
    echo "$(date +%T) batch $P$i two lanes (a=$AA b=$AB) free ${mem} MB roblox ${rob} ollama [${oll}]" >> $LOG
    bash fast/tools/run_pair.sh $P$i $REP "$EA" "$EB" > results/eval/$P$i.pair.log 2>&1
  else
    echo "{\"arm\": \"$LX\"}" > results/eval/$P${i}_a.arm; echo "{\"arm\": \"$LY\"}" > results/eval/$P${i}_b.arm
    echo "$(date +%T) batch $P$i SINGLE lane (free ${mem} MB) roblox ${rob} ollama [${oll}]" >> $LOG
    bash fast/tools/run_alt.sh $P$i 2 "$EX" "$EY" 3 fast fast > results/eval/$P$i.alt.log 2>&1
  fi
  sleep 10
done
echo "CHAIN DONE $P" >> $LOG
