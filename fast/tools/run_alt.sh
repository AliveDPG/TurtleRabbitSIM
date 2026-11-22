#!/bin/bash
# Single lane, arms alternate flight by flight, each flight installs fast/ first.
# usage: bash fast/tools/run_alt.sh NAME SCORES_PER_ARM "ENV_A" "ENV_B" [MAX_ATTEMPTS_PER_ARM] [DIR_A] [DIR_B]
# order a,b per round; from the round after either arm has K/2 scores the order is b,a (swap halfway)
cd /c/Users/darpan/Documents/SpaceTeams/TurtleRabbitSIM || exit 1
NAME=$1; K=$2; EA=$3; EB=$4; MAXA=${5:-$((K + 4))}; DA=${6:-fast}; DB=${7:-fast}; ORDER="a b"
export ORUN_RECORD=0 ORUN_DEBUG_TRUTH=0
ARGS="--cooldown 10 --max-sim-time 400 --unvalidated --no-ground-truth --install-controller"
na=0; nb=0; ta=0; tb=0
while { [ $na -lt $K ] || [ $nb -lt $K ]; } && { [ $ta -lt $MAXA ] || [ $tb -lt $MAXA ]; }; do
  { [ $na -ge $((K / 2)) ] || [ $nb -ge $((K / 2)) ]; } && ORDER="b a"
  for arm in $ORDER; do
    if [ $arm = a ]; then E=$EA; D=$DA; n=$na; tr=$ta; else E=$EB; D=$DB; n=$nb; tr=$tb; fi
    [ $n -ge $K ] && continue; [ $tr -ge $MAXA ] && continue
    mem=$(powershell -NoProfile -Command "[int]((Get-CimInstance Win32_OperatingSystem).FreePhysicalMemory/1024)")
    while [ "$mem" -lt 4000 ]; do echo "$(date +%T) low memory ${mem} MB, waiting" >> results/eval/${NAME}.load; sleep 60; mem=$(powershell -NoProfile -Command "[int]((Get-CimInstance Win32_OperatingSystem).FreePhysicalMemory/1024)"); done
    echo "$(date +%T) arm $arm dir $D free ${mem} MB roblox=$(tasklist | grep -ci roblox)" >> results/eval/${NAME}.load
    env $E python -u scripts/evaluate.py --config default --seeds 8193 --repeat 1 --batch ${NAME}_$arm --mode offscreen --tag L1 --run-args "--controller-dir $D $ARGS" >> results/eval/${NAME}_$arm.out 2>&1
    c=$(grep -c "score=" results/eval/${NAME}_$arm.out)
    if [ $arm = a ]; then na=$c; ta=$((ta + 1)); else nb=$c; tb=$((tb + 1)); fi
    sleep 5
  done
done
echo "DONE $NAME a=$na/$ta b=$nb/$tb"
