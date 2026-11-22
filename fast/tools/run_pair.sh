#!/bin/bash
# Two-lane batch: lane a installs fast/ and flies with env A, lane b flies the same code with env B.
# usage: bash robust/tools/run_pair.sh NAME REPEAT "RB_X=1 RB_Y=0" "RB_X=0" [SEEDS]
cd /c/Users/darpan/Documents/SpaceTeams/TurtleRabbitSIM || exit 1
NAME=$1; REP=$2; EA=$3; EB=$4; SEEDS=${5:-8193}
mkdir results/eval/.queue_lock 2>/dev/null
export ORUN_PARALLEL=1 ORUN_RECORD=0 ORUN_DEBUG_TRUTH=0
ARGS="--cooldown 10 --max-sim-time 400 --unvalidated --no-ground-truth"
tasklist | grep -i roblox > /dev/null && echo "ROBLOX RUNNING at start" > results/eval/${NAME}.note
( env $EA python -u scripts/evaluate.py --config default --seeds $SEEDS --repeat $REP --batch ${NAME}_a --mode offscreen --tag L1 --run-args "--controller-dir fast $ARGS --install-controller" > results/eval/${NAME}_a.out 2>&1 & )
sleep 50
( env $EB python -u scripts/evaluate.py --config default --seeds $SEEDS --repeat $REP --batch ${NAME}_b --mode offscreen --tag L2 --run-args "--controller-dir fast $ARGS" > results/eval/${NAME}_b.out 2>&1 & )
T0=$(date +%s)
while true; do
  na=$(grep -c "score=\|FAILED" results/eval/${NAME}_a.out 2>/dev/null); nb=$(grep -c "score=\|FAILED" results/eval/${NAME}_b.out 2>/dev/null)
  [ "$na" -ge "$REP" ] && [ "$nb" -ge "$REP" ] && break
  [ $(( $(date +%s) - T0 )) -gt $(( REP * 420 + 600 )) ] && echo "TIME CAP" && break
  sleep 20
done
sleep 5
rmdir results/eval/.queue_lock 2>/dev/null
echo "DONE $NAME a=$na b=$nb"
