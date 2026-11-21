#!/bin/bash
cd /c/Users/darpan/Documents/SpaceTeams/TurtleRabbitSIM
until grep -q "CHAIN4 DONE\|DONE kickS2" results/eval/kickS2.pair.log 2>/dev/null; do sleep 20; done
sleep 10; until [ $(tasklist | grep -ci spacecraft) -eq 0 ]; do sleep 10; done
cp robust/dev/*.py robust/
ALL2="RB_WATCH=1 RB_BRAKE=1 RB_NEAR=1 RB_GATE_HOLD=10 RB_GATE_MODE=excess RB_RECOVER=1 RB_RECOVER_DEG=60 RB_RECOVER_BURN_DEG=30 RB_RECOVER_PLAN=1"
KICK="RB_KICK=70 RB_KICK_STRONG=50 RB_KICK_DIR=y"
FIX="RB_RECOVER_DVBURN=1 RB_RECOVER_SCORE=1"
bash robust/tools/run_pair.sh kickY1 2 "$ALL2 $KICK $FIX" "$ALL2 $KICK" > results/eval/kickY1.pair.log 2>&1
sleep 10
bash robust/tools/run_pair.sh kickY2 2 "$ALL2 $KICK" "$ALL2 $KICK $FIX" > results/eval/kickY2.pair.log 2>&1
echo CHAIN5 DONE
