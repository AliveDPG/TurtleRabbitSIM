#!/bin/bash
cd /c/Users/darpan/Documents/SpaceTeams/TurtleRabbitSIM
ALL2="RB_WATCH=1 RB_BRAKE=1 RB_NEAR=1 RB_GATE_HOLD=10 RB_GATE_MODE=excess RB_RECOVER=1 RB_RECOVER_DEG=60 RB_RECOVER_BURN_DEG=30 RB_RECOVER_PLAN=1"
KICK="RB_KICK=70 RB_KICK_STRONG=60"
FIX="RB_RECOVER_DVBURN=1 RB_RECOVER_SCORE=1"
bash robust/tools/run_pair.sh kickS1 2 "$ALL2 $KICK $FIX" "$ALL2 $KICK" > results/eval/kickS1.pair.log 2>&1
sleep 10
bash robust/tools/run_pair.sh kickS2 2 "$ALL2 $KICK" "$ALL2 $KICK $FIX" > results/eval/kickS2.pair.log 2>&1
echo CHAIN4 DONE
