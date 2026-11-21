#!/bin/bash
cd /c/Users/darpan/Documents/SpaceTeams/TurtleRabbitSIM
for i in $(seq 1 400); do grep -q "DONE\|TIME CAP" results/eval/dly25.pair.log 2>/dev/null && break; sleep 20; done
sleep 15
ALL="RB_WATCH=1 RB_BRAKE=1 RB_NEAR=1 RB_GATE_HOLD=10 RB_RECOVER=1"
bash robust/tools/run_pair.sh fin1 6 "$ALL" "RB_WATCH=0" > results/eval/fin1.pair.log 2>&1
sleep 10
bash robust/tools/run_pair.sh fin2 6 "RB_WATCH=0" "$ALL" > results/eval/fin2.pair.log 2>&1
echo CHAIN2 DONE
