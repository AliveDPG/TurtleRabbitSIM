#!/bin/bash
cd /c/Users/darpan/Documents/SpaceTeams/TurtleRabbitSIM
for i in $(seq 1 200); do grep -q "DONE\|TIME CAP" results/eval/eq2.pair.log 2>/dev/null && break; sleep 20; done
sleep 15
cp robust/dev/*.py robust/
bash robust/tools/run_pair.sh dly6 3 "RB_DELAY=6 RB_WATCH=1" "RB_DELAY=6 RB_WATCH=0" > results/eval/dly6.pair.log 2>&1
sleep 10
bash robust/tools/run_pair.sh dly25 3 "RB_DELAY=2.5 RB_WATCH=1" "RB_DELAY=2.5 RB_WATCH=0" > results/eval/dly25.pair.log 2>&1
echo CHAIN1 DONE
