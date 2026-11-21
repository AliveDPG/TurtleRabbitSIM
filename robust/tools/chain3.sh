#!/bin/bash
cd /c/Users/darpan/Documents/SpaceTeams/TurtleRabbitSIM
while true; do a=$(grep -c "score=" results/eval/kick3_a.out); b=$(grep -c "score=" results/eval/kick3_b.out); ta=$(grep -c "score=\|FAILED" results/eval/kick3_a.out); tb=$(grep -c "score=\|FAILED" results/eval/kick3_b.out); { [ $a -ge 3 ] || [ $ta -ge 4 ]; } && { [ $b -ge 3 ] || [ $tb -ge 4 ]; } && [ $(tasklist | grep -ci spacecraft) -eq 0 ] && break; sleep 30; done
sleep 10
ALL2="RB_WATCH=1 RB_BRAKE=1 RB_NEAR=1 RB_GATE_HOLD=10 RB_GATE_MODE=excess RB_RECOVER=1 RB_RECOVER_DEG=60 RB_RECOVER_BURN_DEG=30 RB_RECOVER_PLAN=1"
bash robust/tools/run_alt.sh m2fin 10 "RB_NONE=1" "$ALL2" 14 submission robust > results/eval/m2fin.alt.log 2>&1
echo CHAIN3 DONE
