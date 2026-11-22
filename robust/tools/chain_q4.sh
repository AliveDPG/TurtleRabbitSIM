#!/bin/bash
# Milestone 5: D5 continue, D4 finish, F2 (6 per arm). (chain_q3 stopped 22:20: an `ollama ps | grep` pipe hung
# after the ollama CLI restarted the server; chain_m5.sh now writes ollama ps to a file with a timeout.)
cd /c/Users/darpan/Documents/SpaceTeams/TurtleRabbitSIM
R3="RB_WATCH=1 RB_BRAKE=1 RB_NEAR=1 RB_GATE_HOLD=10 RB_GATE_MODE=excess RB_RECOVER=1 RB_RECOVER_DEG=60 RB_RECOVER_BURN_DEG=30 RB_RECOVER_PLAN=1 RB_RECOVER_DVBURN=1 RB_RECOVER_SCORE=1"
until [ $(tasklist | grep -ci spacecraft) -eq 0 ] && [ ! -d results/eval/.queue_lock ]; do sleep 20; done
I0=1 bash robust/tools/chain_m5.sh m5d5 "$R3 RB_BIG_LOOK=35 RB_BIG_LOOK_EDGE=0.6" blook2 "$R3" r3 12 6 7200
I0=2 bash robust/tools/chain_m5.sh m5d4 "$R3 RB_BRAKE_KEEP=1" bkeep "$R3" r3 12 6 5400
echo '{"arm": "sub"}' > results/eval/m5f2_a.arm; echo '{"arm": "off"}' > results/eval/m5f2_b.arm
bash robust/tools/run_alt.sh m5f2 6 "RB_NONE=1" "RB_NONE=1" 9 submission robust > results/eval/m5f2.alt.log 2>&1
echo "$(date +%T) CHAIN_Q4 DONE" >> results/eval/m5q3.progress
