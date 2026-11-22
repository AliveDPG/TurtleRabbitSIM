#!/bin/bash
# Milestone 5: wait for the running D4 chain (m5d4) to finish, then D5 continue and F2 (6 per arm).
cd /c/Users/darpan/Documents/SpaceTeams/TurtleRabbitSIM
R3="RB_WATCH=1 RB_BRAKE=1 RB_NEAR=1 RB_GATE_HOLD=10 RB_GATE_MODE=excess RB_RECOVER=1 RB_RECOVER_DEG=60 RB_RECOVER_BURN_DEG=30 RB_RECOVER_PLAN=1 RB_RECOVER_DVBURN=1 RB_RECOVER_SCORE=1"
while tasklist //FI "PID eq 76468" | grep -q bash; do sleep 30; done
until [ $(tasklist | grep -ci spacecraft) -eq 0 ] && [ ! -d results/eval/.queue_lock ]; do sleep 20; done
I0=1 bash robust/tools/chain_m5.sh m5d5 "$R3 RB_BIG_LOOK=35 RB_BIG_LOOK_EDGE=0.6" blook2 "$R3" r3 12 6 7200
echo '{"arm": "sub"}' > results/eval/m5f2_a.arm; echo '{"arm": "off"}' > results/eval/m5f2_b.arm
bash robust/tools/run_alt.sh m5f2 6 "RB_NONE=1" "RB_NONE=1" 9 submission robust > results/eval/m5f2.alt.log 2>&1
echo "$(date +%T) CHAIN_Q5 DONE" >> results/eval/m5q3.progress
