#!/bin/bash
# Milestone 5 D: candidate-vs-round-3 head-to-heads, one after the other, after A (m5a) is done.
cd /c/Users/darpan/Documents/SpaceTeams/TurtleRabbitSIM
R3="RB_WATCH=1 RB_BRAKE=1 RB_NEAR=1 RB_GATE_HOLD=10 RB_GATE_MODE=excess RB_RECOVER=1 RB_RECOVER_DEG=60 RB_RECOVER_BURN_DEG=30 RB_RECOVER_PLAN=1 RB_RECOVER_DVBURN=1 RB_RECOVER_SCORE=1"
until grep -q "CHAIN DONE m5a" results/eval/m5a.progress; do sleep 30; done
bash robust/tools/chain_m5.sh m5d1 "$R3 RB_BIG_MARGIN=8 RB_KICK_GATE=0.1" k406 "$R3" r3 12 6 7200
bash robust/tools/chain_m5.sh m5d2 "$R3 RB_BRAKE_KEEP=1" bkeep "$R3" r3 12 6 7200
bash robust/tools/chain_m5.sh m5d3 "$R3 RB_BIG_LOOK=35" blook "$R3" r3 12 6 7200
echo "CHAIN_D DONE" >> results/eval/m5d.progress
