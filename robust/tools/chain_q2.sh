#!/bin/bash
# Milestone 5: after m5d4 - D5 (big look + edge), F3 load curve, G3 kick tests. Stack, E and F2 are started by hand.
cd /c/Users/darpan/Documents/SpaceTeams/TurtleRabbitSIM
R3="RB_WATCH=1 RB_BRAKE=1 RB_NEAR=1 RB_GATE_HOLD=10 RB_GATE_MODE=excess RB_RECOVER=1 RB_RECOVER_DEG=60 RB_RECOVER_BURN_DEG=30 RB_RECOVER_PLAN=1 RB_RECOVER_DVBURN=1 RB_RECOVER_SCORE=1"
R3NOREC="RB_WATCH=1 RB_BRAKE=1 RB_NEAR=1 RB_GATE_HOLD=10 RB_GATE_MODE=excess"
until grep -q "CHAIN DONE m5d4" results/eval/m5d4.progress 2>/dev/null; do sleep 20; done
bash robust/tools/chain_m5.sh m5d5 "$R3 RB_BIG_LOOK=35 RB_BIG_LOOK_EDGE=0.6" blook2 "$R3" r3 12 6 7200
bash robust/tools/chain_m5.sh m5f31 "RB_TICK_SLEEP=50" off50 "$R3 RB_TICK_SLEEP=50" r3_50 6 6 3600
bash robust/tools/chain_m5.sh m5f32 "RB_TICK_SLEEP=150" off150 "$R3 RB_TICK_SLEEP=150" r3_150 6 6 3600
bash robust/tools/chain_m5.sh m5g3 "$R3 RB_KICK=70 RB_KICK_STRONG=50 RB_KICK_DIR=y" krec "$R3NOREC RB_KICK=70 RB_KICK_STRONG=50 RB_KICK_DIR=y" knorec 3 3 3600
echo "CHAIN_Q2 DONE" >> results/eval/m5d.progress
