#!/bin/bash
# Milestone 5: queue after D3 - promote dev (gap between batches), D2 re-run, D5 (big look + edge rule), F3 load curve,
# F2 submission vs robust-off (single lane, different code per flight).
cd /c/Users/darpan/Documents/SpaceTeams/TurtleRabbitSIM
R3="RB_WATCH=1 RB_BRAKE=1 RB_NEAR=1 RB_GATE_HOLD=10 RB_GATE_MODE=excess RB_RECOVER=1 RB_RECOVER_DEG=60 RB_RECOVER_BURN_DEG=30 RB_RECOVER_PLAN=1 RB_RECOVER_DVBURN=1 RB_RECOVER_SCORE=1"
until grep -q "CHAIN_D DONE" results/eval/m5d.progress 2>/dev/null; do sleep 20; done
until [ $(tasklist | grep -ci spacecraft) -eq 0 ]; do sleep 10; done
cp robust/dev/Controller.py robust/dev/guidance.py robust/dev/thrust_law.py robust/
echo "$(date +%T) robust/ <- dev (RB_TICK_SLEEP, RB_BIG_LOOK_EDGE added; defaults off)" >> results/eval/m5d.progress
bash robust/tools/chain_m5.sh m5d4 "$R3 RB_BRAKE_KEEP=1" bkeep "$R3" r3 12 6 7200
bash robust/tools/chain_m5.sh m5d5 "$R3 RB_BIG_LOOK=35 RB_BIG_LOOK_EDGE=0.6" blook2 "$R3" r3 12 6 7200
bash robust/tools/chain_m5.sh m5f31 "RB_TICK_SLEEP=50" off50 "$R3 RB_TICK_SLEEP=50" r3_50 6 6 3600
bash robust/tools/chain_m5.sh m5f32 "RB_TICK_SLEEP=150" off150 "$R3 RB_TICK_SLEEP=150" r3_150 6 6 3600
echo '{"arm": "sub"}' > results/eval/m5f2_a.arm; echo '{"arm": "off"}' > results/eval/m5f2_b.arm
bash robust/tools/run_alt.sh m5f2 12 "RB_NONE=1" "RB_NONE=1" 15 submission robust > results/eval/m5f2.alt.log 2>&1
echo "CHAIN_REST DONE" >> results/eval/m5d.progress
