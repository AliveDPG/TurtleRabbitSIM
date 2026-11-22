#!/bin/bash
# Milestone 5 D: re-run of bkeep vs r3 (m5d4) after chain_d (the first run, m5d2, flew during gpt-oss reload churn).
cd /c/Users/darpan/Documents/SpaceTeams/TurtleRabbitSIM
R3="RB_WATCH=1 RB_BRAKE=1 RB_NEAR=1 RB_GATE_HOLD=10 RB_GATE_MODE=excess RB_RECOVER=1 RB_RECOVER_DEG=60 RB_RECOVER_BURN_DEG=30 RB_RECOVER_PLAN=1 RB_RECOVER_DVBURN=1 RB_RECOVER_SCORE=1"
until grep -q "CHAIN_D DONE" results/eval/m5d.progress 2>/dev/null; do sleep 30; done
bash robust/tools/chain_m5.sh m5d4 "$R3 RB_BRAKE_KEEP=1" bkeep "$R3" r3 12 6 7200
echo "CHAIN_D2 DONE" >> results/eval/m5d.progress
