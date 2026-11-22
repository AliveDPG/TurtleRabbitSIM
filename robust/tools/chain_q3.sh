#!/bin/bash
# Milestone 5 resume: G3 kick tests, F3 load curve, D5, D4 finish, F2. Stack and E are started by hand.
cd /c/Users/darpan/Documents/SpaceTeams/TurtleRabbitSIM
R3="RB_WATCH=1 RB_BRAKE=1 RB_NEAR=1 RB_GATE_HOLD=10 RB_GATE_MODE=excess RB_RECOVER=1 RB_RECOVER_DEG=60 RB_RECOVER_BURN_DEG=30 RB_RECOVER_PLAN=1 RB_RECOVER_DVBURN=1 RB_RECOVER_SCORE=1"
R3NOREC="RB_WATCH=1 RB_BRAKE=1 RB_NEAR=1 RB_GATE_HOLD=10 RB_GATE_MODE=excess"
while [ -d results/eval/.queue_lock ] || [ $(tasklist | grep -ci spacecraft) -gt 0 ] || \
      [ $(powershell -NoProfile -Command "(Get-CimInstance Win32_Process | Where-Object { \$_.CommandLine -match 'evaluate.py' }).Count") -gt 0 ]; do sleep 30; done
echo "$(date +%T) rig free, starting" >> results/eval/m5q3.progress
KY="RB_KICK=70 RB_KICK_STRONG=50 RB_KICK_DIR=y"
KH="RB_KICK=50 RB_KICK_STRONG=120 RB_KICK_DIR=y"
bash robust/tools/chain_m5.sh m5g3 "$R3 $KY" krec "$R3NOREC $KY" knorec 3 3 3600
bash robust/tools/chain_m5.sh m5g4 "$R3 $KH" khrec "$R3NOREC $KH" khnorec 3 3 3600
bash robust/tools/chain_m5.sh m5f31 "RB_TICK_SLEEP=50" off50 "$R3 RB_TICK_SLEEP=50" r3_50 6 6 3600
bash robust/tools/chain_m5.sh m5f32 "RB_TICK_SLEEP=150" off150 "$R3 RB_TICK_SLEEP=150" r3_150 6 6 3600
bash robust/tools/chain_m5.sh m5d5 "$R3 RB_BIG_LOOK=35 RB_BIG_LOOK_EDGE=0.6" blook2 "$R3" r3 12 6 7200
I0=2 bash robust/tools/chain_m5.sh m5d4 "$R3 RB_BRAKE_KEEP=1" bkeep "$R3" r3 12 6 5400
echo '{"arm": "sub"}' > results/eval/m5f2_a.arm; echo '{"arm": "off"}' > results/eval/m5f2_b.arm
if ollama ps 2>/dev/null | grep -q "gpt-oss:20b"; then ollama stop gpt-oss:20b >/dev/null 2>&1; fi
bash robust/tools/run_alt.sh m5f2 12 "RB_NONE=1" "RB_NONE=1" 15 submission robust > results/eval/m5f2.alt.log 2>&1
echo "$(date +%T) CHAIN_Q3 DONE" >> results/eval/m5q3.progress
