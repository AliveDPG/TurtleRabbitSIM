#!/bin/bash
# Milestone 5: while the rig flies (a SpaceCRAFT process exists), stop gpt-oss:20b whenever it is loaded (owner-allowed).
# Stops when the file results/eval/ollama_guard.stop exists. Log: results/eval/ollama_guard.log
cd /c/Users/darpan/Documents/SpaceTeams/TurtleRabbitSIM
while [ ! -e results/eval/ollama_guard.stop ]; do
  if ollama ps 2>/dev/null | grep -q "gpt-oss:20b"; then
    ollama stop gpt-oss:20b >/dev/null 2>&1; echo "$(date +%T) stopped gpt-oss:20b" >> results/eval/ollama_guard.log
  fi
  sleep 30
done
