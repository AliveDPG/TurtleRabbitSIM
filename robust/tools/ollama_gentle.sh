#!/bin/bash
# Milestone 5: stop gpt-oss:20b (owner-allowed) only when a rig runner is waiting for memory (its .load file's last
# line says "low memory" within the last 90 s), at most once per 5 min. Stop file results/eval/ollama_gentle.stop.
cd /c/Users/darpan/Documents/SpaceTeams/TurtleRabbitSIM
last=0
while [ ! -e results/eval/ollama_gentle.stop ]; do
  f=$(ls -t results/eval/*.load 2>/dev/null | head -1)
  if [ -n "$f" ] && tail -1 "$f" | grep -q "low memory" && [ $(( $(date +%s) - $(stat -c %Y "$f") )) -lt 90 ] && [ $(( $(date +%s) - last )) -gt 300 ]; then
    if ollama ps 2>/dev/null | grep -q "gpt-oss:20b"; then
      ollama stop gpt-oss:20b >/dev/null 2>&1; last=$(date +%s); echo "$(date +%T) stopped gpt-oss:20b (runner waiting: $f)" >> results/eval/ollama_guard.log
    fi
  fi
  sleep 20
done
