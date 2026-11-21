#!/bin/bash
# wait until a pair batch log says DONE (or 3 h); usage: wait_done.sh NAME
f=/c/Users/darpan/Documents/SpaceTeams/TurtleRabbitSIM/results/eval/$1.pair.log
for i in $(seq 1 540); do grep -q "DONE\|TIME CAP" $f 2>/dev/null && break; sleep 20; done
cat $f
