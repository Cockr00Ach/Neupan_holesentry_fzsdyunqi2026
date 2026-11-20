#!/bin/bash
# Benchmark the NeuPAN planner settings on the "twitch between the two boards" scenario.
#
#   ./web/bench.sh <name> <planner.yaml>
#
# It restarts the simulation with the given planner config, teleports the robot to the
# start point, records /neupan_status while the robot drives to the goal, and prints the
# trajectory metrics (time, travelled distance, reversals, twitch windows, clearance).
set +u
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$HERE" || exit 1
NAME=${1:?usage: bench.sh NAME PLANNER_YAML}
YAML=${2:?usage: bench.sh NAME PLANNER_YAML}
START=${BENCH_START:-"3.66 -2.02"}
GOAL=${BENCH_GOAL:-"-3.64 2.02"}
DURATION=${BENCH_DURATION:-50}

source /opt/ros/humble/setup.bash
source "$(cd "$HERE/../../../.." && pwd)/install/setup.bash"

bash stop_sim.sh >/dev/null 2>&1
sleep 2
bash run_sim.sh --wait --nav-only --planner-config "$YAML" >/dev/null 2>&1
sleep 14

rm -rf "/tmp/run_$NAME"
ros2 bag record -o "/tmp/run_$NAME" /neupan_status >/tmp/bag_$NAME.log 2>&1 &
BAG=$!
sleep 3

python3 test_sim_twitch.py --start $START --goal $GOAL --timeout "$DURATION" \
    --csv "/tmp/twitch_$NAME.csv" > "/tmp/twitch_$NAME.log" 2>&1

kill -INT $BAG 2>/dev/null
sleep 4
kill -9 $BAG 2>/dev/null

echo "===== $NAME ($YAML) ====="
grep -E "arrived=|reversals=" "/tmp/twitch_$NAME.log" | head -4
python3 web/analyse_run.py --bag "/tmp/run_$NAME" --goal $GOAL --start $START 2>&1 | tail -4
