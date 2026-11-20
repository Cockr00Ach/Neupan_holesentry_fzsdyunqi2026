#!/bin/bash
# Reproducible acceptance check for the NeuPAN <-> Gazebo pipeline.
#
#   ./check_all.sh          # runs the three scenarios below and stops the simulator
#
# A. start-up route from planner.yaml (spawn (-5,3) -> (5,0))
# B. dynamic obstacle: the blue sentry is parked on the red robot's path, the red
#    robot has to go around it and still reach the goal
# C. single goal across the field, including a stop check after arrival
# D. web view: health endpoint, goal sent over HTTP, stop over HTTP
set +u
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
cd "$HERE" || exit 1

WS=$(cd "$HERE/../../../.." && pwd)
source /opt/ros/humble/setup.bash
if [ -f "$WS/install/setup.bash" ]; then
    source "$WS/install/setup.bash"
fi

FAIL=0

echo "=========== A: start-up route from planner.yaml ==========="
bash stop_sim.sh >/dev/null 2>&1
sleep 2
bash run_sim.sh >/dev/null 2>&1
sleep 28
if grep -q "arrived" /tmp/neupan_run/neupan_bridge.log; then
    echo "PASS: $(grep 'arrived' /tmp/neupan_run/neupan_bridge.log | tail -1)"
else
    echo "FAIL: no arrival in the bridge log"
    FAIL=1
fi

echo
echo "=========== B: dynamic obstacle in the path ==========="
bash stop_sim.sh >/dev/null 2>&1
sleep 2
bash run_sim.sh --wait >/dev/null 2>&1
sleep 10
timeout 170 python3 test_sim_dynamic.py --goal 5.5 -1.5 --blocker-goal 1.5 1.5 \
    --timeout 100 | tail -5 || FAIL=1

echo
echo "=========== C: single goal across the field ==========="
bash stop_sim.sh >/dev/null 2>&1
sleep 2
bash run_sim.sh --wait >/dev/null 2>&1
sleep 10
timeout 130 python3 test_sim_goal.py --goal -3.0 -2.0 0 --timeout 100 \
    --tolerance 0.4 | tail -4 || FAIL=1

echo
echo "=========== D: web view / web control ==========="
bash stop_sim.sh >/dev/null 2>&1
sleep 2
bash run_sim.sh --wait --web >/dev/null 2>&1
sleep 10
HEALTH=$(curl -s --max-time 5 http://127.0.0.1:8088/api/health)
echo "health: $HEALTH"
if echo "$HEALTH" | grep -q '"ok": *true\|"ok":true'; then
    echo "PASS: web server up"
else
    echo "FAIL: web server not reachable"
    FAIL=1
fi
curl -s --max-time 5 -X POST http://127.0.0.1:8088/api/goal     -H 'Content-Type: application/json' -d '{"x": -3.0, "y": -2.0}' >/dev/null
sleep 3
MODE=$(curl -s --max-time 5 http://127.0.0.1:8088/api/state | python3 -c     "import sys,json;print(json.load(sys.stdin)['status']['mode'])" 2>/dev/null)
echo "mode after web goal: $MODE"
if [ "$MODE" = "driving" ]; then
    echo "PASS: goal sent from the web UI is being tracked"
else
    echo "FAIL: the robot did not start driving"
    FAIL=1
fi
sleep 8
curl -s --max-time 5 -X POST http://127.0.0.1:8088/api/stop >/dev/null
sleep 2
MODE2=$(curl -s --max-time 5 http://127.0.0.1:8088/api/state | python3 -c     "import sys,json; print(json.load(sys.stdin)['status']['mode'])" 2>/dev/null)
echo "mode after web stop: $MODE2"
if [ "$MODE2" = "idle" ]; then
    echo "PASS: stop clears the goal"
else
    echo "FAIL: stop did not take effect"
    FAIL=1
fi

echo
bash stop_sim.sh >/dev/null 2>&1
echo "logs kept under /tmp/neupan_run"
exit $FAIL
