#!/bin/bash
# Reproducible acceptance check for the NeuPAN <-> Gazebo pipeline.
#
#   ./check_all.sh          # runs the three scenarios below and stops the simulator
#
# A. start-up route from planner.yaml (spawn (-5,3) -> (5,0))
# B. dynamic obstacle: the blue sentry is parked on the red robot's path, the red
#    robot has to go around it and still reach the goal
# C. single goal across the field, including a stop check after arrival
set +u
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
cd "$HERE" || exit 1

source /opt/ros/humble/setup.bash
if [ -f /home/baymaxwish/fzsd2026/install/setup.bash ]; then
    source /home/baymaxwish/fzsd2026/install/setup.bash
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
bash stop_sim.sh >/dev/null 2>&1
echo "logs kept under /tmp/neupan_run"
exit $FAIL
