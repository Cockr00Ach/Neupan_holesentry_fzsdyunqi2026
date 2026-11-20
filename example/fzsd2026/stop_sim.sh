#!/bin/bash
# Stop everything started by run_sim.sh (Gazebo, robots, bridges, RViz).
set +u

KILL_PATTERNS=('neupan_ros2_bridge' 'rviz2' 'ign gazebo' 'rmua19_robot_base'
               'ros_gz_bridge/parameter_bridge' 'rmu_gazebo_simulator/spawn_robots'
               'robot_state_publisher' 'pose_bridge' 'rfid_bridge' 'simple_competition')

for pat in "${KILL_PATTERNS[@]}"; do
    for pid in $(pgrep -f "$pat" 2>/dev/null); do
        [ "$pid" = "$$" ] && continue
        cmd=$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null) || continue
        case "$cmd" in
            *run_sim.sh*|*stop_sim.sh*) continue ;;
        esac
        kill -9 "$pid" 2>/dev/null
    done
done
sleep 1
ros2 daemon stop >/dev/null 2>&1
ros2 daemon start >/dev/null 2>&1
left=$(pgrep -fc 'ign gazebo' 2>/dev/null || echo 0)
echo "stopped. remaining gazebo processes: $left"
