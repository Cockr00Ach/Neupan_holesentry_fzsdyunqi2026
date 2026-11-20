#!/bin/bash
# One-command bring-up for the NeuPAN fzsd2026 sentry simulation.
#
#   ./run_sim.sh                      # headless Gazebo + robots + NeuPAN bridge
#   ./run_sim.sh --rviz               # ... and RViz for visualization
#   ./run_sim.sh --goal 6 -6 0        # drive to a specific goal at start-up
#   ./run_sim.sh --gui                # also open the Gazebo GUI (viewer only)
#   ./run_sim.sh --wait               # do not follow planner.yaml, wait for /goal_pose
#
# Notes
#  * Gazebo runs in server-only mode (-s): with the GUI attached the Sensors system
#    of this Gazebo 6 build does not publish the lidar/camera topics.
#  * The chassis is driven through <robot_ns>/robot_base/chassis_cmd; nothing else
#    (nav2, the referee) is started by this script.
set +u

HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WS=$(cd "$HERE/../../../.." && pwd)

WORLD=rmul_2025
ROBOT=red_standard_robot1
GOAL=""
START_RVIZ=0
START_GUI=0
START_BRIDGE=1
START_REFEREE=0
WAIT_FOR_GOAL=0

while [ $# -gt 0 ]; do
    case "$1" in
        --world) WORLD="$2"; shift 2 ;;
        --robot) ROBOT="$2"; shift 2 ;;
        --goal) GOAL="$2 $3 $4"; shift 4 ;;
        --rviz) START_RVIZ=1; shift ;;
        --gui) START_GUI=1; shift ;;
        --referee) START_REFEREE=1; shift ;;
        --wait) WAIT_FOR_GOAL=1; shift ;;
        --no-bridge) START_BRIDGE=0; shift ;;
        -h|--help) sed -n '2,16p' "$0"; exit 0 ;;
        *) echo "unknown option: $1"; exit 2 ;;
    esac
done

source /opt/ros/humble/setup.bash
if [ -f "$WS/install/setup.bash" ]; then
    source "$WS/install/setup.bash"
else
    echo "error: $WS/install/setup.bash not found - build the workspace first" >&2
    exit 1
fi

SIM_RES=$(ros2 pkg prefix rmu_gazebo_simulator)/share/rmu_gazebo_simulator/resource
ROBOT_MODELS=$(ros2 pkg prefix fzsd2026_robot_description)/share/fzsd2026_robot_description/resource/models
export IGN_GAZEBO_RESOURCE_PATH="$SIM_RES/models:$ROBOT_MODELS:${IGN_GAZEBO_RESOURCE_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$SIM_RES/models:$ROBOT_MODELS:${GZ_SIM_RESOURCE_PATH:-}"

WORLD_SDF="$SIM_RES/worlds/${WORLD}_world.sdf"
if [ ! -f "$WORLD_SDF" ]; then
    echo "error: world file not found: $WORLD_SDF" >&2
    exit 1
fi

LOG_DIR=${NEUPAN_LOG_DIR:-/tmp/neupan_run}
mkdir -p "$LOG_DIR"
echo "logs: $LOG_DIR"

# ---------------------------------------------------------------- stale procs
# Kill leftovers from a previous run. The helper never touches this script, its
# parent shell, or any process whose command line mentions it (e.g. an editor).
KILL_PATTERNS=('ign gazebo' 'rmua19_robot_base' 'ros_gz_bridge/parameter_bridge'
               'rmu_gazebo_simulator/spawn_robots' 'robot_state_publisher'
               'neupan_ros2_bridge' 'pose_bridge' 'rfid_bridge' 'simple_competition')
kill_leftovers() {
    local self_pid=$1
    local pid cmd pat
    for pat in "${KILL_PATTERNS[@]}"; do
        for pid in $(pgrep -f "$pat" 2>/dev/null); do
            [ "$pid" = "$self_pid" ] && continue
            cmd=$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null) || continue
            case "$cmd" in
                *run_sim.sh*|*stop_sim.sh*) continue ;;
            esac
            kill -9 "$pid" 2>/dev/null
        done
    done
}
kill_leftovers $$
sleep 1

# ---------------------------------------------------------------- gazebo
echo "[1/4] starting Gazebo ($WORLD, headless server)"
nohup ign gazebo -v 2 -r -s "$WORLD_SDF" > "$LOG_DIR/gazebo.log" 2>&1 &
GAZEBO_PID=$!
for _ in $(seq 1 30); do
    if timeout 3 ign service -l 2>/dev/null | grep -q "world/default"; then break; fi
    sleep 1
done

if [ "$START_GUI" = 1 ]; then
    echo "      starting the Gazebo GUI viewer"
    nohup ign gazebo -g > "$LOG_DIR/gazebo_gui.log" 2>&1 &
fi

# ---------------------------------------------------------------- robots
echo "[2/4] spawning robots"
nohup ros2 launch rmu_gazebo_simulator spawn_robots.launch.py \
    > "$LOG_DIR/spawn.log" 2>&1 &
for _ in $(seq 1 30); do
    if timeout 3 ros2 topic list 2>/dev/null | grep -q "$ROBOT/chassis_odometry_gt"; then break; fi
    sleep 1
done

nohup ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
    > "$LOG_DIR/clock_bridge.log" 2>&1 &

if [ "$START_REFEREE" = 1 ]; then
    nohup ros2 launch rmu_gazebo_simulator referee_system.launch.py \
        > "$LOG_DIR/referee.log" 2>&1 &
fi

# ---------------------------------------------------------------- bridge
if [ "$START_BRIDGE" = 1 ]; then
    echo "[3/4] starting the NeuPAN bridge"
    EXTRA_ARGS=""
    if [ -n "$GOAL" ]; then
        EXTRA_ARGS="-p startup_goal:=[$GOAL]"
    fi
    if [ "$WAIT_FOR_GOAL" = 1 ]; then
        EXTRA_ARGS="$EXTRA_ARGS -p use_startup_waypoints:=false"
    fi
    nohup python3 "$HERE/neupan_ros2_bridge.py" --ros-args \
        --params-file "$HERE/bridge_params.yaml" $EXTRA_ARGS \
        > "$LOG_DIR/neupan_bridge.log" 2>&1 &
    sleep 3
fi

# ---------------------------------------------------------------- rviz
if [ "$START_RVIZ" = 1 ]; then
    echo "[4/4] starting RViz"
    nohup rviz2 -d "$HERE/neupan.rviz" > "$LOG_DIR/rviz.log" 2>&1 &
else
    echo "[4/4] RViz not requested (use --rviz)"
fi

sleep 2
echo
echo "running:"
echo "  gazebo  pid $GAZEBO_PID   ($LOG_DIR/gazebo.log)"
echo "  topics : /$ROBOT/chassis_odometry_gt /$ROBOT/livox/lidar"
echo "  command: /$ROBOT/robot_base/chassis_cmd"
echo "  goal   : ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \\"
echo "             \"{header: {frame_id: map}, pose: {position: {x: 5.0, y: 0.0}}}\""
echo
echo "stop everything: $(dirname "$0")/stop_sim.sh"
