#!/bin/bash
# Kill ALL simulation-related processes before restart
# This fixes the duplicate node issue (6x accumulation)

echo "Cleaning up simulation processes..."

# Kill Gazebo
pkill -9 -f "gz sim" 2>/dev/null
pkill -9 -f "gzserver" 2>/dev/null
pkill -9 -f "gzclient" 2>/dev/null

# Kill all ros_gz bridges (accumulated from previous launches)
pkill -9 -f "parameter_bridge" 2>/dev/null

# Kill robot base controllers
pkill -9 -f "rmua19_robot_base" 2>/dev/null

# Kill robot state publishers
pkill -9 -f "robot_state_publisher" 2>/dev/null

# Kill referee system
pkill -9 -f "referee_system" 2>/dev/null
pkill -9 -f "simple_competition" 2>/dev/null
pkill -9 -f "pose_bridge" 2>/dev/null
pkill -9 -f "rfid_bridge" 2>/dev/null

# Kill our bridge and RViz
pkill -9 -f "neupan_ros2_bridge" 2>/dev/null
pkill -9 -f "rviz2" 2>/dev/null

# Restart ROS2 daemon to clear stale registrations
ros2 daemon stop 2>/dev/null
ros2 daemon start 2>/dev/null

sleep 1
echo "Done. Remaining simulation processes:"
ps aux | grep -E "parameter_bridge|rmua19|robot_state_pub|referee_system|gz sim" | grep -v grep | wc -l
