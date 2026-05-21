#!/bin/bash
SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Kill ALL leftover processes from any previous run (use -9 to guarantee cleanup)
pkill -9 -f gzserver          2>/dev/null
pkill -9 -f gzclient          2>/dev/null
pkill -9 -f rviz2             2>/dev/null
pkill -9 -f bt_navigator      2>/dev/null
pkill -9 -f controller_server 2>/dev/null
pkill -9 -f planner_server    2>/dev/null
pkill -9 -f recoveries_server 2>/dev/null
pkill -9 -f lifecycle_manager 2>/dev/null
pkill -9 -f slam_toolbox      2>/dev/null
pkill -9 -f go2_viz           2>/dev/null
pkill -9 -f scan_filter       2>/dev/null
pkill -9 -f robot_state_publisher 2>/dev/null
pkill -9 -f waypoint_follower 2>/dev/null
pkill -9 -f map_saver_server  2>/dev/null
pkill -9 -f "ros2 launch"     2>/dev/null
sleep 1

# Reset ROS2 daemon to clear ghost DDS participants from previous runs
source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 daemon stop 2>/dev/null
sleep 1

# FastRTPS reads its profile from /tmp at runtime — copy it there each launch
cp "$SCRIPTS_DIR/fastrtps_udp_only.xml" /tmp/fastrtps_udp_only.xml

source /opt/ros/foxy/setup.bash
source /home/alireza/thesis0/navigation_go2/go2_ws/install/setup.bash

export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastrtps_udp_only.xml

ros2 launch go2_gazebo go2_sim.launch.py
