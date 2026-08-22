#!/bin/bash
# set_initial_pose.sh — Tell slam_toolbox where the robot is (from a saved waypoint).
#
# Run this once after launching the navigation stack when the robot is NOT
# at its home/starting position.  The two map-frame origins will then converge.
#
# Usage:
#   set_initial_pose.sh <waypoint_name>
#   set_initial_pose.sh microwave
#   set_initial_pose.sh sleep area      (multi-word, no quotes needed)

SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CDDS_WS=/home/alireza/thesis1/navigation_go2/go2_ws/src/unitree_ros2/cyclonedds_ws

source /opt/ros/foxy/setup.bash
source "$CDDS_WS/install/setup.bash"
source /home/alireza/thesis1/navigation_go2/go2_ws/install/setup.bash

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enp9s0" priority="default" multicast="default" /></Interfaces></General></Domain></CycloneDDS>'

python3 "$SCRIPTS_DIR/set_initial_pose.py" "$@"
