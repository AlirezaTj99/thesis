#!/bin/bash
# go2_goal.sh — Send a navigation goal to the Go2.
#
# Goals are RELATIVE to the robot's current position and heading.
#   x > 0 = forward,  x < 0 = backward
#   y > 0 = left,     y < 0 = right
#   yaw   = final heading offset in degrees (0 = keep current heading)
#
# Usage:
#   go2_goal.sh <x> <y>            move (x, y) relative to current position
#   go2_goal.sh <x> <y> <yaw_deg>  move and face yaw_deg offset from current
#
# Examples:
#   go2_goal.sh 1.5 0.0       → 1.5 m forward from here
#   go2_goal.sh 1.5 1.0 90    → 1.5 m forward + 1.0 m left, then face left

SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CDDS_WS=/home/alireza/thesis1/navigation_go2/go2_ws/src/unitree_ros2/cyclonedds_ws

source /opt/ros/foxy/setup.bash
source "$CDDS_WS/install/setup.bash"
source /home/alireza/thesis1/navigation_go2/go2_ws/install/setup.bash

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enp9s0" priority="default" multicast="default" /></Interfaces></General></Domain></CycloneDDS>'

python3 "$SCRIPTS_DIR/go2_goal.py" "$@"
