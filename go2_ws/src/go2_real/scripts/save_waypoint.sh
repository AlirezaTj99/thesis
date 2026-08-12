#!/bin/bash
# save_waypoint.sh — Save the robot's current map-frame position as a named waypoint.
#
# The name is saved to ~/maps/waypoints.yaml and can then be used with go2_goal.sh.
#
# Usage:
#   save_waypoint.sh <name>
#   save_waypoint.sh sleep area     (multi-word: no quotes needed)
#
# Examples:
#   save_waypoint.sh home
#   save_waypoint.sh sleep area
#   save_waypoint.sh charging_dock

SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CDDS_WS=/home/alireza/thesis1/navigation_go2/go2_ws/src/unitree_ros2/cyclonedds_ws

source /opt/ros/foxy/setup.bash
source "$CDDS_WS/install/setup.bash"
source /home/alireza/thesis1/navigation_go2/go2_ws/install/setup.bash

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enp9s0" priority="default" multicast="default" /></Interfaces></General></Domain></CycloneDDS>'

python3 "$SCRIPTS_DIR/save_waypoint.py" "$@"
