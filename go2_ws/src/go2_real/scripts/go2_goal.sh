#!/bin/bash
# go2_goal.sh — Send a navigation goal to the Go2.
#
# Goals are in the MAP frame (same as RViz 2D Nav Goal).
# You can also use named waypoints defined in ~/maps/waypoints.yaml.
#
# Usage:
#   go2_goal.sh <x> <y>              go to map coordinates (x, y)
#   go2_goal.sh <x> <y> <yaw_deg>   go to (x, y) and face yaw_deg
#   go2_goal.sh <name>               go to named waypoint
#
# Examples:
#   go2_goal.sh 2.0 0.0        → (2.0, 0.0) in map frame
#   go2_goal.sh 2.0 1.0 90     → (2.0, 1.0), face map +y direction
#   go2_goal.sh home            → named waypoint 'home'
#   go2_goal.sh node1           → named waypoint 'node1'
#
# Edit waypoints:  nano ~/maps/waypoints.yaml

SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CDDS_WS=/home/alireza/thesis1/navigation_go2/go2_ws/src/unitree_ros2/cyclonedds_ws

source /opt/ros/foxy/setup.bash
source "$CDDS_WS/install/setup.bash"
source /home/alireza/thesis1/navigation_go2/go2_ws/install/setup.bash

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="wlp8s0" priority="default" multicast="false" /></Interfaces></General><Discovery><MaxAutoParticipantIndex>50</MaxAutoParticipantIndex><Peers><Peer Address="127.0.0.1"/><Peer Address="192.168.123.18"/></Peers></Discovery></Domain></CycloneDDS>'

python3 "$SCRIPTS_DIR/go2_goal.py" "$@"
