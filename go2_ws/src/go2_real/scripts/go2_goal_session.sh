#!/bin/bash
# go2_goal_session.sh — Pre-configured interactive terminal for sending Nav2 goals.
# Keep this window open. Type goal commands here, then close when done.
#
# Usage (inside this terminal):
#   bash go2_goal.sh <x> <y> [yaw_deg]
#
# Examples:
#   bash go2_goal.sh 1.5 0.0        → 1.5 m forward
#   bash go2_goal.sh 1.5 1.0 90    → 1.5 m forward + 1 m left, face left

SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CDDS_WS=/home/alireza/thesis1/navigation_go2/go2_ws/src/unitree_ros2/cyclonedds_ws

source /opt/ros/foxy/setup.bash
source "$CDDS_WS/install/setup.bash"
source /home/alireza/thesis1/navigation_go2/go2_ws/install/setup.bash

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="wlp8s0" priority="default" multicast="false" /></Interfaces></General><Discovery><MaxAutoParticipantIndex>50</MaxAutoParticipantIndex><Peers><Peer Address="127.0.0.1"/><Peer Address="192.168.123.18"/></Peers></Discovery></Domain></CycloneDDS>'

clear
echo "======================================="
echo "   Go2 Navigation Goal Terminal"
echo "======================================="
echo ""
echo "  By coordinates (map frame):"
echo "    bash $SCRIPTS_DIR/go2_goal.sh <x> <y> [yaw_deg]"
echo "    bash $SCRIPTS_DIR/go2_goal.sh 2.0 0.0"
echo "    bash $SCRIPTS_DIR/go2_goal.sh 2.0 1.0 90"
echo ""
echo "  By named waypoint (~/maps/waypoints.yaml):"
echo "    bash $SCRIPTS_DIR/go2_goal.sh home"
echo "    bash $SCRIPTS_DIR/go2_goal.sh node1"
echo ""
echo "  Save current position as waypoint:"
echo "    bash $SCRIPTS_DIR/save_waypoint.sh sleep area"
echo "    bash $SCRIPTS_DIR/save_waypoint.sh home"
echo ""
echo "  Edit waypoints manually:  nano ~/maps/waypoints.yaml"
echo "======================================="
echo ""

exec bash
