#!/bin/bash
# start_go2_real.sh — Start the full real-robot Nav2+SLAM stack for Unitree Go2 Pro.
#
# This script starts EVERYTHING in one launch:
#   - PointCloud2 → LaserScan conversion
#   - Go2 bridge (sportmodestate → /odom + TF, /cmd_vel → Go2 sport API)
#   - Nav2 stack (controller, planner, recoveries, bt_navigator)
#   - AMCL localization + map server
#   - RViz2 with costmap view and goal setting
#
# ── Network mode ─────────────────────────────────────────────────────────────
# "ethernet" : USB-C/RJ45 cable, PC IP 192.168.123.2/24
# "wifi"     : shared hotspot (phone hotspot or router)
#              Steps:
#                1. Connect robot to hotspot via Unitree app (Settings → WiFi → STA mode)
#                2. Connect this PC's WiFi to the same hotspot
#                3. Find robot's IP: nmap -sn 192.168.43.0/24  (adjust subnet)
#                4. Set ROBOT_WIFI_IP below and run this script
CONNECTION_MODE="wifi"
ROBOT_WIFI_IP="130.251.13.140"   # Orin NX WiFi IP (wlan0 on ter-go2-jetson)

SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CDDS_WS=/home/alireza/thesis1/navigation_go2/go2_ws/src/unitree_ros2/cyclonedds_ws
LIVOX_SDK_LIB=/home/alireza/livox-sdk2/lib

# ── Kill any leftover processes ───────────────────────────────────────────────
pkill -9 -f rviz2                   2>/dev/null
pkill -9 -f bt_navigator            2>/dev/null
pkill -9 -f controller_server       2>/dev/null
pkill -9 -f planner_server          2>/dev/null
pkill -9 -f recoveries_server       2>/dev/null
pkill -9 -f lifecycle_manager       2>/dev/null
pkill -9 -f slam_toolbox            2>/dev/null
pkill -9 -f map_server              2>/dev/null
pkill -9 -f amcl                    2>/dev/null
pkill -9 -f map_to_odom_tf         2>/dev/null
pkill -9 -f static_transform_pub   2>/dev/null
pkill -9 -f go2_viz                 2>/dev/null
pkill -9 -f go2_bridge              2>/dev/null
pkill -9 -f obstacle_repulsion     2>/dev/null
pkill -9 -f velocity_combiner      2>/dev/null
pkill -9 -f body_filter            2>/dev/null
pkill -9 -f go2_goal               2>/dev/null
pkill -9 -f robot_state_publisher   2>/dev/null
pkill -9 -f waypoint_follower       2>/dev/null
pkill -9 -f map_saver_server        2>/dev/null
pkill -9 -f pointcloud_to_laserscan 2>/dev/null
pkill -9 -f livox_ros_driver2       2>/dev/null
pkill -9 -f "ros2 launch"           2>/dev/null
pkill -9 -f "ros2 topic"            2>/dev/null
sleep 1

# ── Source environment ────────────────────────────────────────────────────────
source /opt/ros/foxy/setup.bash

# Custom CycloneDDS 0.10.x — required for Go2 DDS compatibility
# (Go2 uses 0.10.x; ROS2 Foxy ships 0.7.x — messages would be incompatible)
if [ -f "$CDDS_WS/install/setup.bash" ]; then
    source "$CDDS_WS/install/setup.bash"
else
    echo "WARNING: cyclonedds_ws not built yet. Run setup_sdk.sh first."
fi

source /home/alireza/thesis1/navigation_go2/go2_ws/install/setup.bash

# ── DDS + network configuration ───────────────────────────────────────────────
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export LD_LIBRARY_PATH="$LIVOX_SDK_LIB:$LD_LIBRARY_PATH"

if [ "$CONNECTION_MODE" = "wifi" ]; then
    NET_IFACE="wlp8s0"
    ROBOT_IP="$ROBOT_WIFI_IP"
    # WiFi AP blocks multicast between clients — unicast-only discovery required
    export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${NET_IFACE}\" priority=\"default\" multicast=\"false\" /></Interfaces></General><Discovery><MaxAutoParticipantIndex>50</MaxAutoParticipantIndex><Peers><Peer Address=\"127.0.0.1\"/><Peer Address=\"192.168.123.18\"/></Peers></Discovery></Domain></CycloneDDS>"
    echo "[NET] WiFi mode — interface=${NET_IFACE}  robot_ip=${ROBOT_IP}"
else
    NET_IFACE="enp9s0"
    ROBOT_IP="192.168.123.161"
    export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${NET_IFACE}\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain></CycloneDDS>"
    echo "[NET] Ethernet mode — interface=${NET_IFACE}  robot_ip=${ROBOT_IP}"
fi

# ── Companion terminals: open each if not already running ─────────────────────
_open_if_dead() {
    local pattern="$1"   # pgrep pattern to detect if already running
    local title="$2"     # terminal window title
    local cmd="$3"       # command to run inside the new terminal
    if ! pgrep -f "$pattern" >/dev/null 2>&1; then
        echo "  [+] Opening: $title"
        if command -v gnome-terminal >/dev/null 2>&1; then
            gnome-terminal --title="$title" -- bash -c "$cmd; exec bash" 2>/dev/null &
        else
            xterm -title "$title" -e "bash -c '$cmd; exec bash'" 2>/dev/null &
        fi
        sleep 0.5
    else
        echo "  [ok] Already running: $title"
    fi
}

echo ""
echo "=== Checking companion terminals ==="
_open_if_dead "monitor_robot.py"    "Robot Monitor"   "bash ${SCRIPTS_DIR}/run_monitor_robot.sh"
_open_if_dead "go2_goal_session.sh" "Go2 Goal"        "bash ${SCRIPTS_DIR}/go2_goal_session.sh"
echo "===================================="
echo ""

cleanup() {
    pkill -f "ros2 topic" 2>/dev/null
    kill "$CLEAR_PID" 2>/dev/null
}
trap cleanup INT TERM

# Periodic costmap clear — wait 28 s for RTAB-Map to localize, then clear
# every 5 s so self-detection marks and TF-drift artifacts never accumulate.
(sleep 28 \
  && while true; do
       ros2 service call /global_costmap/clear_entirely_global_costmap \
           nav2_msgs/srv/ClearEntireCostmap '{}' > /dev/null 2>&1
       ros2 service call /local_costmap/clear_entirely_local_costmap  \
           nav2_msgs/srv/ClearEntireCostmap '{}' > /dev/null 2>&1
       sleep 5
     done) &
CLEAR_PID=$!

ros2 launch go2_real go2_real.launch.py

cleanup
