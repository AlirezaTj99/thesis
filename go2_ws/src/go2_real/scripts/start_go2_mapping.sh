#!/bin/bash
# start_go2_mapping.sh — Build a 2D map of the environment using Go2 LiDAR.
#
# Usage:
#   ./start_go2_mapping.sh
#
# Procedure:
#   1. Run this script — the robot will stand up and SLAM Toolbox starts.
#   2. Drive the robot around the room using the physical Unitree controller.
#   3. Press Ctrl+C — the map is saved automatically to ~/maps/
#
# Output files:
#   ~/maps/go2_map_YYYYMMDD_HHMMSS.pgm   — occupancy grid image
#   ~/maps/go2_map_YYYYMMDD_HHMMSS.yaml  — map metadata (resolution, origin)

# ── Network mode ─────────────────────────────────────────────────────────────
# "ethernet" : USB-C/RJ45 cable  |  "wifi" : shared hotspot
CONNECTION_MODE="wifi"
ROBOT_WIFI_IP="130.251.13.140"   # Orin NX WiFi IP (wlan0 on ter-go2-jetson)

CDDS_WS=/home/alireza/thesis1/navigation_go2/go2_ws/src/unitree_ros2/cyclonedds_ws
LIVOX_SDK_LIB=/home/alireza/livox-sdk2/lib
MAP_DIR="$HOME/maps"

# ── Kill any leftover processes ────────────────────────────────────────────────
pkill -9 -f rviz2                   2>/dev/null
pkill -9 -f slam_toolbox            2>/dev/null
pkill -9 -f go2_bridge              2>/dev/null
pkill -9 -f pointcloud_to_laserscan 2>/dev/null
pkill -9 -f robot_state_publisher   2>/dev/null
pkill -9 -f "ros2 launch"           2>/dev/null
pkill -9 -f "ros2 topic"            2>/dev/null
sleep 1

# ── Source environment ─────────────────────────────────────────────────────────
source /opt/ros/foxy/setup.bash

if [ -f "$CDDS_WS/install/setup.bash" ]; then
    source "$CDDS_WS/install/setup.bash"
else
    echo "WARNING: cyclonedds_ws not built yet. Run setup_sdk.sh first."
fi

source /home/alireza/thesis1/navigation_go2/go2_ws/install/setup.bash

# ── DDS + network configuration ────────────────────────────────────────────────
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export LD_LIBRARY_PATH="$LIVOX_SDK_LIB:$LD_LIBRARY_PATH"

if [ "$CONNECTION_MODE" = "wifi" ]; then
    NET_IFACE="wlp8s0"
    ROBOT_IP="$ROBOT_WIFI_IP"
    # WiFi AP blocks multicast between clients — unicast-only discovery required
    export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${NET_IFACE}\" priority=\"default\" multicast=\"false\" /></Interfaces></General><Discovery><Peers><Peer Address=\"192.168.123.18\"/></Peers></Discovery></Domain></CycloneDDS>"
    echo "[NET] WiFi mode — interface=${NET_IFACE}  robot_ip=${ROBOT_IP}"
else
    NET_IFACE="enp9s0"
    ROBOT_IP="192.168.123.161"
    export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${NET_IFACE}\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain></CycloneDDS>"
    echo "[NET] Ethernet mode — interface=${NET_IFACE}  robot_ip=${ROBOT_IP}"
fi

mkdir -p "$MAP_DIR"

SLAM_GRAPH="${MAP_DIR}/go2_slam_graph"

# ── Cleanup: save map + serialize pose graph, then shut down ──────────────────
cleanup() {
    echo ""
    echo "=== Saving map via map_saver_cli... ==="
    MAP_NAME="go2_map_$(date '+%Y%m%d_%H%M%S')"
    mkdir -p "$MAP_DIR"
    # Save occupancy grid BEFORE killing launch — slam_toolbox must still be running.
    ros2 run nav2_map_server map_saver_cli \
        -f "${MAP_DIR}/${MAP_NAME}" \
        --ros-args -p save_map_timeout:=15000 2>&1

    echo ""
    echo "=== Serializing pose graph for next session... ==="
    # Serialize the pose graph so next run can continue from here (lifelong mapping).
    ros2 service call /slam_toolbox/serialize_map \
        slam_toolbox/srv/SerializePoseGraph \
        "{filename: '${SLAM_GRAPH}'}" 2>&1
    echo "=== Pose graph saved to ${SLAM_GRAPH}.posegraph ==="

    echo ""
    echo "=== Shutting down launch... ==="
    kill -INT -- "-$LAUNCH_PID" 2>/dev/null
    wait "$LAUNCH_PID" 2>/dev/null
    echo "=== Done. Map: ${MAP_DIR}/${MAP_NAME}.pgm ==="
    pkill -f "ros2 topic" 2>/dev/null
}

trap cleanup INT TERM

# ── Launch: always start fresh ────────────────────────────────────────────────
# Lifelong mapping (auto-loading pose graph) is disabled — initial localization
# is unreliable unless the robot starts at the exact same spot and orientation
# it was in when the previous session ended. Map the full room in one session.
echo "=== Starting fresh mapping session ==="
setsid ros2 launch go2_real go2_mapping.launch.py &
LAUNCH_PID=$!
echo "$LAUNCH_PID" > /tmp/go2_mapping_launch.pid

echo ""
echo "========================================="
echo " Go2 Mapping Session"
echo "========================================="
echo " Drive the robot with the physical controller."
echo " Watch the map grow in RViz."
echo " Press Ctrl+C when done — map saves automatically."
echo "========================================="
echo ""

wait "$LAUNCH_PID"
