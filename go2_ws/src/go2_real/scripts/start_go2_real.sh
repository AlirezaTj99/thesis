#!/bin/bash
# start_go2_real.sh — Start the full real-robot Nav2+SLAM stack for Unitree Go2 Pro.
#
# This script starts EVERYTHING in one launch:
#   - Livox MID360 driver (PointCloud2)
#   - PointCloud2 → LaserScan conversion
#   - Go2 bridge (sportmodestate → /odom + TF, /cmd_vel → Go2 sport API)
#   - Nav2 + SLAM toolbox
#   - RViz2 with costmap view and goal setting
#
# Prerequisites:
#   1. Run setup_sdk.sh once (installs Livox-SDK2 + builds all packages)
#   2. Connect PC to Go2 via Ethernet, set PC IP to 192.168.123.2
#      sudo ip addr add 192.168.123.2/24 dev eth0   (update 'eth0' to your interface)
#   3. Set the LiDAR IP in MID360_config.json (default Go2 LiDAR IP: 192.168.1.1xx)
#   4. Find your Ethernet interface name with: ip link show

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
pkill -9 -f go2_viz                 2>/dev/null
pkill -9 -f go2_bridge              2>/dev/null
pkill -9 -f obstacle_repulsion     2>/dev/null
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

# Add Livox SDK shared library to runtime linker path
export LD_LIBRARY_PATH="$LIVOX_SDK_LIB:$LD_LIBRARY_PATH"

ETH_IFACE="enp9s0"
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${ETH_IFACE}\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain></CycloneDDS>"

cleanup() {
    pkill -f "ros2 topic" 2>/dev/null
}
trap cleanup INT TERM

ros2 launch go2_real go2_real.launch.py

cleanup
