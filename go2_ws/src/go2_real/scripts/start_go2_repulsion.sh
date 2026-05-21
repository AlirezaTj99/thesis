#!/bin/bash
# start_go2_repulsion.sh — Start obstacle-repulsion mode (no Nav2/SLAM).
#
# Launches only what is needed for repulsion:
#   1. robot_state_publisher  (static TFs for the URDF)
#   2. go2_bridge             (odom TF + relay /utlidar/* → RELIABLE topics)
#   3. pointcloud_to_laserscan (PointCloud2 → /scan)
#   4. obstacle_repulsion      (computes and publishes repulsive velocity)
#
# The robot will automatically move away from anything that gets within
# INFLUENCE_DIST metres (default 1.5 m).
#
# Usage:
#   ./start_go2_repulsion.sh
# Optional param overrides (append after --):
#   ./start_go2_repulsion.sh -- --ros-args -p influence_dist:=1.0 -p max_vel:=0.8

CDDS_WS=/home/alireza/thesis1/navigation_go2/go2_ws/src/unitree_ros2/cyclonedds_ws
GO2_WS=/home/alireza/thesis1/navigation_go2/go2_ws
GO2_DESC_WS="$GO2_WS"
ETH_IFACE="enp9s0"

# ── Kill any leftover ROS2 processes ─────────────────────────────────────────
echo "Killing leftover ROS2 processes..."
pkill -9 -f rviz2                   2>/dev/null
pkill -9 -f bt_navigator            2>/dev/null
pkill -9 -f controller_server       2>/dev/null
pkill -9 -f planner_server          2>/dev/null
pkill -9 -f recoveries_server       2>/dev/null
pkill -9 -f lifecycle_manager       2>/dev/null
pkill -9 -f slam_toolbox            2>/dev/null
pkill -9 -f go2_viz                 2>/dev/null
pkill -9 -f go2_bridge              2>/dev/null
pkill -9 -f obstacle_repulsion      2>/dev/null
pkill -9 -f robot_state_publisher   2>/dev/null
pkill -9 -f pointcloud_to_laserscan 2>/dev/null
sleep 1
ros2 daemon stop 2>/dev/null; ros2 daemon start 2>/dev/null; sleep 0.5

# ── Source environment ────────────────────────────────────────────────────────
source /opt/ros/foxy/setup.bash
source "$CDDS_WS/install/setup.bash"
source "$GO2_WS/install/setup.bash"

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${ETH_IFACE}\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain></CycloneDDS>"

# Check robot connectivity
echo "Checking robot link on ${ETH_IFACE}..."
if ! ip link show "${ETH_IFACE}" | grep -q "state UP"; then
    echo "ERROR: ${ETH_IFACE} is not UP. Is the Ethernet cable connected?"
    exit 1
fi
echo "OK — starting nodes."

# ── URDF for robot_state_publisher ───────────────────────────────────────────
URDF_XACRO="$GO2_WS/install/go2_description/share/go2_description/urdf/go2_real.urdf.xacro"
if [ ! -f "$URDF_XACRO" ]; then
    echo "ERROR: URDF not found at $URDF_XACRO (did you build the workspace?)"
    exit 1
fi
ROBOT_DESC=$(xacro "$URDF_XACRO")

# ── 1. robot_state_publisher ─────────────────────────────────────────────────
ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -p robot_description:="$ROBOT_DESC" \
               -p use_sim_time:=false &
RSP_PID=$!
echo "robot_state_publisher PID=$RSP_PID"
sleep 0.5

# ── 2. go2_bridge ─────────────────────────────────────────────────────────────
ros2 run go2_real go2_bridge.py &
BRIDGE_PID=$!
echo "go2_bridge PID=$BRIDGE_PID"
sleep 1.0   # wait for TF to become available

# ── 3. pointcloud_to_laserscan ───────────────────────────────────────────────
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
    --ros-args \
        --remap cloud_in:=/cloud_relay \
        --remap scan:=/scan \
        -p use_sim_time:=false \
        -p target_frame:=base_link \
        -p transform_tolerance:=0.1 \
        -p min_height:=-0.10 \
        -p max_height:=0.50 \
        -p angle_min:=-3.14159265 \
        -p angle_max:=3.14159265 \
        -p angle_increment:=0.01745329 \
        -p scan_time:=0.1 \
        -p range_min:=0.30 \
        -p range_max:=30.0 \
        -p use_inf:=true &
PC2LS_PID=$!
echo "pointcloud_to_laserscan PID=$PC2LS_PID"
sleep 0.5

# ── 4. obstacle_repulsion ────────────────────────────────────────────────────
# Parse optional extra args (everything after --)
EXTRA_ARGS=""
if [[ "$*" == *"--"* ]]; then
    EXTRA_ARGS="${*##*-- }"
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo " Obstacle Repulsion Mode"
echo "  influence_dist : 0.5 m  (change: -p influence_dist:=X)"
echo "  max_vel        : 0.5 m/s (Go2 phone max ~1.5 m/s; change: -p max_vel:=X)"
echo "  gain           : 1.0    (change: -p gain:=X)"
echo " Walk toward the robot — it will move away from you."
echo " Press Ctrl+C to stop."
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

ros2 run go2_real obstacle_repulsion.py $EXTRA_ARGS

# ── Cleanup on Ctrl+C ────────────────────────────────────────────────────────
echo "Shutting down..."
kill $RSP_PID $BRIDGE_PID $PC2LS_PID 2>/dev/null
wait $RSP_PID $BRIDGE_PID $PC2LS_PID 2>/dev/null
echo "Done."
