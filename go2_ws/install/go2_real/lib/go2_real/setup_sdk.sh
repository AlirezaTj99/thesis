#!/bin/bash
# setup_sdk.sh — One-time build of the real Go2 Pro ROS2 SDK stack.
#
# Run this once. After it completes, use start_go2_real.sh to launch.
#
# What this does:
#   1. Builds Livox-SDK2 (C library for Livox Mid360 LiDAR)
#   2. Builds CycloneDDS 0.10.x (required for Go2 DDS compatibility)
#   3. Builds unitree ROS2 message packages (unitree_go, unitree_api)
#   4. Builds livox_ros_driver2, pointcloud_to_laserscan, go2_real

set -e

WS=/home/alireza/thesis1/navigation_go2/go2_ws
CDDS_WS="$WS/src/unitree_ros2/cyclonedds_ws"
LIVOX_SDK_SRC=/tmp/Livox-SDK2
LIVOX_SDK_INSTALL=/home/alireza/livox-sdk2

echo "======================================================================"
echo "  Step 1: Build Livox-SDK2 (no sudo — local install to ~/livox-sdk2)"
echo "======================================================================"
if [ ! -d "$LIVOX_SDK_SRC" ]; then
    git clone https://github.com/Livox-SDK/Livox-SDK2.git "$LIVOX_SDK_SRC"
fi
mkdir -p "$LIVOX_SDK_SRC/build"
cd "$LIVOX_SDK_SRC/build"
cmake .. -DCMAKE_INSTALL_PREFIX="$LIVOX_SDK_INSTALL"
make -j$(nproc)
cmake --install . --prefix "$LIVOX_SDK_INSTALL"
echo "  Livox-SDK2 installed to $LIVOX_SDK_INSTALL"

echo "======================================================================"
echo "  Step 2: Build CycloneDDS 0.10.x (no ROS2 in environment)"
echo "======================================================================"
cd "$CDDS_WS/src"
if [ ! -d "cyclonedds" ]; then
    git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
fi
if [ ! -d "rmw_cyclonedds" ]; then
    git clone https://github.com/ros2/rmw_cyclonedds -b foxy
fi
cd "$CDDS_WS"
# Build cyclone-dds WITHOUT ROS2 sourced (colcon picks up cmake directly)
env -i HOME="$HOME" PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    bash -c "cd $CDDS_WS && colcon build --packages-select cyclonedds"

echo "======================================================================"
echo "  Step 3: Build rmw_cyclonedds_cpp (with ROS2 + custom cyclone sourced)"
echo "======================================================================"
source /opt/ros/foxy/setup.bash
source "$CDDS_WS/install/setup.bash"
cd "$CDDS_WS"
colcon build --packages-select rmw_cyclonedds_cpp 2>&1 | tail -5
echo "  rmw_cyclonedds_cpp built"

echo "======================================================================"
echo "  Step 4: Build main workspace (livox_ros_driver2, go2_real, etc.)"
echo "======================================================================"
source /opt/ros/foxy/setup.bash
source "$CDDS_WS/install/setup.bash"
export LD_LIBRARY_PATH="$LIVOX_SDK_INSTALL/lib:$LD_LIBRARY_PATH"
cd "$WS"
colcon build --packages-select \
    unitree_go unitree_api unitree_hg \
    livox_ros_driver2 pointcloud_to_laserscan \
    go2_description go2_gazebo go2_real \
    --cmake-args -DROS_EDITION=ROS2 \
    2>&1 | tail -20

echo "======================================================================"
echo "  Setup complete!"
echo "======================================================================"
echo ""
echo "Before first run:"
echo "  1. Set your Ethernet interface in start_go2_real.sh (uncomment CYCLONEDDS_URI line)"
echo "     Run: ip link show"
echo "  2. Set PC IP: sudo ip addr add 192.168.123.2/24 dev <interface>"
echo "  3. Edit MID360_config.json with your LiDAR IP (default: usually 192.168.1.1xx)"
echo "     File: $WS/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json"
echo ""
echo "Launch:"
echo "  bash $WS/src/go2_real/scripts/start_go2_real.sh"
