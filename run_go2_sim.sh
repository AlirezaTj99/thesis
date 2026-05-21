#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# Run Go2 Nav2 simulation
# Usage: bash run_go2_sim.sh
# ─────────────────────────────────────────────────────────────────────────────

# Kill any stale simulation processes first
echo "[go2_sim] Killing stale ROS2 / Gazebo processes..."
pkill -f "gzserver|gzclient|gz server|gz client|ros2|rviz2" 2>/dev/null || true
sleep 2

# ── Environment ────────────────────────────────────────────────────────────
source /usr/share/gazebo-11/setup.sh
source /opt/ros/foxy/setup.bash
source /home/alireza/thesis0/navigation_go2/go2_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=42
export DISPLAY=:0
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
export GALLIUM_DRIVER=llvmpipe
export GAZEBO_MODEL_PATH=/opt/ros/foxy/share/turtlebot3_gazebo/models:/usr/share/gazebo-11/models

echo ""
echo "[go2_sim] Starting Go2 simulation..."
echo "[go2_sim] - Gazebo world : turtlebot3_world (environment only)"
echo "[go2_sim] - Robot model  : Unitree Go2 Pro (simplified diff-drive)"
echo "[go2_sim] - Nav2 stack   : AMCL + MPPI + NavFn"
echo ""
echo "[go2_sim] After everything loads (~10s), run in a new terminal:"
echo "  source /opt/ros/foxy/setup.bash"
echo "  source /home/alireza/thesis0/navigation_go2/go2_ws/install/setup.bash"
echo "  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=42"
echo "  python3 /home/alireza/thesis0/navigation_go2/go2_ws/src/go2_gazebo/launch/set_initial_pose.py"
echo ""

ros2 launch go2_gazebo go2_sim.launch.py
