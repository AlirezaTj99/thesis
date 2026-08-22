#!/bin/bash
# 3D mapping session — RTAB-Map + Go2 LiDAR
# Drive the robot around all floors, then Ctrl+C to save the map.
# Map is saved to ~/maps/rtabmap.db automatically.

source /opt/ros/foxy/setup.bash
source /home/alireza/thesis1/navigation_go2/go2_ws/src/unitree_ros2/cyclonedds_ws/install/setup.bash
source /home/alireza/thesis1/navigation_go2/go2_ws/install/setup.bash

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export LD_LIBRARY_PATH=/home/alireza/livox-sdk2/lib:$LD_LIBRARY_PATH
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"wlp8s0\" priority=\"default\" multicast=\"false\" /></Interfaces></General><Discovery><MaxAutoParticipantIndex>50</MaxAutoParticipantIndex><Peers><Peer Address=\"127.0.0.1\"/><Peer Address=\"192.168.123.18\"/></Peers></Discovery></Domain></CycloneDDS>"

# Kill any leftover navigation processes
pkill -9 -f "slam_toolbox"       2>/dev/null
pkill -9 -f "go2_real.launch.py" 2>/dev/null
sleep 1

echo ""
echo "  Starting 3D mapping with RTAB-Map..."
echo "  Drive the robot around all floors, then Ctrl+C when done."
echo "  Map will be saved to ~/maps/rtabmap.db"
echo ""

RESUME=${1:-false}
ros2 launch go2_real go2_rtabmap_mapping.launch.py resume:=$RESUME
