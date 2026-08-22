#!/bin/bash
source /opt/ros/foxy/setup.bash
source /home/alireza/thesis1/navigation_go2/go2_ws/src/unitree_ros2/cyclonedds_ws/install/setup.bash
source /home/alireza/thesis1/navigation_go2/go2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enp9s0" priority="default" multicast="default" /></Interfaces></General></Domain></CycloneDDS>'
python3 /home/alireza/thesis1/navigation_go2/go2_ws/src/go2_real/scripts/go2_goal.py "$@"
