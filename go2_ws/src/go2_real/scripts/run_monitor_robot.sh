#!/bin/bash
source /opt/ros/foxy/setup.bash
source /home/alireza/thesis1/navigation_go2/go2_ws/src/unitree_ros2/cyclonedds_ws/install/setup.bash
source /home/alireza/thesis1/navigation_go2/go2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="wlp8s0" priority="default" multicast="false" /></Interfaces></General><Discovery><MaxAutoParticipantIndex>50</MaxAutoParticipantIndex><Peers><Peer Address="127.0.0.1"/><Peer Address="192.168.123.18"/></Peers></Discovery></Domain></CycloneDDS>'
exec python3 -u /home/alireza/thesis1/navigation_go2/go2_ws/src/go2_real/scripts/monitor_robot.py
