#!/bin/bash
# Usage: bash /tmp/nav_goal.sh X Y [YAW_DEG]
# Example: bash /tmp/nav_goal.sh 0.55 0.55

source /opt/ros/foxy/setup.bash
source /home/alireza/thesis0/navigation_go2/go2_ws/install/setup.bash

X=${1:?"Usage: $0 X Y [YAW_DEG]"}
Y=${2:?"Usage: $0 X Y [YAW_DEG]"}
YAW_DEG=${3:-0}

# Convert yaw degrees to quaternion (rotation around Z)
YAW_RAD=$(echo "$YAW_DEG * 3.14159265358979 / 180.0" | bc -l)
QZ=$(echo "s($YAW_RAD / 2)" | bc -l)
QW=$(echo "c($YAW_RAD / 2)" | bc -l)

echo "Sending goal: x=$X y=$Y yaw=${YAW_DEG}deg"

ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: $X, y: $Y, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: $QZ, w: $QW}}}}" \
  --feedback | tee /tmp/nav_goal_output.txt &

PID=$!
wait $PID

if grep -q "result code: 4" /tmp/nav_goal_output.txt 2>/dev/null || \
   grep -q "status: 4" /tmp/nav_goal_output.txt 2>/dev/null || \
   grep -qi "succeed" /tmp/nav_goal_output.txt 2>/dev/null; then
    echo ""
    echo "SUCCESS — reached goal! ($X, $Y)"
else
    echo ""
    echo "Navigation finished — check RViz2 for result."
fi
