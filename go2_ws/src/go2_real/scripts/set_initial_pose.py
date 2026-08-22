#!/usr/bin/env python3
"""
set_initial_pose.py — tell slam_toolbox where the robot is, from a saved waypoint.

Run this once after launching the navigation stack when the robot is NOT at
the map origin (home position).  slam_toolbox will re-localize from the
given waypoint and the two TF frame origins will converge.

Usage:
  python3 set_initial_pose.py <waypoint_name>
  python3 set_initial_pose.py microwave
  python3 set_initial_pose.py sleep area    (multi-word, no quotes needed)
"""
import math
import os
import sys
import time
import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

WAYPOINTS_FILE = os.path.expanduser('~/maps/waypoints.yaml')


def main():
    if len(sys.argv) < 2:
        print('Usage: set_initial_pose.py <waypoint_name>')
        sys.exit(1)

    name = ' '.join(sys.argv[1:])

    if not os.path.exists(WAYPOINTS_FILE):
        print(f'Waypoints file not found: {WAYPOINTS_FILE}')
        sys.exit(1)

    with open(WAYPOINTS_FILE) as f:
        data = yaml.safe_load(f) or {}

    waypoints = data.get('waypoints', {})
    if name not in waypoints:
        print(f"Waypoint '{name}' not found.")
        print(f"Available: {list(waypoints.keys())}")
        sys.exit(1)

    wp = waypoints[name]
    x       = float(wp['x'])
    y       = float(wp['y'])
    yaw_deg = float(wp.get('yaw', 0.0))
    yaw     = math.radians(yaw_deg)

    rclpy.init()
    node = Node('set_initial_pose')
    pub  = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    time.sleep(0.5)   # wait for subscriber connections

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.position.z = 0.0
    msg.pose.pose.orientation.x = 0.0
    msg.pose.pose.orientation.y = 0.0
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    # Moderate uncertainty — scan matching will fine-tune from here
    msg.pose.covariance[0]  = 0.25   # x variance (~0.5 m)
    msg.pose.covariance[7]  = 0.25   # y variance (~0.5 m)
    msg.pose.covariance[35] = 0.07   # yaw variance (~15 deg)

    msg.header.stamp = node.get_clock().now().to_msg()
    pub.publish(msg)
    time.sleep(0.3)
    msg.header.stamp = node.get_clock().now().to_msg()
    pub.publish(msg)   # send twice to ensure slam_toolbox receives it

    print(f"Initial pose set to '{name}':  x={x:.3f}  y={y:.3f}  yaw={yaw_deg:.1f}°")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
