#!/usr/bin/env python3
"""
Publishes the Go2 initial pose to AMCL ONCE.
Uses sim-time and BEST_EFFORT QoS.
Multiple rapid publications crash the Foxy AMCL particle filter.
Run: python3 set_initial_pose.py
"""
import rclpy
import rclpy.parameter
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from builtin_interfaces.msg import Time
import time


def main():
    rclpy.init()

    node = Node(
        'go2_pose_init',
        parameter_overrides=[
            rclpy.parameter.Parameter(
                'use_sim_time',
                rclpy.parameter.Parameter.Type.BOOL,
                True,
            )
        ],
    )

    qos = QoSProfile(depth=10)
    qos.reliability = ReliabilityPolicy.BEST_EFFORT

    pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', qos)

    # Wait for FastRTPS to discover the AMCL subscriber before publishing
    node.get_logger().info('Waiting 5s for DDS discovery before publishing...')
    end = time.time() + 5.0
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    # stamp=0 → AMCL uses latest available odom transform (avoids sim/wall clock mismatch)
    msg.header.stamp = Time(sec=0, nanosec=0)

    # Go2 spawns at (-0.55, -0.55) facing +x (yaw=0)
    msg.pose.pose.position.x = -0.55
    msg.pose.pose.position.y = -0.55
    msg.pose.pose.position.z = 0.0
    msg.pose.pose.orientation.z = 0.0
    msg.pose.pose.orientation.w = 1.0
    msg.pose.covariance[0]  = 0.25
    msg.pose.covariance[7]  = 0.25
    msg.pose.covariance[35] = 0.068538

    # Publish EXACTLY ONCE — multiple rapid publications crash Foxy AMCL particle filter
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.5)
    node.get_logger().info('Initial pose published once: x=-0.55 y=-0.55 yaw=0')

    # Give AMCL time to process before destroying node
    time.sleep(2.0)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
