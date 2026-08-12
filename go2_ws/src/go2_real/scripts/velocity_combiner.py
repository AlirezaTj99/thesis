#!/usr/bin/env python3
"""
velocity_combiner.py — Vector-adds all velocity sources and sends one combined Twist.

Subscribes:
  /cmd_vel        (geometry_msgs/Twist) — Nav2 navigation commands   (~5 Hz)
  /repulsion_vel  (geometry_msgs/Twist) — obstacle repulsion         (20 Hz)

Publishes:
  /cmd_vel_combined (geometry_msgs/Twist) — vector sum, at 20 Hz → consumed by go2_bridge

Each source is zeroed if no message has arrived within STALE_TIMEOUT seconds,
so the robot stops safely if any source goes down.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

STALE_TIMEOUT = 0.5   # seconds — zero a source if it goes silent


class VelocityCombiner(Node):

    def __init__(self):
        super().__init__('velocity_combiner')

        self._nav2_vel    = Twist()
        self._nav2_stamp  = None
        self._rep_vel     = Twist()
        self._rep_stamp   = None

        self.create_subscription(Twist, '/cmd_vel',       self._nav2_cb, 10)
        self.create_subscription(Twist, '/repulsion_vel', self._rep_cb,  10)
        self._pub = self.create_publisher(Twist, '/cmd_vel_combined', 10)

        self.create_timer(0.05, self._publish)   # 20 Hz

        self.get_logger().info('velocity_combiner started — combining /cmd_vel + /repulsion_vel')

    def _nav2_cb(self, msg: Twist):
        self._nav2_vel   = msg
        self._nav2_stamp = self.get_clock().now()

    def _rep_cb(self, msg: Twist):
        self._rep_vel   = msg
        self._rep_stamp = self.get_clock().now()

    def _publish(self):
        now = self.get_clock().now()

        nav2_vx = nav2_vy = nav2_vyaw = 0.0
        if self._nav2_stamp is not None:
            age = (now - self._nav2_stamp).nanoseconds / 1e9
            if age < STALE_TIMEOUT:
                nav2_vx   = self._nav2_vel.linear.x
                nav2_vy   = self._nav2_vel.linear.y
                nav2_vyaw = self._nav2_vel.angular.z

        rep_vx = rep_vy = 0.0
        if self._rep_stamp is not None:
            age = (now - self._rep_stamp).nanoseconds / 1e9
            if age < STALE_TIMEOUT:
                rep_vx = self._rep_vel.linear.x
                rep_vy = self._rep_vel.linear.y

        combined = Twist()
        combined.linear.x  = nav2_vx  + rep_vx
        combined.linear.y  = nav2_vy  + rep_vy
        combined.angular.z = nav2_vyaw            # repulsion has no yaw component

        self._pub.publish(combined)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityCombiner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
