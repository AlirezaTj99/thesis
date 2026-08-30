#!/usr/bin/env python3
"""
auto_initial_pose.py — publish /initialpose to RTAB-Map at stack startup.

Called from go2_real.launch.py with a 9 s delay (after RTAB-Map is ready).
Reads ~/maps/robot_initial_pose.yaml, which go2_control.sh writes when the
user selects the robot's starting location.  If the file is absent, falls
back to (0, 0, 0°) so the stack still starts cleanly.

Publishing /initialpose tells RTAB-Map exactly which area of the map to
search for matching nodes, giving fast and accurate initial localization
instead of waiting for a lucky loop closure.
"""
import math
import os
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

try:
    import yaml
    _YAML_OK = True
except ImportError:
    _YAML_OK = False

POSE_FILE = os.path.expanduser('~/maps/robot_initial_pose.yaml')


class AutoInitialPose(Node):

    def __init__(self):
        super().__init__('auto_initial_pose')
        self._pub  = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        # Fire once after a short spin to let publishers connect
        self.create_timer(0.5, self._run)

    def _run(self):
        x, y, yaw_deg, name = 0.0, 0.0, 0.0, 'home'

        if _YAML_OK and os.path.exists(POSE_FILE):
            try:
                with open(POSE_FILE) as f:
                    data = yaml.safe_load(f) or {}
                x       = float(data.get('x',       0.0))
                y       = float(data.get('y',       0.0))
                yaw_deg = float(data.get('yaw_deg', 0.0))
                name    = data.get('name', 'home')
            except Exception as e:
                self.get_logger().warn(f'Could not read {POSE_FILE}: {e} — using (0, 0, 0°)')
        else:
            self.get_logger().warn(f'{POSE_FILE} not found — using (0, 0, 0°)')

        yaw = math.radians(yaw_deg)

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        # Moderate uncertainty — RTAB-Map ICP scan matching will refine from here
        msg.pose.covariance[0]  = 0.25   # x  (~0.5 m)
        msg.pose.covariance[7]  = 0.25   # y  (~0.5 m)
        msg.pose.covariance[35] = 0.07   # yaw (~15°)

        msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(msg)
        self.get_logger().info(
            f'Initial pose → name={name!r}  x={x:.3f}  y={y:.3f}  yaw={yaw_deg:.1f}°'
        )

        # Second publish 300 ms later for reliability
        time.sleep(0.3)
        msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(msg)

        raise SystemExit(0)


def main():
    rclpy.init()
    node = AutoInitialPose()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
