#!/usr/bin/env python3
"""
obstacle_repulsion.py — Repulsive velocity field for Unitree Go2.

For each LaserScan beam within `influence_dist`, a repulsive vector is computed
pointing away from that obstacle (anti-beam direction), scaled linearly by
penetration depth into the influence zone.  All vectors are summed to produce a
net Cartesian repulsive velocity in the robot frame, then clamped to `max_vel`.

ROS topics
----------
  Subscribes : /scan            (sensor_msgs/LaserScan)
  Publishes  : /repulsion_vel   (geometry_msgs/Twist)
                 — consumed by velocity_combiner, NOT sent directly to robot

Parameters (set via --ros-args -p key:=value)
----------
  influence_dist  float  0.7    Distance [m] at which repulsion starts
  max_vel         float  0.6    Maximum repulsive speed [m/s] — set by launch file
                                as 1.2 × Nav2 desired_linear_vel
  gain            float  1.3    Proportional scaling of computed velocity
  deadband_vel    float  0.03   Min speed [m/s] before sending — suppresses micro-jitter
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

BEST_EFFORT_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)


class ObstacleRepulsionNode(Node):

    def __init__(self):
        super().__init__('obstacle_repulsion')

        self.declare_parameter('influence_dist', 0.7)
        self.declare_parameter('max_vel',        0.6)
        self.declare_parameter('gain',           1.3)
        self.declare_parameter('deadband_vel',   0.03)

        self._d0       = self.get_parameter('influence_dist').value
        self._max_vel  = self.get_parameter('max_vel').value
        self._gain     = self.get_parameter('gain').value
        self._deadband = self.get_parameter('deadband_vel').value

        self._rep_pub = self.create_publisher(Twist, '/repulsion_vel', 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, BEST_EFFORT_QOS)

        self._vx = 0.0
        self._vy = 0.0
        self.create_timer(0.05, self._timer_cb)   # 20 Hz

        self.get_logger().info(
            f'obstacle_repulsion ready  '
            f'influence={self._d0} m  max_vel={self._max_vel:.2f} m/s  gain={self._gain}'
        )

    def _scan_cb(self, msg: LaserScan):
        fx, fy = 0.0, 0.0
        n_active = 0

        for i, dist in enumerate(msg.ranges):
            if math.isnan(dist) or math.isinf(dist):
                continue
            if dist < 0.05 or dist > self._d0:
                continue

            angle = msg.angle_min + i * msg.angle_increment

            w = 1.0 - dist / self._d0   # linear weight: 0 at boundary → 1 at contact

            fx -= w * math.cos(angle)
            fy -= w * math.sin(angle)
            n_active += 1

        if n_active == 0:
            self._vx = 0.0
            self._vy = 0.0
            return

        speed = math.hypot(fx, fy)
        if speed < 1e-9:
            self._vx = 0.0
            self._vy = 0.0
            return

        nx, ny = fx / speed, fy / speed
        avg_w  = speed / n_active
        vel    = min(self._max_vel, self._gain * self._max_vel * avg_w)

        if vel < self._deadband:
            self._vx = 0.0
            self._vy = 0.0
        else:
            self._vx = nx * vel
            self._vy = ny * vel

        self.get_logger().debug(
            f'active_beams={n_active}  avg_w={avg_w:.3f}  '
            f'vel={vel:.3f}  vx={self._vx:.3f}  vy={self._vy:.3f}'
        )

    def _timer_cb(self):
        # Always publish (including zero) so velocity_combiner sees a live source
        msg = Twist()
        msg.linear.x  = self._vx
        msg.linear.y  = self._vy
        msg.angular.z = 0.0
        self._rep_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleRepulsionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
