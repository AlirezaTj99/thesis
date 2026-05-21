#!/usr/bin/env python3
"""
Removes phantom self-detection returns in the robot's front arc.

The Gazebo ray sensor produces spurious returns following r ≈ 0.5/cos(θ) for
|θ| up to ~70° — a flat-surface pattern at ~0.5 m ahead of the laser that does
not correspond to any real obstacle.

Filter rule per ray:
  if |θ| < 80°  AND  range < (0.5/|cos θ|) + 0.35 m  →  set to inf

The +0.35 m margin absorbs noise while the 80° arc cap keeps the rear 200° of
the scan entirely unfiltered for obstacle detection.
Real arena walls are 2.45 m+ away in every direction, so no real return is
ever within the phantom threshold at any angle in this arc.
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

ARC_RAD   = math.radians(80.0)   # filter only rays in ±80° front arc
MARGIN    = 0.35                  # metres above the phantom pattern to still suppress
MAX_FILT  = 2.2                   # absolute ceiling so we never filter real walls


class ScanFilterNode(Node):
    def __init__(self):
        super().__init__('scan_filter')
        self._pub  = self.create_publisher(LaserScan, '/scan_filtered', 10)
        self._sub  = self.create_subscription(LaserScan, '/scan', self._cb, 10)
        self._threshold: np.ndarray | None = None

    def _build_threshold(self, msg: LaserScan):
        n = len(msg.ranges)
        angles = (msg.angle_min
                  + np.arange(n, dtype=np.float64) * msg.angle_increment)
        cos_abs = np.abs(np.cos(angles))
        cos_abs = np.where(cos_abs < 1e-6, 1e-6, cos_abs)   # avoid /0 near ±90°
        phantom  = 0.5 / cos_abs + MARGIN
        thresh   = np.minimum(phantom, MAX_FILT).astype(np.float32)
        in_arc   = np.abs(angles) < ARC_RAD
        self._threshold = np.where(in_arc, thresh, 0.0).astype(np.float32)

    def _cb(self, msg: LaserScan):
        n = len(msg.ranges)
        if self._threshold is None or len(self._threshold) != n:
            self._build_threshold(msg)

        ranges = np.array(msg.ranges, dtype=np.float32)
        mask = (self._threshold > 0) & (ranges < self._threshold)
        ranges[mask] = math.inf

        out = LaserScan()
        out.header          = msg.header
        out.angle_min       = msg.angle_min
        out.angle_max       = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment  = msg.time_increment
        out.scan_time       = msg.scan_time
        out.range_min       = msg.range_min
        out.range_max       = msg.range_max
        out.ranges          = ranges.tolist()
        out.intensities     = msg.intensities
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ScanFilterNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
