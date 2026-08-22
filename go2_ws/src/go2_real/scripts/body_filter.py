#!/usr/bin/env python3
"""
body_filter.py — removes LiDAR returns from the Go2's own body / legs.

How it works
------------
/cloud_relay is published by the Go2 SDK with frame_id='utlidar_lidar' but the
XYZ data is already in base_link coordinates.  Points that land inside the body
bounding box are discarded (robot sees itself).  Everything outside the box is
published to /cloud_filtered with frame_id='utlidar_lidar' → costmap.

SLAM / ICP odometry / pointcloud_to_laserscan still subscribe to /cloud_relay
(the unfiltered cloud) so their quality is not affected.

Body bounding box in base_link frame (covers full leg reach during trot gait):
    X : -0.55 … +0.55 m  (±body length 0.31 m + leg swing margin)
    Y : -0.35 … +0.35 m  (±body width  0.14 m + leg swing margin)
    Z : -0.30 … +0.20 m  (ground contact level to above LiDAR)
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

# Bounding box in base_link frame — points INSIDE are robot self-returns.
_X_MIN, _X_MAX = -0.55,  0.55
_Y_MIN, _Y_MAX = -0.35,  0.35
_Z_MIN, _Z_MAX = -0.30,  0.20


def _filter(msg: PointCloud2) -> PointCloud2:
    fields = {f.name: f for f in msg.fields}
    step   = msg.point_step
    n      = msg.width * msg.height
    if n == 0:
        return msg

    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, step)

    x_off, y_off, z_off = fields['x'].offset, fields['y'].offset, fields['z'].offset
    xs = raw[:, x_off:x_off+4].copy().ravel().view('<f4')
    ys = raw[:, y_off:y_off+4].copy().ravel().view('<f4')
    zs = raw[:, z_off:z_off+4].copy().ravel().view('<f4')

    # Data is already in base_link frame — use coordinates directly.
    bx = xs
    by = ys
    bz = zs

    # Keep points that fall OUTSIDE the body box
    keep = ~(
        (bx >= _X_MIN) & (bx <= _X_MAX) &
        (by >= _Y_MIN) & (by <= _Y_MAX) &
        (bz >= _Z_MIN) & (bz <= _Z_MAX)
    )

    filtered = raw[keep]

    out = PointCloud2()
    out.header           = msg.header
    out.height      = 1
    out.width       = int(keep.sum())
    out.fields      = msg.fields
    out.is_bigendian = msg.is_bigendian
    out.point_step  = step
    out.row_step    = step * out.width
    out.data        = filtered.tobytes()
    out.is_dense    = msg.is_dense
    return out


class BodyFilter(Node):
    def __init__(self):
        super().__init__('body_filter')
        self.sub = self.create_subscription(
            PointCloud2, '/cloud_relay', self._cb, 10)
        self.pub = self.create_publisher(
            PointCloud2, '/cloud_filtered', 10)
        self.get_logger().info(
            f'body_filter ready  box X[{_X_MIN},{_X_MAX}] '
            f'Y[{_Y_MIN},{_Y_MAX}] Z[{_Z_MIN},{_Z_MAX}] m (base_link frame)')

    def _cb(self, msg: PointCloud2):
        self.pub.publish(_filter(msg))


def main():
    rclpy.init()
    node = BodyFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
