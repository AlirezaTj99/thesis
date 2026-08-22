#!/usr/bin/env python3
"""
stair_cloud_persist.py — Persistent LiDAR point cloud during stair climbing.

IDLE mode:   passes /cloud_relay through to /stair/persist_cloud unchanged.
Stair mode:  accumulates LiDAR frames (decay after DECAY_SEC) and republishes
             merged cloud so RViz shows the full stair structure even when the
             robot turns and the stair leaves the current LiDAR FOV momentarily.

Subscribes:  /cloud_relay              (PointCloud2, BEST_EFFORT)
             /stair_controller/state   (std_msgs/String)
Publishes:   /stair/persist_cloud      (PointCloud2)
"""
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String

DECAY_SEC = 10.0    # seconds to keep accumulated points
MAX_PTS   = 50000   # cap to prevent huge messages when many frames accumulate


class StairCloudPersist(Node):

    def __init__(self):
        super().__init__('stair_cloud_persist')
        be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(PointCloud2, '/cloud_relay',            self._cloud_cb,  be)
        self.create_subscription(String,      '/stair_controller/state', self._state_cb,  10)
        self._pub         = self.create_publisher(PointCloud2, '/stair/persist_cloud', 1)
        self._state       = 'IDLE'
        self._buf         = []   # [(timestamp, np.ndarray Nx3), ...]
        self._last_header = None
        self.create_timer(0.5, self._publish_merged)
        self.get_logger().info('StairCloudPersist ready — idle pass-through active')

    def _state_cb(self, msg: String):
        prev        = self._state
        self._state = msg.data
        if prev != 'IDLE' and self._state == 'IDLE':
            self._buf.clear()
            self.get_logger().info('Stair IDLE — persistent buffer cleared')

    def _cloud_cb(self, msg: PointCloud2):
        self._last_header = msg.header
        pts = self._parse(msg)
        if pts is None:
            return

        if self._state == 'IDLE':
            self._buf.clear()
            self._pub.publish(msg)   # pass-through unchanged in IDLE
        else:
            self._buf.append((time.time(), pts))

    def _publish_merged(self):
        if self._state == 'IDLE' or not self._buf or self._last_header is None:
            return
        now       = time.time()
        self._buf = [(t, p) for t, p in self._buf if now - t < DECAY_SEC]
        if not self._buf:
            return
        merged = np.concatenate([p for _, p in self._buf], axis=0)
        if len(merged) > MAX_PTS:
            idx    = np.random.choice(len(merged), MAX_PTS, replace=False)
            merged = merged[idx]
        self._pub.publish(self._make_cloud(merged))

    def _parse(self, msg: PointCloud2):
        try:
            fields = {f.name: f.offset for f in msg.fields}
            step   = msg.point_step
            n      = msg.width * msg.height
            if n == 0:
                return None
            raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, step)
            def _f32(name, off):
                o = fields.get(name, off)
                return raw[:, o:o+4].copy().view(np.float32).reshape(-1)
            pts = np.stack([_f32('x', 0), _f32('y', 4), _f32('z', 8)], axis=1)
            return pts[np.isfinite(pts).all(axis=1)]
        except Exception:
            return None

    def _make_cloud(self, pts: np.ndarray) -> PointCloud2:
        n   = len(pts)
        dt  = np.dtype([('x', np.float32), ('y', np.float32), ('z', np.float32)])
        arr = np.zeros(n, dtype=dt)
        arr['x'] = pts[:, 0]
        arr['y'] = pts[:, 1]
        arr['z'] = pts[:, 2]
        msg              = PointCloud2()
        msg.header       = self._last_header
        msg.height       = 1
        msg.width        = n
        msg.is_dense     = False
        msg.is_bigendian = False
        msg.point_step   = 12
        msg.row_step     = 12 * n
        msg.fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        msg.data = arr.tobytes()
        return msg


def main():
    rclpy.init()
    node = StairCloudPersist()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
