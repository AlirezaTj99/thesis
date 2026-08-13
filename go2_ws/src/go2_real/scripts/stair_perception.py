#!/usr/bin/env python3
"""
stair_perception.py — Geometric stair detection from 3D LiDAR point cloud.

Pipeline:  ROI crop → voxel downsample → ground estimation →
           height-histogram tread detection → step regularity check → geometry output

Subscribes:  /cloud_relay       (PointCloud2, BEST_EFFORT)
Publishes:   /stair/detected    (Bool)
             /stair/geometry    (String — JSON with stair parameters)
             /stair/cloud_viz   (PointCloud2 — height-coloured ROI for RViz)
             /stair/markers     (MarkerArray — arrow + text label in RViz)
"""
import json
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, String
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Duration

# ── Go2 physical limits ───────────────────────────────────────────────────────
MIN_STEP_H  = 0.05    # m  — below = ramp or noise
MAX_STEP_H  = 0.25    # m  — Go2 maximum obstacle height
MIN_TREAD_D = 0.12    # m  — minimum tread depth
MIN_WIDTH   = 0.70    # m  — Go2 body 354 mm + safety margin
MAX_SLOPE   = 45.0    # deg
MIN_STEPS   = 2       # need at least 2 treads to confirm staircase
MIN_CONF    = 0.50    # confidence threshold for publishing

# ── Region of interest (robot base_link frame) ────────────────────────────────
ROI_X = (0.20, 4.00)   # forward
ROI_Y = (-1.50, 1.50)  # lateral
ROI_Z = (-0.50, 2.00)  # vertical

VOXEL_LEAF  = 0.05    # m — downsample resolution
SKIP_FRAMES = 4       # process every Nth frame (20 Hz cloud → ~5 Hz detection)


class StairPerception(Node):

    def __init__(self):
        super().__init__('stair_perception')

        be_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(PointCloud2, '/cloud_relay', self._cloud_cb, be_qos)

        self._det_pub    = self.create_publisher(Bool,        '/stair/detected',  10)
        self._geo_pub    = self.create_publisher(String,      '/stair/geometry',  10)
        self._cloud_pub  = self.create_publisher(PointCloud2, '/stair/cloud_viz',  1)
        self._marker_pub = self.create_publisher(MarkerArray, '/stair/markers',   10)

        self._frame = 0
        self.get_logger().info('StairPerception ready — listening on /cloud_relay')

    # ── Entry point ───────────────────────────────────────────────────────────

    def _cloud_cb(self, msg: PointCloud2):
        self._frame += 1
        if self._frame % SKIP_FRAMES != 0:
            return

        pts = self._parse_cloud(msg)
        if pts is None or len(pts) < 80:
            self._det_pub.publish(Bool(data=False))
            return

        self._publish_viz_cloud(pts, msg.header)

        result = self._detect_stairs(pts)
        detected = result is not None
        self._det_pub.publish(Bool(data=detected))
        if detected:
            self._geo_pub.publish(String(data=json.dumps(result)))
            self._publish_markers(result, msg.header)

    # ── Point cloud parsing ───────────────────────────────────────────────────

    def _parse_cloud(self, msg: PointCloud2):
        """Parse XYZ from PointCloud2 using numpy byte views (no external deps)."""
        try:
            fields  = {f.name: f.offset for f in msg.fields}
            step    = msg.point_step
            n       = msg.width * msg.height
            if n == 0:
                return None
            raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, step)

            def _f32(name, default_off):
                off = fields.get(name, default_off)
                return raw[:, off:off + 4].copy().view(np.float32).reshape(-1)

            pts = np.stack([_f32('x', 0), _f32('y', 4), _f32('z', 8)], axis=1)
            pts = pts[np.isfinite(pts).all(axis=1)]

            mask = (
                (pts[:, 0] >= ROI_X[0]) & (pts[:, 0] <= ROI_X[1]) &
                (pts[:, 1] >= ROI_Y[0]) & (pts[:, 1] <= ROI_Y[1]) &
                (pts[:, 2] >= ROI_Z[0]) & (pts[:, 2] <= ROI_Z[1])
            )
            return pts[mask]
        except Exception as e:
            self.get_logger().warn(f'Cloud parse error: {e}', throttle_duration_sec=5.0)
            return None

    # ── Detection pipeline ────────────────────────────────────────────────────

    def _detect_stairs(self, pts):
        pts      = self._voxel_downsample(pts, VOXEL_LEAF)
        ground_z = self._estimate_ground(pts)
        treads   = self._find_treads(pts, ground_z)
        if len(treads) < MIN_STEPS:
            return None
        return self._fit_model(treads, ground_z)

    def _voxel_downsample(self, pts, leaf):
        if len(pts) == 0:
            return pts
        origin = pts.min(axis=0)
        vox    = np.floor((pts - origin) / leaf).astype(np.int32)
        _, ui  = np.unique(vox, axis=0, return_index=True)
        return pts[ui]

    def _estimate_ground(self, pts):
        z   = pts[:, 2]
        low = z[z <= np.percentile(z, 15)]
        return float(np.median(low)) if len(low) else float(z.min())

    def _find_treads(self, pts, ground_z):
        """Detect horizontal planes via z-histogram — each peak is a candidate tread."""
        above = pts[pts[:, 2] > ground_z + 0.04]
        if len(above) < 30:
            return []

        z_rel        = above[:, 2] - ground_z
        bin_size     = 0.04   # 4 cm resolution
        bins         = np.arange(0.0, 2.0, bin_size)
        hist, edges  = np.histogram(z_rel, bins=bins)

        treads = []
        for i, count in enumerate(hist):
            if count < 12:
                continue
            z_lo  = ground_z + edges[i]
            z_hi  = ground_z + edges[i + 1]
            band  = above[(above[:, 2] >= z_lo) & (above[:, 2] <= z_hi)]
            if len(band) < 12:
                continue
            if np.std(band[:, 2]) > 0.035:   # reject non-horizontal clusters
                continue
            width = float(band[:, 1].max() - band[:, 1].min())
            if width < MIN_WIDTH:
                continue
            treads.append({
                'z':     float(np.mean(band[:, 2])),
                'x':     float(np.mean(band[:, 0])),
                'y':     float(np.mean(band[:, 1])),
                'width': width,
            })

        treads.sort(key=lambda t: t['z'])
        return treads

    def _fit_model(self, treads, ground_z):
        """Check tread heights form an arithmetic sequence, extract stair geometry."""
        step_heights = [treads[i + 1]['z'] - treads[i]['z'] for i in range(len(treads) - 1)]
        tread_depths = [treads[i + 1]['x'] - treads[i]['x'] for i in range(len(treads) - 1)]

        step_h      = float(np.median(step_heights))
        step_h_std  = float(np.std(step_heights))
        tread_d     = float(np.median(tread_depths))
        width       = float(np.median([t['width'] for t in treads]))

        if not (MIN_STEP_H <= step_h <= MAX_STEP_H):
            return None
        if step_h_std > max(0.04, 0.35 * step_h):   # too irregular
            return None
        if abs(tread_d) < MIN_TREAD_D:
            return None
        if width < MIN_WIDTH:
            return None

        slope_deg = math.degrees(math.atan2(step_h, abs(tread_d))) if abs(tread_d) > 0 else 90.0
        if slope_deg > MAX_SLOPE:
            return None

        xs            = np.array([t['x'] for t in treads])
        ys            = np.array([t['y'] for t in treads])
        direction_deg = math.degrees(math.atan2(float(ys[-1] - ys[0]), float(xs[-1] - xs[0])))

        regularity = max(0.0, 1.0 - step_h_std / (step_h + 0.001))
        step_bonus = min(1.0, len(treads) / 5.0)
        confidence = round(0.6 * regularity + 0.4 * step_bonus, 3)

        if confidence < MIN_CONF:
            return None

        geo = {
            'detected':        True,
            'step_height_m':   round(step_h, 3),
            'tread_depth_m':   round(abs(tread_d), 3),
            'width_m':         round(width, 3),
            'slope_deg':       round(slope_deg, 1),
            'direction_deg':   round(direction_deg, 1),
            'num_steps':       len(treads),
            'first_step_x_m':  round(treads[0]['x'], 3),
            'first_step_y_m':  round(treads[0]['y'], 3),
            'confidence':      confidence,
            'climbable':       True,
        }
        self.get_logger().info(
            f'STAIR: h={step_h:.2f}m  d={abs(tread_d):.2f}m  w={width:.2f}m  '
            f'slope={slope_deg:.1f}°  steps={len(treads)}  conf={confidence:.2f}'
        )
        return geo

    # ── Visualization ──────────────────────────────────────────────────────────

    def _publish_viz_cloud(self, pts, header):
        """Publish ROI cloud with z as intensity — shows up as rainbow height-map in RViz."""
        n  = len(pts)
        dt = np.dtype([('x', np.float32), ('y', np.float32),
                       ('z', np.float32), ('intensity', np.float32)])
        arr = np.zeros(n, dtype=dt)
        arr['x']         = pts[:, 0]
        arr['y']         = pts[:, 1]
        arr['z']         = pts[:, 2]
        arr['intensity'] = pts[:, 2]   # encode height as intensity

        msg = PointCloud2()
        msg.header       = header
        msg.height       = 1
        msg.width        = n
        msg.is_dense     = False
        msg.is_bigendian = False
        msg.point_step   = 16
        msg.row_step     = 16 * n
        msg.fields = [
            PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.data = arr.tobytes()
        self._cloud_pub.publish(msg)

    def _publish_markers(self, geo, header):
        arr = MarkerArray()

        clr = Marker()
        clr.action = Marker.DELETEALL
        arr.markers.append(clr)

        # Green arrow pointing toward first step
        arrow = Marker()
        arrow.header    = header
        arrow.ns        = 'stair'
        arrow.id        = 1
        arrow.type      = Marker.ARROW
        arrow.action    = Marker.ADD
        arrow.scale.x   = 0.08
        arrow.scale.y   = 0.18
        arrow.scale.z   = 0.18
        arrow.color.r   = 0.0
        arrow.color.g   = 1.0
        arrow.color.b   = 0.2
        arrow.color.a   = 0.9
        arrow.lifetime  = Duration(sec=1)
        arrow.points    = [
            Point(x=0.0, y=0.0, z=0.4),
            Point(x=float(geo['first_step_x_m']), y=float(geo['first_step_y_m']), z=0.4),
        ]
        arr.markers.append(arrow)

        # Yellow text label
        label = Marker()
        label.header              = header
        label.ns                  = 'stair'
        label.id                  = 2
        label.type                = Marker.TEXT_VIEW_FACING
        label.action              = Marker.ADD
        label.pose.position.x     = float(geo['first_step_x_m'])
        label.pose.position.y     = float(geo['first_step_y_m'])
        label.pose.position.z     = 1.1
        label.pose.orientation.w  = 1.0
        label.scale.z             = 0.18
        label.color.r             = 1.0
        label.color.g             = 1.0
        label.color.b             = 0.0
        label.color.a             = 1.0
        label.lifetime            = Duration(sec=1)
        label.text = (
            f"STAIR  h={geo['step_height_m']}m  d={geo['tread_depth_m']}m  "
            f"w={geo['width_m']}m\n"
            f"slope={geo['slope_deg']}°  steps={geo['num_steps']}  "
            f"conf={geo['confidence']}"
        )
        arr.markers.append(label)

        self._marker_pub.publish(arr)


def main():
    rclpy.init()
    node = StairPerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
