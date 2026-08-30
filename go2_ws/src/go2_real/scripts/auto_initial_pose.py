#!/usr/bin/env python3
"""
auto_initial_pose.py — publish /initialpose to RTAB-Map at stack startup.

Two-phase approach:
  Phase 1 (immediate): publish initial guess from yaml position + single-scan
    orientation search against the dilated occupancy map.
  Phase 2 (spin collection): while the user rotates the robot for localization,
    collect (scan, odom_cumulative_yaw) pairs. Once >=180 deg of rotation is
    captured, run a multi-sample orientation search: for every candidate initial
    yaw, ALL collected scans are projected onto the map simultaneously. The
    candidate whose aggregate hit fraction is highest = the true orientation.
    This is far more discriminating than single-scan matching because the
    constraint must hold across every sampled angle, not just one.
"""
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy,
                       QoSReliabilityPolicy, QoSHistoryPolicy,
                       qos_profile_sensor_data)
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan

try:
    import numpy as np
    _NUMPY_OK = True
except ImportError:
    _NUMPY_OK = False

try:
    import yaml
    _YAML_OK = True
except ImportError:
    _YAML_OK = False

POSE_FILE        = os.path.expanduser('~/maps/robot_initial_pose.yaml')
YAW_STEP         = 5      # degrees — angular resolution of orientation search
MIN_SCORE        = 0.05   # minimum aggregate hit fraction to trust result
WAIT_TIMEOUT     = 30.0   # seconds to wait for /map+/scan before fallback
PHASE1_MIN_DELAY = 5.0    # delay Phase 1 publish so go2_bridge publishes its /initialpose first
DILATE_R         = 4      # pixels — inflate occupied cells so rays score hits
SAMPLE_INTERVAL  = 0.5    # seconds between spin samples
MIN_SPAN_DEG     = 180.0  # minimum rotation span to trigger multi-sample search
MAX_SPIN_WAIT    = 60.0   # seconds to wait for spin; go2_bridge does ~27s auto-spin

def _dilate_map(occupied, radius):
    result = occupied.copy()
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            if dy * dy + dx * dx <= radius * radius:
                result |= np.roll(np.roll(occupied, dy, axis=0), dx, axis=1)
    return result

class AutoInitialPose(Node):

    def __init__(self):
        super().__init__('auto_initial_pose')
        self._pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        self._map_msg  = None
        self._scan_msg = None
        self._phase    = 'waiting'
        self._start_t  = time.time()
        self._phase2_t = None

        self._x       = 0.0
        self._y       = 0.0
        self._yaw_deg = 0.0
        self._name    = 'home'

        self._dilated   = None
        self._map_meta  = None   # (res, ox, oy, w, h)

        self._samples       = []
        self._odom_yaw_last = None
        self._odom_yaw_cum  = 0.0
        self._last_sample_t = 0.0

        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(OccupancyGrid, '/map',  self._map_cb,  map_qos)
        # /scan and /odom are published BEST_EFFORT (sensor data) — match that QoS
        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._odom_cb, qos_profile_sensor_data)

        self.create_timer(0.5, self._tick)

    def _map_cb(self, msg):
        self._map_msg = msg
        if self._dilated is None and _NUMPY_OK:
            self._precompute_map(msg)

    def _scan_cb(self, msg):
        self._scan_msg = msg

    def _odom_cb(self, msg):
        q   = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if self._odom_yaw_last is None:
            self._odom_yaw_last = yaw
            return
        delta = (yaw - self._odom_yaw_last + math.pi) % (2.0 * math.pi) - math.pi
        self._odom_yaw_cum  += delta
        self._odom_yaw_last  = yaw

    def _precompute_map(self, m):
        w, h  = m.info.width, m.info.height
        data  = np.array(m.data, dtype=np.int8).reshape(h, w)
        occ   = data > 50
        n_occ = int(np.sum(occ))
        self.get_logger().info(
            f'Map: {w}x{h} @ {m.info.resolution:.3f} m/cell — {n_occ} occupied cells'
        )
        if n_occ == 0:
            return
        self._dilated  = _dilate_map(occ, DILATE_R)
        self._map_meta = (m.info.resolution,
                          m.info.origin.position.x,
                          m.info.origin.position.y,
                          w, h)

    def _tick(self):
        if self._phase == 'done':
            return
        elapsed = time.time() - self._start_t
        if self._phase == 'waiting':
            self._phase_waiting(elapsed)
        elif self._phase == 'collecting':
            self._phase_collecting()

    def _phase_waiting(self, elapsed):
        if self._map_msg is None:
            self.get_logger().info('Waiting for /map…', throttle_duration_sec=2.0)
            if elapsed > WAIT_TIMEOUT:
                self.get_logger().warn('No /map after timeout — using yaml yaw, no spin')
                self._load_yaml()
                self._publish(math.radians(self._yaw_deg), 'yaml fallback (no map)')
                self._phase = 'done'
                raise SystemExit(0)
            return

        if self._scan_msg is None:
            self.get_logger().info('Waiting for /scan…', throttle_duration_sec=2.0)
            if elapsed > WAIT_TIMEOUT:
                self.get_logger().warn('No /scan after timeout — using yaml yaw, no spin')
                self._load_yaml()
                self._publish(math.radians(self._yaw_deg), 'yaml fallback (no scan)')
                self._phase = 'done'
                raise SystemExit(0)
            return

        # Wait until go2_bridge has had time to publish its own /initialpose first,
        # so our publication (which comes next) is the last word RTAB-Map receives.
        if elapsed < PHASE1_MIN_DELAY:
            self.get_logger().info(
                f'Map+scan ready, waiting {PHASE1_MIN_DELAY - elapsed:.1f}s '
                '(let bridge publish first)…',
                throttle_duration_sec=1.0,
            )
            return

        self._load_yaml()

        if _NUMPY_OK and self._dilated is not None:
            initial_yaw, score = self._score_single()
            label = f'single-scan best={math.degrees(initial_yaw):.1f}° score={score:.3f}'
        else:
            initial_yaw = math.radians(self._yaw_deg)
            label = 'yaml (numpy unavailable)'

        self._publish(initial_yaw, f'initial guess — {label}')

        if not _NUMPY_OK or self._dilated is None:
            self._phase = 'done'
            raise SystemExit(0)

        self._phase        = 'collecting'
        self._phase2_t     = time.time()
        self._last_sample_t = time.time()
        self.get_logger().info(
            f'Phase 2 started — rotate the robot >{MIN_SPAN_DEG:.0f}° for orientation correction'
        )

    def _phase_collecting(self):
        now = time.time()
        if now - self._last_sample_t >= SAMPLE_INTERVAL and self._scan_msg is not None:
            s      = self._scan_msg
            ranges = np.array(s.ranges, dtype=np.float32)
            angles = (s.angle_min + np.arange(len(ranges), dtype=np.float32) * s.angle_increment)
            valid  = (np.isfinite(ranges) & (ranges > s.range_min) & (ranges < s.range_max))
            if np.any(valid):
                self._samples.append((ranges[valid], angles[valid], self._odom_yaw_cum))
            self._last_sample_t = now

        if len(self._samples) >= 4:
            yaws     = [s[2] for s in self._samples]
            span_deg = math.degrees(max(yaws) - min(yaws))
        else:
            span_deg = 0.0

        elapsed2 = now - self._phase2_t
        self.get_logger().info(
            f'Spin: {len(self._samples)} samples, span={span_deg:.0f}°  '
            f'(need {MIN_SPAN_DEG:.0f}°, timeout in {MAX_SPIN_WAIT - elapsed2:.0f}s)',
            throttle_duration_sec=5.0,
        )

        if span_deg >= MIN_SPAN_DEG or elapsed2 > MAX_SPIN_WAIT:
            if span_deg < 30.0:
                self.get_logger().warn(
                    f'Spin span only {span_deg:.0f}° — odom may not be available; '
                    'keeping single-scan initial guess'
                )
                self._phase = 'done'
                raise SystemExit(0)
            self._run_multi_search(span_deg)

    def _score_single(self):
        res, ox, oy, w, h = self._map_meta
        s      = self._scan_msg
        ranges = np.array(s.ranges, dtype=np.float32)
        angles = (s.angle_min + np.arange(len(ranges), dtype=np.float32) * s.angle_increment)
        valid  = (np.isfinite(ranges) & (ranges > s.range_min) & (ranges < s.range_max))
        ranges, angles = ranges[valid], angles[valid]
        n_rays = len(ranges)
        if n_rays == 0:
            return math.radians(self._yaw_deg), 0.0

        best_yaw, best_score = math.radians(self._yaw_deg), -1.0
        scores = {}
        for yaw_deg in range(0, 360, YAW_STEP):
            yaw = math.radians(yaw_deg)
            ex  = ((self._x + ranges * np.cos(angles + yaw)) - ox) / res
            ey  = ((self._y + ranges * np.sin(angles + yaw)) - oy) / res
            ei  = np.clip(np.round(ex).astype(np.int32), 0, w - 1)
            ej  = np.clip(np.round(ey).astype(np.int32), 0, h - 1)
            sc  = float(np.sum(self._dilated[ej, ei])) / n_rays
            scores[yaw_deg] = sc
            if sc > best_score:
                best_score, best_yaw = sc, yaw

        yaml_deg = int(self._yaw_deg) % 360
        diag = '  '.join(f'{d}°={scores.get(d,0):.2f}' for d in range(0, 360, 45))
        self.get_logger().info(f'Single-scan scores (45° steps): {diag}')
        self.get_logger().info(
            f'Single-scan best: {math.degrees(best_yaw):.0f}° score={best_score:.3f}  '
            f'yaml={yaml_deg}° score={scores.get(yaml_deg,0):.3f}  '
            f'yaml+180={(yaml_deg+180)%360}° score={scores.get((yaml_deg+180)%360,0):.3f}'
        )
        return best_yaw, best_score

    def _run_multi_search(self, span_deg):
        self._phase = 'done'
        res, ox, oy, w, h = self._map_meta
        n_samples = len(self._samples)
        self.get_logger().info(
            f'Multi-sample search: {n_samples} samples, span={span_deg:.0f}°'
        )

        best_offset = 0.0
        best_score  = -1.0
        all_scores  = {}

        for offset_deg in range(0, 360, YAW_STEP):
            offset     = math.radians(offset_deg)
            total_hits = 0
            total_rays = 0

            for ranges, angles, cum_yaw in self._samples:
                abs_yaw    = offset + cum_yaw
                ray_angles = angles + abs_yaw
                ex  = ((self._x + ranges * np.cos(ray_angles)) - ox) / res
                ey  = ((self._y + ranges * np.sin(ray_angles)) - oy) / res
                ei  = np.clip(np.round(ex).astype(np.int32), 0, w - 1)
                ej  = np.clip(np.round(ey).astype(np.int32), 0, h - 1)
                total_hits += int(np.sum(self._dilated[ej, ei]))
                total_rays += len(ranges)

            score = total_hits / total_rays if total_rays > 0 else 0.0
            all_scores[offset_deg] = score
            if score > best_score:
                best_score  = score
                best_offset = offset

        diag = '  '.join(f'{d}°={all_scores.get(d,0):.3f}' for d in range(0, 360, 45))
        self.get_logger().info(f'Multi-sample scores (45° steps): {diag}')
        yaml_deg = int(self._yaw_deg) % 360
        self.get_logger().info(
            f'Multi-sample best: {math.degrees(best_offset):.1f}° score={best_score:.3f}  '
            f'yaml={yaml_deg}° score={all_scores.get(yaml_deg,0):.3f}  '
            f'yaml+180={(yaml_deg+180)%360}° score={all_scores.get((yaml_deg+180)%360,0):.3f}'
        )

        if best_score >= MIN_SCORE:
            # best_offset = initial map-frame yaw. Robot has since rotated by odom_yaw_cum.
            # Publish the CURRENT orientation so RTAB-Map gets the pose RIGHT NOW.
            current_yaw = (best_offset + self._odom_yaw_cum) % (2.0 * math.pi)
            self.get_logger().info(
                f'Current yaw = offset({math.degrees(best_offset):.1f}°) + '
                f'odom_cum({math.degrees(self._odom_yaw_cum):.1f}°) = '
                f'{math.degrees(current_yaw):.1f}°'
            )
            self._publish(
                current_yaw,
                f'multi-sample CORRECTED yaw={math.degrees(current_yaw):.1f}° '
                f'(offset={math.degrees(best_offset):.1f}° score={best_score:.3f})'
            )
        else:
            self.get_logger().warn(
                f'Multi-sample score too low ({best_score:.3f} < {MIN_SCORE}) — '
                'keeping single-scan initial guess'
            )

        raise SystemExit(0)

    def _load_yaml(self):
        if _YAML_OK and os.path.exists(POSE_FILE):
            try:
                with open(POSE_FILE) as f:
                    d = yaml.safe_load(f) or {}
                self._x       = float(d.get('x',       0.0))
                self._y       = float(d.get('y',       0.0))
                self._yaw_deg = float(d.get('yaw_deg', 0.0))
                self._name    = d.get('name', 'home')
            except Exception as e:
                self.get_logger().warn(f'YAML read error: {e} — using (0, 0, 0°)')
        else:
            self.get_logger().warn(f'{POSE_FILE} not found — using (0, 0, 0°)')

    def _publish(self, yaw_rad, reason):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id         = 'map'
        msg.pose.pose.position.x    = self._x
        msg.pose.pose.position.y    = self._y
        msg.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
        msg.pose.covariance[0]  = 0.25
        msg.pose.covariance[7]  = 0.25
        msg.pose.covariance[35] = 0.07

        msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(msg)
        self.get_logger().info(
            f'[{reason}]  name={self._name!r}  '
            f'x={self._x:.3f}  y={self._y:.3f}  yaw={math.degrees(yaw_rad):.1f}°'
        )
        time.sleep(0.3)
        msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(msg)


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
