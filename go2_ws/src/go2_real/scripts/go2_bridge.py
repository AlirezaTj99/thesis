#!/usr/bin/env python3
"""
go2_bridge.py — ROS2 bridge for real Unitree Go2 Pro navigation.

Subscribes:
  /utlidar/robot_odom  (nav_msgs/Odometry)       — LIO-SAM odometry (BEST_EFFORT QoS)
  /utlidar/cloud_base  (sensor_msgs/PointCloud2) — LiDAR in base_link (BEST_EFFORT QoS)
  /cmd_vel_combined    (geometry_msgs/Twist)      — Nav2 velocity commands
  /amcl_pose           (PoseWithCovarianceStamped) — monitors AMCL yaw convergence

Publishes:
  /odom                (nav_msgs/Odometry)        — relayed with RELIABLE QoS for Nav2
  /cloud_relay         (sensor_msgs/PointCloud2)  — relayed with RELIABLE QoS for laserscan
  TF: odom → base_link                            — broadcast directly from LIO-SAM odometry
  /initialpose         (PoseWithCovarianceStamped) — seeds AMCL at known starting position
  /api/sport/request   (unitree_api/Request)      — velocity command to Go2

Startup localization sequence:
  1. t+6s after first odom: send /initialpose (tight XY, ±π yaw uncertainty)
  2. Rotate slowly 360° at 0.2 rad/s — AMCL accumulates scans from all angles
  3. Monitor /amcl_pose yaw covariance until it drops below threshold (converged)
  4. Release Nav2 control — robot is localized
"""

import json
import math
import os
import time
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist, TransformStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2
from unitree_api.msg import Request
from unitree_go.msg import SportModeState
import tf2_ros

ROBOT_SPORT_API_ID_MOVE          = 1008
ROBOT_SPORT_API_ID_BALANCE_STAND = 1002

# Body height (m) below which the robot is considered sitting.
# Standing: ~0.32 m  |  Sitting/lying: ~0.05 m
STANDING_HEIGHT_THRESHOLD = 0.08   # Go2 standing ~0.32m, sitting ~0.05m; 0.08 safely above sitting

# Robot's known starting position — fallback if pose file is missing.
# start_go2_real.sh asks the user and writes ~/maps/robot_initial_pose.yaml before launch.
INITIAL_MAP_X   = 0.0
INITIAL_MAP_Y   = 0.0
INITIAL_MAP_YAW = 0.0   # radians

POSE_FILE = os.path.expanduser('~/maps/robot_initial_pose.yaml')

# Localization scan: drive a tight circle so AMCL sees all orientations.
# Pure yaw (x=0) is silently ignored by Go2 in BalanceStand mode — a small
# forward velocity is required to activate movement.
# Circle radius = vx/vyaw = 0.04/0.4 = 0.1 m  |  empirically one full circuit ≈ 27 s
_SCAN_ROT_VEL  = 0.4    # rad/s yaw component
_SCAN_FWD_VEL  = 0.04   # m/s forward component (circle radius 0.1 m)
_SCAN_ROT_DUR  = 27.0   # seconds — one full circle (empirically measured)

# AMCL yaw covariance (covariance[35]) threshold for declaring convergence.
# Units: rad².  0.1 rad² ≈ ±18° std dev — tight enough for reliable navigation.
_AMCL_YAW_CONVERGED = 0.1
_MAX_STABILITY_WAIT  = 60   # seconds max wait for AMCL convergence before giving up

# Match the Go2 robot's publisher QoS (BEST_EFFORT, VOLATILE)
BEST_EFFORT_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)


class Go2Bridge(Node):

    def __init__(self):
        super().__init__('go2_bridge')

        self._odom_pub        = self.create_publisher(Odometry, '/odom', 10)
        self._imu_pub         = self.create_publisher(Imu, '/imu/data', 10)
        self._cloud_pub       = self.create_publisher(PointCloud2, '/cloud_relay', 10)
        self._req_pub         = self.create_publisher(Request, '/api/sport/request_wifi', 10)
        self._initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self._tf_broadcaster  = tf2_ros.TransformBroadcaster(self)

        self.create_subscription(Odometry, '/utlidar/robot_odom',
                                 self._odom_callback, BEST_EFFORT_QOS)
        self.create_subscription(PointCloud2, '/utlidar/cloud_base',
                                 self._cloud_callback, BEST_EFFORT_QOS)
        self.create_subscription(SportModeState, '/sportmodestate',
                                 self._imu_callback, BEST_EFFORT_QOS)
        self.create_subscription(Twist, '/cmd_vel_combined',
                                 self._cmd_vel_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self._amcl_pose_callback, 10)

        # Last known odom→base_link transform — republished at 50 Hz.
        self._cached_transform = None
        self.create_timer(0.02, self._tf_keepalive)   # 50 Hz

        # One-shot 6 s timer to send /initialpose once AMCL is active.
        self._first_odom_seen    = False
        self._initial_pose_timer = None

        # Localization scan state — robot rotates 360° slowly while AMCL converges.
        self._localizing         = False   # blocks Nav2 cmd_vel during scan + convergence wait
        self._scan_step          = 0
        self._scan_timer         = None
        self._stability_timer    = None
        self._stability_elapsed  = 0.0
        self._amcl_yaw_variance  = float('inf')
        self._diag_timer         = None
        self._last_odom_msg      = None   # for diagnostic yaw logging
        self._last_body_height   = 0.0

        # Cloud relay gate — only forward scans when robot is standing.
        self._robot_standing = False

        # Target velocities from Nav2
        self._vx_target   = 0.0
        self._vy_target   = 0.0
        self._vyaw_target = 0.0
        self._zero_since  = None
        self.create_timer(0.05, self._cmd_vel_keepalive)   # 20 Hz

        # Repulsion velocity — tracked separately so it applies even during localization.
        # During localization _cmd_vel_callback blocks /cmd_vel_combined entirely, so
        # we must read /repulsion_vel directly here to keep obstacle avoidance alive.
        self._rep_vx    = 0.0
        self._rep_vy    = 0.0
        self._rep_stamp = None
        self.create_subscription(Twist, '/repulsion_vel', self._rep_vel_callback, 10)

        self._recovery_timer = self.create_timer(3.0, self._startup_recovery_stand)
        self._balance_timer  = self.create_timer(5.0, self._startup_balance_stand)
        self.get_logger().info('go2_bridge started — relaying odom + cloud with QoS fix')

    # ── Odometry + TF ──────────────────────────────────────────────────────────

    def _log_odom_yaw(self):
        if self._last_odom_msg is None:
            return
        q = self._last_odom_msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        self.get_logger().info(
            f'[LOC-DIAG] body_height={self._last_body_height:.3f}m  '
            f'odom_yaw={math.degrees(yaw):.1f}°  '
            f'sending vyaw={self._vyaw_target:.2f} rad/s'
        )

    def _odom_callback(self, msg: Odometry):
        self._last_odom_msg = msg
        now = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'
        odom.pose            = msg.pose
        odom.twist           = msg.twist
        self._odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp    = now
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation      = msg.pose.pose.orientation
        self._cached_transform = t
        self._tf_broadcaster.sendTransform(t)

        if not self._first_odom_seen:
            self._first_odom_seen = True
            self._initial_pose_timer = self.create_timer(12.0, self._send_initial_pose)

    def _tf_keepalive(self):
        if self._cached_transform is None:
            return
        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_link'
        t.transform       = self._cached_transform.transform
        self._tf_broadcaster.sendTransform(t)

    # ── AMCL initial pose + localization scan sequence ─────────────────────────

    def _send_initial_pose(self):
        self.destroy_timer(self._initial_pose_timer)

        # Read the location the user chose in start_go2_real.sh
        map_x, map_y, map_yaw = INITIAL_MAP_X, INITIAL_MAP_Y, INITIAL_MAP_YAW
        location_name = 'default (0, 0, 0°)'
        try:
            with open(POSE_FILE) as f:
                data = yaml.safe_load(f)
            map_x         = float(data.get('x',       map_x))
            map_y         = float(data.get('y',       map_y))
            map_yaw       = math.radians(float(data.get('yaw_deg', math.degrees(map_yaw))))
            location_name = data.get('name', location_name)
        except Exception as e:
            self.get_logger().warn(f'Could not read {POSE_FILE}: {e} — using defaults (0, 0, 0°)')

        half = map_yaw / 2.0
        ip = PoseWithCovarianceStamped()
        ip.header.stamp    = self.get_clock().now().to_msg()
        ip.header.frame_id = 'map'
        ip.pose.pose.position.x    = map_x
        ip.pose.pose.position.y    = map_y
        ip.pose.pose.position.z    = 0.0
        ip.pose.pose.orientation.x = 0.0
        ip.pose.pose.orientation.y = 0.0
        ip.pose.pose.orientation.z = math.sin(half)
        ip.pose.pose.orientation.w = math.cos(half)
        ip.pose.covariance[0]  = 0.01   # x  ±0.1 m — position is well known
        ip.pose.covariance[7]  = 0.01   # y  ±0.1 m
        ip.pose.covariance[35] = 3.0    # yaw ±π  — nearly unknown, AMCL will find it
        self._initialpose_pub.publish(ip)
        self.get_logger().info(
            f'Initial pose sent: location="{location_name}"  '
            f'({map_x}, {map_y}, yaw={math.degrees(map_yaw):.0f}°) '
            f'— starting localization scan'
        )

        # Start slow 360° rotation immediately after seeding AMCL
        self._localizing  = True
        self._scan_step   = 0
        self._zero_since  = None
        self._scan_timer  = self.create_timer(0.5, self._localization_scan_tick)

    def _localization_scan_tick(self):
        self.destroy_timer(self._scan_timer)

        if self._scan_step >= 1:
            # Circles done — stop and monitor AMCL convergence
            self._vx_target   = 0.0
            self._vy_target   = 0.0
            self._vyaw_target = 0.0
            if self._diag_timer is not None:
                self.destroy_timer(self._diag_timer)
                self._diag_timer = None
            self.get_logger().info(
                'Localization circles complete — waiting for AMCL to converge '
                f'(threshold: yaw variance < {_AMCL_YAW_CONVERGED} rad²)'
            )
            self._stability_elapsed = 0.0
            self._stability_timer = self.create_timer(1.0, self._check_amcl_stability)
            return

        self.get_logger().info(
            f'Localization scan: 1 circle  vx={_SCAN_FWD_VEL} m/s  vyaw={_SCAN_ROT_VEL} rad/s  '
            f'radius={_SCAN_FWD_VEL/_SCAN_ROT_VEL:.2f}m  (~{_SCAN_ROT_DUR:.0f} s)'
        )
        self._vx_target   = _SCAN_FWD_VEL  # must be non-zero — Go2 ignores pure-yaw commands
        self._vy_target   = 0.0
        self._vyaw_target = _SCAN_ROT_VEL
        self._scan_step   = 1
        self._scan_timer  = self.create_timer(_SCAN_ROT_DUR, self._localization_scan_tick)
        # Diagnostic: log odom yaw every 5s during rotation to confirm robot is moving
        self._diag_timer = self.create_timer(5.0, self._log_odom_yaw)

    def _amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        self._amcl_yaw_variance = msg.pose.covariance[35]

    def _check_amcl_stability(self):
        self._stability_elapsed += 1.0
        converged = self._amcl_yaw_variance <= _AMCL_YAW_CONVERGED
        timed_out = self._stability_elapsed >= _MAX_STABILITY_WAIT

        if converged or timed_out:
            self.destroy_timer(self._stability_timer)
            self._localizing = False
            if converged:
                self.get_logger().info(
                    f'AMCL converged (yaw variance={self._amcl_yaw_variance:.3f} rad²) '
                    '— Nav2 enabled'
                )
            else:
                self.get_logger().warn(
                    f'AMCL convergence timeout after {_MAX_STABILITY_WAIT} s '
                    f'(yaw variance={self._amcl_yaw_variance:.3f}) — releasing Nav2 anyway'
                )
        else:
            self.get_logger().info(
                f'Waiting for AMCL: yaw variance={self._amcl_yaw_variance:.3f} '
                f'(need < {_AMCL_YAW_CONVERGED}) — {self._stability_elapsed:.0f}/{_MAX_STABILITY_WAIT} s'
            )

    # ── Sensors ────────────────────────────────────────────────────────────────

    def _imu_callback(self, msg: SportModeState):
        self._last_body_height = msg.body_height
        was_standing = self._robot_standing
        self._robot_standing = msg.body_height > STANDING_HEIGHT_THRESHOLD
        if self._robot_standing != was_standing:
            state = 'standing' if self._robot_standing else 'sitting'
            self.get_logger().info(f'Robot {state} (body_height={msg.body_height:.3f} m)')

        imu = Imu()
        imu.header.stamp    = self.get_clock().now().to_msg()
        imu.header.frame_id = 'base_link'

        q = msg.imu_state.quaternion
        imu.orientation.w = float(q[0])
        imu.orientation.x = float(q[1])
        imu.orientation.y = float(q[2])
        imu.orientation.z = float(q[3])

        g = msg.imu_state.gyroscope
        imu.angular_velocity.x = float(g[0])
        imu.angular_velocity.y = float(g[1])
        imu.angular_velocity.z = float(g[2])

        a = msg.imu_state.accelerometer
        imu.linear_acceleration.x = float(a[0])
        imu.linear_acceleration.y = float(a[1])
        imu.linear_acceleration.z = float(a[2])

        imu.orientation_covariance[0]          = 0.01
        imu.orientation_covariance[4]          = 0.01
        imu.orientation_covariance[8]          = 0.01
        imu.angular_velocity_covariance[0]     = 0.01
        imu.angular_velocity_covariance[4]     = 0.01
        imu.angular_velocity_covariance[8]     = 0.01
        imu.linear_acceleration_covariance[0]  = 0.1
        imu.linear_acceleration_covariance[4]  = 0.1
        imu.linear_acceleration_covariance[8]  = 0.1
        self._imu_pub.publish(imu)

    def _cloud_callback(self, msg: PointCloud2):
        if not self._robot_standing:
            return   # suppress scans while sitting — legs would create ghost obstacles
        msg.header.stamp = self.get_clock().now().to_msg()
        self._cloud_pub.publish(msg)

    # ── Velocity control ───────────────────────────────────────────────────────

    def _rep_vel_callback(self, msg: Twist):
        self._rep_vx    = msg.linear.x
        self._rep_vy    = msg.linear.y
        self._rep_stamp = self.get_clock().now()

    def _cmd_vel_callback(self, msg: Twist):
        if self._localizing:
            return   # localization sequence in progress — block Nav2 commands
        new_vx   = float(msg.linear.x)
        new_vy   = float(msg.linear.y)
        new_vyaw = float(msg.angular.z)
        if new_vx != 0.0 or new_vy != 0.0 or new_vyaw != 0.0:
            self._vx_target   = new_vx
            self._vy_target   = new_vy
            self._vyaw_target = new_vyaw
            self._zero_since  = None
        else:
            if self._zero_since is None:
                self._zero_since = self.get_clock().now()

    def _cmd_vel_keepalive(self):
        if self._zero_since is not None:
            age = (self.get_clock().now() - self._zero_since).nanoseconds / 1e9
            if age >= 0.3:
                self._vx_target  = self._vy_target = self._vyaw_target = 0.0
                self._zero_since = None

        vx, vy, vyaw = self._vx_target, self._vy_target, self._vyaw_target

        # During localization, /cmd_vel_combined is blocked so velocity_combiner cannot
        # deliver repulsion. Inject it directly so the robot avoids obstacles while scanning.
        if self._localizing and self._rep_stamp is not None:
            rep_age = (self.get_clock().now() - self._rep_stamp).nanoseconds / 1e9
            if rep_age < 0.5:
                vx += self._rep_vx
                vy += self._rep_vy

        if vx != 0.0 or vy != 0.0 or vyaw != 0.0:
            self._publish_move(vx, vy, vyaw)

    # ── Sport API commands ─────────────────────────────────────────────────────

    def _startup_recovery_stand(self):
        req = Request()
        req.header.identity.api_id = 1006
        self._req_pub.publish(req)
        self.get_logger().info('RecoveryStand sent — standing up')
        self.destroy_timer(self._recovery_timer)

    def _startup_balance_stand(self):
        req = Request()
        req.header.identity.api_id = ROBOT_SPORT_API_ID_BALANCE_STAND
        self._req_pub.publish(req)
        self.get_logger().info('BalanceStand sent — robot ready for velocity commands')
        self.destroy_timer(self._balance_timer)

    def _publish_move(self, vx: float, vy: float, vyaw: float):
        req = Request()
        req.header.identity.api_id = ROBOT_SPORT_API_ID_MOVE
        req.parameter = json.dumps({'x': vx, 'y': vy, 'z': vyaw})
        self._req_pub.publish(req)


def main(args=None):
    rclpy.init(args=args)
    node = Go2Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down — laying down...')
        time.sleep(0.1)
        down = Request()
        down.header.identity.api_id = 1005   # StandDown
        node._req_pub.publish(down)
        time.sleep(1.5)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
