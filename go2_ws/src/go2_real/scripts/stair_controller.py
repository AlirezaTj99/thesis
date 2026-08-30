#!/usr/bin/env python3
"""
stair_controller.py — Straight-line stair follower.

go2_goal.py delivers the robot to the stair entry waypoint via Nav2 (with an
approach alignment point), then activates this node.  From that point this node
rotates in place to align with the stair bearing, then drives straight to the
stair exit waypoint using odometry-based cross-track correction.

                 IDLE
                  │  activate=True + target received
                  ▼
              ALIGNING  ← rotate in place until yaw matches bearing (±ALIGN_TOL)
                  │  aligned
                  ▼
              CLIMBING  ← forward at CLIMB_VEL, gravity toward centre-line
                  │  arrived (remaining ≤ ARRIVAL_DIST)  OR  pitch flat after peak
                  ▼
              LANDING   ← stop, settle, publish done
                  │
                  ▼
                 IDLE

Activation sequence (from go2_goal.py):
  1. Publish /stair_controller/target  (String JSON)  — bearing + line length
  2. Publish /stair_controller/activate True

Subscribes:
    /stair_controller/activate  (std_msgs/Bool)
    /stair_controller/target    (std_msgs/String)  JSON: {bearing_rad, line_length_m}
    /odom                       (nav_msgs/Odometry)
    /imu/data                   (sensor_msgs/Imu)

Publishes:
    /stair_controller/state     (std_msgs/String)  — repulsion_gate uses this
    /stair_controller/done      (std_msgs/Bool)    — pulses True when finished
    /api/sport/request_wifi     (unitree_api/Request)
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from unitree_api.msg import Request

# ── Motion ────────────────────────────────────────────────────────────────────
CLIMB_VEL       = 0.33    # m/s — forward speed on stairs (+30% from 0.25)

# Yaw alignment before climbing
ALIGN_TOL       = 0.08    # rad (~5°) — yaw error below this → start climbing
ALIGN_KP        = 1.5     # proportional gain for in-place rotation
ALIGN_MAX_YAW   = 0.6     # rad/s — clamp on alignment rotation speed
ALIGN_TIMEOUT   = 5.0     # s — abort if can't align (go2_goal pre-aligns, this is a safety net)

# Cross-track gravity (lateral correction)
MAX_GRAVITY_VEL = 0.35    # m/s lateral — maximum correction velocity
MIN_GRAVITY_VEL = 0.05    # m/s lateral — high-pass cutoff; below this = no correction
GRAVITY_RANGE   = 0.20    # m  — gravity scales linearly from 0 at centre to MAX at this offset

# Completion
ARRIVAL_DIST    = 0.30    # m remaining on the line → declare done
CLIMB_TIMEOUT   = 90.0    # s — hard abort

# IMU-based secondary completion (pitch returns flat after climbing)
CLIMB_PITCH_HIGH = 6.0    # deg — pitch above this confirms we are climbing
CLIMB_PITCH_FLAT = 3.0    # deg — pitch below this AFTER peak = crested stairs

# Safety
MAX_PITCH_DEG   = 50.0
MAX_ROLL_DEG    = 35.0

API_MOVE = 1008
API_STOP = 1003


class StairController(Node):

    def __init__(self):
        super().__init__('stair_controller')

        self.create_subscription(Bool,     '/stair_controller/activate', self._activate_cb, 10)
        self.create_subscription(String,   '/stair_controller/target',   self._target_cb,   10)
        self.create_subscription(Odometry, '/odom',                       self._odom_cb,     10)
        self.create_subscription(Imu,      '/imu/data',                   self._imu_cb,      10)

        self._state_pub = self.create_publisher(String,  '/stair_controller/state', 10)
        self._done_pub  = self.create_publisher(Bool,    '/stair_controller/done',  10)
        self._cmd_pub   = self.create_publisher(Request, '/api/sport/request_wifi', 10)

        self._state       = 'IDLE'
        self._state_enter = time.time()

        # Target stair line — set via /stair_controller/target before activation
        self._bearing  = None   # radians — travel direction in odom frame
        self._line_len = None   # metres  — total length from entry to exit

        # Odometry: updated at ~50 Hz by go2_bridge
        self._odom_x   = 0.0
        self._odom_y   = 0.0
        self._odom_yaw = 0.0

        # Recorded at CLIMBING start
        self._climb_start_x = None
        self._climb_start_y = None

        # IMU
        self._pitch_deg    = 0.0
        self._roll_deg     = 0.0
        self._yaw_rad      = 0.0
        self._pitch_peaked = False

        self.create_timer(0.1, self._loop)   # 10 Hz control loop
        self.get_logger().info('StairController ready — IDLE')

    # ── Callbacks ──────────────────────────────────────────────────────────────

    def _target_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            self._bearing  = float(data['bearing_rad'])
            self._line_len = float(data['line_length_m'])
            self.get_logger().info(
                f'Target set: bearing={math.degrees(self._bearing):.1f}°  '
                f'length={self._line_len:.2f} m'
            )
        except Exception as e:
            self.get_logger().error(f'Bad target message: {e}')

    def _activate_cb(self, msg: Bool):
        if msg.data:
            if self._state != 'IDLE':
                return
            if self._bearing is None or self._line_len is None:
                self.get_logger().error('Activated but /stair_controller/target not received — ignoring')
                return
            self._begin_climb()
        else:
            if self._state != 'IDLE':
                self.get_logger().warn('Stair controller deactivated externally')
                self._finish()

    def _odom_cb(self, msg: Odometry):
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._odom_yaw = math.atan2(siny, cosy)

    def _imu_cb(self, msg: Imu):
        q = msg.orientation
        sinr = 2.0 * (q.w * q.x + q.y * q.z)
        cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self._roll_deg  = math.degrees(math.atan2(sinr, cosr))
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        self._pitch_deg = math.degrees(math.asin(max(-1.0, min(1.0, sinp))))
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._yaw_rad   = math.atan2(siny, cosy)

    # ── Main control loop ──────────────────────────────────────────────────────

    def _loop(self):
        self._state_pub.publish(String(data=self._state))

        if self._state == 'IDLE':
            return

        if not self._safety_ok():
            return

        elapsed = time.time() - self._state_enter

        if self._state == 'ALIGNING':
            self._do_align(elapsed)
        elif self._state == 'CLIMBING':
            self._do_climb(elapsed)
        elif self._state == 'LANDING':
            self._do_landing()

    def _begin_climb(self):
        self._pitch_peaked = False
        self._transition('ALIGNING')
        self.get_logger().info(
            f'ALIGNING: current_yaw={math.degrees(self._odom_yaw):.1f}°  '
            f'target_bearing={math.degrees(self._bearing):.1f}°'
        )

    def _do_align(self, elapsed):
        if elapsed > ALIGN_TIMEOUT:
            self.get_logger().warn('Align timeout — proceeding to climb anyway')
            self._start_climbing()
            return

        err = (self._bearing - self._odom_yaw + math.pi) % (2 * math.pi) - math.pi
        self.get_logger().info(
            f'ALIGNING  yaw={math.degrees(self._odom_yaw):.1f}°  '
            f'target={math.degrees(self._bearing):.1f}°  err={math.degrees(err):.1f}°',
            throttle_duration_sec=1.0,
        )

        if abs(err) < ALIGN_TOL:
            self.get_logger().info(
                f'Aligned (err={math.degrees(err):.1f}°) — starting climb'
            )
            self._stop()
            self._start_climbing()
            return

        vyaw = max(-ALIGN_MAX_YAW, min(ALIGN_MAX_YAW, ALIGN_KP * err))
        self._move(0.0, 0.0, vyaw)

    def _start_climbing(self):
        self._climb_start_x = self._odom_x
        self._climb_start_y = self._odom_y
        self._transition('CLIMBING')
        self.get_logger().info(
            f'CLIMBING: start=({self._climb_start_x:.2f}, {self._climb_start_y:.2f})  '
            f'bearing={math.degrees(self._bearing):.1f}°  length={self._line_len:.2f} m'
        )

    def _do_climb(self, elapsed):
        if elapsed > CLIMB_TIMEOUT:
            self.get_logger().warn('Climb timeout — stopping')
            self._stop()
            self._transition('LANDING')
            return

        # ── Progress along the stair line (odom frame) ───────────────────────
        dx = self._odom_x - self._climb_start_x
        dy = self._odom_y - self._climb_start_y
        cb = math.cos(self._bearing)
        sb = math.sin(self._bearing)

        forward_progress = dx * cb + dy * sb          # metres along stair direction
        cross_track      = dx * (-sb) + dy * cb       # metres perpendicular (+ve = left of line)
        remaining        = self._line_len - forward_progress

        # ── IMU pitch tracking ────────────────────────────────────────────────
        if abs(self._pitch_deg) > CLIMB_PITCH_HIGH:
            self._pitch_peaked = True

        # ── Completion checks ─────────────────────────────────────────────────
        if remaining <= ARRIVAL_DIST:
            self.get_logger().info(
                f'Arrived at stair exit (remaining={remaining:.2f} m) — LANDING'
            )
            self._stop()
            self._transition('LANDING')
            return

        if self._pitch_peaked and abs(self._pitch_deg) < CLIMB_PITCH_FLAT and elapsed > 3.0:
            self.get_logger().info(
                f'Pitch flat after peak ({self._pitch_deg:.1f}°) — crested stairs, LANDING'
            )
            self._stop()
            self._transition('LANDING')
            return

        # ── Cross-track gravity correction ────────────────────────────────────
        # Velocity scales linearly from 0 at centre to MAX_GRAVITY_VEL at GRAVITY_RANGE.
        # High-pass filter: corrections below MIN_GRAVITY_VEL are suppressed.
        gravity_raw = min(abs(cross_track) / GRAVITY_RANGE * MAX_GRAVITY_VEL, MAX_GRAVITY_VEL)
        if gravity_raw < MIN_GRAVITY_VEL:
            gravity_raw = 0.0
        # Positive cross_track = robot left of line → pull right (negative vy in body frame)
        vy = -math.copysign(gravity_raw, cross_track)

        self.get_logger().info(
            f'CLIMBING  fwd={forward_progress:.2f}m  remain={remaining:.2f}m  '
            f'xtrack={cross_track:+.3f}m  vy={vy:+.3f}  pitch={self._pitch_deg:.1f}°  '
            f'peaked={self._pitch_peaked}',
            throttle_duration_sec=0.5,
        )
        self._move(CLIMB_VEL, vy, 0.0)

    def _do_landing(self):
        self._stop()
        time.sleep(0.5)
        self.get_logger().info('Stair climb complete')
        self._done_pub.publish(Bool(data=True))
        self._reset()
        self._transition('IDLE')

    # ── Safety ─────────────────────────────────────────────────────────────────

    def _safety_ok(self):
        if abs(self._pitch_deg) > MAX_PITCH_DEG:
            self.get_logger().error(f'SAFETY STOP: pitch={self._pitch_deg:.1f}°')
            self._stop()
            self._done_pub.publish(Bool(data=True))
            self._reset()
            self._transition('IDLE')
            return False
        if abs(self._roll_deg) > MAX_ROLL_DEG:
            self.get_logger().error(f'SAFETY STOP: roll={self._roll_deg:.1f}°')
            self._stop()
            self._done_pub.publish(Bool(data=True))
            self._reset()
            self._transition('IDLE')
            return False
        return True

    # ── Helpers ────────────────────────────────────────────────────────────────

    def _finish(self):
        self._stop()
        self._done_pub.publish(Bool(data=True))
        self._reset()
        self._transition('IDLE')

    def _reset(self):
        self._bearing       = None
        self._line_len      = None
        self._climb_start_x = None
        self._climb_start_y = None
        self._pitch_peaked  = False

    def _transition(self, new_state):
        self.get_logger().info(f'  {self._state} → {new_state}')
        self._state       = new_state
        self._state_enter = time.time()
        self._state_pub.publish(String(data=new_state))

    def _move(self, vx: float, vy: float, vyaw: float):
        req = Request()
        req.header.identity.api_id = API_MOVE
        req.parameter = json.dumps({'x': round(vx, 4), 'y': round(vy, 4), 'z': round(vyaw, 4)})
        self._cmd_pub.publish(req)

    def _stop(self):
        req = Request()
        req.header.identity.api_id = API_STOP
        self._cmd_pub.publish(req)


def main():
    rclpy.init()
    node = StairController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
