#!/usr/bin/env python3
"""
stair_controller.py — State machine for stair approach, alignment, and climbing.

                IDLE
                 │  activate=True
                 ▼
              APPROACH  ← creep forward until stair confirmed + close enough
                 │  dist < APPROACH_DIST
                 ▼
               ALIGN    ← correct yaw and lateral offset vs stair axis
                 │  yaw_err < tol  AND  lat_err < tol
                 ▼
              CLIMBING  ← forward at CLIMB_VEL, hold centerline
                 │  stair gone > STAIR_LOST_TIMEOUT  OR  timeout
                 ▼
              LANDING   ← stop, brief settle, signal done
                 │
                 ▼
                IDLE

Activation:    /stair_controller/activate  (Bool)
               go2_control.py publishes True/False here.

Subscribes:
    /stair_controller/activate  (Bool)
    /stair/detected             (Bool)
    /stair/geometry             (String — JSON)
    /odom                       (nav_msgs/Odometry)
    /imu/data                   (sensor_msgs/Imu)

Publishes:
    /api/sport/request_wifi     (unitree_api/Request)  — direct robot sport API
    /stair_controller/state     (std_msgs/String)
    /stair_controller/done      (std_msgs/Bool)        — pulses True on completion/abort
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

# ── Motion parameters ─────────────────────────────────────────────────────────
APPROACH_VEL       = 0.18   # m/s  — forward speed while searching
APPROACH_DIST      = 0.70   # m    — stop approach when first step is this close
APPROACH_TIMEOUT   = 30.0   # s    — abort if no confirmed stair found
ALIGN_YAW_TOL      = 12.0   # deg — acceptable yaw error (measurement noise ~±30° so keep generous)
ALIGN_LAT_TOL      = 0.20   # m    — acceptable lateral offset to stair centre
ALIGN_ROT_VEL      = 0.40   # rad/s
ALIGN_LAT_VEL      = 0.08   # m/s
ALIGN_TIMEOUT      = 40.0   # s
CLIMB_VEL          = 0.35   # m/s  — fast enough for trot gait to clear step risers
CLIMB_TIMEOUT      = 45.0   # s    — hard abort
STAIR_LOST_TIMEOUT = 6.0    # s    — fallback: stair gone this long = reached the top
CLIMB_PITCH_HIGH   = 6.0    # deg  — pitch above this confirms we are climbing
CLIMB_PITCH_FLAT   = 3.0    # deg  — pitch below this (after peaking) = back on flat ground
DETECT_CONFIRM     = 3      # consecutive positive detections before trusting
DIR_EMA_ALPHA      = 0.15   # EMA smoothing for direction_deg (low α = more noise rejection)

# ── Safety limits ─────────────────────────────────────────────────────────────
MAX_PITCH_DEG      = 50.0
MAX_ROLL_DEG       = 35.0

# ── Go2 sport API ─────────────────────────────────────────────────────────────
API_STOP           = 1003
API_STAND          = 1006   # RecoveryStand — ensure robot is upright before climb
API_MOVE           = 1008


class StairController(Node):

    def __init__(self):
        super().__init__('stair_controller')

        self.create_subscription(Bool,     '/stair_controller/activate', self._activate_cb, 10)
        self.create_subscription(Bool,     '/stair/detected',            self._detected_cb,  10)
        self.create_subscription(String,   '/stair/geometry',            self._geometry_cb,  10)
        self.create_subscription(Odometry, '/odom',                      self._odom_cb,      10)
        self.create_subscription(Imu,      '/imu/data',                  self._imu_cb,       10)

        self._cmd_pub   = self.create_publisher(Request, '/api/sport/request_wifi', 10)
        self._state_pub = self.create_publisher(String,  '/stair_controller/state',  10)
        self._done_pub  = self.create_publisher(Bool,    '/stair_controller/done',   10)

        self._state        = 'IDLE'
        self._geo          = None
        self._detect_count = 0
        self._pitch_deg    = 0.0
        self._roll_deg     = 0.0
        self._state_enter  = time.time()
        self._stair_lost_t = None   # time when stair detection last dropped to zero
        self._pitch_peaked     = False  # True once pitch exceeded CLIMB_PITCH_HIGH during climb
        self._dir_ema          = 0.0    # EMA-smoothed direction_deg for yaw alignment
        self._dir_ema_init     = False  # True after first valid direction reading
        self._yaw_ok_count     = 0      # consecutive yaw-OK readings needed before lateral phase

        self.create_timer(0.1, self._loop)
        self.get_logger().info('StairController ready — IDLE')

    # ── Subscription callbacks ────────────────────────────────────────────────

    def _activate_cb(self, msg: Bool):
        if msg.data and self._state == 'IDLE':
            self.get_logger().info('Activated — RecoveryStand + BalanceStand then APPROACH')
            self._send_api(API_STAND)    # RecoveryStand — get up from any posture
            time.sleep(2.0)
            self._send_api(1002)         # BalanceStand — unlock velocity control mode
            time.sleep(0.5)
            self._transition('APPROACH')
        elif not msg.data and self._state != 'IDLE':
            self.get_logger().info('Deactivated by operator')
            self._abort('manual_stop')

    def _detected_cb(self, msg: Bool):
        if msg.data:
            self._detect_count = min(self._detect_count + 1, DETECT_CONFIRM + 5)
            self._stair_lost_t = None
        else:
            if self._detect_count > 0:
                self._detect_count -= 1
            if self._detect_count == 0 and self._stair_lost_t is None:
                self._stair_lost_t = time.time()

    def _geometry_cb(self, msg: String):
        try:
            self._geo = json.loads(msg.data)
            # Use bearing to first step (atan2(y, x)) as yaw reference.
            # When stair is directly ahead: x>0, y≈0 → bearing≈0°.
            # Positive bearing → step is LEFT → positive vyaw (CCW) turns left toward it.
            # Negative bearing → step is RIGHT → negative vyaw (CW) turns right toward it.
            fx = self._geo.get('first_step_x_m', 1.0)
            fy = self._geo.get('first_step_y_m', 0.0)
            d  = math.degrees(math.atan2(fy, fx))
            if not self._dir_ema_init:
                self._dir_ema      = d
                self._dir_ema_init = True
            else:
                diff = ((d - self._dir_ema + 180.0) % 360.0) - 180.0
                if abs(diff) > 30.0:   # skip large jumps — bearing should be stable
                    return
                self._dir_ema += DIR_EMA_ALPHA * diff
                if self._dir_ema > 180.0:
                    self._dir_ema -= 360.0
                elif self._dir_ema < -180.0:
                    self._dir_ema += 360.0
        except Exception:
            pass

    def _odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._yaw = math.atan2(siny, cosy)

    def _imu_cb(self, msg: Imu):
        q = msg.orientation
        sinr = 2.0 * (q.w * q.x + q.y * q.z)
        cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self._roll_deg  = math.degrees(math.atan2(sinr, cosr))
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        self._pitch_deg = math.degrees(math.asin(max(-1.0, min(1.0, sinp))))

    # ── Main 10 Hz control loop ───────────────────────────────────────────────

    def _loop(self):
        if self._state == 'IDLE':
            return

        self._state_pub.publish(String(data=self._state))

        if not self._safety_ok():
            return

        elapsed = time.time() - self._state_enter

        if   self._state == 'APPROACH':  self._do_approach(elapsed)
        elif self._state == 'ALIGN':     self._do_align(elapsed)
        elif self._state == 'CLIMBING':  self._do_climb(elapsed)
        elif self._state == 'LANDING':   self._do_landing()

    # ── State behaviours ──────────────────────────────────────────────────────

    def _do_approach(self, elapsed):
        if elapsed > APPROACH_TIMEOUT:
            self.get_logger().warn('Approach timeout — no confirmed stair found')
            self._abort('approach_timeout')
            return

        stair_confirmed = self._detect_count >= DETECT_CONFIRM and self._geo is not None

        if not stair_confirmed:
            self._move(0.2, 0.0, 0.0)   # creep forward
            return

        dist = self._geo.get('first_step_x_m', 999.0)
        lat  = self._geo.get('first_step_y_m', 0.0)
        vy   = self._clamp(-lat * 0.15, -ALIGN_LAT_VEL, ALIGN_LAT_VEL)

        if dist <= APPROACH_DIST:
            self._stop()
            self.get_logger().info(f'Stair at {dist:.2f} m — ALIGN')
            self._transition('ALIGN')
            return

        self._move(APPROACH_VEL, vy, 0.0)

    def _do_align(self, elapsed):
        if elapsed > ALIGN_TIMEOUT:
            self.get_logger().warn('Align timeout')
            self._abort('align_timeout')
            return

        if self._geo is None:
            self._stop()
            return

        # ── Phase 1: yaw — rotate until first step is directly ahead (bearing→0°) ─
        # _dir_ema holds EMA of atan2(first_step_y, first_step_x).
        # Target = 0°: step straight ahead.  Positive = step LEFT → turn LEFT (positive vyaw).
        yaw_err = self._dir_ema   # error from 0°
        yaw_ok  = abs(yaw_err) < ALIGN_YAW_TOL
        if not yaw_ok:
            self._yaw_ok_count = 0
            vyaw = self._clamp(math.radians(yaw_err) * 2.0, -ALIGN_ROT_VEL, ALIGN_ROT_VEL)
            self._move(0.0, 0.0, vyaw)
            self.get_logger().info(
                f'ALIGN yaw  bearing_ema={self._dir_ema:.1f}°  err={yaw_err:.1f}°  vyaw={vyaw:.3f}',
                throttle_duration_sec=2.0,
            )
            return
        self._yaw_ok_count += 1
        if self._yaw_ok_count < 5:
            self._stop()
            self.get_logger().info(
                f'ALIGN yaw OK {self._yaw_ok_count}/5  bearing_ema={self._dir_ema:.1f}°',
                throttle_duration_sec=0.5,
            )
            return

        # ── Phase 2: lateral centering ──────────────────────────────────────────
        lat    = self._geo.get('first_step_y_m', 0.0)
        lat_ok = abs(lat) < ALIGN_LAT_TOL
        self.get_logger().info(
            f'ALIGN lat  lat={lat:.3f}m  lat_ok={lat_ok}  bearing_ema={self._dir_ema:.1f}° (yaw OK)',
            throttle_duration_sec=2.0,
        )

        if lat_ok:
            self._stop()
            time.sleep(0.4)
            self.get_logger().info('Aligned (yaw + lateral OK) — CLIMBING')
            self._transition('CLIMBING')
            return

        vy = self._clamp(-lat * 0.2, -ALIGN_LAT_VEL, ALIGN_LAT_VEL)
        self._move(0.0, vy, 0.0)

    def _do_climb(self, elapsed):
        if elapsed > CLIMB_TIMEOUT:
            self.get_logger().warn('Climb timeout — forcing LANDING')
            self._stop()
            self._transition('LANDING')
            return

        # Track whether pitch has peaked (confirms the robot was actually climbing).
        if abs(self._pitch_deg) > CLIMB_PITCH_HIGH:
            self._pitch_peaked = True

        # Primary top-detection: pitch returns to flat after having peaked.
        # This fires the moment the robot crests the stairs onto level ground,
        # without waiting for the stair detection timer.
        if self._pitch_peaked and abs(self._pitch_deg) < CLIMB_PITCH_FLAT and elapsed > 3.0:
            self.get_logger().info(
                f'Pitch flat (pitch={self._pitch_deg:.1f}°) after climbing — reached top, LANDING'
            )
            self._stop()
            self._transition('LANDING')
            return

        # Fallback: stair detection gone long enough (handles gentle slopes where pitch stays low).
        # Only applies after pitch has peaked — prevents false LANDING from LiDAR dead zone
        # oscillations while the robot is still at the base of the stair.
        stair_gone_long_enough = (
            self._pitch_peaked and
            self._stair_lost_t is not None and
            time.time() - self._stair_lost_t > STAIR_LOST_TIMEOUT and
            elapsed > 3.0
        )
        if stair_gone_long_enough:
            self.get_logger().info('Stair no longer visible — reached top, LANDING')
            self._stop()
            self._transition('LANDING')
            return

        self.get_logger().info(
            f'CLIMBING  pitch={self._pitch_deg:.1f}°  peaked={self._pitch_peaked}',
            throttle_duration_sec=2.0,
        )
        lat = self._geo.get('first_step_y_m', 0.0) if self._geo else 0.0
        vy  = self._clamp(-lat * 0.12, -ALIGN_LAT_VEL, ALIGN_LAT_VEL)
        self._move(CLIMB_VEL, vy, 0.0)

    def _do_landing(self):
        self._stop()
        time.sleep(0.8)
        self.get_logger().info('Climb complete — returning control to operator')
        self._done_pub.publish(Bool(data=True))
        self._transition('IDLE')

    # ── Safety ────────────────────────────────────────────────────────────────

    def _safety_ok(self):
        if abs(self._pitch_deg) > MAX_PITCH_DEG:
            self.get_logger().error(f'SAFETY STOP: pitch={self._pitch_deg:.1f}°')
            self._abort('pitch_limit')
            return False
        if abs(self._roll_deg) > MAX_ROLL_DEG:
            self.get_logger().error(f'SAFETY STOP: roll={self._roll_deg:.1f}°')
            self._abort('roll_limit')
            return False
        return True

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _send_api(self, api_id, param=''):
        req = Request()
        req.header.identity.api_id = api_id
        req.parameter = param
        self._cmd_pub.publish(req)

    def _move(self, vx, vy, vz):
        self._send_api(API_MOVE, json.dumps({
            'x': round(vx, 4), 'y': round(vy, 4), 'z': round(vz, 4),
        }))

    def _stop(self):
        self._send_api(API_STOP)

    def _transition(self, new_state):
        self.get_logger().info(f'  {self._state} → {new_state}')
        self._state       = new_state
        self._state_enter = time.time()
        self._state_pub.publish(String(data=new_state))
        if new_state == 'CLIMBING':
            self._pitch_peaked = False
        if new_state == 'APPROACH':
            self._dir_ema      = 0.0
            self._dir_ema_init = False
            self._yaw_ok_count = 0
            self._detect_count = 0
            self._stair_lost_t = None
            self._geo          = None

    def _abort(self, reason):
        self.get_logger().error(f'Abort ({reason})')
        self._stop()
        self._done_pub.publish(Bool(data=True))
        self._transition('IDLE')

    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))


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
