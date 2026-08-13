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
ALIGN_YAW_TOL      = 0.10   # rad (~6°) — acceptable yaw error to stair axis
ALIGN_LAT_TOL      = 0.12   # m    — acceptable lateral offset to stair centre
ALIGN_ROT_VEL      = 0.22   # rad/s
ALIGN_LAT_VEL      = 0.08   # m/s
ALIGN_TIMEOUT      = 20.0   # s
CLIMB_VEL          = 0.15   # m/s  — slow and steady up the steps
CLIMB_TIMEOUT      = 45.0   # s    — hard abort
STAIR_LOST_TIMEOUT = 4.0    # s    — stair gone this long = reached the top
DETECT_CONFIRM     = 3      # consecutive positive detections before trusting

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

        self.create_timer(0.1, self._loop)
        self.get_logger().info('StairController ready — IDLE')

    # ── Subscription callbacks ────────────────────────────────────────────────

    def _activate_cb(self, msg: Bool):
        if msg.data and self._state == 'IDLE':
            self.get_logger().info('Activated — RecoveryStand then APPROACH')
            self._send_api(API_STAND)
            time.sleep(2.0)
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
            self._move(APPROACH_VEL * 0.4, 0.0, 0.0)   # creep forward
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

        yaw_err = math.radians(self._geo.get('direction_deg', 0.0))
        lat     = self._geo.get('first_step_y_m', 0.0)
        yaw_ok  = abs(yaw_err) < ALIGN_YAW_TOL
        lat_ok  = abs(lat)     < ALIGN_LAT_TOL

        if yaw_ok and lat_ok:
            self._stop()
            time.sleep(0.4)
            self.get_logger().info('Aligned — CLIMBING')
            self._transition('CLIMBING')
            return

        vz = 0.0 if yaw_ok else self._clamp(-yaw_err * 0.8, -ALIGN_ROT_VEL, ALIGN_ROT_VEL)
        vy = 0.0 if lat_ok  else self._clamp(-lat    * 0.2,  -ALIGN_LAT_VEL, ALIGN_LAT_VEL)
        self._move(0.0, vy, vz)

    def _do_climb(self, elapsed):
        if elapsed > CLIMB_TIMEOUT:
            self.get_logger().warn('Climb timeout — forcing LANDING')
            self._stop()
            self._transition('LANDING')
            return

        stair_gone_long_enough = (
            self._stair_lost_t is not None and
            time.time() - self._stair_lost_t > STAIR_LOST_TIMEOUT and
            elapsed > 3.0   # must have climbed for at least 3 s
        )
        if stair_gone_long_enough:
            self.get_logger().info('Stair no longer visible — reached top, LANDING')
            self._stop()
            self._transition('LANDING')
            return

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
