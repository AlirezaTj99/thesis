#!/usr/bin/env python3
"""
obstacle_repulsion.py — Repulsive velocity field for Unitree Go2.

For each LaserScan beam within `influence_dist`, a repulsive vector is computed
pointing away from that obstacle (anti-beam direction), scaled quadratically by
penetration depth into the influence zone.  All vectors are summed to produce a
net Cartesian repulsive velocity in the robot frame, then clamped to `max_vel`.

Result: the robot automatically backs away from whatever is close to it.
Direction is naturally perpendicular to flat surfaces (wall, leg, box), and
diagonal for corners.

Test: run this node, then walk toward the robot — it moves away from you.

ROS topics
----------
  Subscribes : /scan              (sensor_msgs/LaserScan, RELIABLE)
  Publishes  : /api/sport/request (unitree_api/Request)

Parameters (set via --ros-args -p key:=value)
----------
  influence_dist  float  1.5     Distance [m] at which repulsion starts
  max_vel         float  0.5     Maximum repulsive speed [m/s].
                                 Go2 phone-app max ≈ 1.5 m/s; 0.5 is safe indoors.
  gain            float  1.0     Proportional scaling of computed velocity
  deadband_vel    float  0.03    Min speed [m/s] before sending — suppresses micro-jitter

Notes
-----
- Publishes at 20 Hz (keep-alive) so the Go2 safety timeout never fires.
- Works standalone (no Nav2 required), but needs go2_bridge + pointcloud_to_laserscan
  running to have a /scan topic.
- When nav2 is running simultaneously, both this node and go2_bridge publish to
  /api/sport/request.  The most-recent message wins; at 20 Hz vs Nav2's 5 Hz,
  repulsion commands dominate when active.
"""

import math
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
from unitree_api.msg import Request

BEST_EFFORT_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)

SPORT_API_MOVE         = 1008
SPORT_API_STOP         = 1003
SPORT_API_BALANCE_STAND = 1002


class ObstacleRepulsionNode(Node):

    def __init__(self):
        super().__init__('obstacle_repulsion')

        self.declare_parameter('influence_dist', 0.7)
        self.declare_parameter('max_vel',        0.5)
        self.declare_parameter('gain',           1.3)
        self.declare_parameter('deadband_vel',   0.03)

        self._d0       = self.get_parameter('influence_dist').value
        self._max_vel  = self.get_parameter('max_vel').value
        self._gain     = self.get_parameter('gain').value
        self._deadband = self.get_parameter('deadband_vel').value

        self._req_pub = self.create_publisher(Request, '/api/sport/request', 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, BEST_EFFORT_QOS)

        self._vx = 0.0
        self._vy = 0.0
        self.create_timer(0.05, self._timer_cb)   # 20 Hz

        # BalanceStand 2 s after start — ensures the robot accepts velocity cmds.
        # go2_bridge also sends this on startup; the duplicate is harmless.
        self._bs_timer = self.create_timer(2.0, self._send_balance_stand)

        self.get_logger().info(
            f'obstacle_repulsion ready  '
            f'influence={self._d0} m  max_vel={self._max_vel} m/s  gain={self._gain}'
        )

    # ------------------------------------------------------------------
    def _scan_cb(self, msg: LaserScan):
        fx, fy = 0.0, 0.0
        n_active = 0

        for i, dist in enumerate(msg.ranges):
            if math.isnan(dist) or math.isinf(dist):
                continue
            if dist < 0.05 or dist > self._d0:
                continue

            angle = msg.angle_min + i * msg.angle_increment

            # Linear weight: 0 at influence boundary → 1 at contact.
            # Strong enough at range to overcome the Nav2 zero-cmd interference.
            w = 1.0 - dist / self._d0

            # Anti-beam direction = away from obstacle
            fx -= w * math.cos(angle)
            fy -= w * math.sin(angle)
            n_active += 1

        if n_active == 0:
            self._vx = 0.0
            self._vy = 0.0
            return

        # Normalize direction (the summed force may be large but only direction matters)
        speed = math.hypot(fx, fy)
        if speed < 1e-9:
            self._vx = 0.0
            self._vy = 0.0
            return

        nx, ny = fx / speed, fy / speed   # unit repulsion direction

        # Per-beam average weight: 0..1, maps to 0..max_vel after gain
        avg_w = speed / n_active
        vel   = min(self._max_vel, self._gain * self._max_vel * avg_w)

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

    # ------------------------------------------------------------------
    def _send_balance_stand(self):
        req = Request()
        req.header.identity.api_id = SPORT_API_BALANCE_STAND
        self._req_pub.publish(req)
        self.get_logger().info('BalanceStand sent')
        self.destroy_timer(self._bs_timer)

    def _timer_cb(self):
        # Only publish when actively repelling — silence lets Nav2 commands through
        if self._vx != 0.0 or self._vy != 0.0:
            self._send(self._vx, self._vy, 0.0)

    def _send(self, vx: float, vy: float, vyaw: float):
        req = Request()
        req.header.identity.api_id = SPORT_API_MOVE
        req.parameter = json.dumps({
            'x': round(float(vx),   4),
            'y': round(float(vy),   4),
            'z': round(float(vyaw), 4),
        })
        self._req_pub.publish(req)


# ------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = ObstacleRepulsionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Stopping robot...')
        node._send(0.0, 0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
