#!/usr/bin/env python3
"""
map_repulsion.py — Costmap-based repulsive velocity field for Go2.

Reads the global costmap and generates a repulsive velocity pushing the robot
away from high-cost cells within `influence_dist`, mirroring obstacle_repulsion
logic exactly (same weight formula, same gain/scaling, same deadband).

The output is in the robot frame and is forwarded by repulsion_gate alongside
the sensor-based repulsion signal.

ROS topics
----------
  Subscribes : /global_costmap/costmap   (nav_msgs/OccupancyGrid)
  Publishes  : /map_repulsion_vel_raw    (geometry_msgs/Twist)
                 — summed and gated by repulsion_gate → /repulsion_vel

Parameters (set via --ros-args -p key:=value)
----------
  influence_dist  float  0.7    [m] radius in which high-cost cells generate repulsion
  max_vel         float  0.6    Maximum repulsive speed [m/s]
  gain            float  1.3    Proportional scaling — matches obstacle_repulsion default
  deadband_vel    float  0.03   Min speed [m/s] before publishing
  cost_threshold  int    50     OccupancyGrid cost (0-100) above which a cell is active
                                50 ≈ start of Nav2 inflation zone
"""

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time as RclpyTime
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from action_msgs.msg import GoalStatusArray
from tf2_ros import Buffer, TransformListener


class MapRepulsionNode(Node):

    def __init__(self):
        super().__init__('map_repulsion')

        self.declare_parameter('influence_dist', 0.7)
        self.declare_parameter('max_vel',        0.6)
        self.declare_parameter('gain',           1.3)
        self.declare_parameter('deadband_vel',   0.03)
        self.declare_parameter('cost_threshold', 50)

        self._d0       = self.get_parameter('influence_dist').value
        self._max_vel  = self.get_parameter('max_vel').value
        self._gain     = self.get_parameter('gain').value
        self._deadband = self.get_parameter('deadband_vel').value
        self._cost_thr = float(self.get_parameter('cost_threshold').value)

        self._costmap: Optional[OccupancyGrid] = None
        self._nav_active = False

        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 self._costmap_cb, 10)
        self.create_subscription(GoalStatusArray,
                                 '/navigate_to_pose/_action/status',
                                 self._nav_status_cb, 10)
        self._pub = self.create_publisher(Twist, '/map_repulsion_vel_raw', 10)
        self.create_timer(0.05, self._timer_cb)  # 20 Hz

        self.get_logger().info(
            f'map_repulsion ready  influence={self._d0} m  '
            f'max_vel={self._max_vel:.2f} m/s  gain={self._gain}  '
            f'cost_threshold={self._cost_thr:.0f}'
        )

    def _costmap_cb(self, msg: OccupancyGrid):
        self._costmap = msg

    def _nav_status_cb(self, msg: GoalStatusArray):
        was_active = self._nav_active
        # ACCEPTED=1 or EXECUTING=2 means a goal is live
        self._nav_active = any(gs.status in (1, 2) for gs in msg.status_list)
        if self._nav_active != was_active:
            if self._nav_active:
                self.get_logger().info('Navigation active — map repulsion ENABLED')
            else:
                self.get_logger().info('Navigation ended — map repulsion DISABLED')

    def _timer_cb(self):
        zero = Twist()

        if not self._nav_active:
            self._pub.publish(zero)
            return

        if self._costmap is None:
            self._pub.publish(zero)
            return

        # ── Robot pose in map frame ───────────────────────────────────────────
        try:
            tf = self._tf_buffer.lookup_transform('map', 'base_link', RclpyTime())
        except Exception:
            self._pub.publish(zero)
            return

        rx  = tf.transform.translation.x
        ry  = tf.transform.translation.y
        q   = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        # ── Sum repulsion vectors over nearby high-cost cells (map frame) ─────
        cm  = self._costmap
        res = cm.info.resolution
        ox  = cm.info.origin.position.x
        oy  = cm.info.origin.position.y
        cw  = cm.info.width
        ch  = cm.info.height

        r_cells = int(math.ceil(self._d0 / res)) + 1
        cx0 = int((rx - ox) / res)
        cy0 = int((ry - oy) / res)

        fx, fy   = 0.0, 0.0
        n_active = 0

        for dcx in range(-r_cells, r_cells + 1):
            for dcy in range(-r_cells, r_cells + 1):
                ccx = cx0 + dcx
                ccy = cy0 + dcy
                if not (0 <= ccx < cw and 0 <= ccy < ch):
                    continue
                cost = cm.data[ccy * cw + ccx]
                if cost < 0 or cost <= self._cost_thr:   # -1 = unknown; skip low-cost
                    continue

                # Cell centre in world frame
                wx = ox + (ccx + 0.5) * res
                wy = oy + (ccy + 0.5) * res
                dx = rx - wx
                dy = ry - wy
                dist = math.sqrt(dx * dx + dy * dy)

                if dist < 1e-6 or dist >= self._d0:
                    continue

                # Mirrors obstacle_repulsion: linear weight, 0 at boundary → 1 at contact
                w = 1.0 - dist / self._d0
                fx += (dx / dist) * w
                fy += (dy / dist) * w
                n_active += 1

        if n_active == 0:
            self._pub.publish(zero)
            return

        speed = math.hypot(fx, fy)
        if speed < 1e-9:
            self._pub.publish(zero)
            return

        # ── Scale exactly as obstacle_repulsion does ──────────────────────────
        nx, ny = fx / speed, fy / speed
        avg_w  = speed / n_active
        vel    = min(self._max_vel, self._gain * self._max_vel * avg_w)

        if vel < self._deadband:
            self._pub.publish(zero)
            return

        # ── Rotate direction from map frame → robot frame ─────────────────────
        cos_y =  math.cos(yaw)
        sin_y =  math.sin(yaw)
        vx_r  =  nx * cos_y + ny * sin_y
        vy_r  = -nx * sin_y + ny * cos_y

        self.get_logger().debug(
            f'active_cells={n_active}  avg_w={avg_w:.3f}  vel={vel:.3f}  '
            f'vx_r={vx_r * vel:.3f}  vy_r={vy_r * vel:.3f}'
        )

        msg = Twist()
        msg.linear.x = vx_r * vel
        msg.linear.y = vy_r * vel
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapRepulsionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
