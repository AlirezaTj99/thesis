#!/usr/bin/env python3
"""
go2_goal.py — Navigate through a waypoint sequence, with stair-pair detection.

When two consecutive waypoints both carry  property: stair  in waypoints.yaml,
the node:
  1. Inserts an approach alignment waypoint APPROACH_DIST m before stair_entry
     on the exact bearing toward stair_exit, and navigates there via Nav2.
  2. Navigates to stair_entry (final yaw = stair bearing) via Nav2.
  3. Publishes straight-line target (bearing_rad, line_length_m) to
     /stair_controller/target, then publishes True to /stair_controller/activate.
  4. Waits for /stair_controller/done.
  5. Clears local costmap, then global costmap, waits 0.5 s, then resumes Nav2
     at the waypoint AFTER stair_exit (stair_exit itself is skipped).

Repulsion is automatically re-enabled by repulsion_gate.py the moment
stair_controller publishes state='IDLE' — no explicit signal needed here.

Usage:
    go2_goal.py <waypoint_name> [waypoint_name2 ...]
    go2_goal.py <x> <y> [yaw_deg]
    go2_goal.py home stair_point_1 stair_point_2 microwave

Mixed usage is fine:  go2_goal.py home 3.0 1.5 90 kitchen
"""
import json
import math
import os
import sys
import time
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from unitree_api.msg import Request
from tf2_ros import Buffer, TransformListener


APPROACH_DIST      = 1.0   # m — alignment waypoint inserted this far before stair_entry
ALIGN_VERIFY_TOL   = 0.15  # rad (~9°) — heading error below this → activate stair controller
ALIGN_MAX_RETRIES  = 3     # re-send stair_entry Nav2 goal this many times before giving up


class GoalSender(Node):

    def __init__(self, goals):
        super().__init__('go2_goal')
        # goals: list of dicts  {name, x, y, yaw_rad, property}
        self._goals   = goals
        self._current = 0
        self._total   = len(goals)

        self._client  = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._req_pub = self.create_publisher(Request, '/api/sport/request_wifi', 10)

        # Stair controller interface
        self._target_pub   = self.create_publisher(String, '/stair_controller/target',   10)
        self._activate_pub = self.create_publisher(Bool,   '/stair_controller/activate', 10)
        self.create_subscription(Bool, '/stair_controller/done', self._stair_done_cb, 10)

        # Costmap clear service clients
        self._clear_local  = self.create_client(
            ClearEntireCostmap,
            '/local_costmap/clear_entirely_local_costmap',
        )
        self._clear_global = self.create_client(
            ClearEntireCostmap,
            '/global_costmap/clear_entirely_global_costmap',
        )

        # Stair state machine
        self._stair_phase        = None    # None | 'approach' | 'entry' | 'climbing'
        self._stair_entry        = None    # dict: the stair_entry waypoint
        self._stair_bearing      = None    # radians — entry→exit direction in MAP frame
        self._stair_bearing_odom = None    # radians — same bearing converted to ODOM frame via TF
        self._stair_length       = None    # metres  — entry→exit distance
        self._stair_align_retries = 0
        self._resume_timer       = None

        # TF listener — used to convert map-frame bearing to odom frame for stair_controller
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._nav2_ready = False
        self._sit_sent   = False

        # Odometry — used for heading verification before stair handoff
        self._odom_yaw = 0.0
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

        self._cmd_vel = Twist()
        self.create_subscription(Twist, '/cmd_vel', self._vel_cb, 10)

    # ── ROS callbacks ──────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._odom_yaw = math.atan2(siny, cosy)

    def _vel_cb(self, msg):
        self._cmd_vel = msg

    def _stair_done_cb(self, msg: Bool):
        if not msg.data or self._stair_phase != 'climbing':
            return
        self.get_logger().info('Stair controller done — clearing costmaps then resuming')
        self._stair_phase = None
        self._stair_entry = None
        self._do_clear_local()

    # ── Costmap clearing chain: local → global → 0.5 s delay → resume ─────────

    def _do_clear_local(self):
        if self._clear_local.service_is_ready():
            fut = self._clear_local.call_async(ClearEntireCostmap.Request())
            fut.add_done_callback(lambda _: self._do_clear_global())
        else:
            self.get_logger().warn('Local costmap clear service not ready — skipping')
            self._do_clear_global()

    def _do_clear_global(self):
        if self._clear_global.service_is_ready():
            fut = self._clear_global.call_async(ClearEntireCostmap.Request())
            fut.add_done_callback(lambda _: self._after_clear())
        else:
            self.get_logger().warn('Global costmap clear service not ready — skipping')
            self._after_clear()

    def _after_clear(self):
        self.get_logger().info('Costmaps cleared — waiting 0.5 s before resuming Nav2')
        self._resume_timer = self.create_timer(0.5, self._resume_once)

    def _resume_once(self):
        self._resume_timer.cancel()
        self._resume_timer = None
        # _current points at stair_entry; skip stair_exit (+1), jump to +2
        self._current += 2
        if self._current < self._total:
            self.get_logger().info(
                f'Resuming at waypoint {self._current + 1}/{self._total}: '
                f'{self._goals[self._current]["name"]}'
            )
            self.send()
        else:
            self.get_logger().info('All waypoints completed after stair climb')
            self._laydown()
            raise SystemExit(0)

    # ── Main goal dispatch ─────────────────────────────────────────────────────

    def send(self):
        idx = self._current
        wp  = self._goals[idx]

        # Stair look-ahead: current AND next both have property='stair' → stair mode
        if (self._stair_phase is None
                and wp.get('property') == 'stair'
                and idx + 1 < self._total
                and self._goals[idx + 1].get('property') == 'stair'):
            self.get_logger().info(
                f'Stair pair: {wp["name"]} → {self._goals[idx + 1]["name"]}'
            )
            self._begin_stair_approach(wp, self._goals[idx + 1])
            return

        self._send_nav2_goal(wp['x'], wp['y'], wp['yaw_rad'], label=wp['name'])

    def _begin_stair_approach(self, entry, exit_wp):
        bearing = math.atan2(
            exit_wp['y'] - entry['y'],
            exit_wp['x'] - entry['x'],
        )
        length = math.hypot(
            exit_wp['x'] - entry['x'],
            exit_wp['y'] - entry['y'],
        )

        self._stair_entry        = entry
        self._stair_bearing      = bearing
        self._stair_bearing_odom = None   # computed from TF just before activation
        self._stair_length       = length
        self._stair_align_retries = 0

        # Approach point: APPROACH_DIST m before entry, on the entry→exit bearing
        ap_x = entry['x'] - math.cos(bearing) * APPROACH_DIST
        ap_y = entry['y'] - math.sin(bearing) * APPROACH_DIST

        self._stair_phase = 'approach'
        self.get_logger().info(
            f'Approach waypoint: ({ap_x:.2f}, {ap_y:.2f})  '
            f'bearing={math.degrees(bearing):.1f}°  stair_len={length:.2f} m'
        )
        self._send_nav2_goal(ap_x, ap_y, bearing, label='stair-approach')

    # ── Nav2 goal plumbing ─────────────────────────────────────────────────────

    def _send_nav2_goal(self, gx, gy, gyaw, label=''):
        if not self._nav2_ready:
            self.get_logger().info('Waiting for navigate_to_pose server...')
            if not self._client.wait_for_server(timeout_sec=30.0):
                self.get_logger().error(
                    'navigate_to_pose not available after 30 s — is Nav2 running?'
                )
                raise SystemExit(1)
            self._nav2_ready = True

        self.get_logger().info(
            f'[{label}]  x={gx:.2f}  y={gy:.2f}  yaw={math.degrees(gyaw):.1f}°'
        )

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp    = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy
        goal.pose.pose.orientation.z = math.sin(gyaw / 2)
        goal.pose.pose.orientation.w = math.cos(gyaw / 2)

        future = self._client.send_goal_async(goal, feedback_callback=self._feedback)
        future.add_done_callback(self._goal_response)

    def _feedback(self, fb):
        v = self._cmd_vel
        self.get_logger().info(
            f'[{self._current + 1}/{self._total}] phase={self._stair_phase or "nav"}  '
            f'dist={fb.feedback.distance_remaining:.2f} m  '
            f'vx={v.linear.x:+.3f}  wz={v.angular.z:+.3f}',
            throttle_duration_sec=2.0,
        )

    def _goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Goal REJECTED by Nav2')
            raise SystemExit(1)
        self.get_logger().info('Goal accepted')
        handle.get_result_async().add_done_callback(self._result)

    def _result(self, future):
        status = future.result().status

        # ── Stair approach phase ───────────────────────────────────────────────
        if self._stair_phase == 'approach':
            if status == 4:
                self.get_logger().info('Approach point reached — navigating to stair entry')
                self._stair_phase = 'entry'
                e = self._stair_entry
                self._send_nav2_goal(e['x'], e['y'], self._stair_bearing,
                                     label='stair-entry')
            else:
                self.get_logger().error(
                    f'Approach navigation failed (status {status}) — aborting'
                )
                self._stair_phase = None
                self._laydown()
                raise SystemExit(1)
            return

        # ── Stair entry phase ──────────────────────────────────────────────────
        if self._stair_phase == 'entry':
            if status == 4:
                self.get_logger().info('Stair entry reached — checking heading alignment')
                self._check_stair_heading()
            else:
                self.get_logger().error(
                    f'Stair entry navigation failed (status {status}) — aborting'
                )
                self._stair_phase = None
                self._laydown()
                raise SystemExit(1)
            return

        # ── Normal waypoint ────────────────────────────────────────────────────
        if status == 4:
            self.get_logger().info(
                f'Waypoint {self._current + 1}/{self._total} REACHED: '
                f'{self._goals[self._current]["name"]}'
            )
            self._current += 1
            if self._current < self._total:
                self.send()
                return
            self.get_logger().info('All waypoints reached')
        else:
            self.get_logger().warn(
                f'Waypoint {self._current + 1}/{self._total} failed (status {status}) — aborting'
            )
        self._laydown()
        raise SystemExit(0)

    # ── Heading verification before stair handoff ──────────────────────────────

    def _bearing_in_odom(self, bearing_map):
        """Convert a bearing in map frame to odom frame using the live map→odom TF."""
        try:
            t = self._tf_buffer.lookup_transform('odom', 'map', rclpy.time.Time())
            q = t.transform.rotation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            odom_from_map_yaw = math.atan2(siny, cosy)
            return bearing_map + odom_from_map_yaw
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e} — using map bearing as fallback')
            return bearing_map

    def _check_stair_heading(self):
        bearing_odom = self._bearing_in_odom(self._stair_bearing)
        self._stair_bearing_odom = bearing_odom

        err = (bearing_odom - self._odom_yaw + math.pi) % (2 * math.pi) - math.pi
        self.get_logger().info(
            f'Heading check: odom_yaw={math.degrees(self._odom_yaw):.1f}°  '
            f'target_map={math.degrees(self._stair_bearing):.1f}°  '
            f'target_odom={math.degrees(bearing_odom):.1f}°  '
            f'err={math.degrees(err):.1f}°'
        )
        if abs(err) < ALIGN_VERIFY_TOL:
            self.get_logger().info('Heading verified — activating stair controller')
        else:
            self.get_logger().info(
                f'Heading off by {math.degrees(err):.1f}° — stair controller will align in odom frame'
            )
        self._activate_stair_controller()

    # ── Stair controller activation ────────────────────────────────────────────

    def _activate_stair_controller(self):
        # Use odom-frame bearing so stair_controller's odom-based alignment and
        # cross-track calculations are in the same frame.
        bearing_to_send = self._stair_bearing_odom if self._stair_bearing_odom is not None \
                          else self._stair_bearing
        target = json.dumps({
            'bearing_rad':   round(bearing_to_send, 6),
            'line_length_m': round(self._stair_length, 4),
        })
        self._target_pub.publish(String(data=target))
        self.get_logger().info(
            f'Stair target → {target}  '
            f'(map={math.degrees(self._stair_bearing):.1f}°  '
            f'odom={math.degrees(bearing_to_send):.1f}°)'
        )

        # Give stair_controller one spin cycle to receive target before activate
        time.sleep(0.15)

        self._activate_pub.publish(Bool(data=True))
        self._stair_phase = 'climbing'
        self.get_logger().info(
            'Stair controller activated — Nav2 idle, repulsion zeroed, waiting for done'
        )

    # ── Lay-down ───────────────────────────────────────────────────────────────

    def _laydown(self):
        if self._sit_sent:
            return
        self._sit_sent = True
        self.get_logger().info('Waiting for bridge to go silent before lay-down...')
        time.sleep(2.0)
        down = Request()
        down.header.identity.api_id = 1005   # StandDown
        self._req_pub.publish(down)
        time.sleep(1.0)
        self.get_logger().info('Lay-down command sent')


# ── YAML / argv helpers ────────────────────────────────────────────────────────

def _load_all_waypoints():
    path = os.path.expanduser('~/maps/waypoints.yaml')
    if not os.path.exists(path):
        print(f'Error: waypoints file not found: {path}')
        sys.exit(1)
    with open(path) as f:
        data = yaml.safe_load(f)
    return data.get('waypoints', {})


def _lookup_waypoint(name, wps):
    if name not in wps:
        print(f'Error: waypoint "{name}" not found.')
        print(f'Available: {list(wps.keys())}')
        sys.exit(1)
    wp = wps[name]
    return {
        'name':     name,
        'x':        float(wp['x']),
        'y':        float(wp['y']),
        'yaw_rad':  math.radians(float(wp.get('yaw', 0))),
        'property': wp.get('property', 'floor'),
    }


def _parse_goals(argv):
    """Parse argv[1:] into a list of goal dicts.

    Each token is either a named waypoint or the start of an x y [yaw_deg] triple.
    Mixed usage is supported: go2_goal.py home 3.0 1.5 90 kitchen
    """
    wps   = _load_all_waypoints()
    goals = []
    i = 1
    while i < len(argv):
        try:
            x = float(argv[i])
            if i + 1 >= len(argv):
                print(f'Error: x={x} has no y coordinate')
                sys.exit(1)
            y = float(argv[i + 1])
            i += 2
            yaw_deg = 0.0
            if i < len(argv):
                try:
                    yaw_deg = float(argv[i])
                    i += 1
                except ValueError:
                    pass
            goals.append({
                'name':     f'({x:.2f},{y:.2f})',
                'x':        x,
                'y':        y,
                'yaw_rad':  math.radians(yaw_deg),
                'property': 'floor',
            })
        except ValueError:
            goals.append(_lookup_waypoint(argv[i], wps))
            i += 1
    return goals


def main():
    if len(sys.argv) < 2:
        print('Usage:  go2_goal.py <waypoint_name> [waypoint_name2 ...]')
        print('        go2_goal.py <x> <y> [yaw_deg]')
        sys.exit(1)

    goals = _parse_goals(sys.argv)
    if not goals:
        print('Error: no valid goals parsed')
        sys.exit(1)

    rclpy.init()
    node = GoalSender(goals)

    # Allow DDS to discover go2_bridge before sending commands
    rclpy.spin_once(node, timeout_sec=2.0)

    # Stand up and unlock velocity mode
    req = Request()
    req.header.identity.api_id = 1006   # RecoveryStand
    node._req_pub.publish(req)
    node.get_logger().info('RecoveryStand sent — standing up...')
    rclpy.spin_once(node, timeout_sec=0.1)
    time.sleep(2.5)

    req = Request()
    req.header.identity.api_id = 1002   # BalanceStand
    node._req_pub.publish(req)
    node.get_logger().info('BalanceStand sent — ready for navigation')
    rclpy.spin_once(node, timeout_sec=0.1)
    time.sleep(4.0)

    node.send()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node._laydown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
