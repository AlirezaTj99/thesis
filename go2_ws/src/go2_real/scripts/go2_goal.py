#!/usr/bin/env python3
"""
go2_goal.py — Send a navigation goal to Nav2 NavigateToPose action server.

Goals can be given as coordinates or as a named waypoint from ~/maps/waypoints.yaml.

  x, y  = map frame coordinates in metres
  yaw   = final heading in degrees (0 = facing map +x axis)

Usage:
    python3 go2_goal.py <x> <y>               go to coordinates
    python3 go2_goal.py <x> <y> <yaw_deg>     go to coordinates with heading
    python3 go2_goal.py <waypoint_name>        go to named waypoint

Examples:
    go2_goal.py 2.0 0.0          go to (2.0, 0.0) in the map
    go2_goal.py 2.0 1.5 90       go to (2.0, 1.5) and face map +y direction
    go2_goal.py home             go to the 'home' waypoint
    go2_goal.py node1            go to 'node1' from ~/maps/waypoints.yaml
"""
import os
import sys
import math
import time
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from unitree_api.msg import Request


class GoalSender(Node):
    def __init__(self, goals):
        super().__init__('go2_goal')
        self._goals   = list(goals)   # list of (gx, gy, gyaw)
        self._current = 0
        self._total   = len(goals)
        self._client  = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._req_pub = self.create_publisher(Request, '/api/sport/request_wifi', 10)
        self._sit_sent = False
        self._cmd_vel = Twist()
        self.create_subscription(Twist, '/cmd_vel', self._vel_cb, 10)

    def send(self):
        gx, gy, gyaw = self._goals[self._current]
        self.get_logger().info(
            f'Goal {self._current + 1}/{self._total} (map): '
            f'x={gx:.2f} y={gy:.2f} yaw={math.degrees(gyaw):.1f}°'
        )

        if self._current == 0:
            self.get_logger().info('Waiting for navigate_to_pose server...')
            if not self._client.wait_for_server(timeout_sec=30.0):
                self.get_logger().error(
                    'navigate_to_pose server not available after 30 s — '
                    'is the Nav2 stack running? Check the Navigation terminal.'
                )
                raise SystemExit(1)
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy
        goal.pose.pose.orientation.z = math.sin(gyaw / 2)
        goal.pose.pose.orientation.w = math.cos(gyaw / 2)

        future = self._client.send_goal_async(goal, feedback_callback=self._feedback)
        future.add_done_callback(self._goal_response)

    def _vel_cb(self, msg):
        self._cmd_vel = msg

    def _feedback(self, fb):
        v = self._cmd_vel
        self.get_logger().info(
            f'[{self._current + 1}/{self._total}] dist={fb.feedback.distance_remaining:.2f}m  '
            f'cmd_vel: lin.x={v.linear.x:+.3f}  lin.y={v.linear.y:+.3f}  ang.z={v.angular.z:+.3f}'
        )

    def _goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Goal REJECTED by Nav2')
            raise SystemExit(1)
        self.get_logger().info(f'Goal {self._current + 1}/{self._total} accepted — navigating...')
        handle.get_result_async().add_done_callback(self._result)

    def _laydown(self):
        if self._sit_sent:
            return
        self._sit_sent = True
        self.get_logger().info('Waiting for bridge to go silent before lay-down...')
        time.sleep(2.0)
        down = Request()
        down.header.identity.api_id = 1005   # StandDown / lay down
        self._req_pub.publish(down)
        time.sleep(1.0)
        self.get_logger().info('Lay-down command sent')

    def _result(self, future):
        status = future.result().status
        if status == 4:
            self.get_logger().info(f'Waypoint {self._current + 1}/{self._total} REACHED')
            self._current += 1
            if self._current < self._total:
                self.get_logger().info('Moving to next waypoint...')
                self.send()
                return
            self.get_logger().info('All waypoints reached!')
        else:
            self.get_logger().warn(
                f'Waypoint {self._current + 1}/{self._total} failed (status {status}) — aborting chain'
            )
        self._laydown()
        raise SystemExit(0)


def _load_waypoint(name):
    path = os.path.expanduser('~/maps/waypoints.yaml')
    if not os.path.exists(path):
        print(f'Error: waypoints file not found: {path}')
        print('Create ~/maps/waypoints.yaml with your named locations.')
        sys.exit(1)
    with open(path) as f:
        data = yaml.safe_load(f)
    wps = data.get('waypoints', {})
    if name not in wps:
        available = list(wps.keys())
        print(f'Error: waypoint "{name}" not found.')
        print(f'Available waypoints: {available}')
        sys.exit(1)
    wp = wps[name]
    return float(wp['x']), float(wp['y']), math.radians(float(wp.get('yaw', 0)))


def _parse_goals(argv):
    """Parse argv[1:] into a list of (x, y, yaw_rad) tuples.

    Each token is either a named waypoint or the start of an x y [yaw] triple.
    Mixed usage is fine: go2_goal.py home 3.0 1.5 90 kitchen
    """
    goals = []
    i = 1
    while i < len(argv):
        try:
            x = float(argv[i])
            if i + 1 >= len(argv):
                print(f'Error: x={x} provided but no y coordinate follows it')
                sys.exit(1)
            y = float(argv[i + 1])
            i += 2
            if i < len(argv):
                try:
                    yaw = math.radians(float(argv[i]))
                    i += 1
                except ValueError:
                    yaw = 0.0
            else:
                yaw = 0.0
            goals.append((x, y, yaw))
        except ValueError:
            x, y, yaw = _load_waypoint(argv[i])
            goals.append((x, y, yaw))
            i += 1
    return goals


def main():
    if len(sys.argv) < 2:
        print('Usage: go2_goal.py <waypoint_name> [waypoint_name2 ...]')
        print('       go2_goal.py <x> <y> [yaw_deg]')
        sys.exit(1)

    goals = _parse_goals(sys.argv)
    if not goals:
        print('Error: no valid goals parsed from arguments')
        sys.exit(1)

    rclpy.init()
    node = GoalSender(goals)

    # Allow DDS to discover go2_bridge before sending any commands.
    # Without this wait the first publish is lost (bridge not yet visible).
    rclpy.spin_once(node, timeout_sec=2.0)

    # Wake-up: stand the robot up and unlock velocity mode before sending goal.
    # Safe to call even when robot is already standing.
    req = Request()
    req.header.identity.api_id = 1006   # RecoveryStand — stand up from lying
    node._req_pub.publish(req)
    node.get_logger().info('RecoveryStand sent — standing up...')
    rclpy.spin_once(node, timeout_sec=0.1)
    time.sleep(2.5)

    req = Request()
    req.header.identity.api_id = 1002   # BalanceStand — unlock velocity commands
    node._req_pub.publish(req)
    node.get_logger().info('BalanceStand sent — ready for navigation')
    rclpy.spin_once(node, timeout_sec=0.1)
    time.sleep(4.0)   # wait for robot to fully stand and AMCL to publish fresh TF

    node.send()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node._laydown()   # lay down on Ctrl+C or goal reached (idempotent)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
