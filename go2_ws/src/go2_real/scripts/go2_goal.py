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
from geometry_msgs.msg import PoseStamped
from unitree_api.msg import Request


class GoalSender(Node):
    def __init__(self, gx, gy, gyaw_rad):
        super().__init__('go2_goal')
        self._gx   = gx
        self._gy   = gy
        self._gyaw = gyaw_rad
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._req_pub = self.create_publisher(Request, '/api/sport/request_wifi', 10)
        self._sit_sent = False

    def send(self):
        self.get_logger().info(
            f'Goal (map): x={self._gx:.2f} y={self._gy:.2f} yaw={math.degrees(self._gyaw):.1f}°'
        )

        self.get_logger().info('Waiting for navigate_to_pose server...')
        self._client.wait_for_server()

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self._gx
        goal.pose.pose.position.y = self._gy
        goal.pose.pose.orientation.z = math.sin(self._gyaw / 2)
        goal.pose.pose.orientation.w = math.cos(self._gyaw / 2)

        future = self._client.send_goal_async(goal, feedback_callback=self._feedback)
        future.add_done_callback(self._goal_response)

    def _feedback(self, fb):
        self.get_logger().info(f'Distance remaining: {fb.feedback.distance_remaining:.2f} m')

    def _goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Goal REJECTED by Nav2')
            raise SystemExit(1)
        self.get_logger().info('Goal accepted — navigating...')
        handle.get_result_async().add_done_callback(self._result)

    def _laydown(self):
        if self._sit_sent:
            return
        self._sit_sent = True
        # Wait for Nav2 to publish cmd_vel=0 and for go2_bridge's 0.3s zero-holdoff
        # to expire so the bridge is fully silent before we issue StandDown.
        # (StopMove was removed — it makes a lying robot stand up to neutral stance.)
        self.get_logger().info('Waiting for bridge to go silent before lay-down...')
        time.sleep(2.0)
        down = Request()
        down.header.identity.api_id = 1005   # StandDown / lay down
        self._req_pub.publish(down)
        time.sleep(1.0)   # let DDS deliver the message before node is destroyed
        self.get_logger().info('Lay-down command sent')

    def _result(self, future):
        status = future.result().status
        if status == 4:
            self.get_logger().info('GOAL REACHED')
        else:
            self.get_logger().warn(f'Navigation ended with status {status}')
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


def main():
    if len(sys.argv) < 2:
        print('Usage: go2_goal.py <x> <y> [yaw_deg]')
        print('       go2_goal.py <waypoint_name>')
        sys.exit(1)

    # Named waypoint if first arg is not a number
    try:
        gx = float(sys.argv[1])
        if len(sys.argv) < 3:
            print('Usage: go2_goal.py <x> <y> [yaw_deg]')
            sys.exit(1)
        gy   = float(sys.argv[2])
        gyaw = math.radians(float(sys.argv[3])) if len(sys.argv) > 3 else 0.0
    except ValueError:
        gx, gy, gyaw = _load_waypoint(sys.argv[1])

    rclpy.init()
    node = GoalSender(gx, gy, gyaw)

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
