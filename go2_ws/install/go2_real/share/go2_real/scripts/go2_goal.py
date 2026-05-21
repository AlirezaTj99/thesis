#!/usr/bin/env python3
"""
go2_goal.py — Send a navigation goal to Nav2 NavigateToPose action server.

Goals are specified RELATIVE to the robot's current position and heading.
  x > 0  = forward,  x < 0  = backward
  y > 0  = left,     y < 0  = right
  yaw    = final heading offset in degrees (0 = keep current heading)

Usage:
    python3 go2_goal.py <x> <y>
    python3 go2_goal.py <x> <y> <yaw_deg>

Examples:
    go2_goal.py 1.5 0.0        go 1.5 m forward
    go2_goal.py 1.5 1.0 90     go to (1.5 m ahead, 1.0 m left) and face left
"""
import sys
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from unitree_api.msg import Request


def quat_to_yaw(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


class GoalSender(Node):
    def __init__(self, dx, dy, dyaw_rad):
        super().__init__('go2_goal')
        self._dx = dx
        self._dy = dy
        self._dyaw = dyaw_rad
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._tf_buf = Buffer()
        self._tf_listener = TransformListener(self._tf_buf, self)
        self._req_pub = self.create_publisher(Request, '/api/sport/request', 10)
        self._sit_sent = False

    def _get_robot_pose_in_map(self):
        """Compose map→odom and odom→base_link individually (avoids Foxy tf2 timing quirk)."""
        deadline = self.get_clock().now().nanoseconds / 1e9 + 8.0
        while True:
            try:
                mo = self._tf_buf.lookup_transform('map',  'odom',      rclpy.time.Time())
                ob = self._tf_buf.lookup_transform('odom', 'base_link', rclpy.time.Time())
                break
            except Exception:
                if self.get_clock().now().nanoseconds / 1e9 > deadline:
                    self.get_logger().error('Timed out waiting for TF')
                    raise SystemExit(1)
                rclpy.spin_once(self, timeout_sec=0.1)

        yaw_mo = quat_to_yaw(mo.transform.rotation)
        ox = ob.transform.translation.x
        oy = ob.transform.translation.y
        cx = mo.transform.translation.x + math.cos(yaw_mo)*ox - math.sin(yaw_mo)*oy
        cy = mo.transform.translation.y + math.sin(yaw_mo)*ox + math.cos(yaw_mo)*oy
        cyaw = yaw_mo + quat_to_yaw(ob.transform.rotation)
        return cx, cy, cyaw

    def send(self):
        self.get_logger().info('Looking up current robot pose in map frame...')
        cx, cy, cyaw = self._get_robot_pose_in_map()

        # Rotate requested offset by current heading, then add to current pose
        gx   = cx + math.cos(cyaw) * self._dx - math.sin(cyaw) * self._dy
        gy   = cy + math.sin(cyaw) * self._dx + math.cos(cyaw) * self._dy
        gyaw = cyaw + self._dyaw

        self.get_logger().info(
            f'Current pose: x={cx:.2f} y={cy:.2f} yaw={math.degrees(cyaw):.1f}°'
        )
        self.get_logger().info(
            f'Goal (map)  : x={gx:.2f} y={gy:.2f} yaw={math.degrees(gyaw):.1f}°'
        )

        self.get_logger().info('Waiting for navigate_to_pose server...')
        self._client.wait_for_server()

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
        stop = Request()
        stop.header.identity.api_id = 1003   # StopMove first
        self._req_pub.publish(stop)
        time.sleep(0.5)                       # let go2_bridge finish its last Move cmd
        down = Request()
        down.header.identity.api_id = 1004   # StandDown / lay down
        self._req_pub.publish(down)
        self.get_logger().info('Lay-down command sent')

    def _result(self, future):
        status = future.result().status
        if status == 4:
            self.get_logger().info('GOAL REACHED')
        else:
            self.get_logger().warn(f'Navigation ended with status {status}')
        self._laydown()
        raise SystemExit(0)


def main():
    if len(sys.argv) < 3:
        print('Usage: go2_goal.py <x> <y> [yaw_deg]')
        print('  x > 0 = forward, y > 0 = left, yaw = heading offset in degrees')
        sys.exit(1)

    dx   = float(sys.argv[1])
    dy   = float(sys.argv[2])
    dyaw = math.radians(float(sys.argv[3])) if len(sys.argv) > 3 else 0.0

    rclpy.init()
    node = GoalSender(dx, dy, dyaw)

    # Wake-up: stand the robot up and unlock velocity mode before sending goal.
    # Safe to call even when robot is already standing.
    req = Request()
    req.header.identity.api_id = 1006   # RecoveryStand — stand up from lying
    node._req_pub.publish(req)
    node.get_logger().info('RecoveryStand sent — standing up...')
    time.sleep(2.5)

    req = Request()
    req.header.identity.api_id = 1002   # BalanceStand — unlock velocity commands
    node._req_pub.publish(req)
    node.get_logger().info('BalanceStand sent — ready for navigation')
    time.sleep(1.5)

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
