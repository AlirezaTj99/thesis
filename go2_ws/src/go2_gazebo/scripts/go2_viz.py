#!/usr/bin/env python3
import math
import os
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.srv import ClearEntireCostmap

WAYPOINTS_FILE = os.path.expanduser('~/maps/waypoints.yaml')


class Go2VizNode(Node):
    def __init__(self):
        super().__init__('go2_viz')
        self.goal_pub   = self.create_publisher(Marker,      '/viz/goal_marker', 10)
        self.arrows_pub = self.create_publisher(MarkerArray, '/viz/path_arrows', 10)
        self._wp_pub    = self.create_publisher(MarkerArray, '/viz/waypoints',   10)
        self.create_subscription(Path, '/plan', self.plan_cb, 10)

        self._wp_mtime = None
        self._cached_waypoints = {}
        self.create_timer(5.0, self._waypoints_timer_cb)  # republish waypoints every 5 s

        # Velocity watchdog — stops the robot and clears stale costmap data when
        # Nav2 aborts. controller_server publishes at 5 Hz during navigation, so a
        # 0.5 s gap reliably means the goal ended (reached, aborted, or cancelled).
        reliable_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self._stop_pub = self.create_publisher(Twist, '/cmd_vel', reliable_qos)
        self.create_subscription(Twist, '/cmd_vel', self._cmdvel_cb, reliable_qos)
        self._last_cmdvel_time = None
        self._last_was_nonzero = False

        # Costmap clear clients — called after every abort so drifted/stale
        # obstacles don't block the next navigation attempt.
        self._local_clear  = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
        self._global_clear = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')

        self.create_timer(0.1, self._watchdog_cb)

    def _cmdvel_cb(self, msg: Twist):
        self._last_cmdvel_time = self.get_clock().now()
        self._last_was_nonzero = (
            abs(msg.linear.x) > 0.001 or
            abs(msg.linear.y) > 0.001 or
            abs(msg.angular.z) > 0.001
        )

    def _watchdog_cb(self):
        if self._last_cmdvel_time is None or not self._last_was_nonzero:
            return
        elapsed = (self.get_clock().now() - self._last_cmdvel_time).nanoseconds * 1e-9
        if elapsed > 0.5:
            # Send zero velocity so Gazebo diff-drive doesn't hold the last command.
            self._stop_pub.publish(Twist())
            self._last_was_nonzero = False

            # Clear both costmaps so stale/drifted obstacle marks from the aborted
            # run don't corrupt the next navigation attempt. The live LiDAR scan
            # will rebuild accurate obstacle data within 1-2 seconds.
            if self._local_clear.service_is_ready():
                self._local_clear.call_async(ClearEntireCostmap.Request())
            if self._global_clear.service_is_ready():
                self._global_clear.call_async(ClearEntireCostmap.Request())

    def _waypoints_timer_cb(self):
        if os.path.exists(WAYPOINTS_FILE):
            mtime = os.path.getmtime(WAYPOINTS_FILE)
            if mtime != self._wp_mtime:
                self._wp_mtime = mtime
                try:
                    with open(WAYPOINTS_FILE) as f:
                        data = yaml.safe_load(f) or {}
                    self._cached_waypoints = data.get('waypoints', {})
                except Exception:
                    pass
        if self._cached_waypoints:
            self._publish_waypoints(self._cached_waypoints)

    def _publish_waypoints(self, waypoints: dict):
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        # Clear previous markers
        for ns in ('wp_dot', 'wp_label'):
            clr = Marker()
            clr.header.frame_id = 'map'
            clr.header.stamp = stamp
            clr.ns = ns
            clr.action = Marker.DELETEALL
            arr.markers.append(clr)

        for idx, (name, wp) in enumerate(waypoints.items()):
            x = float(wp.get('x', 0.0))
            y = float(wp.get('y', 0.0))

            # Green sphere
            dot = Marker()
            dot.header.frame_id = 'map'
            dot.header.stamp = stamp
            dot.ns = 'wp_dot'
            dot.id = idx
            dot.type = Marker.SPHERE
            dot.action = Marker.ADD
            dot.pose.position.x = x
            dot.pose.position.y = y
            dot.pose.position.z = 0.05
            dot.pose.orientation.w = 1.0
            dot.scale.x = 0.20
            dot.scale.y = 0.20
            dot.scale.z = 0.20
            dot.color.r = 0.0
            dot.color.g = 1.0
            dot.color.b = 0.2
            dot.color.a = 1.0
            arr.markers.append(dot)

            # Text label above the sphere
            lbl = Marker()
            lbl.header.frame_id = 'map'
            lbl.header.stamp = stamp
            lbl.ns = 'wp_label'
            lbl.id = idx
            lbl.type = Marker.TEXT_VIEW_FACING
            lbl.action = Marker.ADD
            lbl.pose.position.x = x
            lbl.pose.position.y = y
            lbl.pose.position.z = 0.40
            lbl.pose.orientation.w = 1.0
            lbl.scale.z = 0.22       # text height in metres
            lbl.color.r = 1.0
            lbl.color.g = 1.0
            lbl.color.b = 1.0
            lbl.color.a = 1.0
            lbl.text = name
            arr.markers.append(lbl)

        self._wp_pub.publish(arr)

    def plan_cb(self, msg):
        if len(msg.poses) < 2:
            return
        self._publish_goal(msg)
        self._publish_arrows(msg)

    def _publish_goal(self, msg):
        goal = msg.poses[-1]
        m = Marker()
        m.header = msg.header
        m.ns = 'goal'
        m.id = 0
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose = goal.pose
        m.pose.position.z = 0.01
        m.scale.x = 0.45
        m.scale.y = 0.45
        m.scale.z = 0.02
        m.color.r = 0.0
        m.color.g = 1.0
        m.color.b = 0.0
        m.color.a = 0.85
        self.goal_pub.publish(m)

    def _publish_arrows(self, msg):
        arr = MarkerArray()

        clr = Marker()
        clr.header = msg.header
        clr.ns = 'path_arrows'
        clr.action = Marker.DELETEALL
        arr.markers.append(clr)

        poses = msg.poses
        n = len(poses)
        step = max(1, n // 15)

        arrow_id = 0
        for i in range(0, n - 1, step):
            p0 = poses[i].pose.position
            p1 = poses[min(i + step, n - 1)].pose.position
            angle = math.atan2(p1.y - p0.y, p1.x - p0.x)

            m = Marker()
            m.header = msg.header
            m.ns = 'path_arrows'
            m.id = arrow_id
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.pose.position.x = p0.x
            m.pose.position.y = p0.y
            m.pose.position.z = 0.05
            m.pose.orientation.z = math.sin(angle / 2)
            m.pose.orientation.w = math.cos(angle / 2)
            m.scale.x = 0.13
            m.scale.y = 0.04
            m.scale.z = 0.04
            m.color.r = 0.0
            m.color.g = 0.75
            m.color.b = 1.0
            m.color.a = 1.0
            arr.markers.append(m)
            arrow_id += 1

        self.arrows_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Go2VizNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
