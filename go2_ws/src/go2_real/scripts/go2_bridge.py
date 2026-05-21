#!/usr/bin/env python3
"""
go2_bridge.py — ROS2 bridge for real Unitree Go2 Pro navigation.

Subscribes:
  /utlidar/robot_odom  (nav_msgs/Odometry)       — LIO-SAM odometry (BEST_EFFORT QoS)
  /utlidar/cloud_base  (sensor_msgs/PointCloud2) — LiDAR in base_link (BEST_EFFORT QoS)
  /cmd_vel             (geometry_msgs/Twist)      — Nav2 velocity commands

Publishes:
  /odom                (nav_msgs/Odometry)        — relayed with RELIABLE QoS for Nav2
  /cloud_relay         (sensor_msgs/PointCloud2)  — relayed with RELIABLE QoS for laserscan
  TF: odom → base_link                            — for Nav2 and SLAM
  /api/sport/request   (unitree_api/Request)      — velocity command to Go2
"""

import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from tf2_ros import TransformBroadcaster
from unitree_api.msg import Request

ROBOT_SPORT_API_ID_MOVE         = 1008
ROBOT_SPORT_API_ID_BALANCE_STAND = 1002

# Match the Go2 robot's publisher QoS (BEST_EFFORT, VOLATILE)
BEST_EFFORT_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)


class Go2Bridge(Node):

    def __init__(self):
        super().__init__('go2_bridge')

        self._tf_broadcaster = TransformBroadcaster(self)
        self._odom_pub  = self.create_publisher(Odometry, '/odom', 10)
        self._cloud_pub = self.create_publisher(PointCloud2, '/cloud_relay', 10)
        self._req_pub   = self.create_publisher(Request, '/api/sport/request', 10)

        # Subscribe to robot topics with BEST_EFFORT to match publisher QoS
        self.create_subscription(Odometry, '/utlidar/robot_odom',
                                 self._odom_callback, BEST_EFFORT_QOS)
        self.create_subscription(PointCloud2, '/utlidar/cloud_base',
                                 self._cloud_callback, BEST_EFFORT_QOS)
        self.create_subscription(Twist, '/cmd_vel',
                                 self._cmd_vel_callback, 10)

        # Target velocities from Nav2
        self._vx_target   = 0.0
        self._vy_target   = 0.0
        self._vyaw_target = 0.0
        self._zero_since  = None   # timestamp when cmd_vel first went to zero (holdoff)
        self.create_timer(0.05, self._cmd_vel_keepalive)   # 20 Hz

        # Stand up then unlock velocity commands on startup.
        # RecoveryStand at 3 s brings robot up from sitting.
        # BalanceStand at 5 s (after recovery completes) unlocks Move commands.
        self._recovery_timer = self.create_timer(3.0, self._startup_recovery_stand)
        self._balance_timer  = self.create_timer(5.0, self._startup_balance_stand)
        self.get_logger().info('go2_bridge started — relaying odom + cloud with QoS fix')

    def _odom_callback(self, msg: Odometry):
        now = self.get_clock().now().to_msg()

        # Relay odometry with fresh timestamp
        odom = Odometry()
        odom.header.stamp        = now
        odom.header.frame_id     = 'odom'
        odom.child_frame_id      = 'base_link'
        odom.pose                = msg.pose
        odom.twist               = msg.twist
        self._odom_pub.publish(odom)

        # Publish TF odom → base_link
        tf = TransformStamped()
        tf.header.stamp       = now
        tf.header.frame_id    = 'odom'
        tf.child_frame_id     = 'base_link'
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = msg.pose.pose.position.z
        tf.transform.rotation      = msg.pose.pose.orientation
        self._tf_broadcaster.sendTransform(tf)

    def _cloud_callback(self, msg: PointCloud2):
        # Relay cloud with original frame (base_link) — QoS is now RELIABLE
        self._cloud_pub.publish(msg)

    def _cmd_vel_callback(self, msg: Twist):
        new_vx   = float(msg.linear.x)
        new_vy   = float(msg.linear.y)
        new_vyaw = float(msg.angular.z)
        if new_vx != 0.0 or new_vy != 0.0 or new_vyaw != 0.0:
            self._vx_target   = new_vx
            self._vy_target   = new_vy
            self._vyaw_target = new_vyaw
            self._zero_since  = None          # cancel any pending zero holdoff
        else:
            if self._zero_since is None:      # start the zero holdoff timer
                self._zero_since = self.get_clock().now()

    def _cmd_vel_keepalive(self):
        # Apply zero only after 0.3 s of continuous zero cmd_vel — filters brief Nav2 gaps
        if self._zero_since is not None:
            age = (self.get_clock().now() - self._zero_since).nanoseconds / 1e9
            if age >= 0.3:
                self._vx_target  = self._vy_target = self._vyaw_target = 0.0
                self._zero_since = None

        vx, vy, vyaw = self._vx_target, self._vy_target, self._vyaw_target

        # Only publish non-zero commands — silence lets obstacle_repulsion take full control
        if vx != 0.0 or vy != 0.0 or vyaw != 0.0:
            self._publish_move(vx, vy, vyaw)

    def _startup_recovery_stand(self):
        req = Request()
        req.header.identity.api_id = 1006  # RecoveryStand — stand up from sit
        self._req_pub.publish(req)
        self.get_logger().info('RecoveryStand sent — standing up')
        self.destroy_timer(self._recovery_timer)

    def _startup_balance_stand(self):
        req = Request()
        req.header.identity.api_id = ROBOT_SPORT_API_ID_BALANCE_STAND
        self._req_pub.publish(req)
        self.get_logger().info('BalanceStand sent — robot ready for velocity commands')
        self.destroy_timer(self._balance_timer)

    def _publish_move(self, vx: float, vy: float, vyaw: float):
        req = Request()
        req.header.identity.api_id = ROBOT_SPORT_API_ID_MOVE
        req.parameter = json.dumps({'x': vx, 'y': vy, 'z': vyaw})
        self._req_pub.publish(req)


def main(args=None):
    rclpy.init(args=args)
    node = Go2Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down — laying down...')
        # keepalive loop has stopped (spin exited) — no more Move commands in flight.
        # StopMove was removed: it makes a lying robot stand up to neutral stance.
        time.sleep(0.1)   # let the last keepalive tick clear from the DDS queue
        down = Request()
        down.header.identity.api_id = 1005   # StandDown / lay down
        node._req_pub.publish(down)
        time.sleep(1.5)   # give DDS time to deliver before tearing down the node
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
