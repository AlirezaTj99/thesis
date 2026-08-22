#!/usr/bin/env python3
"""
repulsion_gate.py — Summing gate for sensor and map repulsion signals.

Combines two repulsion sources:
  /repulsion_vel_raw        — from obstacle_repulsion (live sensor-based)
  /map_repulsion_vel_raw    — from map_repulsion (global costmap-based)

During normal navigation the sum is published as /repulsion_vel.
When the stair controller is active (state != IDLE) both signals are zeroed so
repulsion forces do not fight the stair approach/climb commands.

Subscribes:
    /repulsion_vel_raw          (geometry_msgs/Twist) — from obstacle_repulsion
    /map_repulsion_vel_raw      (geometry_msgs/Twist) — from map_repulsion
    /stair_controller/state     (std_msgs/String)     — from stair_controller
Publishes:
    /repulsion_vel              (geometry_msgs/Twist) — consumed by velocity_combiner
"""
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

STAIR_TIMEOUT = 3.0   # seconds without a state message → assume IDLE


class RepulsionGate(Node):

    def __init__(self):
        super().__init__('repulsion_gate')

        self._stair_state     = 'IDLE'
        self._last_state_time = 0.0

        # Latest value from each repulsion source
        self._sensor_vel = Twist()
        self._map_vel    = Twist()

        self.create_subscription(Twist,  '/repulsion_vel_raw',     self._sensor_cb, 10)
        self.create_subscription(Twist,  '/map_repulsion_vel_raw', self._map_cb,    10)
        self.create_subscription(String, '/stair_controller/state', self._state_cb, 10)
        self._pub = self.create_publisher(Twist, '/repulsion_vel', 10)

        self.create_timer(0.05, self._timer_cb)  # 20 Hz — sum and gate both sources

        self.get_logger().info(
            'RepulsionGate ready — summing sensor + map repulsion signals'
        )

    def _state_cb(self, msg: String):
        self._stair_state     = msg.data
        self._last_state_time = time.time()

    def _sensor_cb(self, msg: Twist):
        self._sensor_vel = msg

    def _map_cb(self, msg: Twist):
        self._map_vel = msg

    def _timer_cb(self):
        stair_active = (
            self._last_state_time > 0.0 and
            time.time() - self._last_state_time < STAIR_TIMEOUT and
            self._stair_state not in ('IDLE', '')
        )

        if stair_active:
            self.get_logger().info(
                f'Repulsion zeroed — stair state: {self._stair_state}',
                throttle_duration_sec=2.0,
            )
            self._pub.publish(Twist())
            return

        combined = Twist()
        combined.linear.x = self._sensor_vel.linear.x + self._map_vel.linear.x
        combined.linear.y = self._sensor_vel.linear.y + self._map_vel.linear.y
        self._pub.publish(combined)


def main():
    rclpy.init()
    node = RepulsionGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
