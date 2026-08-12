#!/usr/bin/env python3
"""
monitor_robot.py — Monitor Go2 battery and temperature over WiFi.

Subscribes to /go2/battery (sensor_msgs/BatteryState) published by
state_bridge on the Orin NX — works without Ethernet cable.

Displays every 5 seconds:
  - Battery:  SoC (%), voltage (V), current (A), temperature (°C)
"""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import BatteryState

RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

REPORT_INTERVAL = 5.0   # seconds between console prints


class RobotMonitor(Node):
    def __init__(self):
        super().__init__('robot_monitor')
        self._last_msg = None
        self._last_print = 0.0
        self.create_subscription(BatteryState, '/go2/battery', self._cb, RELIABLE_QOS)
        self.get_logger().info('Waiting for /go2/battery messages (WiFi)...')

    def _cb(self, msg: BatteryState):
        self._last_msg = msg
        now = time.monotonic()
        if now - self._last_print < REPORT_INTERVAL:
            return
        self._last_print = now

        soc   = msg.percentage * 100.0
        volt  = msg.voltage
        amp   = msg.current
        temp  = msg.temperature
        cells = [f'{v:.3f}' for v in msg.cell_voltage if v > 0.0]

        print(
            f'Battery: {soc:.0f}%  {volt:.2f}V  {amp:.2f}A  {temp:.1f}°C'
            + (f'  cells=[{", ".join(cells)}]V' if cells else ''),
            flush=True,
        )


def main():
    rclpy.init()
    node = RobotMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
