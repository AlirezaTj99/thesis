#!/usr/bin/env python3
"""Save the robot's current map-frame pose as a named waypoint in ~/maps/waypoints.yaml."""
import math
import os
import sys
import yaml

import rclpy
from rclpy.node import Node
import tf2_ros

WAYPOINTS_FILE = os.path.expanduser('~/maps/waypoints.yaml')


class WaypointSaver(Node):
    def __init__(self, name: str):
        super().__init__('waypoint_saver')
        self._name = name
        self._buf = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._buf, self)
        self._timer = self.create_timer(0.1, self._try_save)
        self.get_logger().info(f'Waiting for map → base_link TF …')

    def _try_save(self):
        try:
            t = self._buf.lookup_transform('map', 'base_link', rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return

        x = round(t.transform.translation.x, 3)
        y = round(t.transform.translation.y, 3)
        q = t.transform.rotation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_deg = round(math.degrees(math.atan2(siny_cosp, cosy_cosp)), 1)

        if os.path.exists(WAYPOINTS_FILE):
            with open(WAYPOINTS_FILE) as f:
                data = yaml.safe_load(f) or {}
        else:
            data = {}
        if 'waypoints' not in data:
            data['waypoints'] = {}

        data['waypoints'][self._name] = {'x': x, 'y': y, 'yaw': yaw_deg}

        with open(WAYPOINTS_FILE, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, allow_unicode=True)

        print(f'Saved "{self._name}":  x={x}  y={y}  yaw={yaw_deg}°')
        self._timer.cancel()
        raise SystemExit(0)


def main():
    if len(sys.argv) < 2:
        print('Usage: save_waypoint.py <name>')
        print('       save_waypoint.py "sleep area"')
        sys.exit(1)

    name = ' '.join(sys.argv[1:])   # allow multi-word names without quoting
    rclpy.init()
    node = WaypointSaver(name)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
