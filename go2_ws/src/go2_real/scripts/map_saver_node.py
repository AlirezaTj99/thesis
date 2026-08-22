#!/usr/bin/env python3
"""
map_saver_node.py — Subscribes to /map, keeps latest map in memory,
and saves it to ~/maps/ as .pgm + .yaml when the node shuts down (Ctrl+C).

This avoids starting a new ROS2 node in a cleanup script (which fails because
CycloneDDS is already tearing down by the time cleanup runs).
"""
import os
import sys
import signal
import datetime
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import OccupancyGrid

MAP_DIR = os.path.expanduser('~/maps')


class MapSaverNode(Node):

    def __init__(self):
        super().__init__('map_saver_node')
        self._map = None

        qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, qos)
        self.get_logger().info(f'map_saver_node ready — will save to {MAP_DIR}/ on shutdown')

    def _map_cb(self, msg: OccupancyGrid):
        if self._map is None:
            print('[map_saver_node] /map received — will save on shutdown.', flush=True)
        self._map = msg

    def save(self):
        # Use print() — self.get_logger() may fail after rclpy.shutdown()
        if self._map is None:
            print('[map_saver_node] WARNING: No map received — nothing to save.', flush=True)
            return

        print('[map_saver_node] Saving map...', flush=True)
        os.makedirs(MAP_DIR, exist_ok=True)
        name = f"go2_map_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}"
        pgm_path  = os.path.join(MAP_DIR, f"{name}.pgm")
        yaml_path = os.path.join(MAP_DIR, f"{name}.yaml")

        m = self._map
        width, height = m.info.width, m.info.height

        # Write binary PGM (P5): unknown=205, occupied=0, free=254
        total = width * height
        bar_width = 40
        with open(pgm_path, 'wb') as f:
            f.write(f"P5\n{width} {height}\n255\n".encode())
            for i, v in enumerate(m.data):
                if v == -1:
                    f.write(b'\xcd')
                elif v >= 65:
                    f.write(b'\x00')
                else:
                    f.write(b'\xfe')
                if i % 5000 == 0 or i == total - 1:
                    pct = (i + 1) / total
                    filled = int(bar_width * pct)
                    bar = '█' * filled + '░' * (bar_width - filled)
                    sys.stdout.write(f'\r  Saving map: [{bar}] {pct*100:5.1f}%  {i+1}/{total} px')
                    sys.stdout.flush()
        sys.stdout.write('\n')

        # Write YAML metadata compatible with nav2_map_server
        meta = {
            'image':           f"{name}.pgm",
            'resolution':      float(m.info.resolution),
            'origin':          [float(m.info.origin.position.x),
                                 float(m.info.origin.position.y),
                                 0.0],
            'negate':          0,
            'occupied_thresh': 0.65,
            'free_thresh':     0.25,
        }
        with open(yaml_path, 'w') as f:
            yaml.dump(meta, f, default_flow_style=False)

        print(f'[map_saver_node] === Map saved ===', flush=True)
        print(f'[map_saver_node]     {pgm_path}', flush=True)
        print(f'[map_saver_node]     {yaml_path}', flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = MapSaverNode()

    # On SIGINT/SIGTERM: just call rclpy.shutdown() so spin() returns to the
    # main thread. Do NOT do file I/O inside a signal handler — it's unreliable.
    # The actual save happens below after spin() exits.
    signal.signal(signal.SIGINT,  lambda s, f: rclpy.shutdown())
    signal.signal(signal.SIGTERM, lambda s, f: rclpy.shutdown())

    try:
        rclpy.spin(node)
    except BaseException:
        pass

    node.save()
    try:
        node.destroy_node()
    except Exception:
        pass


if __name__ == '__main__':
    main()
