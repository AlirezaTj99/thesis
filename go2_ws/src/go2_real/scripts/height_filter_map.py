#!/usr/bin/env python3
"""
Publishes a height-filtered 2D occupancy grid to /map.

Base grid      : /rtabmap/grid_map_raw  (RTAB-Map full 2D projection, all DB nodes)
Obstacle voxels: /cloud_map             (RTAB-Map local 3D cloud — current vicinity)

Filtering strategy — conservative, preserves far-away saved-map walls:

  For each OCCUPIED cell in the raw grid:
    • No local voxel in that cell at all          → KEEP  (far-away wall, trust RTAB-Map)
    • Local voxels exist, ≥1 in [min_h, max_h]   → KEEP  (confirmed wall)
    • Local voxels exist, none in [min_h, max_h] → CLEAR (confirmed ground noise)

  FREE / UNKNOWN cells pass through unchanged.

  This means the full saved building map is always visible.  Only cells where the
  robot's current LiDAR scan gives evidence that the voxels are below obstacle
  height get cleared — far-away cells that were not recently scanned are never
  disturbed.

Parameters
  floor_z  float  0.0   floor Z in map frame (m)
  min_h    float  0.20  min obstacle height above floor_z (m)
  max_h    float  2.00  max obstacle height above floor_z (m)
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid


def _xyz_from_cloud(cloud: PointCloud2):
    """Return (xs, ys, zs) float32 numpy arrays from a PointCloud2, NaN-filtered."""
    fields = {f.name: f for f in cloud.fields}
    x_off = fields['x'].offset
    y_off = fields['y'].offset
    z_off = fields['z'].offset
    step  = cloud.point_step
    n     = cloud.width * cloud.height
    if n == 0:
        empty = np.empty(0, dtype=np.float32)
        return empty, empty, empty
    raw = np.frombuffer(bytes(cloud.data), dtype=np.uint8).reshape(n, step)
    xs = raw[:, x_off:x_off+4].copy().ravel().view('<f4')
    ys = raw[:, y_off:y_off+4].copy().ravel().view('<f4')
    zs = raw[:, z_off:z_off+4].copy().ravel().view('<f4')
    ok = np.isfinite(xs) & np.isfinite(ys) & np.isfinite(zs)
    return xs[ok], ys[ok], zs[ok]


class HeightFilterMap(Node):
    def __init__(self):
        super().__init__('height_filter_map')
        self.declare_parameter('floor_z', 0.0)
        self.declare_parameter('min_h',   0.20)
        self.declare_parameter('max_h',   2.00)

        self.floor_z = self.get_parameter('floor_z').value
        self.min_h   = self.get_parameter('min_h').value
        self.max_h   = self.get_parameter('max_h').value

        self.cloud_xyz = None   # (xs, ys, zs) numpy arrays; None until first cloud
        self.last_grid = None

        rel_vol = QoSProfile(depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE)
        rel_tl = QoSProfile(depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(PointCloud2,   '/cloud_map',            self._cloud_cb, rel_tl)
        self.create_subscription(OccupancyGrid, '/rtabmap/grid_map_raw', self._grid_cb,  rel_tl)
        self.pub = self.create_publisher(OccupancyGrid, '/map', rel_tl)

        self.get_logger().info(
            f'height_filter_map: floor_z={self.floor_z:.2f}  '
            f'keep [{self.floor_z + self.min_h:.2f}, {self.floor_z + self.max_h:.2f}] m')

    def _cloud_cb(self, msg: PointCloud2):
        xs, ys, zs = _xyz_from_cloud(msg)
        self.cloud_xyz = (xs, ys, zs)
        z_lo = self.floor_z + self.min_h
        z_hi = self.floor_z + self.max_h
        n_wall = int(np.sum((zs >= z_lo) & (zs <= z_hi)))
        z_range = f'z=[{float(zs.min()):.2f},{float(zs.max()):.2f}]' if len(xs) > 0 else 'z=empty'
        self.get_logger().info(
            f'cloud_map: {n_wall} wall / {len(xs)-n_wall} ground / {len(xs)} total  {z_range}')
        if self.last_grid is not None:
            self._publish(self.last_grid)

    def _grid_cb(self, msg: OccupancyGrid):
        self.last_grid = msg
        if self.cloud_xyz is None:
            self.pub.publish(msg)
            self.get_logger().debug('cloud_map pending — forwarding raw grid')
        else:
            self._publish(msg)

    def _publish(self, grid: OccupancyGrid):
        res = grid.info.resolution
        ox  = grid.info.origin.position.x
        oy  = grid.info.origin.position.y
        w   = grid.info.width
        h   = grid.info.height

        xs, ys, zs = self.cloud_xyz
        z_lo = self.floor_z + self.min_h
        z_hi = self.floor_z + self.max_h

        # Map every local cloud voxel to its 2D grid cell index
        cols = np.floor((xs - ox) / res).astype(np.int32)
        rows = np.floor((ys - oy) / res).astype(np.int32)
        in_bounds = (cols >= 0) & (cols < w) & (rows >= 0) & (rows < h)

        local_idx = rows[in_bounds] * w + cols[in_bounds]
        local_zs  = zs[in_bounds]

        # Cells with ≥1 wall-height voxel (z in [floor_z+min_h, floor_z+max_h])
        wall_mask  = (local_zs >= z_lo) & (local_zs <= z_hi)
        wall_cells = set(local_idx[wall_mask].tolist())

        # All 2D cells touched by any local voxel
        all_local  = set(local_idx.tolist())

        # Ground-only cells: covered locally but NO voxel in wall height range
        ground_cells = all_local - wall_cells

        data = list(grid.data)
        removed = 0
        for i, val in enumerate(data):
            if val == 100 and i in ground_cells:
                data[i] = 0
                removed += 1

        out = OccupancyGrid()
        out.header = grid.header
        out.info   = grid.info
        out.data   = data
        self.pub.publish(out)
        self.get_logger().info(
            f'/map {w}×{h}: removed {removed} ground-confirmed cells, '
            f'wall_cells={len(wall_cells)}, local_coverage={len(all_local)} cells')


def main():
    rclpy.init()
    rclpy.spin(HeightFilterMap())


if __name__ == '__main__':
    main()
