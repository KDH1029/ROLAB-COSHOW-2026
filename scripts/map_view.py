#!/usr/bin/env python3
"""Publish a built map (scripts/map_build.py output) for RViz: point cloud (intensity = height above the floor),
2D grid, the mapping drive and the rulebook track lines + wall, all latched in frame 'map' (= track frame).
    python3 scripts/map_view.py maps/<name> [step=1]      (scripts/view_map.sh starts this and rviz2)
"""
import math
import os
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
sys.path.insert(0, os.path.join(ROOT, 'src/mission/track_day1'))
import track_spec as T  # noqa: E402


def stadium(r, n=200):
    L = T.STRAIGHT / 2
    a = np.linspace(-math.pi / 2, math.pi / 2, n)
    return np.vstack([np.column_stack([L + r * np.cos(a), r * np.sin(a)]),
                      np.column_stack([-L - r * np.cos(a), -r * np.sin(a)]), [[L, -r]]])


class MapView(Node):
    def __init__(self, d, step):
        super().__init__('map_view')
        q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        hdr = Header(frame_id='map')
        hdr.stamp = self.get_clock().now().to_msg()
        src = 'map_clean.npz' if os.path.exists(os.path.join(d, 'map_clean.npz')) and os.environ.get('MAP_RAW') != '1' else 'map_track.npz'
        z = np.load(os.path.join(d, src))
        self.get_logger().info('point cloud: %s (MAP_RAW=1 for everything)' % src)
        xyz, h = z['xyz'][::step].astype(np.float32), z['height'][::step].astype(np.float32)
        pc = PointCloud2(header=hdr, height=1, width=len(xyz), is_bigendian=False, point_step=16, is_dense=True,
                         fields=[PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                                 for i, n in enumerate(('x', 'y', 'z', 'intensity'))])
        pc.row_step = 16 * len(xyz)
        pc.data = np.column_stack([xyz, h]).astype(np.float32).tobytes()
        self.create_publisher(PointCloud2, '/map_cloud', q).publish(pc)

        # 2D grid from map.pgm / map.yaml
        y = {}
        for line in open(os.path.join(d, 'map.yaml')):
            if ':' in line and not line.startswith('#'):
                k, v = line.split(':', 1)
                y[k.strip()] = v.split('#')[0].strip()
        res = float(y['resolution'])
        org = [float(v) for v in y['origin'].strip('[]').split(',')]
        raw = open(os.path.join(d, y['image']), 'rb').read()
        parts = raw.split(b'\n', 3)
        w, hh = map(int, parts[1].split())
        img = np.frombuffer(parts[3], np.uint8).reshape(hh, w)[::-1]
        grid = np.full(img.shape, -1, np.int8)
        grid[img == 0] = 100
        grid[img == 254] = 0
        og = OccupancyGrid(header=hdr)
        og.info.resolution, og.info.width, og.info.height = res, w, hh
        og.info.origin.position.x, og.info.origin.position.y = org[0], org[1]
        og.info.origin.orientation.w = 1.0
        og.data = grid.reshape(-1).tolist()
        self.create_publisher(OccupancyGrid, '/map', q).publish(og)

        tr = z['traj']
        path = Path(header=hdr)
        for p in tr[::5]:
            ps = PoseStamped(header=hdr)
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = float(p[0]), float(p[1]), float(p[2])
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.create_publisher(Path, '/map_traj', q).publish(path)

        ma = MarkerArray()
        lines = [(stadium(T.R_OUTER), (1.0, 1.0, 1.0)), (stadium(T.R_LANE12), (1.0, 1.0, 1.0)),
                 (stadium(T.R_INFIELD), (1.0, 0.85, 0.2))]
        hx, hy = T.WALL_LX / 2, T.WALL_LY / 2
        lines.append((np.array([[-hx, -hy], [hx, -hy], [hx, hy], [-hx, hy], [-hx, -hy]]), (0.6, 0.6, 0.6)))
        for i, (pts, c) in enumerate(lines):
            m = Marker(header=hdr, ns='track', id=i, type=Marker.LINE_STRIP, action=Marker.ADD)
            m.scale.x = 0.02
            m.color = ColorRGBA(r=c[0], g=c[1], b=c[2], a=0.9)
            m.pose.orientation.w = 1.0
            m.points = [Point(x=float(a), y=float(b), z=0.005) for a, b in pts]
            ma.markers.append(m)
        self.create_publisher(MarkerArray, '/map_track_lines', q).publish(ma)
        self.get_logger().info('published %s: %d points (step %d), grid %dx%d, drive %d poses — close RViz to quit'
                               % (os.path.basename(d.rstrip('/')), len(xyz), step, w, hh, len(path.poses)))


def main():
    d = sys.argv[1]
    kv = dict(a.split('=') for a in sys.argv[2:])
    rclpy.init()
    n = MapView(d, int(kv.get('step', 1)))
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
