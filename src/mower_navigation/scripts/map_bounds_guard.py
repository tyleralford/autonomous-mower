#!/usr/bin/env python3

import os
import math
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import Odometry
from mower_msgs.msg import NavStatus

try:
    from PIL import Image  # pillow for robust image size reading
except Exception:
    Image = None


MAP_DIR = Path('/home/tyler/mower_ws/maps')
MAP_YAML = MAP_DIR / 'map.yaml'
MAP_PGM = MAP_DIR / 'map.pgm'
LIFECYCLE_SERVICE = '/lifecycle_manager_navigation/manage_nodes'


def in_bounds(bounds, x: float, y: float) -> bool:
    if bounds is None:
        return False
    xmin, xmax, ymin, ymax = bounds
    return (xmin <= x <= xmax) and (ymin <= y <= ymax)


def map_bounds_from_meta(resolution: float, origin_xy, width: int, height: int):
    if resolution is None or origin_xy is None or width is None or height is None:
        return None
    ox, oy = origin_xy
    xmax = ox + width * resolution
    ymax = oy + height * resolution
    xmin = ox
    ymin = oy
    return (xmin, xmax, ymin, ymax)


class MapBoundsGuard(Node):
    def __init__(self):
        super().__init__('map_bounds_guard')
        self.declare_parameter('map_yaml', str(MAP_YAML))
        self.declare_parameter('map_pgm', str(MAP_PGM))
        self.declare_parameter('lifecycle_service', LIFECYCLE_SERVICE)
        self.declare_parameter('status_topic', '/mower/nav_status')
        self.declare_parameter('auto_start', True)

        self.map_yaml_path = Path(self.get_parameter('map_yaml').get_parameter_value().string_value)
        self.map_pgm_path = Path(self.get_parameter('map_pgm').get_parameter_value().string_value)
        self.lifecycle_service_name = self.get_parameter('lifecycle_service').get_parameter_value().string_value
        self.status_topic = self.get_parameter('status_topic').get_parameter_value().string_value
        self.auto_start = self.get_parameter('auto_start').get_parameter_value().bool_value

        # latched status publisher
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(NavStatus, self.status_topic, qos)

        # Subscribe to global odom
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered/global', self.odom_cb, 10)

        # Service client for lifecycle
        self.lifecycle_cli = self.create_client(ManageLifecycleNodes, self.lifecycle_service_name)

        self.map_lock = threading.Lock()
        self.map_ready = False
        self.bounds = None  # (xmin, xmax, ymin, ymax)
        self.nav_started = False
        self.last_pose = None

        # Initial check timer
        self.create_timer(0.5, self.periodic_check)

    def publish_status(self, ready: bool, reason: str):
        msg = NavStatus()
        msg.ready = ready
        msg.reason = reason
        self.status_pub.publish(msg)

    def load_map_bounds(self):
        if not self.map_yaml_path.exists() or not self.map_pgm_path.exists():
            return None
        try:
            # parse minimal YAML: resolution, origin: [x, y, theta]
            resolution = None
            origin = None
            width = height = None
            with open(self.map_yaml_path, 'r') as f:
                for line in f:
                    line = line.strip()
                    if line.startswith('resolution:'):
                        resolution = float(line.split(':', 1)[1].strip())
                    elif line.startswith('origin:'):
                        # origin: [x, y, yaw]
                        br = line.split('[', 1)[1]
                        vals = br.split(']')[0].split(',')
                        origin = (float(vals[0]), float(vals[1]))
            if Image is not None:
                with Image.open(self.map_pgm_path) as im:
                    width, height = im.size
            else:
                # Fallback: parse PGM header
                with open(self.map_pgm_path, 'rb') as f:
                    header = f.readline().strip()
                    if header != b'P5':
                        return None
                    # skip comments
                    dims = f.readline().strip()
                    while dims.startswith(b'#'):
                        dims = f.readline().strip()
                    parts = dims.split()
                    width = int(parts[0])
                    height = int(parts[1])
            return map_bounds_from_meta(resolution, origin, width, height)
        except Exception as e:
            self.get_logger().warn(f'Failed to read map bounds: {e}')
            return None

    def ensure_map(self):
        with self.map_lock:
            if not self.map_ready:
                b = self.load_map_bounds()
                if b is None:
                    self.publish_status(False, 'waiting_for_map')
                    return False
                self.bounds = b
                self.map_ready = True
                self.get_logger().info(f'Map bounds set to {self.bounds}')
        return True

    def in_bounds(self, x: float, y: float) -> bool:
        return in_bounds(self.bounds, x, y)

    def try_start_nav(self):
        if not self.auto_start or self.nav_started:
            return
        if not self.lifecycle_cli.wait_for_service(timeout_sec=0.1):
            return
        req = ManageLifecycleNodes.Request()
        req.command = ManageLifecycleNodes.Request.STARTUP
        future = self.lifecycle_cli.call_async(req)
        def _done(_):
            res = future.result()
            if res and res.success:
                self.get_logger().info('Nav2 lifecycle STARTUP succeeded')
                self.nav_started = True
                self.publish_status(True, 'ready')
            else:
                self.get_logger().warn('Nav2 lifecycle STARTUP failed')
        future.add_done_callback(_done)

    def odom_cb(self, msg: Odometry):
        self.last_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def periodic_check(self):
        # Ensure map files are present and bounds computed
        if not self.ensure_map():
            return
        # Need a pose to proceed
        if self.last_pose is None:
            self.publish_status(False, 'waiting_for_pose')
            return
        x, y = self.last_pose
        if not self.in_bounds(x, y):
            self.publish_status(False, 'robot_outside_map')
            return
        # All good; start nav if not already
        self.try_start_nav()


def main():
    rclpy.init()
    node = MapBoundsGuard()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
