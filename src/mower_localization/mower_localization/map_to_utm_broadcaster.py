#!/usr/bin/env python3
"""Publish a static transform map -> utm using map_origin_utm.yaml.

Reads maps/map_origin_utm.yaml (or provided directory parameter) which stores:
  utm_origin: [x, y, z]
representing the UTM coordinates of the map frame (map frame 0,0,0 corresponds to UTM x,y,z).

This allows Nav2 to operate in the map frame (small local coordinates) while localization
and datum remain in the UTM world frame.
"""
import os
import yaml
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


class MapToUtmBroadcaster(Node):
    def __init__(self):
        super().__init__('map_to_utm_broadcaster')
        self.declare_parameter('maps_dir', '/home/tyler/mower_ws/maps')
        self.declare_parameter('map_origin_file', 'map_origin_utm.yaml')
        self._static_broadcaster = StaticTransformBroadcaster(self)
        self._timer = self.create_timer(2.0, self._try_publish_once)
        self._published = False
        self.get_logger().info('map_to_utm_broadcaster started')

    def _try_publish_once(self):
        if self._published:
            return
        maps_dir = self.get_parameter('maps_dir').get_parameter_value().string_value
        origin_file = self.get_parameter('map_origin_file').get_parameter_value().string_value
        full_path = os.path.join(maps_dir, origin_file)
        if not os.path.exists(full_path):
            # Wait until recorder generates file
            self.get_logger().debug(f'Waiting for {full_path}')
            return
        try:
            with open(full_path, 'r') as f:
                data = yaml.safe_load(f) or {}
            origin = data.get('utm_origin')
            if not origin or len(origin) < 3:
                raise ValueError('utm_origin missing or malformed')
            tx, ty, tz = float(origin[0]), float(origin[1]), float(origin[2])
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'utm'
            t.child_frame_id = 'map'
            t.transform.translation.x = tx
            t.transform.translation.y = ty
            t.transform.translation.z = tz
            # No rotation between UTM and map (identity)
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            self._static_broadcaster.sendTransform(t)
            self._published = True
            self.get_logger().info(f'Published static TF utm -> map using {full_path}')
            # No longer need timer after publishing
            self._timer.cancel()
        except Exception as e:
            self.get_logger().error(f'Failed to publish transform: {e}')


def main():
    rclpy.init()
    node = MapToUtmBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
