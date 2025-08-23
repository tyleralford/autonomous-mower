#!/usr/bin/env python3
import csv
import os
import yaml
from typing import Optional, Any

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

from .map_generator import generate_map

try:
    from mower_msgs.srv import ManageRecording
except Exception:
    # Allow import before messages are built; will fail at runtime if not built
    ManageRecording = None  # type: ignore


START = 0
STOP = 1
BOUNDARY = 0
KEEPOUT = 1
TRAVEL = 2


class RecorderNode(Node):
    def __init__(self):
        super().__init__('recorder_node')
        self._recording: bool = False
        self._area_type: Optional[int] = None
        self._filename: Optional[str] = None
        self._csv_file = None
        self._csv_writer: Optional[csv.writer] = None
        self._odom_sub = None
        self._gps_once_sub = None

        # Create service
        self._srv = self.create_service(
            ManageRecording, '/mower/manage_recording', self.handle_manage_recording
        )

        # QoS for latched / steady odometry topic
        self._odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.get_logger().info('recorder_node ready: service /mower/manage_recording')

    def handle_manage_recording(self, request: Any, response: Any):
        if request.action == START:
            if self._recording:
                response.success = False
                response.message = 'Already recording'
                return response
            if not request.filename:
                response.success = False
                response.message = 'Filename required for START'
                return response

            # Prepare directory
            out_path = os.path.expanduser(request.filename)
            os.makedirs(os.path.dirname(out_path) or '.', exist_ok=True)

            try:
                self._csv_file = open(out_path, 'w', newline='')
                self._csv_writer = csv.writer(self._csv_file)
                self._csv_writer.writerow(['x', 'y'])
            except Exception as e:
                response.success = False
                response.message = f'Failed to open file: {e}'
                return response

            self._filename = out_path
            self._area_type = int(request.area_type)
            self._recording = True

            # Subscribe to odometry
            self._odom_sub = self.create_subscription(
                Odometry, '/odometry/filtered', self.odom_cb, self._odom_qos
            )

            response.success = True
            response.message = f'START recording to {self._filename} (area_type={self._area_type})'
            self.get_logger().info(response.message)
            return response

        elif request.action == STOP:
            if not self._recording:
                response.success = False
                response.message = 'Not currently recording'
                return response

            # Stop recording
            self._recording = False
            if self._odom_sub is not None:
                self.destroy_subscription(self._odom_sub)
                self._odom_sub = None

            if self._csv_file is not None:
                try:
                    self._csv_file.flush()
                    self._csv_file.close()
                except Exception:
                    pass
                self._csv_file = None
                self._csv_writer = None

            # After successfully stopping, trigger map generation and ensure datum saved
            try:
                maps_dir = os.path.dirname(self._filename) or '.'
                self.get_logger().info(f'Triggering map generation in {maps_dir}')
                # Persist datum once
                self._save_datum_once(maps_dir)

                # Generate map from all CSVs in directory
                map_yaml, map_pgm = generate_map(maps_dir)
                self.get_logger().info(f'Generated map: {map_yaml}, {map_pgm}')
            except Exception as e:
                import traceback
                tb = traceback.format_exc()
                self.get_logger().error(f'Map generation failed: {e}\n{tb}')

            response.success = True
            response.message = f'STOP recording; saved to {self._filename}'
            self.get_logger().info(response.message)
            return response

        else:
            response.success = False
            response.message = f'Unknown action: {request.action}'
            return response

    def odom_cb(self, msg: Odometry):
        if not self._recording or self._csv_writer is None:
            return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        try:
            self._csv_writer.writerow([f'{x:.6f}', f'{y:.6f}'])
        except Exception as e:
            self.get_logger().error(f'Failed to write CSV: {e}')

    def _save_datum_once(self, maps_dir: str):
        """Subscribe once to /gps/fix and save datum.yaml if not present."""
        datum_path = os.path.join(maps_dir, 'datum.yaml')
        if os.path.exists(datum_path):
            return

        # Use the first GPS fix to capture latitude/longitude
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )

        def gps_cb(msg: NavSatFix):
            try:
                # robot_localization/SetDatum expects [lat, lon, yaw, world_frame, base_link_frame]
                data = {
                    'latitude': float(msg.latitude),
                    'longitude': float(msg.longitude),
                    'heading': 0.0,  # default east-aligned heading
                    'world_frame': 'utm',
                    'base_link_frame': 'base_link',
                }
                with open(datum_path, 'w') as f:
                    yaml.safe_dump(data, f)
                self.get_logger().info(f'Saved datum metadata to {datum_path}')
            except Exception as e:
                self.get_logger().error(f'Failed to save datum: {e}')
            finally:
                if self._gps_once_sub is not None:
                    self.destroy_subscription(self._gps_once_sub)
                    self._gps_once_sub = None

        # Subscribe and spin a few cycles to catch a message
        self._gps_once_sub = self.create_subscription(
            NavSatFix, '/gps/fix', gps_cb, qos
        )
        # Let a few cycles run; non-blocking here, callback will tear down sub


def main():
    rclpy.init()
    node = RecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
