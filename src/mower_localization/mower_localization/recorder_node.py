#!/usr/bin/env python3
import csv
import os
from typing import Optional, Any

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry

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
                Odometry, '/odometry/filtered/global', self.odom_cb, self._odom_qos
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
