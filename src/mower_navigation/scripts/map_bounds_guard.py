#!/usr/bin/env python3

from pathlib import Path
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import Odometry
from mower_msgs.msg import NavStatus

try:
    from PIL import Image
except Exception:
    Image = None

MAP_DIR = Path('/home/tyler/mower_ws/maps')
MAP_YAML = MAP_DIR / 'map.yaml'
MAP_PGM = MAP_DIR / 'map.pgm'
LIFECYCLE_SERVICE = '/lifecycle_manager_navigation/manage_nodes'


def map_bounds_from_meta(resolution: float, origin_xy, width: int, height: int):
    if resolution is None or origin_xy is None or width is None or height is None:
        return None
    ox, oy = origin_xy
    return (ox, ox + width * resolution, oy, oy + height * resolution)


class MapBoundsGuard(Node):
    """Navigation guard: ensures map + pose readiness, boundary safety, and lifecycle sequencing."""

    def __init__(self):
        super().__init__('map_bounds_guard')
        # Parameters
        params = [
            ('map_yaml', str(MAP_YAML)),
            ('map_pgm', str(MAP_PGM)),
            ('map_origin_utm_yaml', 'map_origin_utm.yaml'),  # companion file containing utm_origin when map.yaml origin is 0,0,0
            ('lifecycle_service', LIFECYCLE_SERVICE),
            ('status_topic', '/mower/nav_status'),
            ('auto_start', True),
            ('boundary_buffer_m', 0.5),
            # boundary_buffer_m now used only as an interior hysteresis requirement when re-entering
            # (robot must be this far inside previously violated edges before resuming nav)
            ('detection_margin_m', 0.0),  # expands allowable detection bounds outward (rarely needed >0)
            ('verbose', False),
            ('lifecycle_retry_seconds', [0.0, 5.0, 10.0]),
            ('activation_wait_sec', 3.0),
            # Added stabilization delay for re-entry (time inside bounds before RESUME)
            ('reentry_stabilization_sec', 5.0),
            ('check_rate_hz_initial', 10.0),
            ('check_rate_hz_active', 2.0),
            ('max_state_silence_sec', 2.0),
            ('reentry_margin_m', 0.1),
            ('pose_stale_sec', 1.0),
        ]
        for name, default in params:
            self.declare_parameter(name, default)

        gp = lambda n: self.get_parameter(n).get_parameter_value()
        self.map_yaml_path = Path(gp('map_yaml').string_value)
        self.map_pgm_path = Path(gp('map_pgm').string_value)
        self.lifecycle_service_name = gp('lifecycle_service').string_value
        self.status_topic = gp('status_topic').string_value
        self.auto_start = gp('auto_start').bool_value
        self.boundary_buffer = gp('boundary_buffer_m').double_value
        self.detection_margin = gp('detection_margin_m').double_value
        self.map_origin_utm_yaml = gp('map_origin_utm_yaml').string_value
        self.verbose = gp('verbose').bool_value
        try:
            self.lifecycle_schedule = list(gp('lifecycle_retry_seconds').double_array_value)
        except Exception:
            self.lifecycle_schedule = [0.0, 5.0, 10.0]
        self.activation_wait = gp('activation_wait_sec').double_value
        self.reentry_stabilization = gp('reentry_stabilization_sec').double_value
        self.check_rate_initial = gp('check_rate_hz_initial').double_value
        self.check_rate_active = gp('check_rate_hz_active').double_value
        self.max_state_silence = gp('max_state_silence_sec').double_value
        self.reentry_margin = gp('reentry_margin_m').double_value
        self.pose_stale_sec = gp('pose_stale_sec').double_value

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(NavStatus, self.status_topic, qos)
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_cb, 10)
        self.lifecycle_cli = self.create_client(ManageLifecycleNodes, self.lifecycle_service_name)

        # State
        self.map_lock = threading.Lock()
        self.map_ready = False
        self.raw_bounds = None
        self.bounds = None
        self.last_pose = None
        self.last_pose_map_frame = None  # pose converted into map frame for boundary checks
        self.pose_time = None
        self.state = None
        self.start_time = self.get_clock().now()
        self.last_publish_time = self.get_clock().now()
        self.startup_sent_time = None
        self.nav_active_assumed = False
        self.paused_due_to_bounds = False
        # Timestamp when robot first satisfies interior hysteresis on re-entry
        self.reentry_stable_start_time = None
        self.lifecycle_commands_sent = set()
        self.violated_axes = set()  # track which boundaries were crossed for targeted hysteresis
        self.map_frame_shift = (0.0, 0.0)  # utm_origin (utm -> map translation) if map.yaml origin is (0,0,0)

        # Timers
        self.active_timer = None
        self.inactive_timer = self.create_timer(1.0 / max(0.1, self.check_rate_initial), self.periodic_check)

    # ---------- Core helpers ----------
    def publish_status(self, ready: bool, reason: str):
        now = self.get_clock().now()
        if reason == self.state:
            if (now - self.last_publish_time).nanoseconds / 1e9 < self.max_state_silence:
                return
        self.state = reason
        self.last_publish_time = now
        msg = NavStatus()
        msg.ready = ready
        msg.reason = reason
        self.status_pub.publish(msg)
        if self.verbose:
            self.get_logger().info(f'state -> {reason} (ready={ready})')

    def load_map_bounds(self):
        if not self.map_yaml_path.exists() or not self.map_pgm_path.exists():
            return None
        try:
            resolution = None
            origin = None
            with open(self.map_yaml_path, 'r') as f:
                for line in f:
                    line = line.strip()
                    if line.startswith('resolution:'):
                        resolution = float(line.split(':', 1)[1].strip())
                    elif line.startswith('origin:'):
                        vals = line.split('[', 1)[1].split(']')[0].split(',')
                        origin = (float(vals[0]), float(vals[1]))
            # Determine if map.yaml origin is (near) zero and a companion utm origin file exists
            if origin and abs(origin[0]) < 1e-6 and abs(origin[1]) < 1e-6:
                companion = self.map_yaml_path.parent / self.map_origin_utm_yaml
                if companion.exists():
                    try:
                        import yaml
                        with open(companion, 'r') as c:
                            data = yaml.safe_load(c) or {}
                        utm_origin = data.get('utm_origin')
                        if utm_origin and len(utm_origin) >= 2:
                            # utm_origin gives UTM coords of map frame origin -> shift poses into map frame
                            self.map_frame_shift = (float(utm_origin[0]), float(utm_origin[1]))
                            if self.verbose:
                                self.get_logger().info(f'Using companion utm_origin shift {self.map_frame_shift} to convert UTM odom into map frame')
                    except Exception as e:
                        self.get_logger().warn(f'Failed reading companion utm origin file: {e}')
            if Image is not None:
                with Image.open(self.map_pgm_path) as im:
                    width, height = im.size
            else:
                with open(self.map_pgm_path, 'rb') as f:
                    header = f.readline().strip()
                    if header != b'P5':
                        return None
                    dims = f.readline().strip()
                    while dims.startswith(b'#'):
                        dims = f.readline().strip()
                    parts = dims.split()
                    width, height = int(parts[0]), int(parts[1])
            return map_bounds_from_meta(resolution, origin, width, height)
        except Exception as e:
            self.get_logger().warn(f'Failed to read map bounds: {e}')
            return None

    def ensure_map(self):
        with self.map_lock:
            if self.map_ready:
                return True
            b = self.load_map_bounds()
            if b is None:
                self.publish_status(False, 'waiting_for_map')
                return False
            self.raw_bounds = b  # strict map bounds
            rxmin, rxmax, rymin, rymax = b
            # Detection bounds (optionally relaxed by detection_margin)
            dxmin = rxmin - self.detection_margin
            dxmax = rxmax + self.detection_margin
            dymin = rymin - self.detection_margin
            dymax = rymax + self.detection_margin
            self.bounds = (dxmin, dxmax, dymin, dymax)  # used for quick in/out tests
            self.map_ready = True
            self.get_logger().info(
                f'Map bounds raw={b} detection_bounds(with margin {self.detection_margin})->{self.bounds}; '
                f'reentry interior margin per violated side={self.boundary_buffer}'
            )
            return True

    def in_bounds(self, x: float, y: float) -> bool:
        if self.bounds is None:
            return False
        xmin, xmax, ymin, ymax = self.bounds
        return xmin <= x <= xmax and ymin <= y <= ymax

    def send_lifecycle_command(self, command):
        if not self.lifecycle_cli.wait_for_service(timeout_sec=0.3):
            return False
        req = ManageLifecycleNodes.Request()
        req.command = command
        fut = self.lifecycle_cli.call_async(req)
        def _done(_):
            res = fut.result()
            if not res or not res.success:
                self.get_logger().warn(f'lifecycle command {command} failed')
        fut.add_done_callback(_done)
        return True

    def sequence_lifecycle(self):
        if not self.auto_start:
            return
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        if ManageLifecycleNodes.Request.STARTUP not in self.lifecycle_commands_sent:
            if elapsed >= self.lifecycle_schedule[0]:
                if self.send_lifecycle_command(ManageLifecycleNodes.Request.STARTUP):
                    self.lifecycle_commands_sent.add(ManageLifecycleNodes.Request.STARTUP)
                    self.startup_sent_time = now
                    self.publish_status(False, 'starting_nav')
            return
        if not self.nav_active_assumed and self.startup_sent_time is not None:
            if (now - self.startup_sent_time).nanoseconds / 1e9 >= self.activation_wait and not self.paused_due_to_bounds:
                self.nav_active_assumed = True
                self.publish_status(True, 'active')
                self._switch_active_rate()
                return
        if self.paused_due_to_bounds:
            for t in self.lifecycle_schedule[1:]:
                if elapsed >= t and ManageLifecycleNodes.Request.RESUME not in self.lifecycle_commands_sent:
                    if self.send_lifecycle_command(ManageLifecycleNodes.Request.RESUME):
                        self.lifecycle_commands_sent.add(ManageLifecycleNodes.Request.RESUME)
                        self.startup_sent_time = now
                        self.publish_status(False, 'starting_nav')
                        break

    def handle_out_of_bounds(self):
        if self.last_pose_map_frame and self.raw_bounds:
            mx, my = self.last_pose_map_frame
            rxmin, rxmax, rymin, rymax = self.raw_bounds
            self.violated_axes.clear()
            if mx < rxmin:
                self.violated_axes.add('xmin')
            if mx > rxmax:
                self.violated_axes.add('xmax')
            if my < rymin:
                self.violated_axes.add('ymin')
            if my > rymax:
                self.violated_axes.add('ymax')
        if self.nav_active_assumed:
            self.get_logger().warn('Out-of-bounds: pausing Nav2')
            self.send_lifecycle_command(ManageLifecycleNodes.Request.PAUSE)
            self.nav_active_assumed = False
            self.paused_due_to_bounds = True
            self.reentry_stable_start_time = None  # reset stabilization tracking
            # Allow future RESUME (remove prior one if present)
            try:
                self.lifecycle_commands_sent.discard(ManageLifecycleNodes.Request.RESUME)
            except Exception:
                pass
        self.publish_status(False, 'robot_outside_map')

    def odom_cb(self, msg: Odometry):
        self.last_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        # Convert into map frame if shift specified (odom assumed UTM frame)
        if self.map_frame_shift != (0.0, 0.0):
            self.last_pose_map_frame = (
                self.last_pose[0] - self.map_frame_shift[0],
                self.last_pose[1] - self.map_frame_shift[1],
            )
        else:
            self.last_pose_map_frame = self.last_pose
        self.pose_time = self.get_clock().now()

    def _switch_active_rate(self):
        if self.active_timer is None:
            if self.inactive_timer is not None:
                self.inactive_timer.cancel()
                self.inactive_timer = None
            self.active_timer = self.create_timer(1.0 / max(0.1, self.check_rate_active), self.periodic_check)

    def periodic_check(self):
        if not self.ensure_map():
            return
        if self.last_pose is None or self.pose_time is None:
            self.publish_status(False, 'waiting_for_pose')
            return
        if (self.get_clock().now() - self.pose_time).nanoseconds / 1e9 > self.pose_stale_sec:
            self.publish_status(False, 'waiting_for_pose')
            return
        x, y = self.last_pose
        mx, my = self.last_pose_map_frame if self.last_pose_map_frame else (None, None)
        # Detection uses raw (strict) bounds (not expanded) for fast recognition
        if self.raw_bounds and mx is not None:
            rxmin, rxmax, rymin, rymax = self.raw_bounds
            if not (rxmin <= mx <= rxmax and rymin <= my <= rymax):
                self.handle_out_of_bounds()
                return
        else:  # fallback
            if not self.in_bounds(x, y):
                self.handle_out_of_bounds()
                return

        # Hysteresis only on sides actually violated
        if self.paused_due_to_bounds:
            if self.violated_axes and mx is not None:
                rxmin, rxmax, rymin, rymax = self.raw_bounds
                need_more = False
                if 'xmin' in self.violated_axes and mx < rxmin + self.reentry_margin:
                    need_more = True
                if 'xmax' in self.violated_axes and mx > rxmax - self.reentry_margin:
                    need_more = True
                if 'ymin' in self.violated_axes and my < rymin + self.reentry_margin:
                    need_more = True
                if 'ymax' in self.violated_axes and my > rymax - self.reentry_margin:
                    need_more = True
                if need_more:
                    self.publish_status(False, 'robot_outside_map')
                    return
            # At this point interior margin satisfied; apply stabilization delay before RESUME
            now = self.get_clock().now()
            if self.reentry_stable_start_time is None:
                self.reentry_stable_start_time = now
                self.publish_status(False, 'reentry_stabilizing')
                return
            elapsed_stable = (now - self.reentry_stable_start_time).nanoseconds / 1e9
            if elapsed_stable < self.reentry_stabilization:
                self.publish_status(False, 'reentry_stabilizing')
                return
            # Stabilization period complete – issue RESUME then transition to starting_nav state
            if self.send_lifecycle_command(ManageLifecycleNodes.Request.RESUME):
                self.lifecycle_commands_sent.add(ManageLifecycleNodes.Request.RESUME)
                self.startup_sent_time = now
                self.paused_due_to_bounds = False
                self.reentry_stable_start_time = None
                self.publish_status(False, 'starting_nav')
                # Do not proceed to sequence_lifecycle this tick; wait for next cycle
                return
        self.sequence_lifecycle()
        if self.nav_active_assumed:
            self.publish_status(True, 'active')


def main():
    rclpy.init()
    node = MapBoundsGuard()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
