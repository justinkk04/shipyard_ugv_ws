#!/usr/bin/env python3
#WAYPOINT VERSION
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class RouteFollowerNode(Node):
    """
    Waypoint-Based Route Follower Node

    Final architecture behavior:
    - Waits for /mission/state = INSPECTING
    - Captures starting odometry pose
    - Follows a predefined waypoint route
    - Publishes desired motion to /nominal_cmd_vel
    - obstacle_avoidance_node owns final /cmd_vel
    - Publishes mission progress
    - Publishes COMPLETE when the final waypoint is reached

    This is not SLAM, Nav2, or A*.
    This is a predefined-route follower for the known corridor mission.
    """

    def __init__(self):
        super().__init__('route_follower_node')

        # ------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------
        self.declare_parameter('forward_speed', 0.22)
        self.declare_parameter('direct_cmd_vel', False)
        self.declare_parameter('control_rate_hz', 10.0)

        self.declare_parameter('waypoint_tolerance', 0.25)
        self.declare_parameter('waypoint_pass_tolerance', 0.80)
        self.declare_parameter('heading_kp', 1.2)
        self.declare_parameter('max_angular_speed', 0.6)
        self.declare_parameter('heading_slowdown_angle', 0.7)
        self.declare_parameter('heading_stop_angle', 1.2)

        # Relative odom waypoints for the U-shaped corridor.
        # These assume the robot starts near the blue start marker and
        # /odom begins near x=0, y=0.
        self.declare_parameter(
            'waypoints',
            [
                3.0, 0.0,
                6.0, 0.0,
                9.0, 0.0,
                12.0, 0.0
            ]
        )

        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.direct_cmd_vel = bool(self.get_parameter('direct_cmd_vel').value)
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)

        self.waypoint_tolerance = float(
            self.get_parameter('waypoint_tolerance').value
        )
        self.waypoint_pass_tolerance = float(
            self.get_parameter('waypoint_pass_tolerance').value
        )
        self.heading_kp = float(self.get_parameter('heading_kp').value)
        self.max_angular_speed = float(
            self.get_parameter('max_angular_speed').value
        )
        self.heading_slowdown_angle = float(
            self.get_parameter('heading_slowdown_angle').value
        )
        self.heading_stop_angle = float(
            self.get_parameter('heading_stop_angle').value
        )

        self.relative_waypoints = self.parse_waypoints(
            self.get_parameter('waypoints').value
        )

        if len(self.relative_waypoints) == 0:
            self.get_logger().error('No valid waypoints provided. Route follower will not move.')

        # Final demo should use /nominal_cmd_vel.
        # direct_cmd_vel=True is only for standalone testing.
        if self.direct_cmd_vel:
            self.cmd_topic = '/cmd_vel'
        else:
            self.cmd_topic = '/nominal_cmd_vel'

        # ------------------------------------------------------------
        # Mission state
        # ------------------------------------------------------------
        self.mission_state = 'IDLE'
        self.previous_mission_state = 'IDLE'
        self.mission_active = False
        self.completed = False
        self.completion_event_sent = False

        # ------------------------------------------------------------
        # Odometry state
        # ------------------------------------------------------------
        self.latest_x = None
        self.latest_y = None
        self.latest_yaw = None

        self.start_x = None
        self.start_y = None
        self.start_yaw = None

        # Waypoints converted into odom/world frame at mission start.
        self.active_waypoints = []
        self.current_waypoint_index = 0

        # For progress calculation.
        self.segment_lengths = []
        self.cumulative_lengths = []
        self.total_path_length = 0.0

        self.last_no_odom_warn_time = None

        # ------------------------------------------------------------
        # QoS for mission state
        # ------------------------------------------------------------
        state_qos = QoSProfile(depth=10)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # ------------------------------------------------------------
        # Subscribers
        # ------------------------------------------------------------
        self.state_sub = self.create_subscription(
            String,
            '/mission/state',
            self.mission_state_callback,
            state_qos
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # ------------------------------------------------------------
        # Publishers
        # ------------------------------------------------------------
        self.cmd_pub = self.create_publisher(
            Twist,
            self.cmd_topic,
            10
        )

        self.progress_pub = self.create_publisher(
            Float32,
            '/mission/progress',
            10
        )

        self.event_pub = self.create_publisher(
            String,
            '/mission/event',
            10
        )

        # ------------------------------------------------------------
        # Timer
        # ------------------------------------------------------------
        timer_period = 1.0 / self.control_rate_hz
        self.control_timer = self.create_timer(
            timer_period,
            self.control_loop
        )

        self.get_logger().info('Waypoint Route Follower Node started.')
        self.get_logger().info(f'Command topic: {self.cmd_topic}')
        self.get_logger().info(f'Forward speed: {self.forward_speed:.2f} m/s')
        self.get_logger().info(f'Waypoint tolerance: {self.waypoint_tolerance:.2f} m')
        self.get_logger().info(f'Heading Kp: {self.heading_kp:.2f}')
        self.get_logger().info(f'Max angular speed: {self.max_angular_speed:.2f} rad/s')
        self.get_logger().info(f'Relative waypoints: {self.relative_waypoints}')

    # ------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------
    def mission_state_callback(self, msg):
        new_state = msg.data.strip().upper()

        if new_state != self.mission_state:
            self.previous_mission_state = self.mission_state
            self.mission_state = new_state

            self.get_logger().info(
                f'Mission state received: {self.mission_state}'
            )

            if self.mission_state == 'INSPECTING':
                self.start_new_route()
            else:
                self.mission_active = False
                self.publish_stop()

    def odom_callback(self, msg):
        self.latest_x = float(msg.pose.pose.position.x)
        self.latest_y = float(msg.pose.pose.position.y)

        q = msg.pose.pose.orientation
        self.latest_yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

    # ------------------------------------------------------------
    # Route setup
    # ------------------------------------------------------------
    def start_new_route(self):
        self.mission_active = True
        self.completed = False
        self.completion_event_sent = False
        self.current_waypoint_index = 0

        self.start_x = None
        self.start_y = None
        self.start_yaw = None

        self.active_waypoints = []
        self.segment_lengths = []
        self.cumulative_lengths = []
        self.total_path_length = 0.0

        self.publish_progress(0.0)

        self.get_logger().info('Waypoint route follower activated.')
        self.get_logger().info('Waiting for odometry start pose...')

    def capture_start_pose_and_build_route(self):
        self.start_x = self.latest_x
        self.start_y = self.latest_y
        self.start_yaw = self.latest_yaw

        self.active_waypoints = []

        cos_yaw = math.cos(self.start_yaw)
        sin_yaw = math.sin(self.start_yaw)

        for rel_x, rel_y in self.relative_waypoints:
            # Rotate relative mission waypoint by starting yaw,
            # then translate into odom frame.
            goal_x = self.start_x + rel_x * cos_yaw - rel_y * sin_yaw
            goal_y = self.start_y + rel_x * sin_yaw + rel_y * cos_yaw
            self.active_waypoints.append((goal_x, goal_y))

        self.compute_path_lengths()

        self.get_logger().info(
            f'Start odom captured: x={self.start_x:.2f}, '
            f'y={self.start_y:.2f}, yaw={self.start_yaw:.2f}'
        )

        for i, (x, y) in enumerate(self.active_waypoints):
            self.get_logger().info(
                f'Waypoint {i + 1}/{len(self.active_waypoints)}: '
                f'x={x:.2f}, y={y:.2f}'
            )

    def compute_path_lengths(self):
        points = [(self.start_x, self.start_y)] + self.active_waypoints

        self.segment_lengths = []
        self.cumulative_lengths = [0.0]
        running_total = 0.0

        for i in range(1, len(points)):
            x0, y0 = points[i - 1]
            x1, y1 = points[i]
            segment_length = math.hypot(x1 - x0, y1 - y0)

            self.segment_lengths.append(segment_length)
            running_total += segment_length
            self.cumulative_lengths.append(running_total)

        self.total_path_length = running_total

        self.get_logger().info(
            f'Total predefined route length: {self.total_path_length:.2f} m'
        )

    def parse_waypoints(self, flat_list):
        values = list(flat_list)

        if len(values) < 2 or len(values) % 2 != 0:
            self.get_logger().error(
                'Waypoint parameter must contain an even number of values: '
                '[x1, y1, x2, y2, ...]'
            )
            return []

        waypoints = []

        for i in range(0, len(values), 2):
            waypoints.append((float(values[i]), float(values[i + 1])))

        return waypoints

    # ------------------------------------------------------------
    # Main control loop
    # ------------------------------------------------------------
    def control_loop(self):
        if not self.mission_active:
            return

        if self.completed:
            self.publish_stop()
            return

        if self.mission_state != 'INSPECTING':
            self.publish_stop()
            return

        if self.latest_x is None or self.latest_y is None or self.latest_yaw is None:
            self.publish_stop()
            self.warn_no_odom()
            return

        if len(self.relative_waypoints) == 0:
            self.publish_stop()
            return

        if self.start_x is None or self.start_y is None or self.start_yaw is None:
            self.capture_start_pose_and_build_route()

        if self.current_waypoint_index >= len(self.active_waypoints):
            self.complete_route()
            return

        target_x, target_y = self.active_waypoints[self.current_waypoint_index]

        dx = target_x - self.latest_x
        dy = target_y - self.latest_y
        distance_to_waypoint = math.hypot(dx, dy)

        # If close enough to the current waypoint, or if we have passed
        # an intermediate waypoint while avoiding an obstacle, advance.
        waypoint_done, waypoint_reason = self.waypoint_reached_or_passed(
            target_x,
            target_y,
            distance_to_waypoint
        )

        if waypoint_done:
            self.get_logger().info(
                f'Waypoint {self.current_waypoint_index + 1}/'
                f'{len(self.active_waypoints)} {waypoint_reason}. '
                f'Distance error={distance_to_waypoint:.2f} m'
            )

            self.current_waypoint_index += 1

            if self.current_waypoint_index >= len(self.active_waypoints):
                self.complete_route()
                return

            target_x, target_y = self.active_waypoints[self.current_waypoint_index]
            dx = target_x - self.latest_x
            dy = target_y - self.latest_y
            distance_to_waypoint = math.hypot(dx, dy)

        self.publish_progress(
            self.compute_progress_percent(distance_to_waypoint)
        )

        target_heading = math.atan2(dy, dx)
        heading_error = self.normalize_angle(target_heading - self.latest_yaw)

        angular_cmd = self.heading_kp * heading_error
        angular_cmd = self.clamp(
            angular_cmd,
            -self.max_angular_speed,
            self.max_angular_speed
        )

        linear_cmd = self.forward_speed

        # If the robot is badly misaligned, rotate first or slow down.
        abs_heading_error = abs(heading_error)

        if abs_heading_error > self.heading_stop_angle:
            linear_cmd = 0.0
        elif abs_heading_error > self.heading_slowdown_angle:
            linear_cmd = 0.08

        cmd = Twist()
        cmd.linear.x = linear_cmd
        cmd.angular.z = angular_cmd

        self.cmd_pub.publish(cmd)

    def complete_route(self):
        self.publish_stop()
        self.publish_progress(100.0)

        self.completed = True
        self.mission_active = False

        self.get_logger().info('Final waypoint reached. Mission route complete.')
        self.publish_complete_event()

    # ------------------------------------------------------------
    # Helper functions
    # ------------------------------------------------------------
    def compute_progress_percent(self, distance_to_current_waypoint):
        if self.total_path_length <= 0.0:
            return 0.0

        idx = self.current_waypoint_index

        if idx >= len(self.segment_lengths):
            return 100.0

        segment_length = self.segment_lengths[idx]

        distance_completed_before_segment = self.cumulative_lengths[idx]

        if segment_length <= 0.0:
            segment_fraction_done = 1.0
        else:
            segment_fraction_done = 1.0 - (
                distance_to_current_waypoint / segment_length
            )
            segment_fraction_done = self.clamp(segment_fraction_done, 0.0, 1.0)

        completed_distance = (
            distance_completed_before_segment
            + segment_fraction_done * segment_length
        )

        progress_percent = 100.0 * completed_distance / self.total_path_length
        return self.clamp(progress_percent, 0.0, 100.0)

    def waypoint_reached_or_passed(self, target_x, target_y, distance_to_waypoint):
        """
        Decide whether the current waypoint should be advanced.

        Normal case:
        - Advance when the robot is within waypoint_tolerance.

        Obstacle-avoidance case:
        - If this is an intermediate waypoint and the robot has already
          passed it along the route direction, allow the route follower
          to advance instead of turning around to chase an old waypoint.

        The final endpoint is stricter: it must be reached by distance.
        """

        # Normal waypoint reach condition.
        if distance_to_waypoint <= self.waypoint_tolerance:
            return True, 'reached'

        # Do not skip the final endpoint. Final endpoint accuracy matters.
        if self.current_waypoint_index >= len(self.active_waypoints) - 1:
            return False, ''

        # Previous route point is start pose for waypoint 1, otherwise previous waypoint.
        if self.current_waypoint_index == 0:
            prev_x = self.start_x
            prev_y = self.start_y
        else:
            prev_x, prev_y = self.active_waypoints[
                self.current_waypoint_index - 1
            ]

        seg_x = target_x - prev_x
        seg_y = target_y - prev_y
        robot_x = self.latest_x - prev_x
        robot_y = self.latest_y - prev_y

        seg_len_sq = seg_x * seg_x + seg_y * seg_y

        if seg_len_sq <= 1e-6:
            return False, ''

        # Projection tells whether robot has moved beyond the waypoint
        # along the route segment direction.
        projection = (robot_x * seg_x + robot_y * seg_y) / seg_len_sq

        # Lateral distance from the route segment line.
        seg_len = math.sqrt(seg_len_sq)
        lateral_error = abs(seg_x * robot_y - seg_y * robot_x) / seg_len

        if projection >= 1.0 and lateral_error <= self.waypoint_pass_tolerance:
            return True, 'passed'

        return False, ''    

    
    def quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)

        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def clamp(self, value, low, high):
        return max(low, min(value, high))

    def warn_no_odom(self):
        now = self.get_clock().now()

        if self.last_no_odom_warn_time is None:
            self.last_no_odom_warn_time = now
            self.get_logger().warn('No odometry received yet. Holding position.')
            return

        age = (now - self.last_no_odom_warn_time).nanoseconds / 1e9

        if age >= 2.0:
            self.last_no_odom_warn_time = now
            self.get_logger().warn('No odometry received yet. Holding position.')

    # ------------------------------------------------------------
    # Publishers
    # ------------------------------------------------------------
    def publish_stop(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def publish_progress(self, progress_percent):
        msg = Float32()
        msg.data = float(progress_percent)
        self.progress_pub.publish(msg)

    def publish_complete_event(self):
        if self.completion_event_sent:
            return

        msg = String()
        msg.data = 'COMPLETE'
        self.event_pub.publish(msg)

        self.completion_event_sent = True
        self.get_logger().info('Published mission event: COMPLETE')


def main(args=None):
    rclpy.init(args=args)

    node = RouteFollowerNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('Route Follower interrupted by user.')
        node.publish_stop()

    finally:
        try:
            if rclpy.ok():
                node.publish_stop()
        except Exception:
            pass

        node.destroy_node()

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()