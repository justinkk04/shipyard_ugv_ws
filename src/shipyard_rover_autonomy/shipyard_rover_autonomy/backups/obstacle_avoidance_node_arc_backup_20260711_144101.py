#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class ObstacleAvoidanceNode(Node):
    """
    Obstacle Avoidance Node

    Improved behavior:
    - Reads desired motion from /nominal_cmd_vel
    - Reads LiDAR from /scan
    - Publishes final safe command to /cmd_vel
    - Locks avoidance direction to reduce left/right wiggle
    - Uses slow arc motion to move around obstacles
    - Backs up briefly if dangerously close
    - Publishes BLOCKED or SENSOR_INVALID events when needed
    """

    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        # ------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------
        self.declare_parameter('front_clear_distance', 1.20)
        self.declare_parameter('front_release_distance', 1.60)
        self.declare_parameter('danger_distance', 0.45)
        self.declare_parameter('side_clear_distance', 0.25)

        self.declare_parameter('blocked_timeout', 10.0)
        self.declare_parameter('avoid_timeout', 12.0)
        self.declare_parameter('scan_stale_timeout', 1.0)
        self.declare_parameter('control_rate_hz', 10.0)

        self.declare_parameter('turn_speed', 0.55)
        self.declare_parameter('avoid_forward_speed', 0.08)
        self.declare_parameter('backup_speed', -0.08)
        self.declare_parameter('backup_duration', 0.8)
        self.declare_parameter('avoid_commit_duration', 2.0)

        self.front_clear_distance = float(
            self.get_parameter('front_clear_distance').value
        )
        self.front_release_distance = float(
            self.get_parameter('front_release_distance').value
        )
        self.danger_distance = float(
            self.get_parameter('danger_distance').value
        )
        self.side_clear_distance = float(
            self.get_parameter('side_clear_distance').value
        )
        self.blocked_timeout = float(
            self.get_parameter('blocked_timeout').value
        )
        self.avoid_timeout = float(
            self.get_parameter('avoid_timeout').value
        )
        self.scan_stale_timeout = float(
            self.get_parameter('scan_stale_timeout').value
        )
        self.control_rate_hz = float(
            self.get_parameter('control_rate_hz').value
        )
        self.turn_speed = float(
            self.get_parameter('turn_speed').value
        )
        self.avoid_forward_speed = float(
            self.get_parameter('avoid_forward_speed').value
        )
        self.backup_speed = float(
            self.get_parameter('backup_speed').value
        )
        self.backup_duration = float(
            self.get_parameter('backup_duration').value
        )
        self.avoid_commit_duration = float(
            self.get_parameter('avoid_commit_duration').value
        )

        # ------------------------------------------------------------
        # Internal state
        # ------------------------------------------------------------
        self.mission_state = 'IDLE'

        self.latest_scan = None
        self.latest_scan_time = None

        self.latest_nominal_cmd = Twist()
        self.have_nominal_cmd = False

        # Avoidance modes:
        # CLEAR
        # AVOIDING_LEFT
        # AVOIDING_RIGHT
        # BACKING_UP
        # BLOCKED
        self.avoidance_mode = 'CLEAR'
        self.mode_start_time = None

        self.locked_direction = 0
        self.last_avoid_direction = 1  # 1 = left, -1 = right

        self.blocked_start_time = None
        self.blocked_event_sent = False
        self.sensor_invalid_event_sent = False

        self.last_status = None

        # ------------------------------------------------------------
        # QoS for mission state
        # ------------------------------------------------------------
        state_qos = QoSProfile(depth=10)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # ------------------------------------------------------------
        # Subscribers
        # ------------------------------------------------------------
        self.create_subscription(
            String,
            '/mission/state',
            self.mission_state_callback,
            state_qos
        )

        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.create_subscription(
            Twist,
            '/nominal_cmd_vel',
            self.nominal_cmd_callback,
            10
        )

        # ------------------------------------------------------------
        # Publishers
        # ------------------------------------------------------------
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.event_pub = self.create_publisher(
            String,
            '/mission/event',
            10
        )

        self.front_distance_pub = self.create_publisher(
            Float32,
            '/obstacle/front_distance',
            10
        )

        self.left_clearance_pub = self.create_publisher(
            Float32,
            '/obstacle/left_clearance',
            10
        )

        self.right_clearance_pub = self.create_publisher(
            Float32,
            '/obstacle/right_clearance',
            10
        )

        self.status_pub = self.create_publisher(
            String,
            '/obstacle/status',
            10
        )

        # ------------------------------------------------------------
        # Main timer
        # ------------------------------------------------------------
        timer_period = 1.0 / self.control_rate_hz
        self.timer = self.create_timer(timer_period, self.control_loop)

        self.get_logger().info('Improved Obstacle Avoidance Node started.')
        self.get_logger().info('This node owns final /cmd_vel.')
        self.get_logger().info(f'front_clear_distance:   {self.front_clear_distance:.2f} m')
        self.get_logger().info(f'front_release_distance: {self.front_release_distance:.2f} m')
        self.get_logger().info(f'danger_distance:        {self.danger_distance:.2f} m')
        self.get_logger().info(f'side_clear_distance:    {self.side_clear_distance:.2f} m')
        self.get_logger().info(f'turn_speed:             {self.turn_speed:.2f} rad/s')
        self.get_logger().info(f'avoid_forward_speed:    {self.avoid_forward_speed:.2f} m/s')

    # ------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------
    def mission_state_callback(self, msg):
        new_state = msg.data.strip().upper()

        if new_state != self.mission_state:
            self.get_logger().info(f'Mission state received: {new_state}')

        self.mission_state = new_state

        if self.mission_state == 'INSPECTING':
            self.blocked_event_sent = False
            self.sensor_invalid_event_sent = False
            self.blocked_start_time = None

            if self.avoidance_mode not in ['CLEAR']:
                self.set_mode('CLEAR')
        else:
            self.publish_stop()
            self.set_mode('CLEAR')
            self.blocked_start_time = None

    def scan_callback(self, msg):
        self.latest_scan = msg
        self.latest_scan_time = self.get_clock().now()
        self.sensor_invalid_event_sent = False

    def nominal_cmd_callback(self, msg):
        self.latest_nominal_cmd = msg
        self.have_nominal_cmd = True

    # ------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------
    def control_loop(self):
        if self.mission_state != 'INSPECTING':
            self.publish_stop()
            self.publish_status('IDLE_STOP')
            return

        if not self.have_nominal_cmd:
            self.publish_stop()
            self.publish_status('WAITING_FOR_NOMINAL_CMD')
            return

        if self.latest_scan is None or self.latest_scan_time is None:
            self.publish_stop()
            self.publish_status('WAITING_FOR_SCAN')
            return

        now = self.get_clock().now()

        scan_age = (now - self.latest_scan_time).nanoseconds / 1e9
        if scan_age > self.scan_stale_timeout:
            self.publish_stop()
            self.publish_status('SENSOR_INVALID_STALE_SCAN')
            self.publish_mission_event_once('SENSOR_INVALID')
            return

        if not self.scan_message_is_usable(self.latest_scan):
            self.publish_stop()
            self.publish_status('SENSOR_INVALID_BAD_SCAN')
            self.publish_mission_event_once('SENSOR_INVALID')
            return

        front = self.get_sector_min_range(self.latest_scan, -25.0, 25.0)
        left = self.get_sector_min_range(self.latest_scan, 25.0, 120.0)
        right = self.get_sector_min_range(self.latest_scan, -120.0, -25.0)

        self.publish_obstacle_telemetry(front, left, right)

        front_logic = self.range_for_logic(front)
        left_logic = self.range_for_logic(left)
        right_logic = self.range_for_logic(right)

        front_triggered = front_logic <= self.front_clear_distance
        front_danger = front_logic <= self.danger_distance
        front_released = front_logic >= self.front_release_distance

        left_clear = left_logic > self.side_clear_distance
        right_clear = right_logic > self.side_clear_distance

        # ------------------------------------------------------------
        # If dangerously close, back up first.
        # ------------------------------------------------------------
        if front_danger and self.avoidance_mode != 'BACKING_UP':
            direction = self.choose_avoid_direction(left_logic, right_logic)
            self.locked_direction = direction
            self.last_avoid_direction = direction
            self.set_mode('BACKING_UP')
            self.blocked_start_time = now

        # ------------------------------------------------------------
        # State machine
        # ------------------------------------------------------------
        if self.avoidance_mode == 'BACKING_UP':
            self.handle_backup_mode(now)
            return

        if self.avoidance_mode in ['AVOIDING_LEFT', 'AVOIDING_RIGHT']:
            self.handle_avoid_mode(
                now,
                front_logic,
                front_released,
                left_clear,
                right_clear
            )
            return

        if self.avoidance_mode == 'BLOCKED':
            self.publish_stop()
            self.publish_status('BLOCKED_SAFE_STOP')
            self.publish_mission_event_once('BLOCKED')
            return

        # ------------------------------------------------------------
        # CLEAR mode
        # ------------------------------------------------------------
        if not front_triggered:
            self.blocked_start_time = None
            self.publish_status('CLEAR_PASS_THROUGH')
            self.cmd_pub.publish(self.latest_nominal_cmd)
            return

        # Front obstacle detected. Choose one direction and commit.
        direction = self.choose_avoid_direction(left_logic, right_logic)
        self.locked_direction = direction
        self.last_avoid_direction = direction

        if direction > 0:
            self.set_mode('AVOIDING_LEFT')
        else:
            self.set_mode('AVOIDING_RIGHT')

        self.blocked_start_time = now
        self.publish_avoid_arc()

    # ------------------------------------------------------------
    # Mode handlers
    # ------------------------------------------------------------
    def handle_backup_mode(self, now):
        mode_age = self.mode_age(now)

        if mode_age < self.backup_duration:
            cmd = Twist()
            cmd.linear.x = self.backup_speed
            cmd.angular.z = -self.locked_direction * 0.20
            self.cmd_pub.publish(cmd)
            self.publish_status('DANGER_BACKUP')
            return

        if self.locked_direction >= 0:
            self.set_mode('AVOIDING_LEFT')
        else:
            self.set_mode('AVOIDING_RIGHT')

        self.publish_avoid_arc()

    def handle_avoid_mode(
        self,
        now,
        front_logic,
        front_released,
        left_clear,
        right_clear
    ):
        mode_age = self.mode_age(now)

        # If avoidance has been active too long, call it blocked.
        if mode_age >= self.avoid_timeout:
            self.publish_stop()
            self.set_mode('BLOCKED')
            self.publish_status('BLOCKED_AVOID_TIMEOUT')
            self.publish_mission_event_once('BLOCKED')
            return

        # If front and sides are all tight for too long, call it blocked.
        both_sides_blocked = not left_clear and not right_clear
        front_still_close = front_logic <= self.front_clear_distance

        if front_still_close and both_sides_blocked:
            if self.blocked_start_time is None:
                self.blocked_start_time = now
            else:
                blocked_age = (now - self.blocked_start_time).nanoseconds / 1e9
                if blocked_age >= self.blocked_timeout:
                    self.publish_stop()
                    self.set_mode('BLOCKED')
                    self.publish_status('BLOCKED_NO_CLEAR_SIDE')
                    self.publish_mission_event_once('BLOCKED')
                    return
        else:
            self.blocked_start_time = None

        # Hysteresis:
        # Do not release immediately when front barely clears.
        # Keep avoiding for at least avoid_commit_duration.
        if front_released and mode_age >= self.avoid_commit_duration:
            self.set_mode('CLEAR')
            self.blocked_start_time = None
            self.publish_status('CLEAR_RELEASE_TO_ROUTE')
            self.cmd_pub.publish(self.latest_nominal_cmd)
            return

        self.publish_avoid_arc()

    # ------------------------------------------------------------
    # Movement helpers
    # ------------------------------------------------------------
    def publish_avoid_arc(self):
        cmd = Twist()
        cmd.linear.x = self.avoid_forward_speed
        cmd.angular.z = self.locked_direction * abs(self.turn_speed)

        if self.locked_direction >= 0:
            self.publish_status('AVOIDING_LEFT_ARC')
        else:
            self.publish_status('AVOIDING_RIGHT_ARC')

        self.cmd_pub.publish(cmd)

    def publish_stop(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0

        try:
            if rclpy.ok():
                self.cmd_pub.publish(cmd)
        except Exception:
            pass

    def choose_avoid_direction(self, left_logic, right_logic):
        """
        Choose one direction and commit to it.

        1  = left
        -1 = right
        """

        # If one side is clearly better, use it.
        margin = 0.10

        if left_logic > right_logic + margin:
            return 1

        if right_logic > left_logic + margin:
            return -1

        # If nearly tied, keep the previous direction to avoid oscillation.
        return self.last_avoid_direction

    def set_mode(self, new_mode):
        if new_mode != self.avoidance_mode:
            self.get_logger().info(
                f'Avoidance mode: {self.avoidance_mode} -> {new_mode}'
            )
            self.avoidance_mode = new_mode
            self.mode_start_time = self.get_clock().now()

    def mode_age(self, now):
        if self.mode_start_time is None:
            return 0.0

        return (now - self.mode_start_time).nanoseconds / 1e9

    # ------------------------------------------------------------
    # LaserScan helpers
    # ------------------------------------------------------------
    def scan_message_is_usable(self, scan):
        if scan is None:
            return False

        if len(scan.ranges) == 0:
            return False

        if scan.angle_increment == 0.0:
            return False

        if scan.range_max <= scan.range_min:
            return False

        for r in scan.ranges:
            if not math.isnan(r):
                return True

        return False

    def get_sector_min_range(self, scan, min_angle_deg, max_angle_deg):
        min_range = math.inf

        for i, r in enumerate(scan.ranges):
            if math.isnan(r):
                continue

            if math.isinf(r):
                continue

            if r < scan.range_min or r > scan.range_max:
                continue

            angle_rad = scan.angle_min + i * scan.angle_increment
            angle_rad = math.atan2(math.sin(angle_rad), math.cos(angle_rad))
            angle_deg = math.degrees(angle_rad)

            if min_angle_deg <= angle_deg <= max_angle_deg:
                if r < min_range:
                    min_range = r

        return min_range

    def range_for_logic(self, value):
        if math.isinf(value):
            if self.latest_scan is not None:
                return float(self.latest_scan.range_max)
            return 999.0

        return float(value)

    def range_for_publish(self, value):
        if math.isinf(value):
            if self.latest_scan is not None:
                return float(self.latest_scan.range_max)
            return 999.0

        return float(value)

    # ------------------------------------------------------------
    # Telemetry publishers
    # ------------------------------------------------------------
    def publish_obstacle_telemetry(self, front, left, right):
        msg = Float32()
        msg.data = self.range_for_publish(front)
        self.front_distance_pub.publish(msg)

        msg = Float32()
        msg.data = self.range_for_publish(left)
        self.left_clearance_pub.publish(msg)

        msg = Float32()
        msg.data = self.range_for_publish(right)
        self.right_clearance_pub.publish(msg)

    def publish_status(self, status_text):
        msg = String()
        msg.data = status_text
        self.status_pub.publish(msg)

        if status_text != self.last_status:
            self.get_logger().info(f'Obstacle status: {status_text}')
            self.last_status = status_text

    def publish_mission_event_once(self, event_text):
        event_text = event_text.strip().upper()

        if event_text == 'SENSOR_INVALID':
            if self.sensor_invalid_event_sent:
                return
            self.sensor_invalid_event_sent = True

        if event_text == 'BLOCKED':
            if self.blocked_event_sent:
                return
            self.blocked_event_sent = True

        msg = String()
        msg.data = event_text
        self.event_pub.publish(msg)

        self.get_logger().warn(f'Published mission event: {event_text}')


def main(args=None):
    rclpy.init(args=args)

    node = ObstacleAvoidanceNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('Obstacle Avoidance interrupted by user.')

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