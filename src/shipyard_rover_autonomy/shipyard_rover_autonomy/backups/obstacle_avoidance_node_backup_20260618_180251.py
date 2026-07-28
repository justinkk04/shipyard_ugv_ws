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

    Phase 2 behavior:
    - Reads mission state from /mission/state
    - Reads desired motion from /nominal_cmd_vel
    - Reads 2D LiDAR from /scan
    - Publishes final safe command to /cmd_vel
    - Publishes obstacle telemetry
    - Publishes mission events if the robot is blocked or sensor data is invalid

    This node is the ONLY autonomy node that should publish /cmd_vel in Phase 2.
    """

    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        # ----------------------------
        # Parameters
        # ----------------------------
        self.declare_parameter('front_clear_distance', 0.75)
        self.declare_parameter('danger_distance', 0.35)
        self.declare_parameter('side_clear_distance', 0.60)
        self.declare_parameter('blocked_timeout', 10.0)
        self.declare_parameter('scan_stale_timeout', 1.0)
        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('turn_speed', 0.35)

        self.front_clear_distance = float(
            self.get_parameter('front_clear_distance').value
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
        self.scan_stale_timeout = float(
            self.get_parameter('scan_stale_timeout').value
        )
        self.control_rate_hz = float(
            self.get_parameter('control_rate_hz').value
        )
        self.turn_speed = float(
            self.get_parameter('turn_speed').value
        )

        # ----------------------------
        # Internal state
        # ----------------------------
        self.mission_state = 'IDLE'

        self.latest_scan = None
        self.latest_scan_time = None

        self.latest_nominal_cmd = Twist()
        self.have_nominal_cmd = False

        self.blocked_start_time = None
        self.blocked_event_sent = False
        self.sensor_invalid_event_sent = False

        self.last_status = None

        # ----------------------------
        # QoS for mission state
        # ----------------------------
        # This lets the obstacle node receive the latest mission state even if
        # it starts after the mission manager.
        state_qos = QoSProfile(depth=10)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # ----------------------------
        # Subscribers
        # ----------------------------
        self.state_sub = self.create_subscription(
            String,
            '/mission/state',
            self.mission_state_callback,
            state_qos
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.nominal_cmd_sub = self.create_subscription(
            Twist,
            '/nominal_cmd_vel',
            self.nominal_cmd_callback,
            10
        )

        # ----------------------------
        # Publishers
        # ----------------------------
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

        # ----------------------------
        # Main control loop
        # ----------------------------
        timer_period = 1.0 / self.control_rate_hz
        self.control_timer = self.create_timer(
            timer_period,
            self.control_loop
        )

        self.get_logger().info('Obstacle Avoidance Node started.')
        self.get_logger().info('Subscribing to: /mission/state, /scan, /nominal_cmd_vel')
        self.get_logger().info('Publishing final command to: /cmd_vel')
        self.get_logger().info(f'Front clear distance: {self.front_clear_distance:.2f} m')
        self.get_logger().info(f'Danger distance: {self.danger_distance:.2f} m')
        self.get_logger().info(f'Side clear distance: {self.side_clear_distance:.2f} m')
        self.get_logger().info(f'Blocked timeout: {self.blocked_timeout:.2f} s')

    def mission_state_callback(self, msg):
        new_state = msg.data.strip().upper()

        if new_state != self.mission_state:
            self.get_logger().info(f'Mission state received: {new_state}')

        self.mission_state = new_state

        if self.mission_state == 'INSPECTING':
            # Reset one-shot event flags at the start of a new mission.
            self.blocked_event_sent = False
            self.sensor_invalid_event_sent = False
            self.blocked_start_time = None
        else:
            self.publish_stop()
            self.blocked_start_time = None

    def scan_callback(self, msg):
        self.latest_scan = msg
        self.latest_scan_time = self.get_clock().now()

        # If scan comes back after being stale, allow future invalid events again.
        self.sensor_invalid_event_sent = False

    def nominal_cmd_callback(self, msg):
        self.latest_nominal_cmd = msg
        self.have_nominal_cmd = True

    def control_loop(self):
        """
        Main obstacle avoidance loop.

        Runs at control_rate_hz.
        At 10 Hz, the node checks obstacles every 0.1 seconds, which is faster
        than the 2-second response requirement.
        """

        # If mission is not active, robot should not move.
        if self.mission_state != 'INSPECTING':
            self.publish_stop()
            self.publish_status('IDLE_STOP')
            return

        # If no nominal command has arrived, do not move.
        if not self.have_nominal_cmd:
            self.publish_stop()
            self.publish_status('WAITING_FOR_NOMINAL_CMD')
            return

        # If no scan has arrived, do not move.
        if self.latest_scan is None or self.latest_scan_time is None:
            self.publish_stop()
            self.publish_status('WAITING_FOR_SCAN')
            return

        # If scan is stale, stop and publish SENSOR_INVALID.
        now = self.get_clock().now()
        scan_age = (now - self.latest_scan_time).nanoseconds / 1e9

        if scan_age > self.scan_stale_timeout:
            self.publish_stop()
            self.publish_status('SENSOR_INVALID_STALE_SCAN')
            self.publish_mission_event_once('SENSOR_INVALID')
            return

        # If scan message itself is invalid, stop and publish SENSOR_INVALID.
        if not self.scan_message_is_usable(self.latest_scan):
            self.publish_stop()
            self.publish_status('SENSOR_INVALID_BAD_SCAN')
            self.publish_mission_event_once('SENSOR_INVALID')
            return

        # Compute LiDAR sector distances.
        front_distance = self.get_sector_min_range(
            self.latest_scan,
            -20.0,
            20.0
        )

        left_clearance = self.get_sector_min_range(
            self.latest_scan,
            30.0,
            100.0
        )

        right_clearance = self.get_sector_min_range(
            self.latest_scan,
            -100.0,
            -30.0
        )

        self.publish_obstacle_telemetry(
            front_distance,
            left_clearance,
            right_clearance
        )

        # Convert infinite "nothing seen" values to a large value for logic.
        front_logic = self.range_for_logic(front_distance)
        left_logic = self.range_for_logic(left_clearance)
        right_logic = self.range_for_logic(right_clearance)

        front_is_clear = front_logic > self.front_clear_distance
        front_is_danger = front_logic <= self.danger_distance

        left_is_clear = left_logic > self.side_clear_distance
        right_is_clear = right_logic > self.side_clear_distance

        # ----------------------------
        # Case 1: front is clear
        # ----------------------------
        if front_is_clear:
            self.blocked_start_time = None
            self.publish_status('CLEAR_PASS_THROUGH')
            self.cmd_pub.publish(self.latest_nominal_cmd)
            return

        # ----------------------------
        # Case 2: obstacle very close
        # ----------------------------
        if front_is_danger:
            self.publish_stop()
            self.publish_status('DANGER_STOP')

            # If both sides are also blocked, start/continue blocked timer.
            if not left_is_clear and not right_is_clear:
                self.handle_possible_blocked()
            else:
                self.blocked_start_time = None

            return

        # ----------------------------
        # Case 3: obstacle within 0.75 m, but not immediate danger
        # Stop forward motion and turn toward the clearer side.
        # ----------------------------
        avoidance_cmd = Twist()
        avoidance_cmd.linear.x = 0.0

        if left_is_clear or right_is_clear:
            self.blocked_start_time = None

            if left_logic >= right_logic and left_is_clear:
                avoidance_cmd.angular.z = abs(self.turn_speed)
                self.publish_status('OBSTACLE_TURN_LEFT')
            elif right_is_clear:
                avoidance_cmd.angular.z = -abs(self.turn_speed)
                self.publish_status('OBSTACLE_TURN_RIGHT')
            else:
                avoidance_cmd.angular.z = 0.0
                self.publish_status('OBSTACLE_STOP_NO_CLEAR_SIDE')

            self.cmd_pub.publish(avoidance_cmd)
            return

        # ----------------------------
        # Case 4: front blocked and both sides blocked
        # ----------------------------
        self.publish_stop()
        self.publish_status('BLOCKED_WAITING')
        self.handle_possible_blocked()

    def scan_message_is_usable(self, scan):
        """
        Basic validation of the LaserScan message.

        Inf ranges are not automatically invalid. In LaserScan, inf usually means
        no obstacle returned within sensor range.
        """

        if scan is None:
            return False

        if len(scan.ranges) == 0:
            return False

        if scan.angle_increment == 0.0:
            return False

        if scan.range_max <= scan.range_min:
            return False

        # Valid if at least one reading is finite OR at least one reading is inf.
        # NaN-only scans are bad.
        has_non_nan = False

        for r in scan.ranges:
            if not math.isnan(r):
                has_non_nan = True
                break

        return has_non_nan

    def get_sector_min_range(self, scan, min_angle_deg, max_angle_deg):
        """
        Return the minimum finite valid range in an angular sector.

        Angles are normalized to [-180, 180] degrees.

        If no finite obstacle is detected in the sector, return infinity.
        That means the sector is treated as clear.
        """

        min_range = math.inf

        for i, r in enumerate(scan.ranges):
            if math.isnan(r):
                continue

            # Inf means no return. That is not an obstacle, so skip it.
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
        """
        Convert inf to a large clear value for comparisons.
        """

        if math.isinf(value):
            if self.latest_scan is not None:
                return float(self.latest_scan.range_max)
            return 999.0

        return float(value)

    def range_for_publish(self, value):
        """
        Avoid publishing inf when possible. Publish range_max if no obstacle
        is detected in that sector.
        """

        if math.isinf(value):
            if self.latest_scan is not None:
                return float(self.latest_scan.range_max)
            return 999.0

        return float(value)

    def publish_obstacle_telemetry(
        self,
        front_distance,
        left_clearance,
        right_clearance
    ):
        front_msg = Float32()
        front_msg.data = self.range_for_publish(front_distance)
        self.front_distance_pub.publish(front_msg)

        left_msg = Float32()
        left_msg.data = self.range_for_publish(left_clearance)
        self.left_clearance_pub.publish(left_msg)

        right_msg = Float32()
        right_msg.data = self.range_for_publish(right_clearance)
        self.right_clearance_pub.publish(right_msg)

    def handle_possible_blocked(self):
        """
        If the robot is blocked for longer than blocked_timeout, publish BLOCKED.
        """

        now = self.get_clock().now()

        if self.blocked_start_time is None:
            self.blocked_start_time = now
            return

        blocked_duration = (now - self.blocked_start_time).nanoseconds / 1e9

        if blocked_duration >= self.blocked_timeout:
            self.publish_stop()
            self.publish_status('BLOCKED_SAFE_STOP')
            self.publish_mission_event_once('BLOCKED')

    def publish_stop(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0

        try:
            if rclpy.ok():
                self.cmd_pub.publish(cmd)
        except Exception:
            pass

    def publish_status(self, status_text):
        msg = String()
        msg.data = status_text
        self.status_pub.publish(msg)

        if status_text != self.last_status:
            self.get_logger().info(f'Obstacle status: {status_text}')
            self.last_status = status_text

    def publish_mission_event_once(self, event_text):
        """
        Publish mission events to mission_manager_node.

        SENSOR_INVALID and BLOCKED are one-shot events so they do not spam.
        """

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