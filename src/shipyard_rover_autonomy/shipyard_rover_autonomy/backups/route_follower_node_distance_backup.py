#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class RouteFollowerNode(Node):
    """
    Route Follower Node

    Phase 1 behavior:
    - Waits for /mission/state = INSPECTING
    - Captures starting odometry pose
    - Drives forward at a fixed speed
    - Computes distance traveled from starting odom pose
    - Stops when target distance is reached
    - Publishes mission progress
    - Publishes COMPLETE event when endpoint is reached

    Later:
    - This node can publish /nominal_cmd_vel instead of /cmd_vel
    - obstacle_avoidance_node can read /nominal_cmd_vel and publish final /cmd_vel
    """

    def __init__(self):
        super().__init__('route_follower_node')

        # Parameters
        self.declare_parameter('target_distance', 6.0)
        self.declare_parameter('forward_speed', 0.22)
        self.declare_parameter('direct_cmd_vel', True)
        self.declare_parameter('control_rate_hz', 10.0)

        self.target_distance = float(
            self.get_parameter('target_distance').value
        )

        self.forward_speed = float(
            self.get_parameter('forward_speed').value
        )

        self.direct_cmd_vel = bool(
            self.get_parameter('direct_cmd_vel').value
        )

        self.control_rate_hz = float(
            self.get_parameter('control_rate_hz').value
        )

        if self.direct_cmd_vel:
            self.cmd_topic = '/cmd_vel'
        else:
            self.cmd_topic = '/nominal_cmd_vel'

        # Mission state tracking
        self.mission_state = 'IDLE'
        self.previous_mission_state = 'IDLE'
        self.mission_active = False
        self.completed = False

        # Odometry tracking
        self.latest_x = None
        self.latest_y = None
        self.start_x = None
        self.start_y = None
        self.distance_traveled = 0.0

        # Used so we do not spam COMPLETE repeatedly
        self.completion_event_sent = False

        state_qos = QoSProfile(depth=10)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

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

        timer_period = 1.0 / self.control_rate_hz
        self.control_timer = self.create_timer(
            timer_period,
            self.control_loop
        )

        self.get_logger().info('Route Follower Node started.')
        self.get_logger().info(f'Command topic: {self.cmd_topic}')
        self.get_logger().info(f'Target distance: {self.target_distance:.2f} m')
        self.get_logger().info(f'Forward speed: {self.forward_speed:.2f} m/s')

    def mission_state_callback(self, msg):
        """
        Receives mission state from mission_manager_node.
        """

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
        """
        Reads robot position from /odom.
        For Phase 1, we only need x and y.
        """

        self.latest_x = msg.pose.pose.position.x
        self.latest_y = msg.pose.pose.position.y

    def start_new_route(self):
        """
        Resets route-following state when mission starts.
        """

        self.mission_active = True
        self.completed = False
        self.completion_event_sent = False

        self.start_x = None
        self.start_y = None
        self.distance_traveled = 0.0

        self.publish_progress(0.0)

        self.get_logger().info('Route follower activated.')
        self.get_logger().info('Waiting for odometry start pose...')

    def control_loop(self):
        """
        Main control loop.

        If mission is INSPECTING:
        - Capture start pose if needed
        - Compute distance traveled
        - Drive forward until target distance
        - Stop and publish COMPLETE

        If mission is not active:
        - Do not command motion
        """

        if not self.mission_active:
            return

        if self.completed:
            self.publish_stop()
            return

        if self.mission_state != 'INSPECTING':
            self.publish_stop()
            return

        if self.latest_x is None or self.latest_y is None:
            self.get_logger().warn_throttle(
                2.0,
                'No odometry received yet. Holding position.'
            )
            self.publish_stop()
            return

        if self.start_x is None or self.start_y is None:
            self.start_x = self.latest_x
            self.start_y = self.latest_y
            self.distance_traveled = 0.0

            self.get_logger().info(
                f'Start odom captured: x={self.start_x:.2f}, y={self.start_y:.2f}'
            )

        dx = self.latest_x - self.start_x
        dy = self.latest_y - self.start_y

        self.distance_traveled = math.sqrt(dx * dx + dy * dy)

        progress_percent = 100.0 * self.distance_traveled / self.target_distance
        progress_percent = max(0.0, min(progress_percent, 100.0))

        self.publish_progress(progress_percent)

        if self.distance_traveled >= self.target_distance:
            self.publish_stop()
            self.completed = True
            self.mission_active = False

            self.get_logger().info(
                f'Target reached. Distance traveled: {self.distance_traveled:.2f} m'
            )

            self.publish_complete_event()
            return

        self.publish_forward_motion()

    def publish_forward_motion(self):
        """
        Publishes forward velocity command.
        """

        cmd = Twist()
        cmd.linear.x = self.forward_speed
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def publish_stop(self):
        """
        Publishes zero velocity command.
        """

        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def publish_progress(self, progress_percent):
        """
        Publishes mission progress as a percentage from 0 to 100.
        """

        msg = Float32()
        msg.data = float(progress_percent)
        self.progress_pub.publish(msg)

    def publish_complete_event(self):
        """
        Publishes mission completion event to mission_manager_node.
        """

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
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()