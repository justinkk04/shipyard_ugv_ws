#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from std_msgs.msg import String, Float32
from std_srvs.srv import Trigger


class MissionManagerNode(Node):
    """
    Mission Manager Node

    Responsibilities:
    - Starts in IDLE
    - Provides /start_mission service
    - Publishes mission state
    - Publishes elapsed mission time
    - Watches for completion/fault events from other nodes
    - Enforces a 6-minute mission timeout
    """

    def __init__(self):
        super().__init__('mission_manager_node')

        # Mission states
        self.STATE_IDLE = 'IDLE'
        self.STATE_INSPECTING = 'INSPECTING'
        self.STATE_COMPLETE = 'COMPLETE'
        self.STATE_TIMEOUT = 'TIMEOUT'
        self.STATE_SAFE_STOP = 'SAFE_STOP'

        self.mission_state = self.STATE_IDLE
        self.mission_start_time = None
        self.elapsed_time_sec = 0.0

        # 6 minutes = 360 seconds
        self.mission_timeout_sec = 360.0

        # Use transient local QoS for mission state so late subscribers can receive
        # the most recent state.
        state_qos = QoSProfile(depth=10)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.state_pub = self.create_publisher(
            String,
            '/mission/state',
            state_qos
        )

        self.elapsed_pub = self.create_publisher(
            Float32,
            '/mission/elapsed_time',
            10
        )

        self.event_sub = self.create_subscription(
            String,
            '/mission/event',
            self.mission_event_callback,
            10
        )

        self.start_service = self.create_service(
            Trigger,
            '/start_mission',
            self.start_mission_callback
        )

        # Publish mission state and elapsed time periodically.
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.get_logger().info('Mission Manager Node started.')
        self.get_logger().info('Current mission state: IDLE')
        self.publish_state()

    def start_mission_callback(self, request, response):
        """
        Service callback for /start_mission.

        This is triggered by:
        ros2 service call /start_mission std_srvs/srv/Trigger "{}"
        """

        if self.mission_state == self.STATE_INSPECTING:
            response.success = False
            response.message = 'Mission already running.'
            return response

        self.mission_state = self.STATE_INSPECTING
        self.mission_start_time = self.get_clock().now()
        self.elapsed_time_sec = 0.0

        self.get_logger().info('START command received.')
        self.get_logger().info('Mission state changed to INSPECTING.')

        self.publish_state()

        response.success = True
        response.message = 'Mission started.'
        return response

    def mission_event_callback(self, msg):
        """
        Receives mission events from other nodes.

        For Phase 1, the route follower will publish:
        COMPLETE

        Later, obstacle avoidance or health monitoring nodes can publish:
        FAULT
        BLOCKED
        SENSOR_INVALID
        """

        event = msg.data.strip().upper()

        self.get_logger().info(f'Received mission event: {event}')

        if event == 'COMPLETE':
            if self.mission_state == self.STATE_INSPECTING:
                self.mission_state = self.STATE_COMPLETE
                self.get_logger().info('Mission state changed to COMPLETE.')
                self.publish_state()

        elif event in ['FAULT', 'BLOCKED', 'SENSOR_INVALID', 'SAFE_STOP']:
            if self.mission_state == self.STATE_INSPECTING:
                self.mission_state = self.STATE_SAFE_STOP
                self.get_logger().warn('Mission state changed to SAFE_STOP.')
                self.publish_state()

    def timer_callback(self):
        """
        Runs repeatedly.
        Publishes mission state and elapsed time.
        Checks timeout if mission is active.
        """

        if self.mission_state == self.STATE_INSPECTING:
            now = self.get_clock().now()

            if self.mission_start_time is not None:
                elapsed_duration = now - self.mission_start_time
                self.elapsed_time_sec = elapsed_duration.nanoseconds / 1e9

            if self.elapsed_time_sec > self.mission_timeout_sec:
                self.mission_state = self.STATE_TIMEOUT
                self.get_logger().warn('Mission timeout exceeded.')
                self.get_logger().warn('Mission state changed to TIMEOUT.')
                self.publish_state()

        self.publish_state()
        self.publish_elapsed_time()

    def publish_state(self):
        msg = String()
        msg.data = self.mission_state
        self.state_pub.publish(msg)

    def publish_elapsed_time(self):
        msg = Float32()
        msg.data = float(self.elapsed_time_sec)
        self.elapsed_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = MissionManagerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Mission Manager interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()