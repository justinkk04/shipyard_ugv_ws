#!/usr/bin/env python3

import csv
import json
import math
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan


class TelemetryLoggerNode(Node):
    """
    Telemetry Logger Node

    This node does not control the robot.

    It:
    - Watches important mission, obstacle, command, odom, IMU, and scan topics
    - Publishes a compact /ugv/status message
    - Publishes simulated battery percentage
    - Publishes sensor health
    - Writes a CSV mission log at 1 Hz
    - Writes a final mission summary text file
    - Prints a dashboard in one terminal
    """

    def __init__(self):
        super().__init__('telemetry_logger_node')

        # --------------------------------------------------
        # Parameters
        # --------------------------------------------------
        self.declare_parameter('update_rate_hz', 1.0)
        self.declare_parameter('log_dir', os.path.expanduser('~/shipyard_ugv_ws/logs'))
        self.declare_parameter('sensor_stale_timeout', 1.0)
        self.declare_parameter('battery_drain_percent_per_sec', 0.02)

        self.update_rate_hz = float(self.get_parameter('update_rate_hz').value)
        self.log_dir = str(self.get_parameter('log_dir').value)
        self.sensor_stale_timeout = float(self.get_parameter('sensor_stale_timeout').value)
        self.battery_drain_percent_per_sec = float(
            self.get_parameter('battery_drain_percent_per_sec').value
        )

        os.makedirs(self.log_dir, exist_ok=True)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

        self.csv_path = os.path.join(
            self.log_dir,
            f'mission_log_{timestamp}.csv'
        )

        self.summary_path = os.path.join(
            self.log_dir,
            f'mission_summary_{timestamp}.txt'
        )

        # --------------------------------------------------
        # Latest mission values
        # --------------------------------------------------
        self.mission_state = 'UNKNOWN'
        self.previous_mission_state = 'UNKNOWN'
        self.elapsed_time = 0.0
        self.progress = 0.0
        self.last_mission_event = 'NONE'

        # --------------------------------------------------
        # Latest obstacle values
        # --------------------------------------------------
        self.front_distance = -1.0
        self.left_clearance = -1.0
        self.right_clearance = -1.0
        self.obstacle_status = 'UNKNOWN'

        self.obstacle_est_x = 0.0
        self.obstacle_est_y = 0.0
        self.obstacle_est_valid = False

        # --------------------------------------------------
        # Latest command values
        # --------------------------------------------------
        self.nominal_linear_x = 0.0
        self.nominal_angular_z = 0.0

        self.cmd_linear_x = 0.0
        self.cmd_angular_z = 0.0

        # --------------------------------------------------
        # Latest odom values
        # --------------------------------------------------
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0

        # --------------------------------------------------
        # Sensor health timestamps
        # --------------------------------------------------
        self.last_odom_time = None
        self.last_imu_time = None
        self.last_scan_time = None

        self.sensor_health = 'UNKNOWN'

        # --------------------------------------------------
        # Simulated battery
        # --------------------------------------------------
        self.battery_percent = 100.0
        self.last_timer_time = self.get_clock().now()

        # --------------------------------------------------
        # Mission history
        # --------------------------------------------------
        self.state_history = []
        self.event_history = []
        self.obstacle_history = []

        self.seen_active_mission = False
        self.summary_written = False

        # --------------------------------------------------
        # QoS
        # --------------------------------------------------
        state_qos = QoSProfile(depth=10)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        log_path_qos = QoSProfile(depth=10)
        log_path_qos.reliability = ReliabilityPolicy.RELIABLE
        log_path_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # --------------------------------------------------
        # Subscribers
        # --------------------------------------------------
        self.create_subscription(
            String,
            '/mission/state',
            self.mission_state_callback,
            state_qos
        )

        self.create_subscription(
            Float32,
            '/mission/elapsed_time',
            self.elapsed_time_callback,
            10
        )

        self.create_subscription(
            Float32,
            '/mission/progress',
            self.progress_callback,
            10
        )

        self.create_subscription(
            String,
            '/mission/event',
            self.mission_event_callback,
            10
        )

        self.create_subscription(
            Float32,
            '/obstacle/front_distance',
            self.front_distance_callback,
            10
        )

        self.create_subscription(
            Float32,
            '/obstacle/left_clearance',
            self.left_clearance_callback,
            10
        )

        self.create_subscription(
            Float32,
            '/obstacle/right_clearance',
            self.right_clearance_callback,
            10
        )

        self.create_subscription(
            String,
            '/obstacle/status',
            self.obstacle_status_callback,
            10
        )

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )

        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.create_subscription(
            Twist,
            '/nominal_cmd_vel',
            self.nominal_cmd_callback,
            10
        )

        # --------------------------------------------------
        # Publishers
        # --------------------------------------------------
        self.status_pub = self.create_publisher(
            String,
            '/ugv/status',
            10
        )

        self.battery_pub = self.create_publisher(
            Float32,
            '/ugv/battery_percent',
            10
        )

        self.sensor_health_pub = self.create_publisher(
            String,
            '/ugv/sensor_health',
            10
        )

        self.log_path_pub = self.create_publisher(
            String,
            '/ugv/log_path',
            log_path_qos
        )

        # --------------------------------------------------
        # CSV file setup
        # --------------------------------------------------
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        self.csv_writer.writerow([
            'wall_time',
            'mission_state',
            'elapsed_time_sec',
            'progress_percent',
            'last_mission_event',
            'battery_percent',
            'sensor_health',
            'odom_x',
            'odom_y',
            'odom_yaw_rad',
            'front_distance_m',
            'left_clearance_m',
            'right_clearance_m',
            'obstacle_status',
            'obstacle_est_valid',
            'obstacle_est_x',
            'obstacle_est_y',
            'nominal_linear_x',
            'nominal_angular_z',
            'cmd_linear_x',
            'cmd_angular_z'
        ])

        self.csv_file.flush()

        # --------------------------------------------------
        # Timer
        # --------------------------------------------------
        timer_period = 1.0 / self.update_rate_hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.publish_log_path()

        self.get_logger().info('Telemetry Logger Node started.')
        self.get_logger().info(f'CSV log file: {self.csv_path}')
        self.get_logger().info(f'Summary file: {self.summary_path}')
        self.get_logger().info(f'Update rate: {self.update_rate_hz:.2f} Hz')

    # --------------------------------------------------
    # Mission callbacks
    # --------------------------------------------------
    def mission_state_callback(self, msg):
        new_state = msg.data.strip().upper()

        if new_state != self.mission_state:
            wall_time = datetime.now().strftime('%H:%M:%S')
            transition = f'{wall_time}: {self.mission_state} -> {new_state}'
            self.state_history.append(transition)

            self.previous_mission_state = self.mission_state
            self.mission_state = new_state

            if new_state == 'INSPECTING':
                self.seen_active_mission = True
                self.summary_written = False

            self.get_logger().info(f'Mission state changed: {transition}')

    def elapsed_time_callback(self, msg):
        self.elapsed_time = float(msg.data)

    def progress_callback(self, msg):
        self.progress = float(msg.data)

    def mission_event_callback(self, msg):
        event = msg.data.strip().upper()

        if event == '':
            return

        self.last_mission_event = event

        wall_time = datetime.now().strftime('%H:%M:%S')
        entry = f'{wall_time}: {event}'
        self.event_history.append(entry)

        self.get_logger().info(f'Mission event logged: {event}')

    # --------------------------------------------------
    # Obstacle callbacks
    # --------------------------------------------------
    def front_distance_callback(self, msg):
        self.front_distance = float(msg.data)
        self.update_obstacle_estimate()

    def left_clearance_callback(self, msg):
        self.left_clearance = float(msg.data)

    def right_clearance_callback(self, msg):
        self.right_clearance = float(msg.data)

    def obstacle_status_callback(self, msg):
        new_status = msg.data.strip()

        if new_status != self.obstacle_status:
            wall_time = datetime.now().strftime('%H:%M:%S')
            entry = (
                f'{wall_time}: {new_status}, '
                f'front={self.front_distance:.2f} m, '
                f'est=({self.obstacle_est_x:.2f}, {self.obstacle_est_y:.2f})'
            )
            self.obstacle_history.append(entry)

        self.obstacle_status = new_status

    # --------------------------------------------------
    # Sensor callbacks
    # --------------------------------------------------
    def odom_callback(self, msg):
        self.last_odom_time = self.get_clock().now()

        self.odom_x = float(msg.pose.pose.position.x)
        self.odom_y = float(msg.pose.pose.position.y)

        q = msg.pose.pose.orientation
        self.odom_yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

        self.update_obstacle_estimate()

    def imu_callback(self, msg):
        self.last_imu_time = self.get_clock().now()

    def scan_callback(self, msg):
        self.last_scan_time = self.get_clock().now()

    # --------------------------------------------------
    # Command callbacks
    # --------------------------------------------------
    def cmd_vel_callback(self, msg):
        self.cmd_linear_x = float(msg.linear.x)
        self.cmd_angular_z = float(msg.angular.z)

    def nominal_cmd_callback(self, msg):
        self.nominal_linear_x = float(msg.linear.x)
        self.nominal_angular_z = float(msg.angular.z)

    # --------------------------------------------------
    # Main timer
    # --------------------------------------------------
    def timer_callback(self):
        now = self.get_clock().now()

        self.update_battery(now)
        self.update_sensor_health(now)
        self.update_obstacle_estimate()

        self.publish_status()
        self.publish_battery()
        self.publish_sensor_health()
        self.publish_log_path()

        self.write_csv_row()
        self.print_dashboard()

        if self.mission_state in ['COMPLETE', 'SAFE_STOP', 'TIMEOUT']:
            if self.seen_active_mission and not self.summary_written:
                self.write_summary()
                self.summary_written = True

        self.last_timer_time = now

    # --------------------------------------------------
    # Helper logic
    # --------------------------------------------------
    def update_battery(self, now):
        dt = (now - self.last_timer_time).nanoseconds / 1e9

        if dt < 0.0:
            dt = 0.0

        if self.mission_state == 'INSPECTING':
            self.battery_percent -= self.battery_drain_percent_per_sec * dt
            self.battery_percent = max(0.0, self.battery_percent)

    def update_sensor_health(self, now):
        stale_sensors = []

        if self.is_stale(now, self.last_odom_time):
            stale_sensors.append('ODOM')

        if self.is_stale(now, self.last_imu_time):
            stale_sensors.append('IMU')

        if self.is_stale(now, self.last_scan_time):
            stale_sensors.append('SCAN')

        if len(stale_sensors) == 0:
            self.sensor_health = 'OK'
        elif len(stale_sensors) == 1:
            self.sensor_health = f'WARN_{stale_sensors[0]}_STALE'
        else:
            self.sensor_health = 'FAULT_MULTIPLE_SENSOR_STALE'

    def is_stale(self, now, last_time):
        if last_time is None:
            return True

        age = (now - last_time).nanoseconds / 1e9
        return age > self.sensor_stale_timeout

    def update_obstacle_estimate(self):
        """
        Simple estimated obstacle location:

        obstacle_est_x = robot_x + front_distance * cos(yaw)
        obstacle_est_y = robot_y + front_distance * sin(yaw)

        This is not full mapping. It is just a simple report-friendly estimate.
        """

        if self.front_distance <= 0.0:
            self.obstacle_est_valid = False
            return

        if math.isnan(self.front_distance) or math.isinf(self.front_distance):
            self.obstacle_est_valid = False
            return

        self.obstacle_est_x = self.odom_x + self.front_distance * math.cos(self.odom_yaw)
        self.obstacle_est_y = self.odom_y + self.front_distance * math.sin(self.odom_yaw)
        self.obstacle_est_valid = True

    def quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    # --------------------------------------------------
    # Publishers
    # --------------------------------------------------
    def publish_status(self):
        status_data = {
            'mission_state': self.mission_state,
            'elapsed_time_sec': round(self.elapsed_time, 2),
            'progress_percent': round(self.progress, 2),
            'battery_percent': round(self.battery_percent, 2),
            'robot_x': round(self.odom_x, 3),
            'robot_y': round(self.odom_y, 3),
            'robot_yaw_rad': round(self.odom_yaw, 3),
            'front_distance_m': round(self.front_distance, 3),
            'left_clearance_m': round(self.left_clearance, 3),
            'right_clearance_m': round(self.right_clearance, 3),
            'obstacle_status': self.obstacle_status,
            'obstacle_est_valid': self.obstacle_est_valid,
            'obstacle_est_x': round(self.obstacle_est_x, 3),
            'obstacle_est_y': round(self.obstacle_est_y, 3),
            'sensor_health': self.sensor_health,
            'last_mission_event': self.last_mission_event,
            'cmd_linear_x': round(self.cmd_linear_x, 3),
            'cmd_angular_z': round(self.cmd_angular_z, 3)
        }

        msg = String()
        msg.data = json.dumps(status_data)
        self.status_pub.publish(msg)

    def publish_battery(self):
        msg = Float32()
        msg.data = float(self.battery_percent)
        self.battery_pub.publish(msg)

    def publish_sensor_health(self):
        msg = String()
        msg.data = self.sensor_health
        self.sensor_health_pub.publish(msg)

    def publish_log_path(self):
        msg = String()
        msg.data = f'csv={self.csv_path}; summary={self.summary_path}'
        self.log_path_pub.publish(msg)

    # --------------------------------------------------
    # CSV and summary files
    # --------------------------------------------------
    def write_csv_row(self):
        wall_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

        self.csv_writer.writerow([
            wall_time,
            self.mission_state,
            f'{self.elapsed_time:.2f}',
            f'{self.progress:.2f}',
            self.last_mission_event,
            f'{self.battery_percent:.2f}',
            self.sensor_health,
            f'{self.odom_x:.3f}',
            f'{self.odom_y:.3f}',
            f'{self.odom_yaw:.3f}',
            f'{self.front_distance:.3f}',
            f'{self.left_clearance:.3f}',
            f'{self.right_clearance:.3f}',
            self.obstacle_status,
            self.obstacle_est_valid,
            f'{self.obstacle_est_x:.3f}',
            f'{self.obstacle_est_y:.3f}',
            f'{self.nominal_linear_x:.3f}',
            f'{self.nominal_angular_z:.3f}',
            f'{self.cmd_linear_x:.3f}',
            f'{self.cmd_angular_z:.3f}'
        ])

        self.csv_file.flush()

    def write_summary(self):
        with open(self.summary_path, 'w') as f:
            f.write('Shipyard UGV Mission Summary\n')
            f.write('============================\n\n')

            f.write('Final Mission Outcome\n')
            f.write('---------------------\n')
            f.write(f'Final mission state:     {self.mission_state}\n')
            f.write(f'Elapsed mission time:    {self.elapsed_time:.2f} sec\n')
            f.write(f'Final route progress:    {self.progress:.2f} %\n')
            f.write(f'Last mission event:      {self.last_mission_event}\n')
            f.write(f'Simulated battery:       {self.battery_percent:.2f} %\n')
            f.write(f'Sensor health:           {self.sensor_health}\n\n')

            f.write('Final Robot Estimate\n')
            f.write('--------------------\n')
            f.write(f'Odom x:                  {self.odom_x:.3f} m\n')
            f.write(f'Odom y:                  {self.odom_y:.3f} m\n')
            f.write(f'Odom yaw:                {self.odom_yaw:.3f} rad\n\n')

            f.write('Final Obstacle Data\n')
            f.write('-------------------\n')
            f.write(f'Obstacle status:         {self.obstacle_status}\n')
            f.write(f'Front distance:          {self.front_distance:.3f} m\n')
            f.write(f'Left clearance:          {self.left_clearance:.3f} m\n')
            f.write(f'Right clearance:         {self.right_clearance:.3f} m\n')
            f.write(f'Obstacle estimate valid: {self.obstacle_est_valid}\n')
            f.write(f'Obstacle est x:          {self.obstacle_est_x:.3f} m\n')
            f.write(f'Obstacle est y:          {self.obstacle_est_y:.3f} m\n\n')

            f.write('Final Command Data\n')
            f.write('------------------\n')
            f.write(f'Nominal linear.x:        {self.nominal_linear_x:.3f}\n')
            f.write(f'Nominal angular.z:       {self.nominal_angular_z:.3f}\n')
            f.write(f'Final cmd linear.x:      {self.cmd_linear_x:.3f}\n')
            f.write(f'Final cmd angular.z:     {self.cmd_angular_z:.3f}\n\n')

            f.write('Mission State History\n')
            f.write('---------------------\n')
            for item in self.state_history:
                f.write(item + '\n')

            f.write('\nMission Event History\n')
            f.write('---------------------\n')
            for item in self.event_history:
                f.write(item + '\n')

            f.write('\nObstacle Status History\n')
            f.write('-----------------------\n')
            for item in self.obstacle_history:
                f.write(item + '\n')

            f.write('\nLog Files\n')
            f.write('---------\n')
            f.write(f'CSV log:     {self.csv_path}\n')
            f.write(f'Summary log: {self.summary_path}\n')

        self.get_logger().info(f'Mission summary written: {self.summary_path}')

    # --------------------------------------------------
    # Dashboard
    # --------------------------------------------------
    def print_dashboard(self):
        os.system('clear')

        print('============================================================')
        print('SHIPYARD UGV TELEMETRY DASHBOARD')
        print('============================================================')
        print(f'Mission State:        {self.mission_state}')
        print(f'Elapsed Time:         {self.elapsed_time:7.2f} sec')
        print(f'Progress:             {self.progress:7.2f} %')
        print(f'Last Mission Event:   {self.last_mission_event}')
        print(f'Battery:              {self.battery_percent:7.2f} %')
        print(f'Sensor Health:        {self.sensor_health}')
        print('------------------------------------------------------------')
        print('Robot Estimate')
        print(f'Odom Position:        x={self.odom_x:7.3f}, y={self.odom_y:7.3f}')
        print(f'Yaw:                  {self.odom_yaw:7.3f} rad')
        print('------------------------------------------------------------')
        print('Obstacle Data')
        print(f'Obstacle Status:      {self.obstacle_status}')
        print(f'Front Distance:       {self.front_distance:7.3f} m')
        print(f'Left Clearance:       {self.left_clearance:7.3f} m')
        print(f'Right Clearance:      {self.right_clearance:7.3f} m')
        print(f'Obstacle Estimate:    valid={self.obstacle_est_valid}, '
              f'x={self.obstacle_est_x:7.3f}, y={self.obstacle_est_y:7.3f}')
        print('------------------------------------------------------------')
        print('Command Comparison')
        print(f'Nominal Cmd:          linear.x={self.nominal_linear_x:7.3f}, '
              f'angular.z={self.nominal_angular_z:7.3f}')
        print(f'Final /cmd_vel:       linear.x={self.cmd_linear_x:7.3f}, '
              f'angular.z={self.cmd_angular_z:7.3f}')
        print('------------------------------------------------------------')
        print(f'CSV Log:              {self.csv_path}')
        print(f'Summary File:         {self.summary_path}')
        print('============================================================')
        print('Press CTRL+C to stop telemetry logger.')

    def destroy_node(self):
        try:
            self.csv_file.flush()
            self.csv_file.close()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = TelemetryLoggerNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('Telemetry Logger interrupted by user.')

    finally:
        node.destroy_node()

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()