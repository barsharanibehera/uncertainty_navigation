#!/usr/bin/env python3

import math
import csv
import time
import os

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray


class GoalNavigator(Node):

    def __init__(self):
        super().__init__('goal_navigator')

        self.declare_parameter('goal_x', 1.0)
        self.declare_parameter('goal_y', 1.0)
        
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.odom_received = False
        self.uncertainty_received = False
        self.scan_received = False
        self.goal_reached = False
        self.is_avoiding = False

        self.front_std = 0.0
        self.left_std = 0.0
        self.right_std = 0.0

        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')

        csv_path = os.path.expanduser('~/navigation_results.csv')
        self.csv_file = open(csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Time', 'Distance_to_Goal', 'Front_Distance', 'Front_Std', 'Uncertainty_Class', 'Linear_Speed', 'Angular_Speed'])
        self.start_time = time.time()
        
        self.cmd_publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.uncertainty_subscriber = self.create_subscription(Float32MultiArray, '/navigation_uncertainty', self.uncertainty_callback, 10)
        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Goal Navigator started')
        self.get_logger().info(f'Logging data to: {csv_path}')
        self.get_logger().info(f'Dynamic Goal Set = ({self.goal_x:.2f}, {self.goal_y:.2f})')

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        self.odom_received = True

    def uncertainty_callback(self, msg):
        if len(msg.data) < 6:
            return
        self.front_std = msg.data[1]
        self.left_std = msg.data[3]
        self.right_std = msg.data[5]
        self.uncertainty_received = True

    def scan_callback(self, msg):
        self.front_distance = self.get_sector_distance(msg, 0.0, 25.0)
        self.left_distance = self.get_sector_distance(msg, 90.0, 25.0)
        self.right_distance = self.get_sector_distance(msg, -90.0, 25.0)
        self.scan_received = True

    def get_sector_distance(self, scan, center_degrees, width_degrees):
        center = math.radians(center_degrees)
        width = math.radians(width_degrees)
        minimum_distance = float('inf')
        for i, distance in enumerate(scan.ranges):
            if not math.isfinite(distance):
                continue
            angle = scan.angle_min + (i * scan.angle_increment)
            angle_difference = math.atan2(math.sin(angle - center), math.cos(angle - center))
            if abs(angle_difference) <= width:
                if scan.range_min <= distance <= scan.range_max:
                    minimum_distance = min(minimum_distance, distance)
        return minimum_distance

    def classify_uncertainty(self, std_dev):
        if std_dev < 0.10:
            return 'LOW'
        elif std_dev < 0.30:
            return 'MEDIUM'
        else:
            return 'HIGH'

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def stop_robot(self):
        try:
            if rclpy.ok():
                cmd = TwistStamped()
                cmd.twist.linear.x = 0.0
                cmd.twist.angular.z = 0.0
                self.cmd_publisher.publish(cmd)
        except Exception:
            pass

    def obstacle_avoidance(self, cmd, distance_to_goal):
        trigger_threshold = 0.60
        clear_threshold = 0.85

        if distance_to_goal < self.front_distance:
            self.is_avoiding = False
            return False

        if self.front_distance < trigger_threshold:
            self.is_avoiding = True
        elif self.front_distance > clear_threshold:
            self.is_avoiding = False

        if not self.is_avoiding:
            return False

        cmd.twist.linear.x = 0.02
        left_score = self.left_distance - self.left_std
        right_score = self.right_distance - self.right_std

        if left_score > right_score:
            cmd.twist.angular.z = 0.45
            self.get_logger().warn(f'Avoiding LEFT | Left space={self.left_distance:.2f}m')
        else:
            cmd.twist.angular.z = -0.45
            self.get_logger().warn(f'Avoiding RIGHT | Right space={self.right_distance:.2f}m')
        return True

    def control_loop(self):
        if self.goal_reached:
            self.stop_robot()
            return
        if not self.odom_received or not self.uncertainty_received or not self.scan_received:
            return

        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        distance = math.sqrt(dx * dx + dy * dy)

        if distance < 0.25:
            self.stop_robot()
            self.goal_reached = True
            self.get_logger().info('==================================================')
            self.get_logger().info(f'🏆 GOAL REACHED! Final Position=({self.x:.2f}, {self.y:.2f})')
            self.get_logger().info('==================================================')
            current_time = round(time.time() - self.start_time, 2)
            self.csv_writer.writerow([current_time, distance, self.front_distance, self.front_std, 'LOW', 0.0, 0.0])
            self.csv_file.flush()
            return

        desired_yaw = math.atan2(dy, dx)
        heading_error = self.normalize_angle(desired_yaw - self.yaw)
        
        front_uncertainty = self.classify_uncertainty(self.front_std)
        left_uncertainty = self.classify_uncertainty(self.left_std)
        right_uncertainty = self.classify_uncertainty(self.right_std)

        # --- MODIFIED: Clear, readable progress banner ---
        self.get_logger().info('--------------------------------------------------')
        self.get_logger().info(f'🎯 TARGET: ({self.goal_x:.2f}, {self.goal_y:.2f}) | 📍 CURRENT: ({self.x:.2f}, {self.y:.2f})')
        self.get_logger().info(f'📏 DISTANCE REMAINING: {distance:.2f} meters')
        self.get_logger().info(f'📡 Laser -> Front: {self.front_distance:.2f}m | Left: {self.left_distance:.2f}m | Right: {self.right_distance:.2f}m')
        self.get_logger().info(f'📊 Uncertainty -> Front: {front_uncertainty} | Left: {left_uncertainty} | Right: {right_uncertainty}')
        self.get_logger().info('--------------------------------------------------')

        cmd = TwistStamped()
        obstacle_detected = self.obstacle_avoidance(cmd, distance)
        
        if not obstacle_detected:
            if abs(heading_error) > math.radians(12):
                cmd.twist.linear.x = 0.0
                cmd.twist.angular.z = max(-0.8, min(0.8, 1.5 * heading_error))
            else:
                if distance > 1.0:
                    linear_speed = 0.20
                elif distance > 0.50:
                    linear_speed = 0.12
                else:
                    linear_speed = 0.07

                if front_uncertainty == 'HIGH':
                    linear_speed *= 0.50
                    self.get_logger().warn('⚠️ HIGH uncertainty -> Speed reduced by 50%')
                elif front_uncertainty == 'MEDIUM':
                    linear_speed *= 0.75
                    self.get_logger().warn('⚠️ MEDIUM uncertainty -> Speed reduced by 25%')

                cmd.twist.linear.x = linear_speed
                cmd.twist.angular.z = 0.0

        self.cmd_publisher.publish(cmd)

        current_time = round(time.time() - self.start_time, 2)
        self.csv_writer.writerow([
            current_time, 
            round(distance, 3), 
            round(self.front_distance, 3), 
            round(self.front_std, 3), 
            front_uncertainty, 
            round(cmd.twist.linear.x, 3),
            round(cmd.twist.angular.z, 3)
        ])

    def destroy_node(self):
        self.stop_robot()
        try:
            self.csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GoalNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
