#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray


class GoalNavigator(Node):

    def __init__(self):

        super().__init__('goal_navigator')

        # ==================================================
        # GOAL
        # ==================================================

        self.goal_x = 2.0
        self.goal_y = 2.0

        # ==================================================
        # ROBOT STATE
        # ==================================================

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.odom_received = False
        self.uncertainty_received = False
        self.scan_received = False

        # Prevent repeated GOAL REACHED messages
        self.goal_reached = False

        # ==================================================
        # UNCERTAINTY VALUES
        # ==================================================

        self.front_std = 0.0
        self.left_std = 0.0
        self.right_std = 0.0

        # ==================================================
        # LASER DISTANCES
        # ==================================================

        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')

        # ==================================================
        # PUBLISHER
        # ==================================================

        self.cmd_publisher = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )

        # ==================================================
        # SUBSCRIBERS
        # ==================================================

        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.uncertainty_subscriber = self.create_subscription(
            Float32MultiArray,
            '/navigation_uncertainty',
            self.uncertainty_callback,
            10
        )

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # ==================================================
        # CONTROL TIMER
        # ==================================================

        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

        self.get_logger().info(
            'Goal Navigator started'
        )

        self.get_logger().info(
            f'Goal = ({self.goal_x:.2f}, {self.goal_y:.2f})'
        )

    # ======================================================
    # ODOMETRY CALLBACK
    # ======================================================

    def odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        siny_cosp = 2.0 * (
            q.w * q.z +
            q.x * q.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            q.y * q.y +
            q.z * q.z
        )

        self.yaw = math.atan2(
            siny_cosp,
            cosy_cosp
        )

        self.odom_received = True

    # ======================================================
    # UNCERTAINTY CALLBACK
    # ======================================================

    def uncertainty_callback(self, msg):

        if len(msg.data) < 6:
            return

        # Data format:
        #
        # [front_distance,
        #  front_std,
        #  left_distance,
        #  left_std,
        #  right_distance,
        #  right_std]

        self.front_std = msg.data[1]
        self.left_std = msg.data[3]
        self.right_std = msg.data[5]

        self.uncertainty_received = True

    # ======================================================
    # LASER SCAN CALLBACK
    # ======================================================

    def scan_callback(self, msg):

        self.front_distance = self.get_sector_distance(
            msg,
            0.0,
            25.0
        )

        self.left_distance = self.get_sector_distance(
            msg,
            90.0,
            25.0
        )

        self.right_distance = self.get_sector_distance(
            msg,
            -90.0,
            25.0
        )

        self.scan_received = True

    # ======================================================
    # GET MINIMUM DISTANCE FROM LASER SECTOR
    # ======================================================

    def get_sector_distance(
        self,
        scan,
        center_degrees,
        width_degrees
    ):

        center = math.radians(center_degrees)
        width = math.radians(width_degrees)

        minimum_distance = float('inf')

        for i, distance in enumerate(scan.ranges):

            if not math.isfinite(distance):
                continue

            angle = scan.angle_min + (
                i * scan.angle_increment
            )

            angle_difference = math.atan2(
                math.sin(angle - center),
                math.cos(angle - center)
            )

            if abs(angle_difference) <= width:

                if scan.range_min <= distance <= scan.range_max:

                    minimum_distance = min(
                        minimum_distance,
                        distance
                    )

        return minimum_distance

    # ======================================================
    # UNCERTAINTY CLASSIFICATION
    # ======================================================

    def classify_uncertainty(self, std_dev):

        if std_dev < 0.10:
            return 'LOW'

        elif std_dev < 0.30:
            return 'MEDIUM'

        else:
            return 'HIGH'

    # ======================================================
    # ANGLE NORMALIZATION
    # ======================================================

    def normalize_angle(self, angle):

        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    # ======================================================
    # STOP ROBOT
    # ======================================================

    def stop_robot(self):

        cmd = TwistStamped()

        cmd.twist.linear.x = 0.0
        cmd.twist.angular.z = 0.0

        self.cmd_publisher.publish(cmd)

    # ======================================================
    # OBSTACLE AVOIDANCE
    # ======================================================

    def obstacle_avoidance(self, cmd):

        # Obstacle threshold
        obstacle_threshold = 0.55

        # --------------------------------------------------
        # NO FRONT OBSTACLE
        # --------------------------------------------------

        if self.front_distance > obstacle_threshold:

            return False

        # --------------------------------------------------
        # OBSTACLE DETECTED
        # --------------------------------------------------

        self.get_logger().warn(
            f'OBSTACLE AHEAD | '
            f'Front={self.front_distance:.2f} m'
        )

        # Stop forward movement
        cmd.twist.linear.x = 0.0

        # --------------------------------------------------
        # Compare LEFT and RIGHT
        # --------------------------------------------------

        left_score = (
            self.left_distance -
            self.left_std
        )

        right_score = (
            self.right_distance -
            self.right_std
        )

        # --------------------------------------------------
        # Choose safer direction
        # --------------------------------------------------

        if left_score > right_score:

            cmd.twist.angular.z = 0.45

            self.get_logger().warn(
                f'Avoiding obstacle -> LEFT | '
                f'Left={self.left_distance:.2f} m '
                f'Uncertainty={self.left_std:.2f}'
            )

        else:

            cmd.twist.angular.z = -0.45

            self.get_logger().warn(
                f'Avoiding obstacle -> RIGHT | '
                f'Right={self.right_distance:.2f} m '
                f'Uncertainty={self.right_std:.2f}'
            )

        return True

    # ======================================================
    # MAIN CONTROL LOOP
    # ======================================================

    def control_loop(self):

        # --------------------------------------------------
        # Already reached goal
        # --------------------------------------------------

        if self.goal_reached:

            self.stop_robot()

            return

        # --------------------------------------------------
        # Wait for odometry
        # --------------------------------------------------

        if not self.odom_received:

            return

        # --------------------------------------------------
        # Wait for uncertainty
        # --------------------------------------------------

        if not self.uncertainty_received:

            return

        # --------------------------------------------------
        # Wait for laser
        # --------------------------------------------------

        if not self.scan_received:

            return

        # ==================================================
        # CALCULATE DISTANCE TO GOAL
        # ==================================================

        dx = self.goal_x - self.x
        dy = self.goal_y - self.y

        distance = math.sqrt(
            dx * dx +
            dy * dy
        )

        # ==================================================
        # GOAL CHECK
        # ==================================================

        if distance < 0.20:

            self.stop_robot()

            self.goal_reached = True

            self.get_logger().info(
                f'GOAL REACHED! '
                f'Position=({self.x:.2f}, {self.y:.2f})'
            )

            return

        # ==================================================
        # DESIRED HEADING
        # ==================================================

        desired_yaw = math.atan2(
            dy,
            dx
        )

        # ==================================================
        # HEADING ERROR
        # ==================================================

        heading_error = self.normalize_angle(
            desired_yaw - self.yaw
        )

        heading_error_deg = math.degrees(
            heading_error
        )

        # ==================================================
        # UNCERTAINTY CLASSIFICATION
        # ==================================================

        front_uncertainty = self.classify_uncertainty(
            self.front_std
        )

        left_uncertainty = self.classify_uncertainty(
            self.left_std
        )

        right_uncertainty = self.classify_uncertainty(
            self.right_std
        )

        # ==================================================
        # DISPLAY STATUS
        # ==================================================

        self.get_logger().info(
            f'Position=({self.x:.2f}, {self.y:.2f}) | '
            f'Distance={distance:.2f} m'
        )

        self.get_logger().info(
            f'Laser -> '
            f'Front={self.front_distance:.2f} m | '
            f'Left={self.left_distance:.2f} m | '
            f'Right={self.right_distance:.2f} m'
        )

        self.get_logger().info(
            f'Uncertainty -> '
            f'Front: {front_uncertainty} '
            f'(std={self.front_std:.2f}) | '
            f'Left: {left_uncertainty} '
            f'(std={self.left_std:.2f}) | '
            f'Right: {right_uncertainty} '
            f'(std={self.right_std:.2f})'
        )

        # ==================================================
        # CREATE COMMAND
        # ==================================================

        cmd = TwistStamped()

        # ==================================================
        # OBSTACLE AVOIDANCE FIRST
        # ==================================================

        obstacle_detected = self.obstacle_avoidance(cmd)

        if obstacle_detected:

            self.cmd_publisher.publish(cmd)

            return

        # ==================================================
        # TURN TOWARD GOAL
        # ==================================================

        if abs(heading_error) > math.radians(12):

            cmd.twist.linear.x = 0.0

            angular_speed = 1.5 * heading_error

            angular_speed = max(
                -0.8,
                min(0.8, angular_speed)
            )

            cmd.twist.angular.z = angular_speed

            self.get_logger().info(
                f'Turning | '
                f'Heading Error={heading_error_deg:.1f} deg'
            )

        # ==================================================
        # MOVE TOWARD GOAL
        # ==================================================

        else:

            # --------------------------------------------------
            # Base speed
            # --------------------------------------------------

            if distance > 1.0:

                linear_speed = 0.20

            elif distance > 0.50:

                linear_speed = 0.12

            else:

                linear_speed = 0.07

            # --------------------------------------------------
            # HIGH FRONT UNCERTAINTY
            # --------------------------------------------------

            if front_uncertainty == 'HIGH':

                linear_speed *= 0.50

                self.get_logger().warn(
                    'HIGH front uncertainty -> '
                    'CAUTIOUS MOVEMENT'
                )

            # --------------------------------------------------
            # MEDIUM FRONT UNCERTAINTY
            # --------------------------------------------------

            elif front_uncertainty == 'MEDIUM':

                linear_speed *= 0.75

                self.get_logger().warn(
                    'MEDIUM front uncertainty -> '
                    'REDUCED SPEED'
                )

            cmd.twist.linear.x = linear_speed
            cmd.twist.angular.z = 0.0

            self.get_logger().info(
                f'Moving toward goal | '
                f'Speed={linear_speed:.2f} m/s'
            )

        # ==================================================
        # PUBLISH
        # ==================================================

        self.cmd_publisher.publish(cmd)

    # ======================================================
    # SHUTDOWN
    # ======================================================

    def destroy_node(self):

        self.stop_robot()

        super().destroy_node()


# ==========================================================
# MAIN
# ==========================================================

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


# ==========================================================
# PROGRAM ENTRY
# ==========================================================

if __name__ == '__main__':

    main()
