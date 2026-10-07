import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray


class GoalNavigator(Node):

    def __init__(self):
        super().__init__('goal_navigator')

        # ==================================================
        # GOAL POSITION
        # ==================================================

        self.goal_x = 5.0
        self.goal_y = 4.0

        # ==================================================
        # CURRENT ROBOT POSITION
        # ==================================================

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.odom_received = False

        # ==================================================
        # UNCERTAINTY DATA
        # ==================================================

        self.uncertainty_received = False

        self.front_avg = 0.0
        self.front_std = 0.0

        self.left_avg = 0.0
        self.left_std = 0.0

        self.right_avg = 0.0
        self.right_std = 0.0

        # ==================================================
        # ODOMETRY SUBSCRIBER
        # ==================================================

        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # ==================================================
        # UNCERTAINTY SUBSCRIBER
        # ==================================================

        self.uncertainty_subscriber = self.create_subscription(
            Float32MultiArray,
            '/navigation_uncertainty',
            self.uncertainty_callback,
            10
        )

        # ==================================================
        # VELOCITY PUBLISHER
        # ==================================================

        self.cmd_publisher = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
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
            'Uncertainty-Aware Goal Navigator Started!'
        )

        self.get_logger().info(
            f'Goal: X={self.goal_x:.2f} m, '
            f'Y={self.goal_y:.2f} m'
        )

    # ======================================================
    # NORMALIZE ANGLE
    # ======================================================

    def normalize_angle(self, angle):

        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    # ======================================================
    # QUATERNION TO YAW
    # ======================================================

    def quaternion_to_yaw(self, q):

        siny_cosp = 2.0 * (
            q.w * q.z +
            q.x * q.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            q.y * q.y +
            q.z * q.z
        )

        return math.atan2(
            siny_cosp,
            cosy_cosp
        )

    # ======================================================
    # ODOMETRY CALLBACK
    # ======================================================

    def odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        self.yaw = self.quaternion_to_yaw(
            msg.pose.pose.orientation
        )

        self.odom_received = True

    # ======================================================
    # UNCERTAINTY CALLBACK
    # ======================================================

    def uncertainty_callback(self, msg):

        if len(msg.data) < 6:
            return

        self.front_avg = msg.data[0]
        self.front_std = msg.data[1]

        self.left_avg = msg.data[2]
        self.left_std = msg.data[3]

        self.right_avg = msg.data[4]
        self.right_std = msg.data[5]

        self.uncertainty_received = True

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
    # STOP ROBOT
    # ======================================================

    def stop_robot(self):

        cmd = TwistStamped()

        cmd.twist.linear.x = 0.0
        cmd.twist.angular.z = 0.0

        self.cmd_publisher.publish(cmd)

    # ======================================================
    # MAIN CONTROL LOOP
    # ======================================================

    def control_loop(self):

        # --------------------------------------------------
        # Wait for odometry
        # --------------------------------------------------

        if not self.odom_received:

            self.get_logger().info(
                'Waiting for odometry...'
            )

            return

        # --------------------------------------------------
        # Wait for uncertainty data
        # --------------------------------------------------

        if not self.uncertainty_received:

            self.get_logger().info(
                'Waiting for uncertainty data...'
            )

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
        # CHECK GOAL
        # ==================================================

        if distance < 0.20:

            self.stop_robot()

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
        # DISPLAY UNCERTAINTY
        # ==================================================

        self.get_logger().info(
            f'Position=({self.x:.2f}, {self.y:.2f}) | '
            f'Distance={distance:.2f} m'
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
            # Reduce speed when front uncertainty is HIGH
            # --------------------------------------------------

            if front_uncertainty == 'HIGH':

                linear_speed *= 0.5

                self.get_logger().warn(
                    'HIGH front uncertainty -> '
                    'CAUTIOUS MOVEMENT'
                )

            # --------------------------------------------------
            # Reduce speed when front uncertainty is MEDIUM
            # --------------------------------------------------

            elif front_uncertainty == 'MEDIUM':

                linear_speed *= 0.75

                self.get_logger().warn(
                    'MEDIUM front uncertainty -> '
                    'REDUCED SPEED'
                )

            # --------------------------------------------------
            # Move forward
            # --------------------------------------------------

            cmd.twist.linear.x = linear_speed
            cmd.twist.angular.z = 0.0

            self.get_logger().info(
                f'Moving toward goal | '
                f'Speed={linear_speed:.2f} m/s'
            )

        # ==================================================
        # PUBLISH COMMAND
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
# PROGRAM ENTRY POINT
# ==========================================================

if __name__ == '__main__':

    main()
