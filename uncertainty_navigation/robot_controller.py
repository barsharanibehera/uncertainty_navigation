import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32MultiArray


class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')

        # -------------------------------------------------
        # LiDAR subscriber
        # -------------------------------------------------
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # -------------------------------------------------
        # Uncertainty subscriber
        # -------------------------------------------------
        self.uncertainty_subscriber = self.create_subscription(
            Float32MultiArray,
            '/navigation_uncertainty',
            self.uncertainty_callback,
            10
        )

        # -------------------------------------------------
        # Velocity publisher
        # -------------------------------------------------
        self.cmd_publisher = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )

        # -------------------------------------------------
        # Store latest uncertainty
        # -------------------------------------------------
        self.front_std = 0.0
        self.left_std = 0.0
        self.right_std = 0.0

        self.uncertainty_received = False

        # Remember current turning direction
        self.turn_direction = 0

        self.get_logger().info(
            'Adaptive Uncertainty-Aware Robot Controller Started!'
        )

    # =====================================================
    # UNCERTAINTY CALLBACK
    # =====================================================

    def uncertainty_callback(self, msg):

        if len(msg.data) < 6:
            self.get_logger().warn(
                'Invalid uncertainty message received'
            )
            return

        # Message format:
        #
        # [front_avg, front_std,
        #  left_avg,  left_std,
        #  right_avg, right_std]

        self.front_std = msg.data[1]
        self.left_std = msg.data[3]
        self.right_std = msg.data[5]

        self.uncertainty_received = True

    # =====================================================
    # UNCERTAINTY CLASSIFICATION
    # =====================================================

    def classify_uncertainty(self, std_dev):

        if std_dev < 0.10:
            return 'LOW'

        elif std_dev < 0.30:
            return 'MEDIUM'

        else:
            return 'HIGH'

    # =====================================================
    # GET MINIMUM DISTANCE
    # =====================================================

    def get_region_min(self, ranges):

        valid_values = []

        for distance in ranges:

            if not math.isfinite(distance):
                continue

            if distance < 0.12:
                continue

            if distance > 3.5:
                continue

            valid_values.append(distance)

        if valid_values:
            return min(valid_values)

        return 3.5

    # =====================================================
    # LIDAR CALLBACK
    # =====================================================

    def scan_callback(self, msg):

        front_ranges = []
        left_ranges = []
        right_ranges = []

        # -------------------------------------------------
        # Divide LiDAR into regions
        # -------------------------------------------------

        for i, distance in enumerate(msg.ranges):

            if not math.isfinite(distance):
                continue

            if distance < msg.range_min:
                continue

            if distance > msg.range_max:
                continue

            angle = (
                msg.angle_min
                + i * msg.angle_increment
            )

            # Normalize angle
            while angle > math.pi:
                angle -= 2.0 * math.pi

            while angle < -math.pi:
                angle += 2.0 * math.pi

            # FRONT
            if -0.52 <= angle <= 0.52:

                front_ranges.append(distance)

            # LEFT
            elif 0.52 < angle <= 1.57:

                left_ranges.append(distance)

            # RIGHT
            elif -1.57 <= angle < -0.52:

                right_ranges.append(distance)

        # -------------------------------------------------
        # Minimum obstacle distances
        # -------------------------------------------------

        front = self.get_region_min(front_ranges)
        left = self.get_region_min(left_ranges)
        right = self.get_region_min(right_ranges)

        # -------------------------------------------------
        # Uncertainty labels
        # -------------------------------------------------

        front_uncertainty = self.classify_uncertainty(
            self.front_std
        )

        left_uncertainty = self.classify_uncertainty(
            self.left_std
        )

        right_uncertainty = self.classify_uncertainty(
            self.right_std
        )

        # -------------------------------------------------
        # Display sensor information
        # -------------------------------------------------

        self.get_logger().info(
            f'Front: {front:.2f} m | '
            f'Left: {left:.2f} m | '
            f'Right: {right:.2f} m'
        )

        self.get_logger().info(
            f'Uncertainty -> '
            f'Front: {front_uncertainty} | '
            f'Left: {left_uncertainty} | '
            f'Right: {right_uncertainty}'
        )

        # -------------------------------------------------
        # Create velocity command
        # -------------------------------------------------

        cmd = TwistStamped()

        # =================================================
        # CASE 1: ROBOT IS SURROUNDED
        # =================================================

        if (
            front < 0.45
            and left < 0.45
            and right < 0.45
        ):

            self.get_logger().warn(
                'Robot surrounded -> MOVING BACKWARD'
            )

            cmd.twist.linear.x = -0.06
            cmd.twist.angular.z = 0.0

        # =================================================
        # CASE 2: OBSTACLE DIRECTLY AHEAD
        # =================================================

        elif front < 0.60:

            # Turn toward side with more space

            if left > right:

                self.turn_direction = 1

                self.get_logger().warn(
                    'Obstacle ahead -> TURNING LEFT'
                )

                cmd.twist.linear.x = 0.02
                cmd.twist.angular.z = 0.7

            else:

                self.turn_direction = -1

                self.get_logger().warn(
                    'Obstacle ahead -> TURNING RIGHT'
                )

                cmd.twist.linear.x = 0.02
                cmd.twist.angular.z = -0.7

        # =================================================
        # CASE 3: HIGH FRONT UNCERTAINTY
        # =================================================

        elif self.front_std >= 0.30:

            self.get_logger().warn(
                'HIGH front uncertainty -> '
                'VERY CAUTIOUS MOVEMENT'
            )

            cmd.twist.linear.x = 0.06

            # Keep robot approximately straight
            cmd.twist.angular.z = 0.0

        # =================================================
        # CASE 4: MEDIUM FRONT UNCERTAINTY
        # =================================================

        elif self.front_std >= 0.10:

            self.get_logger().info(
                'MEDIUM front uncertainty -> '
                'CAUTIOUS MOVEMENT'
            )

            cmd.twist.linear.x = 0.10
            cmd.twist.angular.z = 0.0

        # =================================================
        # CASE 5: LOW UNCERTAINTY
        # =================================================

        else:

            self.get_logger().info(
                'LOW uncertainty -> NORMAL MOVEMENT'
            )

            cmd.twist.linear.x = 0.15
            cmd.twist.angular.z = 0.0

        # -------------------------------------------------
        # Publish command
        # -------------------------------------------------

        self.cmd_publisher.publish(cmd)

    # =====================================================
    # MAIN
    # =====================================================


def main(args=None):

    rclpy.init(args=args)

    node = RobotController()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        # Safely stop robot

        if rclpy.ok():

            stop_cmd = TwistStamped()

            stop_cmd.twist.linear.x = 0.0
            stop_cmd.twist.angular.z = 0.0

            try:
                node.cmd_publisher.publish(stop_cmd)
            except Exception:
                pass

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
