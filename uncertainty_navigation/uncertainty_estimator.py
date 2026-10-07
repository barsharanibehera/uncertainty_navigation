import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

import math


class UncertaintyEstimator(Node):

    def __init__(self):
        super().__init__('uncertainty_estimator')

        # --------------------------------
        # LiDAR subscriber
        # --------------------------------

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # --------------------------------
        # Uncertainty publisher
        # --------------------------------

        self.uncertainty_publisher = self.create_publisher(
            Float32MultiArray,
            '/navigation_uncertainty',
            10
        )

        self.get_logger().info(
            'Uncertainty Estimator Started!'
        )

    # --------------------------------
    # Calculate average and standard deviation
    # --------------------------------

    def calculate_region_stats(self, values):

        if not values:
            return 0.0, 0.0

        average = sum(values) / len(values)

        variance = sum(
            (x - average) ** 2
            for x in values
        ) / len(values)

        standard_deviation = math.sqrt(variance)

        return average, standard_deviation

    # --------------------------------
    # Classify uncertainty
    # --------------------------------

    def classify_uncertainty(self, std_dev):

        if std_dev < 0.10:
            return 'LOW'

        elif std_dev < 0.30:
            return 'MEDIUM'

        else:
            return 'HIGH'

    # --------------------------------
    # LiDAR callback
    # --------------------------------

    def scan_callback(self, msg):

        front_ranges = []
        left_ranges = []
        right_ranges = []

        # --------------------------------
        # Divide LiDAR into regions
        # --------------------------------

        for i, distance in enumerate(msg.ranges):

            # Ignore invalid values
            if not math.isfinite(distance):
                continue

            if distance < msg.range_min:
                continue

            if distance > msg.range_max:
                continue

            # Convert index to angle
            angle = (
                msg.angle_min
                + i * msg.angle_increment
            )

            # Normalize angle to -pi ... +pi
            if angle > math.pi:
                angle -= 2.0 * math.pi

            # --------------------------------
            # FRONT
            # -30° to +30°
            # --------------------------------

            if -0.52 <= angle <= 0.52:

                front_ranges.append(distance)

            # --------------------------------
            # LEFT
            # +30° to +90°
            # --------------------------------

            elif 0.52 < angle <= 1.57:

                left_ranges.append(distance)

            # --------------------------------
            # RIGHT
            # -90° to -30°
            # --------------------------------

            elif -1.57 <= angle < -0.52:

                right_ranges.append(distance)

        # --------------------------------
        # Calculate statistics
        # --------------------------------

        front_avg, front_std = self.calculate_region_stats(
            front_ranges
        )

        left_avg, left_std = self.calculate_region_stats(
            left_ranges
        )

        right_avg, right_std = self.calculate_region_stats(
            right_ranges
        )

        # --------------------------------
        # Classify uncertainty
        # --------------------------------

        front_uncertainty = self.classify_uncertainty(
            front_std
        )

        left_uncertainty = self.classify_uncertainty(
            left_std
        )

        right_uncertainty = self.classify_uncertainty(
            right_std
        )

        # --------------------------------
        # Publish uncertainty data
        #
        # Data format:
        #
        # [front_avg, front_std,
        #  left_avg,  left_std,
        #  right_avg, right_std]
        # --------------------------------

        uncertainty_msg = Float32MultiArray()

        uncertainty_msg.data = [
            float(front_avg),
            float(front_std),

            float(left_avg),
            float(left_std),

            float(right_avg),
            float(right_std)
        ]

        self.uncertainty_publisher.publish(
            uncertainty_msg
        )

        # --------------------------------
        # Display results
        # --------------------------------

        self.get_logger().info(
            f'Front: {front_avg:.2f} m | '
            f'Uncertainty: {front_uncertainty} '
            f'(std={front_std:.2f})'
        )

        self.get_logger().info(
            f'Left: {left_avg:.2f} m | '
            f'Uncertainty: {left_uncertainty} '
            f'(std={left_std:.2f})'
        )

        self.get_logger().info(
            f'Right: {right_avg:.2f} m | '
            f'Uncertainty: {right_uncertainty} '
            f'(std={right_std:.2f})'
        )


def main(args=None):

    rclpy.init(args=args)

    node = UncertaintyEstimator()

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
