import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math


class LidarSubscriber(Node):

    def __init__(self):
        super().__init__('lidar_subscriber')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.get_logger().info('LiDAR Subscriber Started!')

    def scan_callback(self, msg):

        valid_ranges = [
            distance
            for distance in msg.ranges
            if math.isfinite(distance)
            and msg.range_min <= distance <= msg.range_max
        ]

        if valid_ranges:
            nearest_obstacle = min(valid_ranges)

            self.get_logger().info(
                f'Nearest obstacle: {nearest_obstacle:.2f} meters'
            )

        else:
            self.get_logger().info(
                'No obstacle detected within LiDAR range'
            )


def main(args=None):

    rclpy.init(args=args)

    node = LidarSubscriber()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
