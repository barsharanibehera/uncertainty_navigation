import rclpy
from rclpy.node import Node


class HelloRobot(Node):

    def __init__(self):
        super().__init__('hello_robot')

        self.get_logger().info("Hello Robot Node Started!")

        # Create a timer that calls timer_callback every 1 second
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("I am alive...")


def main(args=None):
    rclpy.init(args=args)

    node = HelloRobot()

    # Keep the node running
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
