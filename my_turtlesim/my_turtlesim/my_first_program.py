import rclpy
from rclpy.node import Node


class MyFirstProgram(Node):

    def __init__(self):
        super().__init__('my_first_program')
        self.get_logger().info('Hello, World!')


def main(args=None):
    rclpy.init(args=args)

    my_first_program = MyFirstProgram()

    my_first_program.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()