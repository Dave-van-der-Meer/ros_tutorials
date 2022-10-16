import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from turtlesim.msg import Pose


class MySimpleSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.my_subscriber = self.create_subscription(
            Pose,
            'turtle1/pose',
            self.listener_callback,
            10)
        # prevent warning that se.fmy_subscriber is not used
        self.my_subscriber

    def listener_callback(self, msg):
        self.get_logger().info(f'Turtle found at x: {msg.x}; y: {msg.y}')


def main(args=None):
    rclpy.init(args=args)

    my_simple_subscriber = MySimpleSubscriber()

    rclpy.spin(my_simple_subscriber)

     # destroy the node when it is not used anymore
    my_simple_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()