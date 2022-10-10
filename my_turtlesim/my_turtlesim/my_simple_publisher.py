import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MySimplePublisher(Node):

    def __init__(self):
        super().__init__('my_simple_publisher')
        self.get_logger().info('Creating simple publisher!')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        my_velocity = Twist()
        my_velocity.linear.x = 0.5
        my_velocity.angular.z = 0.5
        self.publisher_.publish(my_velocity)
        self.get_logger().info('Publishing: "%s"' % my_velocity)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    my_simple_publisher = MySimplePublisher()

    rclpy.spin(my_simple_publisher)

    my_simple_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()