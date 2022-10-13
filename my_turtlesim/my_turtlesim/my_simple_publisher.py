import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MySimplePublisher(Node):

    def __init__(self):
        super().__init__('my_simple_publisher')
        self.get_logger().info('Creating simple publisher!')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publish_velocity_callback)
        self.i = 0

    def publish_velocity_callback(self):
        my_velocity = Twist()
        my_velocity.linear.x = 0.5
        my_velocity.angular.z = 0.5
        self.publisher_.publish(my_velocity)
        self.get_logger().info(f"Publishing velocity: \n\t linear.x: {my_velocity.linear.x}; \n\t linear.z: {my_velocity.linear.x}")
        self.i += 1


def main(args=None):
    # initiate ROS2
    rclpy.init(args=args)

    # create an instance of the node
    my_simple_publisher = MySimplePublisher()

    # keep the node alive intil pressing CTRL+C
    rclpy.spin(my_simple_publisher)
    
    # destroy the node when it is not used anymore
    my_simple_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()