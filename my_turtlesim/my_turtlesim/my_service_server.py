import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist


class MyServiceServer(Node):

    def __init__(self):
        super().__init__('my_service_server')
        self.my_service = self.create_service(Trigger, 'draw_circle', self.draw_circle_callback)
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

    def draw_circle_callback(self, request, response):
        request # request must be specified even if it is not used
        self.get_logger().info('Received request to draw a circle!')
        response.success = True
        response.message = "Starting to draw a circle!"
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publish_velocity_callback)
        self.i = 0

        return response

    def publish_velocity_callback(self):
        my_velocity = Twist()
        my_velocity.linear.x = 0.5
        my_velocity.angular.z = 0.5
        self.publisher_.publish(my_velocity)
        self.get_logger().info(f"Publishing velocity: \n\t linear.x: {my_velocity.linear.x}; \n\t linear.z: {my_velocity.linear.x}")
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    my_service_server = MyServiceServer()

    rclpy.spin(my_service_server)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
