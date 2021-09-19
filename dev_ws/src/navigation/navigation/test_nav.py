# ECE 5730 - Robotics
# navigation.py - Nav control node to send velocity to motor control node.

# Import libraries.
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from geometry_msgs.msg import Twist


class TestPublisher(Node):

    def __init__(self):
        super().__init__('test_publisher')
        self.publisher_ = self.create_publisher(Twist, 'motor_feedback', 10)
        timer_period = 5.0  # 10 Hz. (change it to 0.1 for 10 Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = float(self.i + 100)
        msg.angular.z = float(500)
        self.publisher_.publish(msg)
        self.get_logger().info('Arduino says: Linear: "%.2f", Angular: "%.2f"' % (msg.linear.x, msg.angular.z))
        self.i += 1

class TestSubscriber(Node):
    def __init__(self):
        super().__init__('test_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            'motor_control',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard from MotorControl: (Linear: "%.2f", Angular: "%.2f")' % (msg.linear.x, msg.linear.z))


def main(args=None):
    rclpy.init(args=args)
    
    try:
        test_pub = TestPublisher()
        test_sub = TestSubscriber()
        # Run all callbacks in the main thread
        executor = SingleThreadedExecutor()
        # Add nodes to executor.
        executor.add_node(test_pub)
        executor.add_node(test_sub)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            test_pub.destroy_node()
            test_sub.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
