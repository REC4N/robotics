# ECE 5730 - Robotics
# navigation.py - Nav control node to send velocity to motor control node.

# Import libraries.
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from geometry_msgs.msg import Twist


class MovementPublisher(Node):

    def __init__(self):
        super().__init__('movement_publisher')
        self.publisher_ = self.create_publisher(Twist, 'motor_control', 10)
        timer_period = 1.0  # 10 Hz. (change it to 0.1 for 10 Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = float(self.i)
        msg.angular.z = float(20)
        self.publisher_.publish(msg)
        self.get_logger().info('MotorControl says: Linear: "%.2f", Angular: "%.2f"' % (msg.linear.x, msg.angular.z))
        self.i += 1

class MovementSubscriber(Node):
    def __init__(self):
        super().__init__('movement_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            'motor_feedback',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Arduino sent: Linear: "%.2f", Angular: "%.2f"' % (msg.linear.x, msg.angular.z))

def main(args=None):
    rclpy.init(args=args)
    
    try:
        movement_pub = MovementPublisher()
        movement_sub = MovementSubscriber()
        # Run all callbacks in the main thread
        executor = SingleThreadedExecutor()
        # Add nodes to executor.
        executor.add_node(movement_pub)
        executor.add_node(movement_sub)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            movement_pub.destroy_node()
            movement_sub.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
