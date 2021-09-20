# ECE 5730 - Robotics
# navigation.py - Nav control node to send velocity to motor control node.

# Import libraries.
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from geometry_msgs.msg import Twist

# Global variables:
# Keyboard movement variables.
keyboard_vx = 0.0
keyboard_vtheta = 0.0

class MovementPublisher(Node):
    """
    Movement publisher that sends a twist message to an Arduino node a velocity command.
    """

    def __init__(self):
        # Create node
        super().__init__('movement_publisher')
        # Create publisher node
        self.publisher_ = self.create_publisher(Twist, 'motor_control', 10)
        # Send messages every 10 Hz
        timer_period = 0.1  # seconds 
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        global keyboard_vx
        global keyboard_vtheta
        # Publish a twist message.
        msg = Twist()
        msg.linear.x = float(keyboard_vx)
        msg.angular.z = float(keyboard_vtheta)
        self.publisher_.publish(msg)
        self.get_logger().info('(Vx: "%2.2f", Vtheta: "%2.2f")' % (msg.linear.x, msg.angular.z))

class MovementSubscriber(Node):
    """
    Movement subscriber that recieves from an Arduino a twist message for odometry and error detection.
    """

    def __init__(self):
        # Create node.
        super().__init__('movement_subscriber')
        # Create subscriber node to a 'motor_feedback' topic and expects a Twist message.
        self.subscription = self.create_subscription(
            Twist,
            'motor_feedback',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # Print useful information to terminal.
        self.get_logger().info('Arduino sent: Linear: "%.2f", Angular: "%.2f"' % (msg.linear.x, msg.angular.z))

class KeyboardSubscriber(Node):
    """
    Keyboard subscriber that takes information from a 'teleop_twist_keyboard' node as a twist message.
    """

    def __init__(self):
        # Create node.
        super().__init__('keyboard_subscriber')
        # Create subscriber node to a 'cmd_vel' topic and expects a Twist message.
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',  # Topic of teleop_twist_keyboard node.
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        global keyboard_vx
        global keyboard_vtheta
        keyboard_vx = msg.linear.x
        keyboard_vtheta = msg.angular.z

def main(args=None):
    # Initialize rclpy
    rclpy.init(args=args)
    try:
        # Create Publishers and Subsribers in Navigation module.
        movement_pub = MovementPublisher()
        movement_sub = MovementSubscriber()
        keyboard_sub = KeyboardSubscriber()
        # Run all callbacks in the main thread
        executor = SingleThreadedExecutor()
        # Add nodes to executor.
        executor.add_node(movement_pub)
        executor.add_node(movement_sub)
        executor.add_node(keyboard_sub)
        try:
            # Run all nodes on main thread.
            executor.spin()
        finally:
            # Clean nodes when program is finished.
            executor.shutdown()
            movement_pub.destroy_node()
            movement_sub.destroy_node()
            keyboard_sub.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
