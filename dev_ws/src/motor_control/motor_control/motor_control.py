# ECE 5730 - Robotics
# motor_control.py - Motor control node to manage velocity of motor from Arduino.

# Import libraries.
import rclpy
from rclpy.node import Node
import serial

from geometry_msgs.msg import Twist

# Global variables.
g_node = None
g_ser = None

# Subscription from Navigation module on Jetson.
def sub_callback(msg):
    global g_node
    global g_ser
    g_node.get_logger().info('I heard "%f"'%msg.linear.x)
    g_ser.write("12,45\n".encode())

def main(args=None):
    global g_node
    global g_ser
    # Initialize rclpy
    rclpy.init(args=args)
    # Create node for Arduino
    g_node = rclpy.create_node('arduino_publisher')
    # Create Publisher indicating current platform velocity.
    publisher = g_node.create_publisher(Twist, 'platform_vel', 1)
    # Create Subscriber listening for desired platform velocity.
    subscriber = g_node.create_subscription(Twist, 'target_vel', sub_callback, 1)
    # Create Twist message to be published.
    msg = Twist()
    # Open Serial port on Jetson at 115000 baudrate to communicate with Arduino.
    g_ser=serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    while rclpy.ok():
        # Read line from Serial Port.
        line = g_ser.readline(10)
        # Decode line into to values that were separated by a coma.
        vals = line.decode().split(',')
        if(len(vals) != 2):
        # Make sure that we got at least two values from Serial port.
            print("Got ",len(vals)," vals from ",line)
            continue
        # Show debug info.
        print("Got ",len(vals)," vals from ",line)
        #TODO: Change next two lines to calculation of actual Vx and Vtheta!
        msg.linear.x= float(vals[0])    #TODO
        msg.angular.z= float(vals[1])   #TODO
        # Show Serial info for debug.
        g_node.get_logger().info('Serial Rx: %s'%str(line))
        # Publish Twist message to Navigation module containing current platform speed.
        publisher.publish(msg)
        # TODO: Is this spin referring to subscriber or publisher node?
        rclpy.spin_once(g_node, timeout_sec=0)
    # Clean node.
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

