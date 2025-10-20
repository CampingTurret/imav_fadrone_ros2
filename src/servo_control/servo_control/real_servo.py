#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from .driver import ServoDriver

class RealServo(Node):
    def __init__(self):
        super().__init__('real_servo')

        # Create subscribers
        self.servo_position_subscriber = self.create_subscription(Bool, '/servo_position', self.servo_position_callback, 10)
        self.servo_position_subscriber

        # Initialize servo
        self.servo = ServoDriver()

    def servo_position_callback(self, msg):
        if msg.data:
            self.servo.extend()
            self.get_logger().info("Extend servo")
        else:
            self.servo.retract()
            self.get_logger().info("Retract servo")

def main(args=None):
    rclpy.init(args=args)
    node = RealServo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
