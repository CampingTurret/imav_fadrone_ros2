#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class SimServo(Node):
    def __init__(self):
        super().__init__('sim_servo')

        # Create subscribers
        self.servo_position_subscriber = self.create_subscription(Bool, '/servo_position', self.servo_position_callback, 10)
        self.servo_position_subscriber

    def servo_position_callback(self, msg):
        if msg.data:
            self.get_logger().info("Extend servo")
        else:
            self.get_logger().info("Retract servo")

def main(args=None):
    rclpy.init(args=args)
    node = SimServo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
