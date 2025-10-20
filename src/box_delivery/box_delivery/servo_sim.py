#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
import time

class ServoSim(Node):
    def __init__(self):
        super().__init__('servo_sim')

        # Create subscribers
        self.servo_position_subscriber = self.create_subscription(String, '/servo_position', self.servo_position_callback, 10)
        self.servo_position_subscriber

    def servo_position_callback(self, msg):
        self.get_logger().info(msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = ServoSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
