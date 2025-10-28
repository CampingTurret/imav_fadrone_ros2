#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from math import nan
import numpy as np
from std_msgs.msg import Bool

class MarkerSub(Node):
    def __init__(self):
        super().__init__('marker_sub')

        self.marker_contact_subscriber = self.create_subscription(Bool, '/marker_topic', self.marker_detect_callback, 10)
        self.marker_contact_subscriber

    def marker_detect_callback(self, msg):
        if msg.data:
            self.get_logger().info("True")
        else:
            self.get_logger().info("False")


def main(args=None):
    rclpy.init(args=args)
    node = MarkerSub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
