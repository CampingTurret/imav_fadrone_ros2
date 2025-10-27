#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from gpiozero import Button

class MarkerDetect(Node):
    def __init__(self):
        super().__init__('marker_detect')

        # Create publisher
        self.marker_publisher = self.create_publisher(Bool, '/marker_detect', 10)

        # Initialize variables
        self.marker = Button(17)

        # Create timer
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz   

    def publish_marker_data(self, marker):
        marker_msg = Bool()
        marker_msg.data = marker.is_pressed
        self.marker_publisher.publish(marker_msg)
        self.get_logger().info(f"Contacted? {marker.is_pressed}")

    def timer_callback(self):
        self.publish_marker_data(self.marker)

def main(args=None):
    rclpy.init(args=args)
    node = MarkerDetect()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

