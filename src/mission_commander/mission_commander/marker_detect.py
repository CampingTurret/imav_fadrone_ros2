#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from gpiozero import Button

class MarkerDetect(Node):
    def __init__(self):
        super().__init__('marker_detect')

        # Create publisher
        self.marker_publisher = self.create_publisher(Bool, '/marker_topic', 10)

        # Initialize variables
        self.marker = Button(17)
        self.marker_msg = Bool(data=False)

        # Create timer
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz   


    def timer_callback(self):
        self.marker_msg.data = self.marker.is_pressed
        self.marker_msg.data = True
        self.marker_publisher.publish(self.marker_msg)
        self.get_logger().info(f'Publishing {self.marker_msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = MarkerDetect()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

