#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Bool
from px4_msgs.msg import OffboardControlMode, VehicleStatus, ActuatorMotors, VehicleCommand
import time

class ServoTest(Node):
    def __init__(self):
        super().__init__('servo_test')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Create publishers
        self.offboard_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.servo_publisher = self.create_publisher(ActuatorMotors, '/fmu/in/actuator_motors', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_status_subscriber = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        # Initialize variables
        self.vehicle_status = VehicleStatus()
        self.servo_min = -1.0
        self.servo_max = 1.0
        self._current_pos = self.servo_min
        self.armed = False

        # Timers: heartbeat > 2 Hz; servo toggle every 5 s
        self.create_timer(0.1, self.publish_offboard_control_heartbeat_signal)  # 10 Hz
        self.create_timer(1.0, self.servo_timer_cb)

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info('Arming command sent')

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info('Disarming command sent')

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_publisher.publish(msg)

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control heartbeat signal."""
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.direct_actuator = True
        self.offboard_publisher.publish(msg)


    # Main loop: State Machine
    def servo_timer_cb(self) -> None:
        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            if not self.armed:
                self.arm()
                self.armed = True

            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_ACTUATOR, self._current_pos)
            self.get_logger().info(f"Motor signal {self._current_pos}")
            self._current_pos = self.servo_max if self._current_pos <= self.servo_min + 1e-6 else self.servo_min
        
        else:
            self.get_logger().info("Waiting for OFFBOARD mode")
            self.disarm()
            self.armed = False


def main(args=None):
    rclpy.init(args=args)
    node = ServoTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

