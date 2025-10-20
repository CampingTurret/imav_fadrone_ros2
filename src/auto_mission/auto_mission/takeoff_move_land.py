#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
import time

class TakeoffMoveLand(Node):
    def __init__(self):
        super().__init__('takeoff_move_land')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Create publishers
        self.offboard_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_altitude = -5.0  # meters
        self.move_distance = 5.0      # meters
 
        # Create timer
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.started = False
        self.stage = 0
        self.start_time = time.time()

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status
    
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info('Arming command sent')

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info('Disarming command sent')

    def set_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)  # custom_mode=6 → OFFBOARD
        self.get_logger().info('Set OFFBOARD mode command sent')

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info('Land command sent')

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

    def publish_position_setpoint(self, x: float, y: float, z: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control heartbeat signal."""
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_publisher.publish(msg)

    # ---- Main loop ----
    def timer_callback(self) -> None:
        self.publish_offboard_control_heartbeat_signal()

        if not self.started and self.offboard_setpoint_counter == 10:
            self.set_offboard_mode()
            self.arm()
            self.started = True

        # Stage 0: Takeoff to z = −5.0 m
        if self.started and self.vehicle_local_position.z > self.takeoff_altitude and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_altitude)

        # Stage 1: Move forward 5 m in x
        elif self.vehicle_local_position.z <= self.takeoff_altitude and self.vehicle_local_position.x < self.move_distance :
            self.publish_position_setpoint(self.move_distance, 0.0, self.takeoff_altitude)


        # Stage 2: Land (descend) and disarm
        elif self.vehicle_local_position.z <= self.takeoff_altitude and self.vehicle_local_position.x >= self.move_distance:
            self.land()
            self.disarm()
            self.get_logger().info('Mission complete')
            exit(0)

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = TakeoffMoveLand()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()