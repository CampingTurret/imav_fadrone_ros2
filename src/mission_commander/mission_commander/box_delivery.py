#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np
from std_msgs.msg import Bool
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleAttitude
import time

class BoxDelivery(Node):
    def __init__(self):
        super().__init__('box_delivery')

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
        self.vehicle_yaw_subscriber = self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.vehicle_attitude_callback, qos_profile)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.vehicle_attitude = VehicleAttitude()
        self.takeoff_altitude = -1.5  # meters
        self.started = False
        self.stage = 0

        self.retract_servo()
        self.pos_hold_counter = 0

        # Generate waypoints (x, y, yaw)
        self.waypoints = [
            [1.0, 1.0, 0.0],
            [1.0, 1.0, np.pi],
            [0.0, 1.0, np.pi],
            [-1.0, 1.0, np.pi],
            [-1.0, 1.0, -np.pi/2],
            [-1.0, 0.0, -np.pi/2],
            [-1.0, -1.0, -np.pi/2],
            [-1.0, -1.0, 0.0],
            [0.0, -1.0, 0.0],
            [1.0, -1.0, 0.0],
            [1.0, -1.0, np.pi/2],
            [1.0, 0.0, np.pi/2]
        ]
        self.waypoint_index = 0
 
        # Create timer
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz   
        self.start_time = time.time()

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def vehicle_attitude_callback(self, vehicle_attitude):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_attitude = vehicle_attitude
    
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info('Arming command sent')

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info('Disarming command sent')

    def set_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.get_logger().info('Set OFFBOARD mode command sent')

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info('Land command sent')

    def retract_servo(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_ACTUATOR, -1.0)
        self.get_logger().info("Retract servo")

    def extend_servo(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_ACTUATOR, 1.0)
        self.get_logger().info("Extend servo")

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

    # Publish trajectory setpoint
    def publish_position_setpoint(self, x: float, y: float, z: float, yaw: float):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {[x, y, z]}, yaw setpoint {[yaw]}")

    # Publish offboard control signal
    def publish_offboard_control_heartbeat_signal(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_publisher.publish(msg)

    # Check if target reached
    def target_reached(self, vehicle_pos_xy, target_pos_xy, tol=0.1):
        if np.linalg.norm(vehicle_pos_xy - target_pos_xy) <= tol:
            return True
        else:
            return False
        
    def altitude_reached(self, vehicle_alt, target_alt, tol=0.1):
        if abs(vehicle_alt - target_alt) <= tol:
            return True
        else:
            return False
        
    def yaw_reached(self, vehicle_q, target_yaw, tol=0.08):
        w, x, y, z = vehicle_q
        vehicle_yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y**2 + z**2))

        if abs(vehicle_yaw - target_yaw) <= tol:
            return True
        else:
            return False


    # Main loop: State Machine
    def timer_callback(self) -> None:
        self.publish_offboard_control_heartbeat_signal()
        
        # Abort mission, manual take over
        # if self.started and not self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
        #     self.get_logger().info("Mission aborted")
        #     rclpy.shutdown()
        #     return
        
        # Stage 0: Arm and ready
        if not self.started and self.stage == 0 and self.offboard_setpoint_counter >= 10 :
            self.set_offboard_mode()
            self.get_logger().info("Arming drone")
            self.arm()
            self.started = True
            self.stage = 1
            self.retract_servo()

        # elif not self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
        #     self.get_logger().info("Waiting for OFFBOARD mode")

        vehicle_pos = np.array([self.vehicle_local_position.x , self.vehicle_local_position.y])
        vehicle_q = self.vehicle_attitude.q

        # Stage 1: Takeoff to z = −1.5 m
        if self.started and self.stage == 1:
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_altitude, 0.0)
            self.retract_servo()
            if self.altitude_reached(self.vehicle_local_position.z, self.takeoff_altitude) and self.target_reached(vehicle_pos, np.array([0.0, 0.0])):
                self.stage = 2

        # Stage 2: Follow trajectory to target
        elif self.started and self.stage == 2:
            target_pos = self.waypoints[self.waypoint_index]
            self.publish_position_setpoint(target_pos[0], target_pos[1], self.takeoff_altitude, target_pos[2])
            self.retract_servo()

            if self.altitude_reached(self.vehicle_local_position.z, self.takeoff_altitude) and self.target_reached(vehicle_pos, target_pos[0:2], tol=0.15) and self.yaw_reached(vehicle_q, target_pos[2]):
                self.waypoint_index += 1

                if self.waypoint_index >= len(self.waypoints):
                    self.stage = 3
        
        # Stage 3: Drop box
        elif self.started and self.stage == 3:
            target_pos = self.waypoints[-1]
            self.publish_position_setpoint(target_pos[0], target_pos[1], self.takeoff_altitude, target_pos[2])
            self.extend_servo()
            if self.altitude_reached(self.vehicle_local_position.z, self.takeoff_altitude) and self.target_reached(vehicle_pos, target_pos[0:2]) and self.pos_hold_counter >= 30:
                self.retract_servo()
                self.stage = 4
            else:
                self.pos_hold_counter += 1

        # Stage 4: Return home
        elif self.started and self.stage == 4:
            target_pos = np.array([0.0, 0.0, 0.0])
            self.publish_position_setpoint(target_pos[0], target_pos[1], self.takeoff_altitude, target_pos[2])
            self.retract_servo()
            if self.altitude_reached(self.vehicle_local_position.z, self.takeoff_altitude) and self.target_reached(vehicle_pos, target_pos[0:2]) and self.yaw_reached(vehicle_q, target_pos[2]):
                self.stage = 5

        # Stage 5: Land (descend) and disarm
        elif self.stage == 5:
            self.land()
            self.disarm()
            self.get_logger().info('Mission complete')
            rclpy.shutdown()
            return

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = BoxDelivery()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
