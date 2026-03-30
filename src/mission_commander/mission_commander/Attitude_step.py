#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np
from std_msgs.msg import Bool
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleAttitude
from px4_msgs.msg import VehicleAttitudeSetpoint

import time

from px4_msgs.msg import (
    OffboardControlMode,
    VehicleCommand,
    VehicleAttitudeSetpoint,
    VehicleLocalPosition
)

class MinimalStepInput(Node):
    def __init__(self):
        super().__init__('minimal_step_input')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # --- Publishers ---
        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)

        self.attitude_pub = self.create_publisher(VehicleAttitudeSetpoint,'/fmu/in/vehicle_attitude_setpoint', qos_profile)

        self.command_pub = self.create_publisher(VehicleCommand,'/fmu/in/vehicle_command', qos_profile)

        # --- Subscribers ---
        self.local_pos = VehicleLocalPosition()
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.position_callback, qos_profile)

        # --- State ---
        self.stage = 0
        self.start_time = time.time()

        # Timer @ 33 Hz
        self.timer = self.create_timer(0.03, self.loop)

    def position_callback(self, msg):
        self.local_pos = msg

    # ------------------------------------------------------------
    # Helper: send attitude + thrust setpoint
    # ------------------------------------------------------------
    def send_attitude(self, roll, pitch, yaw, thrust):
        msg = VehicleAttitudeSetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        # Euler → quaternion
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        msg.q_d = [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy
        ]

        msg.thrust = thrust
        self.attitude_pub.publish(msg)

    # ------------------------------------------------------------
    # Helper: offboard heartbeat
    # ------------------------------------------------------------
    def send_heartbeat(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.attitude = True
        msg.thrust_and_torque = True
        self.offboard_pub.publish(msg)

    # ------------------------------------------------------------
    # Helper: vehicle command
    # ------------------------------------------------------------
    def command(self, cmd, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = cmd
        msg.param1 = p1
        msg.param2 = p2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    # ------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------
    def loop(self):
        self.send_heartbeat()

        # --- Stage 0: send a few setpoints before switching to offboard ---
        if self.stage == 0:
            self.send_attitude(0.0, 0.0, 0.0, 0.6)
            if time.time() - self.start_time > 1.0:
                self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 6)  # OFFBOARD
                self.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1)
                self.stage = 1
                self.start_time = time.time()

        # --- Stage 1: takeoff (simple thrust hold) ---
        elif self.stage == 1:
            self.send_attitude(0.0, 0.0, 0.0, 0.65)
            if self.local_pos.z < -1.0:  # reached -1 m altitude
                self.stage = 2
                self.start_time = time.time()

        # --- Stage 2: apply step input ---
        elif self.stage == 2:
            # Example: 10° roll step
            roll_step = np.deg2rad(10)
            self.send_attitude(roll_step, 0.0, 0.0, 0.6)

            if time.time() - self.start_time > 3.0:
                self.stage = 3

        # --- Stage 3: land ---
        elif self.stage == 3:
            self.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.stage = 4

        # --- Stage 4: done ---
        elif self.stage == 4:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = MinimalStepInput()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
