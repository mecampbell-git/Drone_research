#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode, 
    TrajectorySetpoint, 
    VehicleCommand, 
    VehicleLocalPosition, 
    VehicleStatus,
    VehicleGlobalPosition
)
from ament_index_python.packages import get_package_share_directory
import math
import os
from enum import Enum, auto

import csv

class MissionState(Enum):
    WARMUP = auto()
    TRAVEL = auto()
    WAIT = auto()
    RTL = auto()
    LAND = auto()

class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_global_position_subscriber = self.create_subscription(
            VehicleGlobalPosition, '/fmu/out/vehicle_global_position', self.vehicle_global_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1', self.vehicle_status_callback, qos_profile)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_global_position = VehicleGlobalPosition()
        self.vehicle_status = VehicleStatus()

        # self.mission = []
        # self.mission.append([100, 100, -30, 5]) #NED + delay at endpoint
        # self.mission.append([50, 150, -40, 0]) 
        # self.mission.append([150, 100, -60, 10]) 
        # self.mission.append([0, 150, -20, 2]) 

        # A bare name is looked up in the package's installed missions/ folder;
        # a path (absolute, or relative to the current directory) is used as is.
        self.declare_parameter('mission_file', 'mission_box.csv')
        mission_file = self.get_parameter('mission_file').value
        if os.path.dirname(mission_file) == '':
            mission_file = os.path.join(
                get_package_share_directory('px4_offboard'), 'missions', mission_file)
        with open(mission_file, mode="r", newline="", encoding="utf-8") as f:
            reader = csv.reader(f)
            self.mission = [[float(cell) for cell in row] for row in reader if row]
        self.get_logger().info(f'Loaded {len(self.mission)} waypoints from {mission_file}')

        self.mcnt = len(self.mission)
        self.lcnt = 0

        self.state = MissionState.WARMUP

        self.posz = 0.0 #NED
        self.posx = 0
        self.posy = 0
        self.yaw = 0.0
        self.delay = 0
        self.dcnt = 0

        self.position = [0.0, 0.0, 0.0]

        self.loop_hz = 20
        self.land_cnt = 0       # timer cycles spent waiting for PX4 to enter land mode
        self.done = False       # set when PX4 has taken over the landing; main() then exits
        # Create a timer to publish control commands
        self.timer = self.create_timer(1.0 / self.loop_hz, self.timer_callback)
        self.timergps = self.create_timer(1.0, self.timergps_callback) 

    def vehicle_global_position_callback(self, vehicle_global_position):
        """Callback function for vehicle_global_position topic subscriber."""
        self.vehicle_global_position = vehicle_global_position

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def rtl(self):
        """Switch to RTL mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
        self.get_logger().info("Switching to RTL mode")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float, yaw: float = 0.0) -> None:
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw

        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

        if (msg.position != self.position).any():
            self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")
            self.position = msg.position

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timergps_callback(self) -> None:
        print(f"Lat = {self.vehicle_global_position.lat:.6f}, Long = {self.vehicle_global_position.lon:.6f}, Alt = {self.vehicle_global_position.alt:.6f}")


    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        match self.state:
            case MissionState.WARMUP:
                self.publish_position_setpoint(self.posx, self.posy, self.posz, 0.0)
                self.offboard_setpoint_counter += 1
                if self.offboard_setpoint_counter == 10:
                    self.engage_offboard_mode()
                    self.arm()
                    self.posx, self.posy, self.posz, self.delay = self.mission[self.lcnt]
                    self.lcnt += 1
                    self.get_logger().info(f"Next waypoint: {self.posx}, {self.posy}, {self.posz}, delay: {self.delay}")
                    self.state = MissionState.TRAVEL
            case MissionState.TRAVEL:
                distance = ((self.vehicle_local_position.x - self.posx) ** 2 + (self.vehicle_local_position.y - self.posy) ** 2 + (self.vehicle_local_position.z - self.posz) ** 2) ** 0.5

                dxy = ((self.vehicle_local_position.x - self.posx) ** 2 + (self.vehicle_local_position.y - self.posy) ** 2 ) ** 0.5
                print(f"dxy = {dxy:.2f}")

                if distance > 0.2 and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                    if dxy > 1.0:
                        self.yaw = math.atan2(self.posy - self.vehicle_local_position.y, self.posx - self.vehicle_local_position.x)    # radians, 0 = north, +pi/2 = east

                    self.publish_position_setpoint(self.posx, self.posy, self.posz, self.yaw)
                    print(f"Pos = [{self.vehicle_local_position.x:.2f}, {self.vehicle_local_position.y:.2f}, {self.vehicle_local_position.z:.2f}]  d = {distance:.2f}  yaw = {self.yaw:.2f} heading = {self.vehicle_local_position.heading:.2f}")

                elif distance <= 0.2:
                    self.get_logger().info(f"Reached waypoint: {self.posx}, {self.posy}, {self.posz}, delay: {self.delay}")
                    if self.delay > 0:
                        self.dcnt = 0
                        self.state = MissionState.WAIT
                    elif self.lcnt < self.mcnt:
                        self.posx, self.posy, self.posz, self.delay = self.mission[self.lcnt]
                        self.lcnt += 1
                        self.get_logger().info(f"Next waypoint: {self.posx}, {self.posy}, {self.posz}, delay: {self.delay}")
                    else:
                        if self.posx == 0 and self.posy == 0:
                            self.state = MissionState.LAND
                        else:
                            self.state = MissionState.RTL
            case MissionState.WAIT:
                self.dcnt += 1
                self.yaw = self.yaw + .05  # Increment yaw for a slow rotation during wait
                self.publish_position_setpoint(self.posx, self.posy, self.posz, self.yaw)
                if self.dcnt >= self.delay * self.loop_hz:
                    if self.lcnt < self.mcnt:
                        self.posx, self.posy, self.posz, self.delay = self.mission[self.lcnt]
                        self.lcnt += 1
                        self.get_logger().info(f"Next waypoint: {self.posx}, {self.posy}, {self.posz}, delay: {self.delay}")
                        self.state = MissionState.TRAVEL
                    else:
                        if self.posx == 0 and self.posy == 0:
                            self.state = MissionState.LAND
                        else:
                            self.state = MissionState.RTL
            case MissionState.RTL:
                # self.get_logger().info("Mission complete. Returning to launch.")
                # Commands are BEST_EFFORT, so resend once a second until PX4 confirms land mode.
                # PX4 then lands and disarms on its own, and the node can stop.
                if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_RTL:
                    self.get_logger().info("Mission complete. PX4 is returning to launch.")
                    self.done = True
                elif self.land_cnt >= 5 * self.loop_hz:
                    self.get_logger().error("PX4 did not enter return to launch mode within 5 s. Stopping anyway.")
                    self.done = True
                else:
                    if self.land_cnt % self.loop_hz == 0:
                        self.rtl()
                    self.land_cnt += 1
            case MissionState.LAND:
                # self.get_logger().info("Mission complete. Returning to launch.")
                # self.rtl()
                # Commands are BEST_EFFORT, so resend once a second until PX4 confirms land mode.
                # PX4 then lands and disarms on its own, and the node can stop.
                if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
                    self.get_logger().info("Mission complete. PX4 is landing.")
                    self.done = True
                elif self.land_cnt >= 5 * self.loop_hz:
                    self.get_logger().error("PX4 did not enter land mode within 5 s. Stopping anyway.")
                    self.done = True
                else:
                    if self.land_cnt % self.loop_hz == 0:
                        self.land()
                    self.land_cnt += 1




def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    try:
        while rclpy.ok() and not offboard_control.done:
            rclpy.spin_once(offboard_control, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        offboard_control.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
