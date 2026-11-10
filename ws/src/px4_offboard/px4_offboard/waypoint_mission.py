#!/usr/bin/env python3
"""
Waypoint mission as a PX4 ROS 2 custom mode, in Python (px4_ros2_py).

The "Waypoints" mode flies a mission CSV (rows of north,east,down,delay_s in the local NED
frame, the same files as position_mission_control.py) with PX4's Goto setpoint, so PX4's
own smoother plans the motion. Waypoints with delay 0 are flown through with a rounded
turn; the others, and the last, are stopped at. The mode executor runs the whole flight:
arm, PX4 Takeoff, Waypoints, PX4 Land, then the program exits once the drone has disarmed.

Parameters (--ros-args -p name:=value):
  mission_file       bare name: read from this package's installed missions/ folder;
                     anything containing '/' is used as a path (default mission_box.csv)
  max_speed          horizontal speed limit [m/s] (default 5.0). PX4 caps Goto at its
                     MPC_XY_CRUISE parameter (default 5), so faster needs MPC_XY_CRUISE
                     raised too; PX4 also scales acceleration by max_speed / MPC_XY_CRUISE,
                     so keep the two equal.
  acceptance_radius  stop points: horizontal distance counted as "reached" [m] (default 0.5)
  lookahead          fly-through waypoints (delay 0): how far the drone aims along the path,
                     on top of 2 s of travel at its current speed [m]; larger = wider,
                     faster turns. On mission_box.csv (90 deg corners, 5 m/s), 2 m slows
                     to ~1 m/s and cuts ~2.8 m; 6 m slows to 2.4-3.9 m/s and cuts
                     5.3-6.5 m (default 6.0)
  autostart          true: start the flight as soon as PX4 accepts the mode (default).
                     false: wait until the mode is selected (QGC, RC, or a vehicle command).

Subscribes to /detections (vision_msgs/Detection2DArray from object_detector.py); the mode
logs the tracked objects about once a second while it flies.
"""

import csv
import math
import os
import threading
import time

from ament_index_python.packages import get_package_share_directory
import numpy as np
import px4_ros2
from px4_msgs.msg import VehicleCommand
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from vision_msgs.msg import Detection2DArray

from px4_offboard.pixel_distance import pixel_to_ground_ned, target_waypoint


Result = px4_ros2.components.Result

# PX4's Goto smoother approaches its target at about this many m/s per metre of distance
# left (PX4's MPC_XY_TRAJ_P default; measured 0.5-0.6 in SITL) [1/s]
APPROACH_GAIN = 0.5

# Detections older than this are ignored, e.g. if the detector has stopped [s, wall clock]
DETECTIONS_MAX_AGE = 1.0


def load_mission(path):
    """Return a list of (position_ned, delay_s) from a north,east,down,delay CSV."""
    with open(path, newline='', encoding='utf-8') as f:
        rows = [row for row in csv.reader(f) if row]
    mission = []
    for row in rows:
        if len(row) != 4:
            raise ValueError(f'Expected north,east,down,delay in {path}: {row}')
        north, east, down, delay = (float(cell) for cell in row)
        mission.append((np.array([north, east, down]), delay))
    if not mission:
        raise ValueError(f'No waypoints in {path}')
    return mission


class WaypointMode(px4_ros2.components.ModeBase):
    """
    Fly the mission's waypoints in order.

    A waypoint with a delay > 0, and the last waypoint, are stop points: the drone stops
    there and holds for the delay. A waypoint with delay 0 is flown through with a rounded
    turn. Goto always plans to stop at its target, so instead of the waypoint the mode sends
    a "carrot": the point `lookahead` metres further along the path than the drone. Near a
    fly-through waypoint the carrot slides round the corner onto the next leg, so the drone
    turns without stopping; near a stop point the carrot stops at it, so the drone brakes.
    Goto slows down as it nears its target, so the carrot is `lookahead` plus 2 s of travel
    at the current speed ahead: the drone keeps its speed on straights and turns wider when
    faster (about 12 m ahead at 5 m/s). A larger lookahead gives wider, faster turns.
    """

    def __init__(self, px4_node, logger, mission, max_speed, acceptance_radius, lookahead):
        super().__init__(node=px4_node, mode_name='Waypoints')
        self.goto = px4_ros2.control.MulticopterGotoSetpointType(self)
        self.traj = px4_ros2.control.TrajectorySetpointType(self)

        self.vehicle_attitude = px4_ros2.odometry.Attitude(self)
        self.local_position = px4_ros2.odometry.LocalPosition(self)

        self.logger = logger
        self.points = [p for p, _ in mission]
        self.delays = [d for _, d in mission]
        self.max_speed = max_speed
        self.cruise_speed = 6
        self.acceptance_radius = acceptance_radius
        self.lookahead = lookahead
        # Latest /detections message and when it arrived (time.monotonic()). Set from the
        # rclpy executor thread by on_detections(); read here in update_setpoint().
        self.detections = None
        self.detections_time = 0.0

        self.mount_rpy_frd = np.array([0.0, -1.05, 0.0])
        self.fov_rad = np.radians((85.3, 59.9))  # sim camera; see pixel_distance.py
        self.img_size = (1920, 1200)
        self.rpy = np.array([0.0, 0.0, 0.0])

    def on_detections(self, msg):
        """Store the latest Detection2DArray (called from the rclpy thread)."""
        self.detections = msg
        self.detections_time = time.monotonic()

    def current_detections(self):
        """Return the latest detections, or [] if none have arrived recently."""
        msg = self.detections
        if msg is None or time.monotonic() - self.detections_time > DETECTIONS_MAX_AGE:
            return []
        return msg.detections

    def get_detection(self, position):
        self.rpy = self.get_attitude()
        det = False
        for d in self.current_detections():

            u = d.bbox.center.position.x
            v = d.bbox.center.position.y + d.bbox.size_y / 2.0  # bottom of the box
            p1_ned = pixel_to_ground_ned(u, v, position, self.rpy, self.mount_rpy_frd, self.fov_rad, self.img_size)

            d_to_target = np.linalg.norm(p1_ned - self.target_pos)

            if not self.targeting and self.target_id != d.id and d_to_target > 5.0:
                self.targeting = True
                self.target_wp = target_waypoint(position, p1_ned, 5.0, -self.mount_rpy_frd[1])
                self.target_id = d.id
                self.target_start = position
                self.target_delay = 10.0
                self.target_pos = p1_ned
                det = True

                self.logger.info(
                    f'Current: ({position[0]:.1f}, {position[1]:.1f}, {position[2]:.1f}) Target: ({self.target_wp[0]:.1f}, {self.target_wp[1]:.1f}, {self.target_wp[2]:.1f}) '
                    f'Tpos ({p1_ned[0]:.2f}, {p1_ned[1]:.2f}, {p1_ned[2]:.2f})' )
        return det


    def log_detections(self, position):
        objects = []
        self.rpy = self.get_attitude()
        for d in self.current_detections():
            hypothesis = d.results[0].hypothesis if d.results else None
            label = hypothesis.class_id if hypothesis else '?'
            score = hypothesis.score if hypothesis else 0.0
            u = d.bbox.center.position.x
            v = d.bbox.center.position.y
            p1_ned = pixel_to_ground_ned(u, v, position, self.rpy, self.mount_rpy_frd, self.fov_rad, self.img_size)

            objects.append(f'{label} #{d.id} at ({u:.0f}, '
                        #    f'{v:.0f}) px {score:.2f}')
                           f'{v:.0f}) pos ({p1_ned[0]:.2f}, {p1_ned[1]:.2f}, {p1_ned[2]:.2f})')
        self.logger.info(
            f'At ({position[0]:.1f}, {position[1]:.1f}, {position[2]:.1f}): '
            f'{len(objects)} object(s)' + (': ' + ', '.join(objects) if objects else ''),
            throttle_duration_sec=1.0)
        return len(objects)
    
    def get_attitude(self):
        r = self.vehicle_attitude.roll()
        p = self.vehicle_attitude.pitch()
        y = self.vehicle_attitude.yaw()
        return np.array([r, p, y])

    def log_attitude(self):
        r = self.vehicle_attitude.roll()
        p = self.vehicle_attitude.pitch()
        y = self.vehicle_attitude.yaw()
        self.rpy = np.array([r, p, y])
        self.logger.info(f'roll: {r:.2f}, pitch: {p:.2f}, yaw: {y:.2f}', throttle_duration_sec=1.0)

    def is_stop(self, i):
        if self.targeting or self.delays[i] > 0 or i == len(self.points) - 1:
            ret = True
        else :
            ret = False
        return ret
        # return self.delays[i] > 0 or i == len(self.points) - 1

    def segment_start(self, i):
        return self.start if i == 0 else self.points[i - 1]

    def carrot(self, i, along, distance):
        """Point `along + distance` metres along the path from the start of segment i.

        Never goes past the next stop point.
        """
        remaining = along + distance
        while True:
            if not self.targeting:
                a, b = self.segment_start(i), self.points[i]
            else:
                a, b = self.target_start, self.target_wp
            length = np.linalg.norm(b - a)
            # print(i, remaining, length)
            if remaining <= length:
                return a + (b - a) * (remaining / length)
            if self.is_stop(i):
                return b
            remaining -= length
            i += 1

    def on_activate(self):
        self.start = self.local_position.position_ned()
        self.index = 0              # waypoint the drone is heading to
        self.waiting = False
        self.elapsed = 0.0          # time on the current leg or hold [s]
        self.heading = self.local_position.heading()
        self.targeting = False
        self.target_id = 0
        self.target_pos = np.array([0.0, 10000.0, 1000.0])

    def on_deactivate(self):
        pass

    def update_setpoint(self, dt_s):
        # After completed(), PX4 may still call this until it switches mode: hold the last point
        if self.index == len(self.points):
            self.goto.update(self.points[-1], heading=self.heading)
            return

        i = self.index
        position = self.local_position.position_ned()
        # self.get_detection(position)
        self.log_detections(position)

        if not self.targeting:
            a, b = self.segment_start(i), self.points[i]
        else:
            a, b = self.target_start, self.target_wp

        length = np.linalg.norm(b - a)
        # Progress along the current segment: the drone's position projected onto it
        along = 0.0
        if length > 1e-3:
            along = float(np.clip((position - a) @ (b - a) / length, 0, length))
        # Goto slows down as it nears its target (speed ~ APPROACH_GAIN * distance), so the
        # carrot must stay at least speed / APPROACH_GAIN ahead or the drone slows to it.
        velocity = self.local_position.velocity_ned()
        speed_xy = math.hypot(velocity[0], velocity[1])
        distance = self.lookahead + speed_xy / APPROACH_GAIN
        target = self.carrot(i, along, distance)

        # self.log_attitude()

        # Face the direction of travel; hold the last heading close to the target, where the
        # direction to it becomes noisy.
        to_target = target - position
        if math.hypot(to_target[0], to_target[1]) > 1.0:
            self.heading = math.atan2(to_target[1], to_target[0])
        self.goto.update(target, heading=self.heading, max_horizontal_speed=self.cruise_speed)
        self.elapsed += dt_s

        if not self.is_stop(i):
            # Fly-through: move on once the drone has crossed the line that bisects the turn,
            # which it does while cutting the corner (or once close, for a U-turn).
            incoming = (b - a) / length if length > 1e-3 else np.zeros(3)
            out = self.points[i + 1] - b
            outgoing = out / np.linalg.norm(out) if np.linalg.norm(out) > 1e-3 else incoming
            bisector = incoming + outgoing
            if np.linalg.norm(bisector) > 1e-3:
                passed = (position - b) @ bisector >= 0
            else:
                passed = np.linalg.norm(position - b) < self.acceptance_radius
            if passed:
                speed = np.linalg.norm(self.local_position.velocity_ned())
                self.logger.info(
                    f'Passed waypoint {i} ({b[0]:.1f}, {b[1]:.1f}, {b[2]:.1f}) after '
                    f'{self.elapsed:.1f} s, {np.linalg.norm(position - b):.1f} m from it '
                    f'at {speed:.1f} m/s')
                self.index += 1
                self.elapsed = 0.0
            return

        if not self.waiting:
            error = b - position
            speed = np.linalg.norm(self.local_position.velocity_ned())
            if math.hypot(error[0], error[1]) > self.acceptance_radius or abs(error[2]) > 0.3 \
                    or speed > 0.3:
                return
            self.logger.info(
                f'Reached waypoint {i} ({b[0]:.1f}, {b[1]:.1f}, {b[2]:.1f}) after '
                f'{self.elapsed:.1f} s')
            self.waiting = True
            self.elapsed = 0.0

        if self.targeting:
            delay = self.target_delay
            self.yaw_rate = 0.0
        else:
            delay = self.delays[i]
            self.yaw_rate = 2*np.pi / self.delays[i] if self.delays[i] > 0 else 0.0

        # if self.elapsed < self.delays[i]:
        if self.elapsed < delay:
            self.heading += self.yaw_rate * dt_s
            if self.heading > math.pi:
                self.heading -= 2*math.pi
            return

        self.waiting = False
        self.elapsed = 0.0
        if not self.targeting:
            self.index += 1
        if self.index == len(self.points):
            self.completed(Result.success)

        self.targeting = False

class MissionExecutor(px4_ros2.components.ModeExecutorBase):
    """Arm, take off, fly the Waypoints mode, land, then shut down once disarmed."""

    def __init__(self, mode, px4_node, logger, autostart):
        activation = (px4_ros2.components.ModeExecutorActivation.immediately if autostart
                      else px4_ros2.components.ModeExecutorActivation.always)
        super().__init__(mode, activation)
        self.mode = mode
        self.px4_node = px4_node
        self.logger = logger
        # 'waiting' -> 'flying' (arming .. waypoints) -> 'ending' (final Return/Land) -> 'done';
        # 'interrupted' if PX4 or a pilot took over. main() uses it to decide what to do on Ctrl-C.
        self.phase = 'waiting'

    def on_activate(self):
        self.logger.info('Waiting until PX4 is ready to arm')
        self.wait_ready_to_arm(self.then(self.arm_vehicle))

    def on_deactivate(self, reason):
        # A failsafe or a mode switch (RC, QGC) took over. PX4 now flies; just report it.
        self.phase = 'interrupted'
        self.logger.warn(f'Mission interrupted ({reason.name})')

    def then(self, next_step):
        """Wrap next_step so it only runs if the previous step succeeded."""
        def on_completed(result):
            if result == Result.success:
                next_step()
            else:
                self.logger.error(f'Mission aborted: {result.name}')
        return on_completed

    def arm_vehicle(self):
        self.phase = 'flying'
        self.arm(self.then(self.take_off))

    def take_off(self):
        self.logger.info('Taking off')
        self.takeoff(self.then(self.fly_waypoints))

    def fly_waypoints(self):
        self.logger.info('Flying waypoints')
        self.schedule_mode(self.mode.id(), self.then(self.land_vehicle))

    def land_vehicle(self):
        self.logger.info('Mission complete, landing')
        self.phase = 'ending'
        # self.land(self.then(self.wait_disarmed))
        self.rtl(self.then(self.wait_disarmed))

    def wait_disarmed(self):
        self.wait_until_disarmed(self.then(self.finish))

    def finish(self):
        self.logger.info('Landed and disarmed')
        self.phase = 'done'
        self.px4_node.shutdown()


def command_return(node, logger):
    """Send PX4 a Return (RTL) command from the rclpy node."""
    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=1)
    publisher = node.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)
    msg = VehicleCommand()
    msg.command = VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH
    msg.target_system = 1
    msg.target_component = 1
    msg.source_system = 1
    msg.source_component = 1
    msg.from_external = True
    logger.warn('Stopped mid-flight: commanding PX4 to Return')
    # BEST_EFFORT: send a few times so one gets through
    for _ in range(5):
        msg.timestamp = node.get_clock().now().nanoseconds // 1000
        publisher.publish(msg)
        time.sleep(0.1)


def main(args=None):
    # rclpy handles parameters, logging and perception topics; the px4_ros2 node is a
    # separate rclcpp node that talks to PX4. Ctrl-C shuts the px4_ros2 node down, so rclpy
    # must not shut down too: it sends the Return command on the way out.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = Node('waypoint_mission')
    logger = node.get_logger()
    mission_file = node.declare_parameter('mission_file', 'mission_box.csv').value
    max_speed = node.declare_parameter('max_speed', 10.0).value
    acceptance_radius = node.declare_parameter('acceptance_radius', 0.5).value
    lookahead = node.declare_parameter('lookahead', 6.0).value
    autostart = node.declare_parameter('autostart', True).value

    if os.path.dirname(mission_file) == '':
        mission_file = os.path.join(
            get_package_share_directory('px4_offboard'), 'missions', mission_file)
    mission = load_mission(mission_file)
    logger.info(f'Loaded {len(mission)} waypoints from {mission_file}')

    px4_node = px4_ros2.Node('waypoint_mission_px4')
    mode = WaypointMode(px4_node, logger, mission, max_speed, acceptance_radius, lookahead)
    executor = MissionExecutor(mode, px4_node, logger, autostart)
    if not executor.register():
        logger.error('PX4 did not accept the mode registration')
        return

    # px4_node.spin() only drives the px4_ros2 node, so the rclpy node (subscriptions) spins
    # in its own thread. spin() releases the GIL, so the two run side by side.
    node.create_subscription(Detection2DArray, '/detections', mode.on_detections, 10)
    ros_executor = SingleThreadedExecutor()
    ros_executor.add_node(node)
    ros_thread = threading.Thread(target=ros_executor.spin, daemon=True)
    ros_thread.start()

    try:
        px4_node.spin()     # returns after finish() or Ctrl-C
    except KeyboardInterrupt:
        pass
    finally:
        # Stopped between arming and the final Return/Land: PX4 may be in one of its own
        # modes (e.g. Takeoff, then Hold), where losing this program is not a failsafe and
        # the drone would hover until someone intervenes.
        if executor.phase == 'flying':
            command_return(node, logger)
        # The mode and executor hold references into px4_node: destroy them first, or
        # Python may free the node first and the process crashes on exit.
        del executor, mode
        del px4_node
        ros_executor.shutdown()
        ros_thread.join(timeout=1.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
