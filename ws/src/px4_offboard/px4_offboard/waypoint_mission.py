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
"""

import csv
import math
import os
import time

from ament_index_python.packages import get_package_share_directory
import numpy as np
import px4_ros2
from px4_msgs.msg import VehicleCommand
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions

Result = px4_ros2.components.Result

# PX4's Goto smoother approaches its target at about this many m/s per metre of distance
# left (PX4's MPC_XY_TRAJ_P default; measured 0.5-0.6 in SITL) [1/s]
APPROACH_GAIN = 0.5


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
        self.local_position = px4_ros2.odometry.LocalPosition(self)
        self.logger = logger
        self.points = [p for p, _ in mission]
        self.delays = [d for _, d in mission]
        self.max_speed = max_speed
        self.acceptance_radius = acceptance_radius
        self.lookahead = lookahead

    def is_stop(self, i):
        return self.delays[i] > 0 or i == len(self.points) - 1

    def segment_start(self, i):
        return self.start if i == 0 else self.points[i - 1]

    def carrot(self, i, along, distance):
        """Point `along + distance` metres along the path from the start of segment i.

        Never goes past the next stop point.
        """
        remaining = along + distance
        while True:
            a, b = self.segment_start(i), self.points[i]
            length = np.linalg.norm(b - a)
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

    def on_deactivate(self):
        pass

    def update_setpoint(self, dt_s):
        # After completed(), PX4 may still call this until it switches mode: hold the last point
        if self.index == len(self.points):
            self.goto.update(self.points[-1], heading=self.heading)
            return

        i = self.index
        position = self.local_position.position_ned()
        a, b = self.segment_start(i), self.points[i]
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

        # Face the direction of travel; hold the last heading close to the target, where the
        # direction to it becomes noisy.
        to_target = target - position
        if math.hypot(to_target[0], to_target[1]) > 1.0:
            self.heading = math.atan2(to_target[1], to_target[0])
        self.goto.update(target, heading=self.heading, max_horizontal_speed=self.max_speed)
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

        if self.elapsed < self.delays[i]:
            return

        self.waiting = False
        self.elapsed = 0.0
        self.index += 1
        if self.index == len(self.points):
            self.completed(Result.success)


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
    # rclpy handles parameters and logging (and later, perception topics); the px4_ros2 node
    # is a separate rclcpp node that talks to PX4. Ctrl-C shuts the px4_ros2 node down, so
    # rclpy must not shut down too: it sends the Return command on the way out.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = Node('waypoint_mission')
    logger = node.get_logger()
    mission_file = node.declare_parameter('mission_file', 'mission_box.csv').value
    max_speed = node.declare_parameter('max_speed', 5.0).value
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
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
