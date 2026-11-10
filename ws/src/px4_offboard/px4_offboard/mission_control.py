#!/usr/bin/env python3
"""
Waypoint mission as a PX4 ROS 2 custom mode, in Python (px4_ros2_py).

The "Waypoints" mode flies a mission json with PX4's Goto setpoint, so PX4's
own smoother plans the motion. Waypoints with flythrough=True are flown through with a rounded
turn; the others are stopped at. The mode executor runs the whole flight:
arm, PX4 Takeoff, Waypoints, PX4 Land, then the program exits once the drone has disarmed.

Parameters (--ros-args -p name:=value):
  mission_file       bare name: read from this package's installed missions/ folder;
                     anything containing '/' is used as a path (default mission_target.json)

Waypoint params are [a, b, c, speed], read according to the mission's params.wp_format:
  "NED"  north, east, down [m] in PX4's local frame
  "LLA"  latitude, longitude [deg], altitude above home [m, up] (QGC's "Relative" altitude).
         Converted to local NED when the mode activates, with PX4's projection about its local
         origin; "home" is where the drone was when it armed (as for PX4's own home). The
         executor will not arm until PX4 publishes that origin (needs a GPS fix).

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
from px4_msgs.msg import VehicleCommand, VehicleLocalPosition
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from vision_msgs.msg import Detection2DArray

from px4_offboard.geo import global_to_local
from px4_offboard.pixel_distance import pixel_to_ground_ned, target_waypoint

import json
from px4_offboard.mission_header import MissionCommand, TargetMode

WP_FORMATS = ('NED', 'LLA')

# While waiting for PX4's global reference before arming, log this often [s, wall clock]
GEO_WAIT_LOG_PERIOD = 10.0

Result = px4_ros2.components.Result

# PX4's Goto smoother approaches its target at about this many m/s per metre of distance
# left (PX4's MPC_XY_TRAJ_P default; measured 0.5-0.6 in SITL) [1/s]
APPROACH_GAIN = 0.5

# Detections older than this are ignored, e.g. if the detector has stopped [s, wall clock]
DETECTIONS_MAX_AGE = 1.0


def load_mission(path):
    with open(path, "r") as file:
        mission = json.load(file)
    return mission

class WaypointMode(px4_ros2.components.ModeBase):
    """
    Fly the mission's waypoints in order.

    A waypoint with a flythrough flag set to True are stop points. 
    Goto always plans to stop at its target, so instead of the waypoint the mode sends
    a "carrot": the point `lookahead` metres further along the path than the drone. Near a
    fly-through waypoint the carrot slides round the corner onto the next leg, so the drone
    turns without stopping; near a stop point the carrot stops at it, so the drone brakes.
    Goto slows down as it nears its target, so the carrot is `lookahead` plus 2 s of travel
    at the current speed ahead: the drone keeps its speed on straights and turns wider when
    faster (about 12 m ahead at 5 m/s). A larger lookahead gives wider, faster turns.
    """

    def __init__(self, px4_node, logger, mission):
        super().__init__(node=px4_node, mode_name='Waypoints')
        self.goto = px4_ros2.control.MulticopterGotoSetpointType(self)
        self.traj = px4_ros2.control.TrajectorySetpointType(self)

        self.vehicle_attitude = px4_ros2.odometry.Attitude(self)
        self.local_position = px4_ros2.odometry.LocalPosition(self)

        self.logger = logger

        # self.max_speed = mission['params']['max_speed']  # TODO: not used, Not sure if needed
        self.acceptance_radius = mission['params']['acceptance_radius']
        self.lookahead = mission['params']['lookahead']

        # Latest /detections message and when it arrived (time.monotonic()). Set from the
        # rclpy executor thread by on_detections(); read here in update_setpoint().
        self.detections = None
        self.detections_time = 0.0

        mission["camera"]["mount_rpy_frd"] = np.array(mission["camera"]["mount_rpy_frd"])
        mission["camera"]["fov"] = np.radians(mission["camera"]["fov"]) 
        mission["camera"]["img_size"] = tuple(mission["camera"]["img_size"])

        self.wp_format = mission['params'].get('wp_format', 'NED').upper()
        if self.wp_format not in WP_FORMATS:
            raise ValueError(f'wp_format must be one of {WP_FORMATS}, not {self.wp_format!r}')

        self.wp_indices = []
        for i in range(len(mission["items"])):
            if mission["items"][i]["command"] == MissionCommand.WP.value:
                mission["items"][i]["params"] = np.array(mission["items"][i]["params"], dtype=float)
                self.wp_indices.append(i)
        # LLA: the waypoints as loaded; convert_waypoints() writes their NED into the items
        self.wp_lla = {i: mission["items"][i]["params"].copy() for i in self.wp_indices}

        # Latest PX4 vehicle_local_position, for its global reference (set from the rclpy thread
        # by on_local_position()), and the local down of home, set by the executor once armed.
        # (PX4's home_position is only published when it changes, so it can't be relied on.)
        self.local_ref = None
        self.home_down = None
        self.geo_error = None

        self.mount_rpy_frd = mission["camera"]["mount_rpy_frd"]
        self.fov_rad = mission["camera"]["fov"]
        self.img_size = mission["camera"]["img_size"]
        self.items = mission["items"]

        self.rpy = np.array([0.0, 0.0, 0.0])

        self.target_mode = TargetMode.OFF.value

    def on_detections(self, msg):
        """Store the latest Detection2DArray (called from the rclpy thread)."""
        self.detections = msg
        self.detections_time = time.monotonic()

    def on_local_position(self, msg):
        """Store PX4's local position (called from the rclpy thread) for its global reference."""
        self.local_ref = msg

    def geo_missing(self):
        """What LLA conversion still lacks from PX4, or None if it can convert."""
        ref = self.local_ref
        if ref is None:
            return 'no vehicle_local_position received'
        if not ref.xy_global:
            return 'PX4 has no global reference for its local frame (no GPS fix yet?)'
        return None

    def convert_waypoints(self):
        """Convert the LLA waypoints to local NED. Returns an error string, or None."""
        missing = self.geo_missing()
        if missing:
            return missing
        if self.home_down is None:
            return 'home altitude not recorded at arming'
        ref, home_down = self.local_ref, self.home_down
        self.logger.info(
            f'Converting LLA waypoints: local origin ({ref.ref_lat:.7f}, {ref.ref_lon:.7f}), '
            f'home at down {home_down:.2f}')
        for i, (lat, lon, alt, speed) in self.wp_lla.items():
            north, east = global_to_local(lat, lon, ref.ref_lat, ref.ref_lon)
            down = home_down - alt
            self.items[i]["params"] = np.array([north, east, down, speed])
            self.logger.info(f'  item {i}: ({lat:.7f}, {lon:.7f}, {alt:.1f} m) -> '
                             f'NED ({north:.1f}, {east:.1f}, {down:.1f})')
        return None

    def current_detections(self):
        """Return the latest detections, or [] if none have arrived recently."""
        msg = self.detections
        if msg is None or time.monotonic() - self.detections_time > DETECTIONS_MAX_AGE:
            return []
        return msg.detections

# TODO: get_detection() and log_detections() are currently unused and need to be changed (future)
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

    def segment_start(self, i):
        return self.start if i == 0 else self.items[self.wp_indices[i - 1]]["params"][0:3]

    def carrot(self, i, along, distance):
        """Point `along + distance` metres along the path from the start of segment i.

        Never goes past the next stop point.
        """
        remaining = along + distance
        while True:
            # if not self.targeting:
            #     a, b = self.segment_start(i), self.points[i]
            # else:
            #     a, b = self.target_start, self.target_wp
            wi = self.wp_indices[i] 
            a, b = self.segment_start(i), self.items[wi]["params"][0:3]

            length = np.linalg.norm(b - a)

            if remaining <= length:
                return a + (b - a) * (remaining / length)
            if not self.items[wi]['flythrough']:
                return b
            remaining -= length
            i += 1

    def on_activate(self):
        self.start = self.local_position.position_ned()
        self.index = 0              
        self.windex = 0
        self.elapsed = 0.0          # time on the current leg or hold [s]
        self.heading = self.local_position.heading()
        self.targeting = False
        self.target_id = 0
        self.target_pos = np.array([0.0, 10000.0, 1000.0])
        self.geo_error = None
        if self.wp_format == 'LLA':
            self.geo_error = self.convert_waypoints()
            if self.geo_error:
                self.logger.error(f'Cannot convert LLA waypoints: {self.geo_error}')

    def on_deactivate(self):
        pass

    def update_setpoint(self, dt_s):
        # LLA conversion failed in on_activate(): hold where the mode started and report failure
        # (the executor then commands Return). Keep holding until PX4 switches mode.
        if self.geo_error:
            self.goto.update(self.start, heading=self.heading)
            if self.index != len(self.items):
                self.index = len(self.items)
                self.completed(Result.rejected)
            return

        # After completed(), PX4 may still call this until it switches mode: hold the last point
        if self.index == len(self.items):
            self.goto.update(self.items[self.wp_indices[-1]]["params"][0:3], heading=self.heading)
            return

        i = self.index
        position = self.local_position.position_ned()
        self.elapsed += dt_s

# TODO: add target mode handling (future)
        # if self.target_mode == TargetMode.MOVE_TO_TARGET.value:
        #     self.get_detection(position)
        # elif self.target_mode != TargetMode.OFF.value:
        #     self.log_detections(position)

        # if not self.targeting:
        #     a, b = self.segment_start, self.points[i]
        # else:
        #     a, b = self.target_start, self.target_wp

        match self.items[i]["command"]:
            case MissionCommand.WP.value:
                wp = self.items[i]["params"][0:3]
                vel = self.items[i]["params"][3]

                wi = self.wp_indices[self.windex]
                a, b = self.segment_start(self.windex), wp

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
                target = self.carrot(self.windex, along, distance)

                # Face the direction of travel; hold the last heading close to the target, where the
                # direction to it becomes noisy.
                to_target = target - position
                if math.hypot(to_target[0], to_target[1]) > 1.0:
                    self.heading = math.atan2(to_target[1], to_target[0])

                self.goto.update(target, heading=self.heading, max_horizontal_speed=vel)

                if self.items[i]['flythrough']:
                    # Fly-through: move on once the drone has crossed the line that bisects the turn,
                    # which it does while cutting the corner (or once close, for a U-turn).
                    incoming = (b - a) / length if length > 1e-3 else np.zeros(3)

                    wi = self.wp_indices[self.windex + 1] 
                    out = self.items[wi]["params"][0:3] - b

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
                        self.windex += 1
                        self.elapsed = 0.0
                    return
                else:
                    error = b - position
                    speed = np.linalg.norm(self.local_position.velocity_ned())
                    if math.hypot(error[0], error[1]) > self.acceptance_radius or abs(error[2]) > 0.3 \
                            or speed > 0.3:
                        return
                    self.logger.info(
                        f'Reached waypoint {i} ({b[0]:.1f}, {b[1]:.1f}, {b[2]:.1f}) after '
                        f'{self.elapsed:.1f} s')
                    self.index += 1
                    self.windex += 1
                    self.elapsed = 0.0
            case MissionCommand.PAUSE.value:
                wpin = self.wp_indices[self.windex - 1] # hold at the last waypoint
                wp = self.items[wpin]["params"][0:3]
                self.goto.update(wp, heading=self.heading)

                if self.elapsed < self.items[i]["params"]:
                    return
                self.logger.info(f'Pause {self.items[i]["params"]} s complete')
                self.index += 1
                self.elapsed = 0.0
            case MissionCommand.SCAN360.value:    
                wpin = self.wp_indices[self.windex - 1] # hold at the last waypoint
                wp = self.items[wpin]["params"][0:3]

                yaw_rate = 2*np.pi / self.items[i]["params"] if self.items[i]["params"] > 0 else 0.0
                self.heading += yaw_rate * dt_s
                if self.heading > math.pi:
                    self.heading -= 2*math.pi

                self.goto.update(wp, heading=self.heading)

                if self.elapsed < self.items[i]["params"]:
                    return
                self.logger.info(f'Scan 360 {self.items[i]["params"]} s complete')
                self.index += 1
                self.elapsed = 0.0
            case MissionCommand.TARGET.value:
                self.target_mode = self.items[i]["mode"]
                self.index += 1
            case MissionCommand.RECORD.value:
                # TODO: implement video recording 
                self.index += 1
            case MissionCommand.LAND.value:
                # TODO: implement landing
                self.index += 1
            case MissionCommand.RTL.value:   
                self.index = len(self.items)  # abort the mission and finish it (the executor will command PX4 to Return)
            case _:
                self.logger.error(f'Unknown command {self.items[i]["command"]} at item {i}')
                self.index = len(self.items)  # abort the mission and finish it (the executor will command PX4 to Return)

        if self.index == len(self.items):
            self.completed(Result.success)

        # self.targeting = False

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
        self.geo_wait_logged = None
        self.wait_ready_to_arm(self.then(self.check_geo))

    def check_geo(self):
        """LLA missions: wait (indefinitely; Ctrl-C to give up) for PX4's global reference.

        Polls by re-waiting for ready-to-arm, which completes on the next vehicle_status.
        """
        missing = self.mode.geo_missing() if self.mode.wp_format == 'LLA' else None
        if missing is None:
            self.arm_vehicle()
            return
        now = time.monotonic()
        if self.geo_wait_logged is None or now - self.geo_wait_logged > GEO_WAIT_LOG_PERIOD:
            self.geo_wait_logged = now
            self.logger.warn(f'Not arming yet: {missing}')
        self.wait_ready_to_arm(self.then(self.check_geo))

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
        # Home, for LLA altitudes: where the drone armed (PX4 sets its home there too)
        self.mode.home_down = float(self.mode.local_position.position_ned()[2])
        self.logger.info('Taking off')
        self.takeoff(self.then(self.fly_waypoints))

    def fly_waypoints(self):
        self.logger.info('Flying waypoints')
        self.schedule_mode(self.mode.id(), self.on_waypoints_done)

    def on_waypoints_done(self, result):
        if result == Result.success:
            self.land_vehicle()
        elif result == Result.deactivated:
            # A failsafe or a pilot took over (on_deactivate() follows and reports it)
            self.logger.error(f'Mission aborted: {result.name}')
        else:
            # The mode itself failed (e.g. LLA conversion): bring the drone back
            self.logger.error(f'Waypoints failed ({result.name}), returning')
            self.phase = 'ending'
            self.rtl(self.then(self.wait_disarmed))

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
    mission_file = node.declare_parameter('mission_file', 'mission_target.json').value

    if os.path.dirname(mission_file) == '':
        mission_file = os.path.join(
            get_package_share_directory('px4_offboard'), 'missions', mission_file)
    mission = load_mission(mission_file)
    n_wp = sum(item['command'] == MissionCommand.WP.value for item in mission['items'])
    logger.info(f'Loaded {len(mission["items"])} items ({n_wp} waypoints, '
                f'{mission["params"].get("wp_format", "NED")}) from {mission_file}')

    px4_node = px4_ros2.Node('waypoint_mission_px4')
    mode = WaypointMode(px4_node, logger, mission)
    executor = MissionExecutor(mode, px4_node, logger, mission['params']['autostart'])
    if not executor.register():
        logger.error('PX4 did not accept the mode registration')
        return

    # px4_node.spin() only drives the px4_ros2 node, so the rclpy node (subscriptions) spins
    # in its own thread. spin() releases the GIL, so the two run side by side.
    node.create_subscription(Detection2DArray, '/detections', mode.on_detections, 10)
    px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
    node.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
                             mode.on_local_position, px4_qos)
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
        # The rclpy subscriptions hold the mode's bound methods (on_detections, ...), which
        # keep the mode alive after `del mode`: destroy the rclpy node first.
        ros_executor.shutdown()
        ros_thread.join(timeout=1.0)
        node.destroy_node()
        # The mode and executor hold references into px4_node: destroy them first, or
        # Python may free the node first and the process crashes on exit.
        del executor, mode
        del px4_node
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
