# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Layout

One repo for the whole project: design notes, ROS 2 code and simulation assets.

- `docs/`: research and design notes. `06-autonomy-roadmap.md` defines the milestones (M0 = SITL square). `05-software-stack.md` pins versions.
- `camera/`: vendor PDFs for the e-con See3CAM_24CUG (the camera bought) and e-CAM25_CUONX (rejected: no JetPack 7.2 driver).
- `ws/`: the colcon workspace root. `build/`, `install/` and `log/` are gitignored.
  - `ws/src/px4_offboard`, `ws/src/px4_bringup`: two `ament_python` packages, originally from [MechaMind-Labs/ROS2-PX4_Drone_Teleoperation_Using_Joystick](https://github.com/MechaMind-Labs/ROS2-PX4_Drone_Teleoperation_Using_Joystick) (upstream README kept as `ws/src/UPSTREAM_README.md`), now modified and owned here.
  - `ws/src/px4_msgs`: git submodule of `PX4/px4_msgs`, branch `release/1.17`. Clone with `--recursive` or run `git submodule update --init`.
- `sim/`: Gazebo models (`my_x500_base`, `my_x500`, `my_x500_mono_cam`) and PX4 airframes (`22001_gz_my_x500`, `22002_gz_my_x500_mono_cam`). `sim/link_into_px4.sh` symlinks them into `~/PX4-Autopilot`, which otherwise stays a stock v1.17.0 checkout. **Edit them here, never in the PX4 tree.**

The upstream project targets Humble/22.04. This machine runs **ROS 2 Jazzy, Ubuntu 24.04, Gazebo Harmonic (system `gz-harmonic` from the OSRF apt repo), PX4 v1.17.0** in `~/PX4-Autopilot`, and Micro XRCE-DDS Agent v2.4.3 built from `~/src/Micro-XRCE-DDS-Agent` and installed to `~/.local/bin/MicroXRCEAgent` (no sudo).

### Sim assets

- `my_x500_base` is stock `x500_base` renamed. It references the stock meshes and textures (`model://x500_base/...`) instead of carrying copies.
- `my_x500` changes the stock `x500` motor model: `maxRotVelocity` 1032, `motorConstant` 1.23e-5, `momentConstant` 0.014. The `22001` airframe matches it with `MPC_THR_HOVER 0.73` and `SIM_BAT_DRAIN 1080`.
- `my_x500_mono_cam` adds PX4's stock `mono_cam` (1280×960, `horizontal_fov` 1.74 rad, 30 Hz, forward-facing) at `.12 .03 .242`. It does not yet match the real See3CAM_24CUG (1920×1200, 104.6° HFOV = 1.826 rad, mounted facing down).
- PX4 spawns models from `file://${PX4_GZ_MODELS}/<name>/model.sdf`, so they must sit in `Tools/simulation/gz/models`. That is why they are symlinked rather than added through `GZ_SIM_RESOURCE_PATH`.
- PX4 only installs airframes listed in `ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt`. The link script adds missing entries. Rerun `make px4_sitl` after adding an airframe.
- `px4_msgs` must match the PX4 firmware version. Since PX4 v1.16 the bridge appends `_vN` to topics whose `MESSAGE_VERSION` > 0, so the nodes use `/fmu/out/vehicle_status_v1` and `/fmu/out/vehicle_local_position_v1`. Upgrading PX4 means updating both, and checking topic suffixes against the `MESSAGE_VERSION` in each `.msg`.

## Commands

```bash
# One-time, or after adding a model/airframe under sim/
sim/link_into_px4.sh                           # optional arg: path to PX4-Autopilot

# Build, from ws/
cd ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install                 # symlink-install: edit Python without rebuilding
colcon build --packages-select px4_offboard
source install/setup.bash

# Tests are only the ament linters (flake8, pep257, copyright)
colcon test --packages-select px4_offboard && colcon test-result --verbose

# Run
ros2 launch px4_bringup minimal.launch.py      # add world:=baylands etc. (choices = PX4 gz worlds)
ros2 launch px4_offboard keyboard_teleop.launch.py
ros2 launch px4_offboard joystick_teleop.launch.py
ros2 run px4_offboard velocity_offboard_control.py
ros2 run px4_offboard position_mission_control.py
```

`ws/sim_commands.sh` is a scratch list of commands, not a script to run. Console-script names include the `.py` suffix (see `px4_offboard/setup.py`). A new node must be added to `entry_points` there. New launch or config files are installed only if they match the `launch/*.launch.py` or `config/*.yaml` globs.

## Architecture

`px4_bringup/launch/minimal.launch.py` opens `gnome-terminal` windows running `make px4_sitl gz_<model>` in `~/PX4-Autopilot` (the `model` variable, currently `my_x500_mono_cam`), then `MicroXRCEAgent udp4 -p 8888` 3 s later. Those paths and binaries are hardcoded. It also starts `ros_gz_bridge` (Gazebo camera → `/camera/image_raw`, `/camera/camera_info`) and `px4_offboard`'s `camera_monitor.py`. `config/bridge_config.yaml` is a `string.Template`: `${world}` and `${model}` are substituted from the `world` arg and `model` variable into a temp file at launch, because Gazebo sensor topics are `/world/<world>/model/<model>_0/...`.

The teleop data flow is: teleop node → ROS topics → `velocity_offboard_control.py` → `/fmu/in/*` → Micro XRCE-DDS Agent → PX4 SITL.

- **Teleop nodes** (`teleop_keyboard.py`, `teleop_joystick.py`) do not talk to PX4. At 20 Hz they publish:
  - `geometry_msgs/Twist` on `/offboard_velocity_cmd`
  - `std_msgs/Bool` on `/arm_message` (true = arm, false = disarm)

  The joystick node subscribes to `/joy` from the `joy` package's `joy_node`, which is configured by `config/joy_config.yaml`. It uses an Xbox-style button mapping: A=0 arms, X=3 toggles auto-hover, Y=4 disarms. When disarmed, it stops publishing entirely.
- **`velocity_offboard_control.py`** is the only node the teleop launch files use to talk to PX4. It:
  - publishes `OffboardControlMode` (velocity only) and `TrajectorySetpoint` at 50 Hz;
  - on arm, sends 30 warm-up setpoints, then `VEHICLE_CMD_DO_SET_MODE` (param1=1, param2=6, OFFBOARD), then ARM;
  - hovers (zero velocity) if no Twist arrives for 5 s;
  - clamps velocities to ±3 m/s horizontal, ±2 m/s vertical and ±2 rad/s yaw.

  The warm-up loop blocks with `time.sleep` inside a callback.
- **Frame convention for `/offboard_velocity_cmd`:** the Twist is effectively body FRD / NED, not ROS FLU. `linear.z` is passed straight through as NED z, so **negative means climb**. Both teleop nodes produce negative z for "throttle up". The controller rotates `(linear.x, linear.y)` by the current yaw, taken from the `/fmu/out/vehicle_attitude` quaternion, into NED. `angular.z` is sent as `yawspeed`. Keep this convention when you add a new command source.
- **QoS:** every topic to or from PX4 uses BEST_EFFORT + TRANSIENT_LOCAL, KEEP_LAST. Internal topics use the same profile. The exception is the `/joy` subscription, which must be RELIABLE + VOLATILE to match `joy_node`.
- **Camera QoS:** subscribe to `/camera/image_raw` RELIABLE with a small depth. A 1280×960 rgb8 frame is ~3.7 MB; BEST_EFFORT subscribers (including `ros2 topic hz` and rqt image view, which hardcode sensor-data QoS) drop about half the frames and report 9–17 Hz although the sim produces 30 Hz.
- `position_offboard_control.py` is a standalone example that takes off to 5 m in position mode and then lands. `position_mission_control.py` flies a waypoint list with a WARMUP/TRAVEL/WAIT/RTL state machine. No launch file uses either. `px4_offboard/test.py` is a scratch file.

## Gotchas

- `keyboard_teleop.launch.py` runs the keyboard node in a new `gnome-terminal`, because the node needs a TTY for raw `termios` input. That terminal sources `/opt/ros/$ROS_DISTRO/setup.bash` and this workspace's `install/setup.bash`, which it finds via `get_package_prefix`.
- In `teleop_keyboard.py`, the arrow-key escape codes `[C` and `[D` are assigned to the opposite labels, and the vy signs are flipped too, so the net motion is correct. Fix both together or neither.
- **Arming without QGroundControl:** in PX4 v1.17 the `gz_x500` airframe sets `NAV_DLL_ACT 2`, so PX4 denies arming ("Preflight Fail: No connection to the GCS") unless a GCS is connected. `minimal.launch.py` fixes this by waiting for `commander`, then running `px4-param set NAV_DLL_ACT 0` and `px4-param save`. `PX4_PARAM_NAV_DLL_ACT=0` does **not** work: `rcS` applies env overrides before the airframe script, 0 equals the firmware default so it isn't stored, and the airframe's `set-default` then wins.
- `velocity_offboard_control.py` logs "ARMED - Ready to fly!" without checking `arming_state`. To verify, echo `/fmu/out/vehicle_status_v1` (arming_state 2 = armed, nav_state 14 = offboard) or grep PX4's console for `Arming denied`.
- Headless testing: `sleep infinity | HEADLESS=1 make px4_sitl gz_my_x500_mono_cam` from `~/PX4-Autopilot`. The idle stdin pipe stops the `pxh>` prompt from spinning; kill the `sleep` afterwards. PX4 client commands (`px4-param`, `px4-listener`, `px4-commander`) are in `~/PX4-Autopilot/build/px4_sitl_default/bin/` and talk to the running instance. Echoing `/fmu/out/*` with `ros2 topic echo` needs `--qos-reliability best_effort`.
