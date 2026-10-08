# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Layout

One repo for the whole project: design notes, ROS 2 code and simulation assets.

- `docs/`: research and design notes. `06-autonomy-roadmap.md` defines the milestones (M0 = SITL square). `05-software-stack.md` pins versions.
- `camera/`: vendor PDFs for the e-con See3CAM_24CUG (the camera bought) and e-CAM25_CUONX (rejected: no JetPack 7.2 driver).
- `ws/`: the colcon workspace root. `build/`, `install/` and `log/` are gitignored.
  - `ws/src/px4_offboard`, `ws/src/px4_bringup`: two `ament_python` packages, originally from [MechaMind-Labs/ROS2-PX4_Drone_Teleoperation_Using_Joystick](https://github.com/MechaMind-Labs/ROS2-PX4_Drone_Teleoperation_Using_Joystick) (upstream README kept as `ws/src/UPSTREAM_README.md`), now modified and owned here.
  - `ws/src/px4_msgs`: git submodule of `PX4/px4_msgs`, branch `release/1.17`. Clone with `--recursive` or run `git submodule update --init`.
  - `ws/src/px4-ros2-interface-lib`: git submodule of `Auterion/px4-ros2-interface-lib`, branch `release/1.17` (matches PX4 v1.17 and `px4_msgs`; check with its `scripts/check-message-compatibility.py`). C++ only on this branch; the Python bindings exist only on `main` (see `px4_ros2_py`). It also contains twelve `example_*` packages, skipped with `--packages-skip-regex '^example_'`.
  - `ws/src/px4_ros2_py`: the interface library's Python bindings, vendored from its `main` branch and patched to build against `release/1.17`, with the executor/local-position functions upstream hasn't bound yet. Its `README.md` lists every change; drop it when upgrading to a PX4 whose library branch ships them.
  - `ws/src/px4_missions`: `ament_cmake` C++ package built on the interface library (see Architecture).
- `sim/`: Gazebo models (`my_x500_base`, `my_x500`, `my_x500_mono_cam`, `see3cam_24cug`, `my_x500_see3cam_down`) and PX4 airframes (`22001_gz_my_x500`, `22002_gz_my_x500_mono_cam`, `22003_gz_my_x500_see3cam_down`). `sim/link_into_px4.sh` symlinks them into `~/PX4-Autopilot` and applies `sim/patches/` (skipping patches already applied); apart from those patches PX4 stays a stock v1.17.0 checkout. **Edit them here, never in the PX4 tree.**

The upstream project targets Humble/22.04. This machine runs **ROS 2 Jazzy, Ubuntu 24.04, Gazebo Harmonic (system `gz-harmonic` from the OSRF apt repo), PX4 v1.17.0** in `~/PX4-Autopilot`, and Micro XRCE-DDS Agent v2.4.3 built from `~/src/Micro-XRCE-DDS-Agent` and installed to `~/.local/bin/MicroXRCEAgent` (no sudo).

### Sim assets

- `my_x500_base` is stock `x500_base` renamed. It references the stock meshes and textures (`model://x500_base/...`) instead of carrying copies.
- `my_x500` changes the stock `x500` motor model: `maxRotVelocity` 1032, `motorConstant` 1.23e-5, `momentConstant` 0.014. The `22001` airframe matches it with `MPC_THR_HOVER 0.73` and `SIM_BAT_DRAIN 1080`.
- `my_x500_mono_cam` adds PX4's stock `mono_cam` (1280×960, `horizontal_fov` 1.74 rad, 30 Hz, forward-facing) at `.12 .03 .242`.
- `see3cam_24cug` models the real camera: 1920×1200, `horizontal_fov` 1.531 rad (87.7°), 30 Hz, looking along +X of `camera_link`. It keeps `mono_cam`'s link and sensor names (`camera_link`, `imager`), so `bridge_config.yaml` works for either. Gazebo renders a pinhole, which can't match the lens's 104.6°/61.6°/128.2° H/V/D FOV (strong barrel distortion at the sides and corners), so the FOV is that of the 3.0 mm focal length undistorted (fx ≈ 1000 px): the centre scale and VFOV match, the outer edges of the real frame are missing. The real camera does 55–60 fps; the sim uses 30 because each rgb8 frame is ~6.9 MB. Mass (30 g) is an estimate.
- `my_x500_see3cam_down` (airframe `22003`) mounts `see3cam_24cug` under the body at model pose `0 0 .20`, pitched +90° so it looks straight down with the nose at the top of the image. The landing skids fill the left and right edges of the frame. The mount position is a placeholder until the real mount is built.
- PX4 spawns models from `file://${PX4_GZ_MODELS}/<name>/model.sdf`, so they must sit in `Tools/simulation/gz/models`. That is why they are symlinked rather than added through `GZ_SIM_RESOURCE_PATH`.
- PX4 only installs airframes listed in `ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt`. The link script adds missing entries. Rerun `make px4_sitl` after adding an airframe.
- **Magnetometer fix (`sim/patches/`).** Stock v1.17.0 + gz-sim 8.x feeds the EKF a mirrored magnetometer field: Gazebo's legacy mode puts NED components on ENU axes and PX4's bridge only swaps body axes, which can't undo a world-frame mirror. Symptom: in-flight heading resets of up to ~90°, curved legs, and the position estimate wandering 0.5–1 m at waypoints. The patches backport PX4 PR #28877 (Gazebo `use_units_gauss`/`use_earth_frame_ned` = false in `gz_bridge/server.config`, FLU→FRD and tesla→gauss in `GZBridge.cpp`) and PX4-gazebo-models PR #161 (mag noise 3e-7 T in the `Tools/simulation/gz` submodule). Any model with a magnetometer, including `my_x500_base`, must set its noise in tesla (3e-7); 0.0001 is then 1 G of noise and PX4 refuses to arm. Drop the patches when upgrading to a PX4 that includes #28877.
- `px4_msgs` must match the PX4 firmware version. Since PX4 v1.16 the bridge appends `_vN` to topics whose `MESSAGE_VERSION` > 0, so the nodes use `/fmu/out/vehicle_status_v1` and `/fmu/out/vehicle_local_position_v1`. Upgrading PX4 means updating both, and checking topic suffixes against the `MESSAGE_VERSION` in each `.msg`.

## Commands

```bash
# One-time, or after adding a model/airframe under sim/
sim/link_into_px4.sh                           # optional arg: path to PX4-Autopilot

# Build, from ws/
cd ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-skip-regex '^example_'   # symlink-install: edit Python without rebuilding; C++ (px4_missions) still needs a rebuild
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
ros2 run px4_missions waypoint_mission           # arms, takes off, flies the CSV, lands, exits; no offboard node needed
ros2 run px4_offboard waypoint_mission.py        # the same mission in Python (px4_ros2_py); same parameters

# Stop everything: launches, PX4, Gazebo, agent, nodes, and the sim terminal windows
sim/kill_sim.sh

# Plot the newest SITL flight log: XY track + target + waypoints, velocity, heading
tools/plot_flight.py                           # --mission mission1.csv, --save DIR, or a .ulg path
```

`ws/sim_commands.sh` is a scratch list of commands, not a script to run. Console-script names include the `.py` suffix (see `px4_offboard/setup.py`). A new node must be added to `entry_points` there. New launch, config or mission files are installed only if they match the `launch/*.launch.py`, `config/*.yaml` or `missions/*.csv` globs; adding a new file needs one `colcon build`, editing an installed one doesn't.

## Architecture

`px4_bringup/launch/minimal.launch.py` opens `gnome-terminal` windows running `make px4_sitl gz_<model>` in `~/PX4-Autopilot` (the `model` variable, currently `my_x500_see3cam_down`), then `MicroXRCEAgent udp4 -p 8888` 3 s later. Those paths and binaries are hardcoded. It also starts `ros_gz_bridge` (Gazebo camera → `/camera/image_raw`, `/camera/camera_info`) and `px4_offboard`'s `camera_monitor.py`. `config/bridge_config.yaml` is a `string.Template`: `${world}` and `${model}` are substituted from the `world` arg and `model` variable into a temp file at launch, because Gazebo sensor topics are `/world/<world>/model/<model>_0/...`.

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
- **Camera QoS:** subscribe to `/camera/image_raw` RELIABLE with a small depth. A 1280×960 rgb8 frame is ~3.7 MB (1920×1200 is ~6.9 MB); BEST_EFFORT subscribers (including `ros2 topic hz` and rqt image view, which hardcode sensor-data QoS) drop about half the frames and report 9–17 Hz although the sim produces 30 Hz.
- **`px4_missions/waypoint_mission`** is the custom-mode alternative to raw offboard. It registers a PX4 mode named "Waypoints" plus a mode executor. With `autostart:=true` (default) the executor activates as soon as PX4 accepts the registration and runs: wait ready to arm → arm → PX4 Takeoff (default `MIS_TAKEOFF_ALT`, 2.5 m) → Waypoints → PX4 Land → wait until disarmed → `rclcpp::shutdown()`. With `autostart:=false` it waits until the mode is selected (QGC, RC, or a `VEHICLE_CMD_DO_SET_MODE`). The mode flies the same CSVs as `position_mission_control.py` (`mission_file`, looked up in `px4_offboard`'s installed `missions/`) with PX4's `MulticopterGotoSetpointType`, so PX4's smoother plans each leg; `max_speed` (default 5 m/s) caps horizontal speed and `acceptance_radius` (0.5 m, plus |dz| < 0.3 m and speed < 0.3 m/s) defines "reached". A failsafe or a mode switch deactivates the executor and PX4 takes over.
- **`px4_offboard/waypoint_mission.py`** is the Python version of `px4_missions/waypoint_mission` (same mode name and flight sequence), plus fly-through turns that the C++ version lacks. Waypoints with delay > 0, and the last waypoint, are stop points (as in C++). Waypoints with delay 0 are flown through: the mode sends Goto a "carrot" `lookahead + speed / 0.5` m further along the path than the drone, which slides round the corner. The `/ 0.5` is because Goto approaches its target at speed ≈ 0.5–0.6 × distance (PX4's `MPC_XY_TRAJ_P` 0.5), so a closer carrot makes the drone crawl. A fly-through waypoint counts as passed once the drone crosses the line bisecting the turn. `lookahead` (default 6 m) trades corner speed for corner cutting; see the docstring for measured values. **`max_speed` cannot exceed PX4's `MPC_XY_CRUISE`** (default 5 m/s): PX4's `GotoControl` clamps the Goto speed to it and scales `MPC_ACC_HOR` by `max_speed / MPC_XY_CRUISE`, so to fly at v set both to v (`px4-param set MPC_XY_CRUISE v`; it is saved, and also sets the speed of PX4's own Return/Mission). It runs two nodes in one process: an rclpy node `waypoint_mission` for parameters and logging (and the place for perception subscriptions), and the `px4_ros2.Node` `waypoint_mission_px4`, a C++ rclcpp node that talks to PX4 and whose `spin()` drives the mode and executor callbacks. The px4_ros2 node cannot read ROS parameters. On exit, delete the mode and executor before the px4_ros2 node or the process segfaults. After `completed()`, PX4 can still call `update_setpoint` until it switches mode, so the mode must keep sending a setpoint. Ctrl-C: rclcpp's handler shuts the px4_ros2 node down, so PX4 only failsafes if one of *our* modes was active; during PX4's own Takeoff it would just go to Hold and hover. So rclpy is initialised with `SignalHandlerOptions.NO` (its node survives Ctrl-C), the executor tracks `phase`, and if the program stops between arming and the final Return/Land it publishes `VEHICLE_CMD_NAV_RETURN_TO_LAUNCH` from the rclpy node on the way out (tested during Takeoff and during Waypoints). The "Failed to delete datareader" errors at exit are harmless. Testing Ctrl-C from a script: a background job (`cmd &`) in a non-interactive shell ignores SIGINT; use `timeout -s INT <s> ros2 run ...` in the foreground.
- `position_offboard_control.py` is a standalone example that takes off to 5 m in position mode and then lands. `position_mission_control.py` flies a waypoint list with a WARMUP/TRAVEL/WAIT/RTL state machine. Waypoints come from a CSV (rows of `north,east,down,delay_s`, NED metres) set by the `mission_file` parameter: a bare name such as `mission1.csv` is read from the package's installed `missions/` folder (default `mission_box.csv`), anything with a `/` is used as a path. No launch file uses either. `px4_offboard/test.py` is a scratch file.

## Gotchas

- The launch files export `DRONE_SIM_WINDOW=1` inside every `gnome-terminal` they open. `sim/kill_sim.sh` uses it to find those windows' shells (which end in `exec bash` and ignore SIGTERM) and closes them with SIGHUP. Keep the marker in any new terminal window.
- `gnome-terminal` fails with `symbol lookup error ... /snap/core20/...libpthread.so.0` when started from a snap app's environment (e.g. the VS Code snap's integrated terminal). Run the launch files from a normal terminal.
- `keyboard_teleop.launch.py` runs the keyboard node in a new `gnome-terminal`, because the node needs a TTY for raw `termios` input. That terminal sources `/opt/ros/$ROS_DISTRO/setup.bash` and this workspace's `install/setup.bash`, which it finds via `get_package_prefix`.
- In `teleop_keyboard.py`, the arrow-key escape codes `[C` and `[D` are assigned to the opposite labels, and the vy signs are flipped too, so the net motion is correct. Fix both together or neither.
- **Arming without QGroundControl:** in PX4 v1.17 the `gz_x500` airframe sets `NAV_DLL_ACT 2`, so PX4 denies arming ("Preflight Fail: No connection to the GCS") unless a GCS is connected. `minimal.launch.py` fixes this by waiting for `commander`, then running `px4-param set NAV_DLL_ACT 0` and `px4-param save`. `PX4_PARAM_NAV_DLL_ACT=0` does **not** work: `rcS` applies env overrides before the airframe script, 0 equals the firmware default so it isn't stored, and the airframe's `set-default` then wins.
- `velocity_offboard_control.py` logs "ARMED - Ready to fly!" without checking `arming_state`. To verify, echo `/fmu/out/vehicle_status_v1` (arming_state 2 = armed, nav_state 14 = offboard) or grep PX4's console for `Arming denied`.
- Flight logs are in `~/PX4-Autopilot/build/px4_sitl_default/rootfs/log/<UTC date>/`, named by UTC time; `pyulog` is installed. `vehicle_local_position_groundtruth` / `vehicle_attitude_groundtruth` give the true state, but are logged ~0.12 s late relative to the estimate, so shift them before comparing at speed.
- Headless testing: `sleep infinity | HEADLESS=1 make px4_sitl gz_my_x500_see3cam_down` from `~/PX4-Autopilot`. The idle stdin pipe stops the `pxh>` prompt from spinning; kill the `sleep` afterwards. PX4 client commands (`px4-param`, `px4-listener`, `px4-commander`) are in `~/PX4-Autopilot/build/px4_sitl_default/bin/` and talk to the running instance. Echoing `/fmu/out/*` with `ros2 topic echo` needs `--qos-reliability best_effort`.
