# px4_ros2_py (vendored, patched for PX4 v1.17)

Python bindings for `px4_ros2_cpp`, copied from
[Auterion/px4-ros2-interface-lib](https://github.com/Auterion/px4-ros2-interface-lib) `main`
(commit `ac2e8ae`, 2026-09-25), because the `release/1.17` branch used here has no Python
package. Built against `ws/src/px4-ros2-interface-lib` (`release/1.17`).

Changes from upstream:

- **Adapted** the `ModeBase` trampoline: in 1.17 `on_activate`/`on_deactivate` are pure virtual and
  `update_setpoint` is not (the reverse of `main`).
- **Removed** (not in the 1.17 C++ library): `ModeBase.on_failsafe_deferred`,
  `ModeBase.defer_failsafes`, and the `user_selectable` constructor argument.
- **Added** (exist in 1.17 C++, not yet bound upstream):
  - `ModeExecutorBase`: `arm`, `disarm`, `wait_ready_to_arm`, `wait_until_disarmed`, and an
    `activation` constructor argument (`components.ModeExecutorActivation.only_when_armed`,
    `.always`, `.immediately`).
  - `odometry.LocalPosition`: `velocity_ned`, `velocity_xy_valid`, `velocity_z_valid`, `heading`.
  - `Node.shutdown()`, which makes `spin()` return.

Not yet bound, wanted later (see `docs/open-questions.md` Q10):

- `px4_ros2::TrajectorySetpointType` (`control/setpoint_types/experimental/trajectory.hpp`):
  position/velocity/acceleration/yaw setpoints, for precise velocity control in custom modes.
  Bind `update(velocity, acceleration, yaw, yaw_rate)` and the `TrajectorySetpoint` struct overload
  (all fields optional), following `control/setpoint_types/multicopter/goto.cpp`, and register it
  in `bind_main.cpp` under `control`.

Drop this package when upgrading to a PX4 release whose interface-library branch ships
`px4_ros2_py` with these features.
