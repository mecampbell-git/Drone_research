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
  - `odometry.Attitude` (`px4_ros2::OdometryAttitude`, not bound upstream at all): `on_update`,
    `attitude` (quaternion as a `(w, x, y, z)` tuple, like `utils`), `roll`, `pitch`, `yaw`.
  - `Node.shutdown()`, which makes `spin()` return.
  - `control.TrajectorySetpointType` (`experimental/trajectory.hpp` in C++): `update(velocity,
    acceleration=None, yaw=None, yaw_rate=None)`, `update(setpoint)` and
    `update_position(position)`; and `control.TrajectorySetpoint(*, position=None, velocity=None,
    acceleration=None, yaw=None, yaw_rate=None)`, where each vector is an (x, y, z) tuple whose
    entries may be None (axis not controlled), plus read/write fields `position_ned_m_x` etc.
    Unlike Goto it is not capped by `MPC_XY_CRUISE` (only `MPC_XY_VEL_MAX`); flight-tested at
    7 m/s with velocity XY + position Z, acceleration feed-forward and position hold.
- **Changed** `Node.spin()` to release the GIL while it blocks, so other Python threads (e.g. an
  rclpy executor for perception subscriptions) keep running. Upstream holds it, which starves them.

Drop this package when upgrading to a PX4 release whose interface-library branch ships
`px4_ros2_py` with these features.
