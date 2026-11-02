/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

// Added for PX4 v1.17 (see README.md): binds px4_ros2::TrajectorySetpointType and
// px4_ros2::TrajectorySetpoint (control/setpoint_types/experimental/trajectory.hpp).

#include "trajectory.hpp"

#include <array>
#include <optional>

#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <rclcpp/rclcpp.hpp>

namespace py = pybind11;

namespace
{
// A per-axis vector from Python: (x, y, z), where any entry may be None (axis not controlled)
using OptionalAxes = std::optional<std::array<std::optional<float>, 3>>;

void setAxes(
  const OptionalAxes& axes, std::optional<float>& x, std::optional<float>& y,
  std::optional<float>& z)
{
  if (axes) {
    x = (*axes)[0];
    y = (*axes)[1];
    z = (*axes)[2];
  }
}
}  // namespace

void bindSetpointTrajectory(py::module& m)
{
  using px4_ros2::TrajectorySetpoint;
  using px4_ros2::TrajectorySetpointType;

  // TrajectorySetpoint: every field optional; unset fields are not controlled (NaN to PX4)
  py::class_<TrajectorySetpoint> setpoint(m, "TrajectorySetpoint");
  setpoint
      .def(py::init([](const OptionalAxes& position, const OptionalAxes& velocity,
                       const OptionalAxes& acceleration, std::optional<float> yaw,
                       std::optional<float> yaw_rate) {
             TrajectorySetpoint sp;
             setAxes(position, sp.position_ned_m_x, sp.position_ned_m_y, sp.position_ned_m_z);
             setAxes(velocity, sp.velocity_ned_m_s_x, sp.velocity_ned_m_s_y,
                     sp.velocity_ned_m_s_z);
             setAxes(acceleration, sp.acceleration_ned_m_s2_x, sp.acceleration_ned_m_s2_y,
                     sp.acceleration_ned_m_s2_z);
             sp.yaw_ned_rad = yaw;
             sp.yaw_rate_ned_rad_s = yaw_rate;
             return sp;
           }),
           py::kw_only(), py::arg("position") = std::nullopt, py::arg("velocity") = std::nullopt,
           py::arg("acceleration") = std::nullopt, py::arg("yaw") = std::nullopt,
           py::arg("yaw_rate") = std::nullopt,
           "Per-axis NED setpoint. position/velocity/acceleration are (x, y, z) tuples whose "
           "entries may be None (axis not controlled); yaw [rad], yaw_rate [rad/s].")
      .def_readwrite("position_ned_m_x", &TrajectorySetpoint::position_ned_m_x)
      .def_readwrite("position_ned_m_y", &TrajectorySetpoint::position_ned_m_y)
      .def_readwrite("position_ned_m_z", &TrajectorySetpoint::position_ned_m_z)
      .def_readwrite("velocity_ned_m_s_x", &TrajectorySetpoint::velocity_ned_m_s_x)
      .def_readwrite("velocity_ned_m_s_y", &TrajectorySetpoint::velocity_ned_m_s_y)
      .def_readwrite("velocity_ned_m_s_z", &TrajectorySetpoint::velocity_ned_m_s_z)
      .def_readwrite("acceleration_ned_m_s2_x", &TrajectorySetpoint::acceleration_ned_m_s2_x)
      .def_readwrite("acceleration_ned_m_s2_y", &TrajectorySetpoint::acceleration_ned_m_s2_y)
      .def_readwrite("acceleration_ned_m_s2_z", &TrajectorySetpoint::acceleration_ned_m_s2_z)
      .def_readwrite("yaw_ned_rad", &TrajectorySetpoint::yaw_ned_rad)
      .def_readwrite("yaw_rate_ned_rad_s", &TrajectorySetpoint::yaw_rate_ned_rad_s);

  py::class_<TrajectorySetpointType, std::shared_ptr<TrajectorySetpointType>> trajectory(
      m, "TrajectorySetpointType");
  trajectory
      .def(py::init<px4_ros2::Context&, bool>(), py::arg("context"),
           py::arg("local_position_is_optional") = false)
      .def("update",
           py::overload_cast<const Eigen::Vector3f&, const std::optional<Eigen::Vector3f>&,
                             std::optional<float>, std::optional<float>>(
               &TrajectorySetpointType::update),
           py::arg("velocity"), py::arg("acceleration") = std::nullopt,
           py::arg("yaw") = std::nullopt, py::arg("yaw_rate") = std::nullopt,
           "Velocity [m/s] (NED), optional acceleration feed-forward [m/s^2], yaw [rad] or "
           "yaw_rate [rad/s]")
      .def("update",
           py::overload_cast<const TrajectorySetpoint&>(&TrajectorySetpointType::update),
           py::arg("setpoint"), "Any per-axis mix of position/velocity/acceleration/yaw")
      .def("update_position", &TrajectorySetpointType::updatePosition, py::arg("position"),
           "Position [m] (NED); prefer MulticopterGotoSetpointType for plain go-to-point");
}
