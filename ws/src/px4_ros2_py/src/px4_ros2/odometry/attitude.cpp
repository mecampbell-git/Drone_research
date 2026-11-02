/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

// Added for PX4 v1.17 (see README.md)

#include "attitude.hpp"

#include <pybind11/functional.h>

#include <px4_ros2/odometry/attitude.hpp>
#include <rclcpp/rclcpp.hpp>

namespace py = pybind11;

void bindAttitude(py::module& m)
{
  py::class_<px4_ros2::OdometryAttitude, std::shared_ptr<px4_ros2::OdometryAttitude>> attitude(
      m, "Attitude");

  attitude
      .def("on_update",
           [](px4_ros2::OdometryAttitude& self, const std::function<void()>& callback) {
             // We don't pass the message, otherwise we'd have to register the type as well
             self.onUpdate([callback](const auto& msg) { callback(); });
           })
      .def(
          "attitude",
          [](const px4_ros2::OdometryAttitude& self) {
            // There is no automatic type conversion for Eigen::Quaternion, so we use a tuple
            const auto q = self.attitude();
            return py::make_tuple(q.w(), q.x(), q.y(), q.z());
          },
          "Attitude quaternion (w, x, y, z), body FRD to NED")
      .def("roll", &px4_ros2::OdometryAttitude::roll, "Roll [rad]")
      .def("pitch", &px4_ros2::OdometryAttitude::pitch, "Pitch [rad]")
      .def("yaw", &px4_ros2::OdometryAttitude::yaw, "Yaw [rad]");

  // Constructor
  attitude.def(py::init<px4_ros2::Context&>(), py::arg("context"));
}
