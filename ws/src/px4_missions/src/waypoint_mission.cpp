/****************************************************************************
 * Waypoint mission as a PX4 ROS 2 custom mode.
 *
 * The "Waypoints" mode flies the waypoints of a mission CSV (rows of north,east,down,delay_s
 * in the local NED frame, same format as px4_offboard's position_mission_control.py) using
 * PX4's Goto setpoint, so PX4's own smoother plans each leg. The mode executor runs the
 * whole flight: arm, PX4 Takeoff, Waypoints, PX4 Land, then exits once disarmed.
 *
 * Parameters:
 *   mission_file   bare name: read from px4_offboard's installed missions/ folder; anything
 *                  containing '/' is used as a path (default mission_box.csv)
 *   max_speed      horizontal speed limit [m/s] (default 5.0)
 *   acceptance_radius  horizontal distance counted as "reached" [m] (default 0.5)
 *   autostart      true: start the flight as soon as PX4 accepts the mode (default).
 *                  false: wait until the mode is selected (QGC, RC, or a vehicle command).
 ****************************************************************************/

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/control/setpoint_types/multicopter/goto.hpp>
#include <px4_ros2/odometry/local_position.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

struct Waypoint
{
  Eigen::Vector3f position_ned;  // [m]
  float delay_s;
};

static std::vector<Waypoint> loadMission(const std::string & path)
{
  std::ifstream file(path);
  if (!file) {
    throw std::runtime_error("Cannot open mission file " + path);
  }
  std::vector<Waypoint> mission;
  std::string line;
  while (std::getline(file, line)) {
    if (line.find_first_not_of(" \t\r") == std::string::npos) {
      continue;
    }
    std::stringstream row(line);
    std::string cell;
    std::vector<float> values;
    while (std::getline(row, cell, ',')) {
      values.push_back(std::stof(cell));
    }
    if (values.size() != 4) {
      throw std::runtime_error("Expected north,east,down,delay in " + path + ": " + line);
    }
    mission.push_back({{values[0], values[1], values[2]}, values[3]});
  }
  if (mission.empty()) {
    throw std::runtime_error("No waypoints in " + path);
  }
  return mission;
}

class WaypointMode : public px4_ros2::ModeBase
{
public:
  explicit WaypointMode(rclcpp::Node & node)
  : ModeBase(node, Settings{"Waypoints"})
  {
    std::string file = node.declare_parameter<std::string>("mission_file", "mission_box.csv");
    _max_speed = node.declare_parameter<double>("max_speed", 5.0);
    _acceptance_radius = node.declare_parameter<double>("acceptance_radius", 0.5);

    if (file.find('/') == std::string::npos) {
      file = ament_index_cpp::get_package_share_directory("px4_offboard") + "/missions/" + file;
    }
    _mission = loadMission(file);
    RCLCPP_INFO(node.get_logger(), "Loaded %zu waypoints from %s", _mission.size(), file.c_str());

    _goto_setpoint = std::make_shared<px4_ros2::MulticopterGotoSetpointType>(*this);
    _local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
  }

  void onActivate() override
  {
    _index = 0;
    _waiting = false;
    _heading = _local_position->heading();
    _leg_start = node().get_clock()->now();
  }

  void onDeactivate() override {}

  void updateSetpoint(float dt_s) override
  {
    // After completed(), PX4 may still call this until it switches mode: hold the last point
    if (_index == _mission.size()) {
      _goto_setpoint->update(_mission.back().position_ned, _heading);
      return;
    }
    const Waypoint & wp = _mission[_index];
    const Eigen::Vector3f error = wp.position_ned - _local_position->positionNed();
    const float error_xy = error.head<2>().norm();

    // Face the direction of travel; hold the last heading close to the waypoint, where the
    // direction to it becomes noisy.
    if (error_xy > 1.f) {
      _heading = std::atan2(error.y(), error.x());
    }
    _goto_setpoint->update(wp.position_ned, _heading, _max_speed);

    const rclcpp::Time now = node().get_clock()->now();

    if (!_waiting) {
      const bool reached = error_xy < _acceptance_radius && std::fabs(error.z()) < 0.3f &&
        _local_position->velocityNed().norm() < 0.3f;
      if (!reached) {
        return;
      }
      RCLCPP_INFO(
        node().get_logger(), "Reached waypoint %zu (%.1f, %.1f, %.1f) after %.1f s",
        _index, wp.position_ned.x(), wp.position_ned.y(), wp.position_ned.z(),
        (now - _leg_start).seconds());
      _waiting = true;
      _wait_start = now;
    }

    if ((now - _wait_start).seconds() < wp.delay_s) {
      return;
    }

    _waiting = false;
    _leg_start = now;
    if (++_index == _mission.size()) {
      completed(px4_ros2::Result::Success);
    }
  }

private:
  std::vector<Waypoint> _mission;
  float _max_speed;
  float _acceptance_radius;
  size_t _index{0};
  bool _waiting{false};
  float _heading{0.f};
  rclcpp::Time _leg_start;
  rclcpp::Time _wait_start;
  std::shared_ptr<px4_ros2::MulticopterGotoSetpointType> _goto_setpoint;
  std::shared_ptr<px4_ros2::OdometryLocalPosition> _local_position;
};

class MissionExecutor : public px4_ros2::ModeExecutorBase
{
public:
  explicit MissionExecutor(px4_ros2::ModeBase & owned_mode)
  : ModeExecutorBase(settings(owned_mode.node()), owned_mode),
    _node(owned_mode.node())
  {
  }

  enum class State
  {
    WaitReadyToArm,
    Arming,
    TakingOff,
    Waypoints,
    Landing,
    WaitUntilDisarmed,
  };

  void onActivate() override
  {
    runState(State::WaitReadyToArm, px4_ros2::Result::Success);
  }

  void onDeactivate(DeactivateReason reason) override
  {
    // A failsafe or a mode switch (RC, QGC) took over. PX4 now flies; just report it.
    RCLCPP_WARN(
      _node.get_logger(), "Mission interrupted (%s)",
      reason == DeactivateReason::FailsafeActivated ? "failsafe" : "mode changed");
  }

  void runState(State state, px4_ros2::Result previous_result)
  {
    if (previous_result != px4_ros2::Result::Success) {
      RCLCPP_ERROR(
        _node.get_logger(), "Mission aborted before state %i: %s", static_cast<int>(state),
        resultToString(previous_result));
      return;
    }

    switch (state) {
      case State::WaitReadyToArm:
        RCLCPP_INFO(_node.get_logger(), "Waiting until PX4 is ready to arm");
        waitReadyToArm([this](px4_ros2::Result r) {runState(State::Arming, r);});
        break;

      case State::Arming:
        arm([this](px4_ros2::Result r) {runState(State::TakingOff, r);});
        break;

      case State::TakingOff:
        RCLCPP_INFO(_node.get_logger(), "Taking off");
        takeoff([this](px4_ros2::Result r) {runState(State::Waypoints, r);});
        break;

      case State::Waypoints:
        RCLCPP_INFO(_node.get_logger(), "Flying waypoints");
        scheduleMode(ownedMode().id(), [this](px4_ros2::Result r) {runState(State::Landing, r);});
        break;

      case State::Landing:
        RCLCPP_INFO(_node.get_logger(), "Mission complete, landing");
        land([this](px4_ros2::Result r) {runState(State::WaitUntilDisarmed, r);});
        break;

      case State::WaitUntilDisarmed:
        waitUntilDisarmed(
          [this](px4_ros2::Result r) {
            RCLCPP_INFO(_node.get_logger(), "Landed and disarmed (%s)", resultToString(r));
            rclcpp::shutdown();
          });
        break;
    }
  }

private:
  static Settings settings(rclcpp::Node & node)
  {
    const bool autostart = node.declare_parameter<bool>("autostart", true);
    return Settings{}.activate(
      autostart ? Settings::Activation::ActivateImmediately :
      Settings::Activation::ActivateAlways);
  }

  rclcpp::Node & _node;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<px4_ros2::NodeWithModeExecutor<MissionExecutor, WaypointMode>>(
      "waypoint_mission"));
  rclcpp::shutdown();
  return 0;
}
