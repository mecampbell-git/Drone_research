cmake_minimum_required(VERSION 3.8)
project(px4_missions)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(px4_ros2_cpp REQUIRED)

add_executable(waypoint_mission src/waypoint_mission.cpp)
target_link_libraries(waypoint_mission PUBLIC
  Eigen3::Eigen px4_ros2_cpp::px4_ros2_cpp rclcpp::rclcpp ament_index_cpp::ament_index_cpp)

install(TARGETS waypoint_mission DESTINATION lib/${PROJECT_NAME})

ament_package()
