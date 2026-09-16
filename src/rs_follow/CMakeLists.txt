cmake_minimum_required(VERSION 3.8)
project(rs_follow)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(rs_follow_node src/rs_follow_node.cpp)

target_include_directories(rs_follow_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(rs_follow_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  std_msgs)

install(TARGETS rs_follow_node
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/pointcloud_status.py
  scripts/follow_scenario_test.py
  scripts/control_smoothness_check.py
  scripts/follow_sim_visualize.py
  scripts/follow_pysim.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config docs
  DESTINATION share/${PROJECT_NAME})

ament_package()
