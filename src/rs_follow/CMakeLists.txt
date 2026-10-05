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

# Behavioural test for the safety layer. Runs on synthetic scans with no Gazebo
# and no ROS runtime, so a safety regression is caught in milliseconds instead
# of in an eight-minute simulation sweep. Build and run with:
#     colcon build --packages-select rs_follow --cmake-args -DBUILD_TESTING=ON
#     ./build/rs_follow/test_safety
if(BUILD_TESTING)
  add_executable(test_safety test/test_safety.cpp)
  target_include_directories(test_safety PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  ament_target_dependencies(test_safety
    rclcpp
    sensor_msgs
    geometry_msgs
    nav_msgs
    visualization_msgs
    std_msgs)

  # Auto-select must pick a person, not a wall (it once locked a trap's back
  # wall and held station on it while the person stood 4.5 m away).
  add_executable(test_autoselect test/test_autoselect.cpp)
  target_include_directories(test_autoselect PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  ament_target_dependencies(test_autoselect
    rclcpp
    sensor_msgs
    geometry_msgs
    nav_msgs
    visualization_msgs
    std_msgs)

  # Prints the speed-vs-clearance curve, so the safety margins are chosen from
  # numbers instead of by trial and error in simulation.
  add_executable(test_governor_curve test/test_governor_curve.cpp)
  target_include_directories(test_governor_curve PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
endif()

install(PROGRAMS
  scripts/pointcloud_status.py
  scripts/follow_scenario_test.py
  scripts/control_smoothness_check.py
  scripts/follow_sim_visualize.py
  scripts/follow_pysim.py
  scripts/web_target_ui.py
  scripts/web_ui.py
  scripts/ws_server.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config docs web
  DESTINATION share/${PROJECT_NAME})

ament_package()
