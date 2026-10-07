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

# The algorithm itself. Plain CMake, zero ROS dependency -- see
# src/rs_follow_core. Everything this node used to do internally now lives
# there, so the same code can be driven from a non-ROS upper computer, from a
# unit test with no middleware, or from Python via pybind11.
find_package(rs_follow_core REQUIRED)

include_directories(include)

# ---------------------------------------------------------------------------
# The node is a thin ROS adapter over rs_follow_core.
# ---------------------------------------------------------------------------
add_executable(rs_follow_node src/rs_follow_node.cpp)

target_include_directories(rs_follow_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(rs_follow_node rs_follow_core::rs_follow_core)

ament_target_dependencies(rs_follow_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  std_msgs)

install(TARGETS rs_follow_node
  DESTINATION lib/${PROJECT_NAME})

# ---------------------------------------------------------------------------
# Behavioural tests.
#
# These drive FollowController directly with hand-built scans. They now link
# against rs_follow_core and need NO ROS runtime, so a safety regression is
# caught in milliseconds instead of in an eight-minute simulation sweep.
#
#     colcon build --packages-select rs_follow --cmake-args -DBUILD_TESTING=ON
#     ./build/rs_follow/test_safety
#     ./build/rs_follow/test_autoselect
#     ./build/rs_follow/test_governor_curve
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  # The safety layer: side walls must not freeze a forward drive, a doorway must
  # stay passable, an obstacle dead ahead must limit speed, the low band must see
  # a step, and the followed person must never count as an obstacle.
  add_executable(test_safety test/test_safety.cpp)
  target_include_directories(test_safety PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_link_libraries(test_safety rs_follow_core::rs_follow_core)

  # Auto-select must pick a person, not a wall (it once locked a trap's back
  # wall and held station on it while the person stood 4.5 m away).
  add_executable(test_autoselect test/test_autoselect.cpp)
  target_include_directories(test_autoselect PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_link_libraries(test_autoselect rs_follow_core::rs_follow_core)

  # Prints the speed-vs-clearance curve, so the safety margins are chosen from
  # numbers instead of by trial and error in simulation.
  add_executable(test_governor_curve test/test_governor_curve.cpp)
  target_include_directories(test_governor_curve PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_link_libraries(test_governor_curve rs_follow_core::rs_follow_core)
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
