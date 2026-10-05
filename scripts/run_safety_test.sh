#!/usr/bin/env bash
# Fast behavioural test for the safety layer: compiles test_safety.cpp against
# the headers and runs it. No Gazebo, no ROS runtime, no colcon build needed.
set -e
H="$HOME/m20_chase/src/rs_follow/include"
T="$HOME/m20_chase/src/rs_follow/test"
OUT=/tmp/test_safety
g++ -std=c++17 -O1 -Wall -Wextra \
  -I"$H" -I/opt/ros/humble/include/rclcpp -I/opt/ros/humble/include/rcutils \
  -I/opt/ros/humble/include/rcl -I/opt/ros/humble/include/rmw \
  -I/opt/ros/humble/include/rosidl_runtime_c \
  -I/opt/ros/humble/include/rosidl_typesupport_interface \
  -I/opt/ros/humble/include/builtin_interfaces \
  -I/opt/ros/humble/include/sensor_msgs \
  -I/opt/ros/humble/include/std_msgs \
  -I/opt/ros/humble/include/geometry_msgs \
  -I/opt/ros/humble/include/rosidl_runtime_cpp \
  -I/opt/ros/humble/include/rosidl_typesupport_introspection_cpp \
  -I/opt/ros/humble/include/rosidl_dynamic_typesupport \
  -I/opt/ros/humble/include/rcl_yaml_param_parser \
  -I/opt/ros/humble/include/ament_index_cpp \
  -I/opt/ros/humble/include/libstatistics_collector \
  -I/opt/ros/humble/include/statistics_msgs \
  -I/opt/ros/humble/include/tracetools \
  -I/opt/ros/humble/include/rcl_interfaces \
  -I/opt/ros/humble/include/rcl_logging_interface \
  -I/opt/ros/humble/include/service_msgs \
  -I/opt/ros/humble/include/type_description_interfaces \
  -I/opt/ros/humble/include/rosgraph_msgs \
  "$T/test_safety.cpp" -o "$OUT" 2>&1 | head -30
"$OUT"