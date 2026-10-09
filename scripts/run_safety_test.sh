#!/usr/bin/env bash
# Fast behavioural test for the safety layer: compiles test_safety.cpp against
# the headers and runs it. No Gazebo, no ROS runtime, no colcon build needed.
set -euo pipefail
REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
H="$REPO_ROOT/src/rs_follow/include"
T="$REPO_ROOT/src/rs_follow/test"
ROS_PREFIX="${ROS_PREFIX:-/opt/ros/humble}"
OUT="$(mktemp /tmp/rs_follow_test_safety.XXXXXX)"
trap 'rm -f -- "$OUT"' EXIT
# ROS installs package headers in separate immediate include directories.
# Include the whole installed header closure rather than maintaining a partial list.
INCLUDES=(-I"$H" -I"$ROS_PREFIX/include")
for dir in "$ROS_PREFIX"/include/*; do
  [[ -d "$dir" ]] && INCLUDES+=(-I"$dir")
done
g++ -std=c++17 -O1 -Wall -Wextra "${INCLUDES[@]}" \
  "$T/test_safety.cpp" -o "$OUT" \
  -L"$ROS_PREFIX/lib" -Wl,-rpath,"$ROS_PREFIX/lib" -lrclcpp -lrcl -lrcutils
"$OUT"