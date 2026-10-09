#!/usr/bin/env bash
set -eo pipefail
REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
SIM_WS="${SIM_WS:-$HOME/.cache/m20_chase/gazebo_ws}"
ws="$SIM_WS"
if [[ "$ws" == "$REPO_ROOT" || "$ws" == "$REPO_ROOT/"* ]]; then
  echo 'SIM_WS must be outside the repository (build artifacts are not source).' >&2
  exit 1
fi
if ! sudo apt-get install --no-remove -y libdw-dev libelf-dev gazebo ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-robot-localization ros-humble-xacro ros-humble-velodyne-description ros-humble-velodyne-gazebo-plugins; then
  echo 'Dependency installation failed. Check enabled Humble/Gazebo apt repositories and matching libdw/libelf runtime/dev versions; no downgrade/removal is requested. Resolve apt candidates before retrying.' >&2
  exit 1
fi
mkdir -p "$ws/src"
if [[ ! -d "$ws/src/champ/.git" ]]; then
  git clone --branch ros2 https://github.com/chvmp/champ.git "$ws/src/champ"
fi
git -C "$ws/src/champ" checkout 049b33ccaf77847bbea168cbb0a11c260ce119cd
git -C "$ws/src/champ" submodule update --init --recursive
source /opt/ros/humble/setup.bash
cd "$ws"
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
