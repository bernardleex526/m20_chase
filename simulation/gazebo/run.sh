#!/usr/bin/env bash
set -eo pipefail
REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
FIXTURES="$REPO_ROOT/simulation/gazebo"
SIM_WS="${SIM_WS:-$HOME/.cache/m20_chase/gazebo_ws}"
source /opt/ros/humble/setup.bash
source "$SIM_WS/install/setup.bash"
source "$REPO_ROOT/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-73}"
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11374}"
out="${1:-$HOME/.cache/m20_chase/gazebo-results}"
mkdir -p "$out"
setsid ros2 launch "$FIXTURES/dynamics.launch.py" > "$out/gazebo.log" 2>&1 &
sim_pid=$!
trap 'kill -INT -- "-$sim_pid" 2>/dev/null || true; wait "$sim_pid" || true' EXIT
python3 "$FIXTURES/scenarios.py" --output-dir "$out"
