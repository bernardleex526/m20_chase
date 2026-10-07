#!/usr/bin/env bash
# Run the obstacle-placement matrix against the live rs_follow_node.
# No Gazebo: synthetic PointCloud2 is published straight at the node, so the
# obstacle position is exact and each case settles in ~1 s instead of ~30 s.
source /opt/ros/humble/setup.bash
source "$HOME/m20_chase/install/setup.bash"
export ROS_DOMAIN_ID=42
export RCUTILS_LOGGING_BUFFERED_STREAM=1

OUT="$HOME/m20_chase/evidence"
mkdir -p "$OUT"
RIG="$HOME/obs_matrix2.py"
[ -f "$RIG" ] || RIG=/home/m20/obs_matrix2.py

cleanup() {
  [[ -n "${FOPID:-}" ]] && kill $FOPID 2>/dev/null
  sleep 1
  [[ -n "${FOPID:-}" ]] && kill -9 $FOPID 2>/dev/null
}
trap cleanup EXIT

echo "=== start rs_follow_node (Gazebo NOT started) ==="
ros2 run rs_follow rs_follow_node --ros-args \
  -p input_topic:=/rslidar_points \
  -p active:=false \
  -p odom_topic:=/no_odom \
  -p compensate_slip:=false \
  -p follow_dist:=1.0 \
  -p height_min:=-0.60 \
  -p height_max:=1.50 \
  -p cmd_vel_topic:=/cmd_vel \
  -p auto_select_front:=false \
  -p enable_lateral:=true \
  > "$OUT/obs_matrix2_node.log" 2>&1 &
FOPID=$!
sleep 5
kill -0 $FOPID 2>/dev/null || { echo "!! node died"; cat "$OUT/obs_matrix2_node.log"; exit 1; }

echo "=== run matrix ==="
python3 "$RIG" 2>&1 | tee "$OUT/obs_matrix2.txt"
echo "=== DONE ==="
