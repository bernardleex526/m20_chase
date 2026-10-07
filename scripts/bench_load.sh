#!/usr/bin/env bash
# Realistic-load benchmark: synthetic RoboSense-sized clouds at rs_follow.
# gzserver is NOT needed here - bench_cloud.py replaces the sensor, so the only
# CPU measured is the one the algorithm itself would burn on the robot.
source /opt/ros/humble/setup.bash
source "$HOME/m20_chase/install/setup.bash"
export ROS_DOMAIN_ID=42
pkill -f '[r]s_follow_node'; pkill -f '[g]zserver'; pkill -f '[b]ench_cloud'; sleep 1

run_case() {
  local N=$1 HZ=$2 SECS=$3
  echo
  echo "############ $N points @ $HZ Hz ############"

  stdbuf -oL ros2 run rs_follow rs_follow_node --ros-args \
    -p input_topic:=/rslidar_points -p cmd_vel_topic:=/cmd_vel \
    -p odom_topic:=/odom -p publish_scan_debug:=true -p active:=false \
    >/tmp/bench_node.log 2>&1 &
  sleep 4

  python3 /home/m20/bench_cloud.py "$N" "$SECS" "$HZ" >/tmp/bench_pub.log 2>&1 &
  PUB=$!
  sleep 3

  local PID
  PID=$(pgrep -f 'lib/rs_follow/rs_follow_node' | head -1)
  if [[ -z "$PID" ]]; then echo "!! node not running"; cat /tmp/bench_node.log; return; fi

  local HZ_CLK T0 T1
  HZ_CLK=$(getconf CLK_TCK)
  read -r u0 s0 <<< "$(awk '{print $14, $15}' /proc/$PID/stat)"
  T0=$(date +%s.%N)
  wait $PUB 2>/dev/null
  T1=$(date +%s.%N)
  read -r u1 s1 <<< "$(awk '{print $14, $15}' /proc/$PID/stat)"

  local DT CPU
  DT=$(echo "$T1 - $T0" | bc)
  CPU=$(echo "scale=1; (($u1+$s1)-($u0+$s0))/$HZ_CLK/$DT*100" | bc)
  echo "wall ${DT}s   rs_follow_node CPU = ${CPU} % of one core"
  grep VmRSS /proc/$PID/status | sed 's/^/  /'
  grep Threads /proc/$PID/status | sed 's/^/  /'
  echo "  node log:"; tail -2 /tmp/bench_node.log | sed 's/^/    /'
  echo "  publisher: $(cat /tmp/bench_pub.log | tail -1)"

  pkill -f '[r]s_follow_node'; sleep 1
}

run_case 6144   10 12      # what Gazebo actually delivers (ray sensor)
run_case 32768  10 12      # RoboSense RS-Helios 16-class density
run_case 131072 10 12      # RoboSense Ruby / high-density 128-beam

pkill -f '[b]ench_cloud'
echo "=== done ==="
