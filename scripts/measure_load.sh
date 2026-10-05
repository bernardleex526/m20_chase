#!/usr/bin/env bash
# Targeted process accounting for the follow pipeline.
source /opt/ros/humble/setup.bash
source "$HOME/m20_chase/install/setup.bash"
export ROS_DOMAIN_ID=42
pkill -f '[g]zserver'; pkill -f '[r]s_follow_node'; sleep 1

gzserver /mnt/e/小说/new/m20_chase_verify/gazebo/follow_test.sdf >/tmp/res_gz.log 2>&1 &
sleep 7

stdbuf -oL ros2 run rs_follow rs_follow_node --ros-args \
  -p input_topic:=/rslidar_points -p cmd_vel_topic:=/cmd_vel \
  -p odom_topic:=/odom -p publish_scan_debug:=true -p active:=false \
  >/tmp/res_node.log 2>&1 &
sleep 6

# The `ros2 run` wrapper is a python process; the real node is the child binary.
NODE_PID=$(pgrep -f 'lib/rs_follow/rs_follow_node' | head -1)
[[ -z "$NODE_PID" ]] && NODE_PID=$(pgrep -f 'rs_follow_node --ros-args' | tail -1)
echo "node pid = $NODE_PID"
echo "cmdline  = $(tr '\0' ' ' < /proc/$NODE_PID/cmdline 2>/dev/null | cut -c1-160)"

python3 /home/m20/res_drive.py >/tmp/res_driver.log 2>&1 &
DRV=$!
sleep 3

echo
echo "=== per-process (utime+stime over 10 s wall) ==="
read u0 s0 <<< $(awk '{print $14, $15}' /proc/$NODE_PID/stat)
GZ_PID=$(pgrep -x gzserver | head -1)
read gu0 gs0 <<< $(awk '{print $14, $15}' /proc/$GZ_PID/stat)
HZ=$(getconf CLK_TCK)
T0=$(date +%s.%N)
sleep 10
T1=$(date +%s.%N)
read u1 s1 <<< $(awk '{print $14, $15}' /proc/$NODE_PID/stat)
read gu1 gs1 <<< $(awk '{print $14, $15}' /proc/$GZ_PID/stat)
DT=$(echo "$T1 - $T0" | bc)

NODE_CPU=$(echo "scale=2; (($u1+$s1)-($u0+$s0))/$HZ/$DT*100" | bc)
GZ_CPU=$(echo "scale=2; (($gu1+$gs1)-($gu0+$gs0))/$HZ/$DT*100" | bc)
echo "rs_follow_node  CPU = ${NODE_CPU} % of one core  (100% = 1 core saturated)"
echo "gzserver        CPU = ${GZ_CPU} % of one core"

echo
echo "=== RSS (VmRSS) ==="
printf "rs_follow_node  "; grep VmRSS /proc/$NODE_PID/status
printf "gzserver        "; grep VmRSS /proc/$GZ_PID/status

echo
echo "=== threads ==="
printf "rs_follow_node  "; grep Threads /proc/$NODE_PID/status
printf "gzserver        "; grep Threads /proc/$GZ_PID/status

echo
echo "=== rates ==="
for t in /rslidar_points /rs_follow/scan /cmd_vel /model_states /rs_follow/target; do
  r=$(timeout 6 ros2 topic hz $t 2>/dev/null | grep -m1 'average rate' | awk '{print $3}')
  echo "  $t  ${r:-n/a} Hz"
done

echo
echo "=== node log tail ==="
tail -6 /tmp/res_node.log

echo
echo "=== all matching pids ==="
ps -eo pid,pcpu,rss,comm,args | grep -E 'rs_follow_node' | grep -v grep | cut -c1-140

kill $DRV 2>/dev/null
pkill -f '[r]s_follow_node'
pkill -f '[g]zserver'
echo "=== done ==="
