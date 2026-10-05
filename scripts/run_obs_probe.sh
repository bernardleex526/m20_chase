#!/usr/bin/env bash
# Reproduce the S8 geometry in the live world and probe what the obstacle field sees.
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:$GAZEBO_MODEL_PATH
pkill -f '[g]zserver'; sleep 1
gzserver "$HOME/m20_chase/gazebo/all_scenarios.sdf" >/tmp/gz_probe.log 2>&1 &
sleep 8
# S8: robot at origin, step 2.0 m ahead, target 4.5 m ahead
gz model -w all_scenarios -m target_person -x 4.5 -y 0 -z 0.85
gz model -w all_scenarios -m step_low -x 2.0 -y 0 -z 0.075
sleep 2
# Drive the robot forward 0.9 m (the distance at which S8 froze) then probe.
ros2 run rs_follow rs_follow_node --ros-args \
  -p input_topic:=/rslidar_points -p active:=false -p odom_topic:=/odom \
  -p height_min:=-0.40 -p height_max:=1.50 \
  -p enable_low_band:=true -p low_height_min:=-0.75 -p low_height_max:=-0.45 \
  -p cmd_vel_topic:=/cmd_vel -p auto_select_front:=false \
  > /tmp/probe_node.log 2>&1 &
NPID=$!
sleep 5
python3 - <<'PY' &
import time, rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool
rclpy.init(); n = Node("drive")
bp = n.create_publisher(PointStamped, "/rs_follow/bind_target", 10)
ep = n.create_publisher(Bool, "/rs_follow/enable", 10)
time.sleep(1)
p = PointStamped(); p.header.frame_id = "rslidar"; p.point.x = 4.5
for _ in range(5):
    bp.publish(p); time.sleep(0.1)
ep.publish(Bool(data=True))
time.sleep(9)          # let it drive forward
ep.publish(Bool(data=False))
time.sleep(0.5)
rclpy.shutdown()
PY
sleep 11
python3 "$HOME/obs_probe.py" 30
kill $NPID 2>/dev/null
pkill -f '[g]zserver'
echo "=== done ==="
