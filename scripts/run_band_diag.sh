#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:$GAZEBO_MODEL_PATH
pkill -f '[g]zserver'; sleep 1
gzserver "$HOME/m20_chase/gazebo/all_scenarios.sdf" >/tmp/gz3.log 2>&1 &
sleep 8
# put the target 4 m ahead, everything else parked (the S1 layout)
gz model -w all_scenarios -m target_person -x 4.0 -y 0.0 -z 0.85
sleep 1
python3 "$HOME/band_diag.py"
pkill -f '[g]zserver'
echo "=== done ==="
