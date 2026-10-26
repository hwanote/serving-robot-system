#!/usr/bin/env bash
# Send one NavigateToPose goal (map frame).   usage: goto.sh X Y [YAW_DEG]
#   bash ~/serving-robot-system/scripts/dev/goto.sh 0.0 -2.4 0
set -eu
[ $# -ge 2 ] || { echo "usage: $0 X Y [YAW_DEG]"; exit 1; }
X=$1; Y=$2; YAW=${3:-0}
set +u   # ROS setup scripts read unset variables
source /opt/ros/jazzy/setup.bash
set -u
read -r QZ QW < <(python3 -c "import math;a=math.radians($YAW)/2;print(math.sin(a),math.cos(a))")
echo "goal: x=$X y=$Y yaw=${YAW}deg"
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: $X, y: $Y, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: $QZ, w: $QW}}}}"
