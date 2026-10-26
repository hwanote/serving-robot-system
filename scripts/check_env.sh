#!/usr/bin/env bash
# Environment check for serving-robot-system (Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic).
# Run it after installing the packages and after building the workspace:
#   source /opt/ros/jazzy/setup.bash
#   source ~/serving-robot-system/ros_ws/install/setup.bash
#   bash ~/serving-robot-system/scripts/check_env.sh
# Prints [OK]/[FAIL] per item with a hint on how to fix a failure.
set -u
ok=0; ng=0
pass() { echo "  [OK]   $*"; ok=$((ok+1)); }
fail() { echo "  [FAIL] $*"; ng=$((ng+1)); }

echo "== 1. 운영체제 / ROS"
. /etc/os-release 2>/dev/null
if [ "${VERSION_ID:-}" = "24.04" ]; then pass "Ubuntu 24.04"; else fail "Ubuntu ${VERSION_ID:-?} (권장: 24.04)"; fi
if [ "${ROS_DISTRO:-}" = "jazzy" ]; then pass "ROS_DISTRO=jazzy"; else fail "ROS가 불러와지지 않았어요 -> source /opt/ros/jazzy/setup.bash"; fi

echo "== 2. Gazebo"
if command -v gz >/dev/null 2>&1; then
  v=$(gz sim --versions 2>/dev/null | head -1)
  pass "gz 명령 있음 (${v:-version unknown})"
else
  fail "gz 명령이 없어요 -> sudo apt install ros-jazzy-ros-gz"
fi

echo "== 3. 필요한 ROS 패키지"
for p in nav2_bringup nav2_amcl nav2_map_server nav2_controller nav2_planner \
         nav2_behaviors nav2_bt_navigator nav2_velocity_smoother \
         nav2_rotation_shim_controller nav2_regulated_pure_pursuit_controller \
         nav2_lifecycle_manager nav2_costmap_2d nav2_navfn_planner \
         ros_gz_sim ros_gz_bridge robot_state_publisher xacro rviz2 \
         tf2_ros_py lifecycle_msgs; do
  if ros2 pkg prefix "$p" >/dev/null 2>&1; then pass "$p"; else fail "$p -> rosdep install 로 설치 (README 참고)"; fi
done

echo "== 4. 이 프로젝트 패키지 (빌드 후)"
for p in serving_robot_description serving_robot_gazebo serving_robot_navigation \
         serving_robot_bringup serving_robot_interfaces serving_robot_perception; do
  if ros2 pkg prefix "$p" >/dev/null 2>&1; then pass "$p"
  else fail "$p -> cd ros_ws && colcon build --symlink-install && source install/setup.bash"; fi
done

echo "== 5. 남아 있는 이전 실행 (있으면 시간이 뒤섞여 TF가 꼬여요)"
left=$(pgrep -af "gz sim|ruby|parameter_bridge|nav2_|amcl|map_server|rviz2" 2>/dev/null | grep -v -E "pgrep|check_env" || true)
if [ -z "$left" ]; then pass "남은 프로세스 없음"
else fail "남은 프로세스가 있어요 (모두 종료한 뒤 다시 시작하세요: docs/RUN_GUIDE.md '정리')"; echo "$left" | sed 's/^/         /'; fi

echo "----"
echo "결과: OK $ok / FAIL $ng"
[ "$ng" -eq 0 ] && echo "환경이 준비됐어요." || echo "FAIL 항목을 먼저 해결해 주세요."
exit $ng
