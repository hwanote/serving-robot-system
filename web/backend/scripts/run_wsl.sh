#!/bin/bash
# WSL 에서 백엔드 실행: 처음 한 번 가상환경을 만들고, 이후엔 바로 실행. 인자는 그대로 python -m app 에 전달
# 예) scripts/run_wsl.sh --adapter ros --config config/fleet.yaml
cd "$(dirname "$0")/.." || exit 1
V=$HOME/backend_venv
if [ ! -x "$V/bin/python" ]; then
  python3 -m venv "$V" && "$V/bin/pip" install -r requirements.txt || exit 1
fi
[ -f /opt/ros/jazzy/setup.bash ] && source /opt/ros/jazzy/setup.bash
[ -f "$HOME/serving-robot-system/ros_ws/install/setup.bash" ] && source "$HOME/serving-robot-system/ros_ws/install/setup.bash"
exec "$V/bin/python" -m app "$@"
