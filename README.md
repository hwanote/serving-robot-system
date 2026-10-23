# Gazebo 기반 다중 자율 주행 서빙로봇 시스템

카페 같은 매장에서 서빙 로봇이 자율 주행으로 음료를 배달하는 시스템이에요.
이 저장소는 먼저 Gazebo 시뮬레이션으로 만들고, 같은 코드를 실물 로봇에 거의 그대로 옮기는 것을 목표로 해요.

## 구성

| 폴더 | 내용 |
|---|---|
| `ros_ws/src/` | ROS 2 패키지 (`serving_robot_bringup` / `_gazebo` / `_navigation` / `_description` / `_perception` / `_interfaces`) |
| `config/common/` | 팀 공용 설정 (속도 제한, 로봇 시작 위치 등. 값의 단일 출처) |
| `maps/` | 매장 지도 (`stores/demo_store` = 데모용 합성 지도) |
| `scripts/` | `check_env.sh` (환경 점검), `dev/` (개발자 개인 환경용 도구) |
| `docs/` | 실행 가이드 |

시뮬레이션 전용 부분(Gazebo, 로봇 생성, 브리지)과 로봇 입출력 규약, 그 위의 공통 로직(위치 추정, 내비게이션)이
분리돼 있어서, 실물로 옮길 때는 로봇 드라이버만 같은 토픽 이름으로 제공하면 돼요.
자세한 규약은 `ros_ws/src/serving_robot_bringup/launch/robot_base.launch.py` 첫 부분에 있어요.

## 환경 (권장)

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic (`ros-jazzy-ros-gz`로 함께 설치돼요)
- Windows에서는 **WSL2 + Ubuntu 24.04**로 같은 방식으로 실행할 수 있어요 (한 대의 PC에서 확인한 순서: [docs/RUN_GUIDE.md](docs/RUN_GUIDE.md)의 "WSL2에서 실행하기")
- GPU가 없는 가상머신(VMware 등)이면 Gazebo/RViz를 실행하는 터미널마다 `export LIBGL_ALWAYS_SOFTWARE=1`이 필요해요.

## 설치

```bash
# 1) ROS 2 Jazzy가 설치돼 있어야 해요 (https://docs.ros.org/en/jazzy/Installation.html)
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
                    ros-jazzy-ros-gz python3-colcon-common-extensions python3-rosdep

# 2) rosdep 초기화 (컴퓨터에서 처음 한 번만)
sudo rosdep init
rosdep update

# 3) 저장소 받기
git clone https://github.com/hwanote/serving-robot-system.git ~/serving-robot-system
cd ~/serving-robot-system/ros_ws

# 4) 의존 패키지 설치 + 빌드
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

# 5) 환경 점검 (모두 [OK]여야 해요)
bash ~/serving-robot-system/scripts/check_env.sh
```

`[FAIL]` 항목이 있으면 같은 줄에 해결 방법이 적혀 있어요.

## 실행

터미널별 순서와 성공 조건은 **[docs/RUN_GUIDE.md](docs/RUN_GUIDE.md)** 를 보세요.
한 줄 요약:

```bash
source ~/serving-robot-system/ros_ws/install/setup.bash
ros2 launch serving_robot_bringup operation_mode.launch.py
```

## 개발 메모

- `scripts/dev/`의 도구(`vm_sync.sh` 등)는 Windows + VMware 공유 폴더 환경을 쓰는 개발자 개인용이에요. 다른 환경에서는 필요 없어요.
- 팀 공용 값(속도 제한 등)은 `config/common/*.yaml`에만 두고, 코드나 다른 설정에 복사하지 않아요.
