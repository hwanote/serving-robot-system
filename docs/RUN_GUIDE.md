# 실행 가이드 (시뮬레이션)

설치와 빌드가 끝난 뒤의 실행 순서예요. 설치는 [README.md](../README.md)를 먼저 보세요.
각 단계 아래의 **성공 조건**이 보이면 다음으로 넘어가면 돼요. 안 보이면 맨 아래 문제 해결을 보세요.

---

## WSL2에서 실행하기 (Windows)

VM 대신 Windows의 WSL2(Ubuntu 24.04)에서도 같은 코드로 실행돼요. 아래는 **한 대의 PC에서 확인한 순서**예요
(2026-10-21, WSL2 + Ubuntu 24.04.4 + ROS 2 Jazzy). 다른 PC에서는 다를 수 있어요.

1. 확인 (PowerShell): `wsl -l -v` 에서 Ubuntu가 **VERSION 2** 이고, Ubuntu 안에서 `cat /etc/os-release` 가 24.04 인지 봐요.
2. 메모리 (권장): WSL은 기본으로 PC 메모리의 절반만 써요. Gazebo + Nav2 + YOLO를 같이 켜면 빠듯할 수 있어서,
   16 GB PC 기준으로 `%UserProfile%\.wslconfig` 파일을 아래처럼 만든 뒤 PowerShell에서 `wsl --shutdown` 하고 다시 열어요 (`free -h` 로 확인).

   ```ini
   [wsl2]
   memory=10GB
   swap=8GB
   ```

3. 설치: [README.md](../README.md)의 "설치" 절을 그대로 따라요. ROS가 이미 있어도 `ros-jazzy-ros-gz`, `python3-rosdep` 이 빠져 있을 수 있어요
   (`sudo apt install -y ros-jazzy-ros-gz python3-rosdep`, 그다음 `sudo rosdep init`, `rosdep update`).
4. 저장소는 **WSL 홈(`~/`)에 clone** 하고 거기서 빌드와 실행을 해요. Windows 폴더(`/mnt/c/...`) 안에서는 느리고
   줄바꿈, 실행 권한 문제가 생겨요.
5. 창: Gazebo와 RViz 창은 WSLg 로 Windows 화면에 떠요 (`echo $DISPLAY $WAYLAND_DISPLAY` 에 값이 있어야 해요).
   창이 안 뜨거나 Gazebo가 바로 죽으면 `export LIBGL_ALWAYS_SOFTWARE=1` 을 해 보세요.
6. 실행 권한: ROS 노드 스크립트 12개는 커밋 `cd47e39` 부터 실행 권한(100755)이 있어요. 그보다 이전 커밋을 받았다면
   `launch` 가 `executable ... not found` 로 멈춰요 → 최신 브랜치를 받거나
   `chmod +x ros_ws/src/serving_robot_bringup/scripts/*.py ros_ws/src/serving_robot_navigation/scripts/*/*.py`.
7. 사람 감지(YOLO)는 아래 "사람 감지" 절의 설치를 먼저 해야 해요. 처음 한 번 확인할 때는
   `operation_mode.launch.py person_limit:=false` (시뮬레이션 전용)로 켜면 YOLO 없이 주행만 볼 수 있어요.

---

## 0. 시작 전 점검 (처음 한 번, 그리고 이전 실행을 비정상 종료했을 때)

```bash
source ~/serving-robot-system/ros_ws/install/setup.bash
bash ~/serving-robot-system/scripts/check_env.sh
```

성공 조건: 모든 항목이 `[OK]` (특히 5번 "남은 프로세스 없음").
이전 실행이 남아 있으면 시뮬레이션 시간이 뒤섞여 위치 추정이 실패해요. 아래 "정리"를 먼저 하세요.

---

## 1. 터미널 1: 전체 시작 (한 번에)

```bash
# 가상머신처럼 GPU가 없는 환경에서만:
export LIBGL_ALWAYS_SOFTWARE=1

source ~/serving-robot-system/ros_ws/install/setup.bash
ros2 launch serving_robot_bringup operation_mode.launch.py
```

내부에서 아래 순서로 자동 진행돼요. 각 단계가 실제로 준비될 때까지 기다린 뒤에 다음 단계를 시작해요.

| 단계 | 하는 일 | 성공 조건 (터미널 1에 나오는 줄) |
|---|---|---|
| 1 | Gazebo 월드(카페) 시작 | `[operation_mode] Gazebo is up` / Gazebo 창에 카페가 보이고 재생 중(⏸ 표시) |
| 2 | 로봇 생성 + 브리지 | `[operation_mode] robot is publishing scan/odom/TF` / Gazebo에 로봇이 보임 |
| 3 | 위치 추정(지도 + AMCL) | `[operation_mode] localization is running (map->odom)` |
| 4 | Nav2 (경로 계획 + 주행 제어) | `[operation_mode] READY - send a goal ...` |
| 5 | RViz (마지막에 시작) | RViz 창이 뜨고 지도 위에 로봇과 레이저가 보임 |

**전체 성공 조건**: `READY` 줄이 나오면 끝이에요. RViz는 그 직후에 따라서 떠요 (Nav2가 켜지는 동안 CPU를 아끼려고 마지막에 시작해요). 보통 1~2분 걸려요 (가상머신은 더 걸릴 수 있어요).
RViz 터미널 로그의 `GLSL link result` 에러와 `Message Filter dropping` 경고는 시작 직후 한두 번 나올 수 있고 무시해도 돼요.

속도 명령은 `Nav2 → velocity_smoother → collision_monitor → soft_stop_limiter → /robot_01/cmd_vel` 순서로 지나가요.
collision_monitor가 로봇 바로 앞의 마지막 관문이에요. 4단계의 성공 조건에는 `collision_monitor`도 active여야 해요.
거리는 차체 가장자리 기준이고, 값은 `config/common/runtime_limits.yaml`에서 읽어요.
**지도에 있는 것(벽, 카운터, 테이블 기둥, 화분)은 "알고 있는 고정 장애물"로 보고** 정지와 감속 구간에서 제외해요
(`dynamic_scan_filter`가 라이다 점에서 걸러 내요). 그래서 벽 앞 코너에서도 회전할 수 있어요.
의자, 사람, 소지품처럼 지도에 없는 것에는 아래 표대로 반응해요. 하드 정지는 걸러내지 않은 원본 스캔을 써서 벽이든 뭐든 10 cm 안이면 서요.
**전제**: 지도는 고정된 물체만 놓고 만들어야 해요 (맵핑할 때 의자와 사람을 치워요).

| 구간 | 거리 | 동작 | 켜지는 때 |
|---|---|---|---|
| 하드 정지 | 10 cm (`hard_stop_distance_m`) | 정지 | 항상 (모든 방향) |
| 정지 | 0.8 m (`person_stop_distance_m`) | 정지 | 앞으로 갈 때만 (제자리 회전은 막지 않음) |
| 감속 | 1.5 m (`person_slow_distance_m`) | 속도를 0.10 m/s (`slow_linear_mps`)로 제한 | 앞으로 갈 때만 (제자리 회전은 막지 않음) |

**부드러운 정지**: `soft_stop_limiter`가 감속을 0.30 m/s²(`soft_stop_decel_mps2`)로 제한해서 음료가 흔들리지 않게 서요.
하드 정지(10 cm)만 예외로 즉시 서요 (마지막 방어선). 이 노드가 멈추면 로봇에 명령이 안 가서 가만히 서요.

**재출발 규칙**: 정지 구간(0.8 m) 때문에 선 뒤에는 장애물이 사라져도 바로 출발하지 않아요.
앞쪽 1.0 m 안에 아무것도 없는 상태가 2초 이어지면 그때 다시 가요 (`resume_clear_distance_m`, `resume_hold_s`).
기다리는 동안 제자리 회전은 할 수 있어요. 재출발 직후에도 장애물이 1.5 m 안에 있으면 감속 구간이 0.10 m/s로 제한해요.

**충돌 래치**: 몸체에 무언가 닿으면(접촉 센서 `/robot_01/contact`) `soft_stop_limiter`가 출력을 즉시 0으로 만들고
해제 전까지 어떤 명령도 통과시키지 않아요. 로그에 `COLLISION #n: the robot touched <대상>` 이 한 줄 남고,
`/robot_01/collision_latched`가 `true`가 돼요. 접촉 횟수는 이 로그로 세요 (합격선: 접촉 0회).

```bash
# 해제 (마지막 접촉 뒤 3초가 지나야 받아줘요: collision_release_s)
ros2 service call /robot_01/clear_collision std_srvs/srv/Trigger
```

해제 뒤에도 앞쪽 1.0 m가 2초 비어야 앞으로 가요 (재출발 규칙). 후진과 제자리 회전은 바로 돼요.
단, 10 cm 하드 정지 안쪽에 갇혀 있으면 하드 정지가 계속 막아요 → 장애물을 치우거나 로봇을 옮겨 주세요 (알려진 한계).

막힘이 15초/30초 넘게 이어질 때의 알람과 우회는 아래 "경로 실행기" 절을 보세요.

RViz 없이 실행하려면:

```bash
ros2 launch serving_robot_bringup operation_mode.launch.py rviz:=false
```

---

## 2. 터미널 2: 이름으로 이동 (주행 시험)

새 터미널을 열어서 실행해요. 목적지는 좌표가 아니라 **이름**으로 줘요(`config/common/waypoints.yaml`).

```bash
source ~/serving-robot-system/ros_ws/install/setup.bash

# 사용할 수 있는 이름 보기
ros2 run serving_robot_navigation go_to.py --list

# 한 곳으로 이동
ros2 run serving_robot_navigation go_to.py table_01

# 여러 곳을 차례로 (한 곳이라도 실패하면 멈춰요)
ros2 run serving_robot_navigation go_to.py kitchen table_03 standby_01
```

성공 조건: 각 목적지마다 `SUCCEEDED`가 찍히고 로봇이 그 자리에서 정해진 방향을 보고 멈춰요.
이름은 별칭도 돼요 (`desk` = `kitchen`, `home` = `standby_01`).

RViz에서는 위쪽 도구의 **2D Goal Pose**를 누르고 지도를 클릭해서(드래그로 방향 지정) 목표를 줄 수도 있어요.

**어떻게 가나요**: `go_to.py`는 통로 그래프(`waypoints.yaml`의 노드와 직선 통로)를 따라 가요.
현재 위치에서 가장 가까운 노드를 찾고, 직선으로 다음 노드까지 가서 멈추고, 다음 구간 방향으로 제자리 회전한 뒤 다시 직선으로 가요.
출력에 `route from node ...`와 `leg 1/3 ...`처럼 구간이 보여요. 로봇이 노드에서 0.6 m 넘게 떨어져 있으면
Nav2에 목적지를 한 번에 주는 예전 방식으로 가요 (`--direct`로 일부러 예전 방식을 쓸 수도 있어요).
`--executor`를 붙이면 이 역할을 로봇 안쪽의 경로 실행기가 대신해요 (아래 절).

**동작 특성** (설계 기준): 로봇은 직선 이동과 제자리 회전만 해요. 방향이 다르면 먼저 제자리에서 돌고 출발해요.
속도 상한은 `config/common/runtime_limits.yaml`(0.25 m/s, 0.8 rad/s)이에요.
(각 구간 안에서는 Nav2가 직선에 가까운 경로를 만들어요.)

### 경로 실행기 (막힘 대응)

```bash
ros2 run serving_robot_navigation go_to.py --executor table_01 kitchen standby_01
```

`route_executor`(operation_mode와 함께 켜져요)가 통로 그래프를 따라 구간마다 **직선 레일**(로봇 위치 → 다음 노드, 5 cm 간격 자세 목록)을 Nav2 제어기에 직접 줘요(`follow_path`).
플래너를 거치지 않아서 앞에 사람이나 장애물이 있어도 피해서 휘지 않고 **정면에서 멈춰요**. 돌아가는 건 아래 30초 우회 규칙만 해요.
노드에 도착하면 제자리에서 다음 방향으로 돌아요. 레일에서 0.15 m 넘게 벗어나면 멈추고 `OFF_RAIL` 알람, 3초 뒤 그 자리에서 다음 노드로 직선 재출발해요.
서 있는 시간도 재요: 정지 신호(장애물 hold, 사람 STOP)가 있거나, **신호가 없어도 3초 동안 움직이지 않으면**(5 cm 이동도 0.15 rad 회전도 없음) 멈춘 것으로 세요.
시간은 시뮬레이션 시계예요 (VM이 느리면 실제 시간은 더 걸려요). 충돌 래치로 서 있는 시간은 세지 않아요.

| 서 있은 시간 | 하는 일 |
|---|---|
| 15초 | 알람 `BLOCKED` (계속 기다려요). `cause`: `obstacle`(정지 신호) / `not_moving`(신호 없이 안 움직임) |
| 30초 | 막힌 통로를 60초 동안 피하고, 방금 지나온 노드로 돌아가(제자리 회전 후 직진) 다른 직선 통로로 우회해요 (`DETOUR_STARTED`) |
| 우회할 통로가 없을 때 | 알람 `NO_DETOUR`, 그 자리에서 기다리며 5초마다 다시 찾아요 |
| 한 목적지에 우회 3회째 | 알람 `DETOUR_LIMIT`, 기다려요 |

바로 우회하려면 (UI 버튼은 나중에 붙어요):

```bash
ros2 service call /robot_01/request_detour std_srvs/srv/Trigger
ros2 service call /robot_01/cancel_mission std_srvs/srv/Trigger   # 임무 취소
```

토픽: 명령 `/robot_01/mission_cmd`(JSON), 상태 `/robot_01/mission_status`(JSON, 1초에 2번), 알람 `/robot_01/alarm`(JSON: code, level, text, zone, destination).
상태 JSON의 `hold`는 장애물 정지(Collision Monitor 정지 구역, limiter), `semantic_stop`은 사람 감지기의 STOP(카메라가 본 사람이나 LiDAR가 본 장애물), `semantic_source`(`camera`/`lidar`)는 그 STOP을 낸 센서예요. 둘 중 하나라도 true이거나 `stalled`(신호 없이 3초 동안 안 움직임)가 true면 대기 시간(`stopped_s`)이 올라가고, 셋 다 아니고 로봇이 움직일 때만 0으로 돌아가요.
수치(15초, 30초, 60초, 3회, 5초, 안 움직임 3초/5 cm/0.15 rad)는 `config/common/runtime_limits.yaml`의 `blocked_response`, 레일(5 cm, 0.15 m, 3초)은 `rail`에 있어요.
로봇이 노드에서 0.6 m 넘게 떨어져 있으면 통로 그래프 없이 Nav2에 직접 목표를 주고(`navigate_to_pose`, 플래너가 경로를 그려요), 그 경우엔 레일도 우회도 없어요.

시험용 박스를 미리 통로에 놓으려면: `python3 ~/serving-robot-system/scripts/dev/spawn_box.py --at -3.2 -0.6` (지도 좌표 x y), 치우기는 `--remove`.

### 서빙 순서와 배달 이상 (180초)

```bash
ros2 run serving_robot_navigation serve.py table_01            # 터미널 A: 주문 (알람과 상태가 계속 나와요)
ros2 run serving_robot_navigation serve.py --confirm load       # 터미널 B: 주방에서 적재를 마쳤을 때
ros2 run serving_robot_navigation serve.py --confirm handover   # 터미널 B: 테이블에서 고객이 받았을 때
```

주방 → (적재 확인 대기) → 테이블 → (수령 확인 대기) → `standby_01` 순서예요. 확인은 서비스
`/robot_01/confirm_load`, `/robot_01/confirm_handover`(Trigger)로도 줄 수 있어요 (UI 버튼은 나중에 붙어요).

| 상황 | 하는 일 |
|---|---|
| 주방에서 적재 대기 | 알람 `WAIT_LOAD`, 30초마다 반복, 계속 기다려요 |
| 테이블에서 수령 대기 | 알람 `WAIT_HANDOVER`, 60초 뒤 `HANDOVER_TIMEOUT`(30초마다 반복), 계속 기다려요 |
| 주방 출발 후 테이블 도착까지 180초 초과 (움직일 수 있을 때) | `DELIVERY_OVERTIME` + `RETURN_STARTED` → 주방 복귀 → `RETURNED_TO_KITCHEN`(목적지 포함), 상태 `PAUSED` |
| 180초 초과인데 움직일 수 없을 때 | `DELIVERY_OVERTIME_STUCK`(목적지와 위치 포함) 한 번, 움직일 수 있게 되면 위 복귀 절차 |
| `PAUSED` 중 새 주문 | 거절, 알람 `HANDOVER_REQUIRED`. `--confirm handover`를 주면 `HANDOVER_CONFIRMED` 후 다시 받아요 |
| 다른 임무가 진행 중일 때 새 주문 | 거절, 알람 `BUSY` |

주문이 거절되면 `serve.py`(그리고 `go_to.py --executor`)는 `REJECTED ...`를 출력하고 바로 끝나요 (종료 코드 1).
실행 중에 Ctrl-C를 누르면 **자기 임무만** 취소해요 (취소 명령에 자기 `id`를 붙여서 보내요). 다른 창의 임무는 건드리지 않아요.

180초에는 장애물 대기 시간이 포함되고, 충돌 래치 시간만 빠져요. 수치는 `runtime_limits.yaml`의 `delivery`에 있어요.
시험용으로 시간을 줄이려면 `ros2 launch ... operation_mode.launch.py delivery_limit_s:=30 auto_confirm_s:=5`처럼 인자를 줘요
(아무것도 안 주면 yaml 값이에요).

### 위치를 잃었을 때 (LOCALIZATION_LOST → 선택)

로봇이 자기 위치를 잃으면(`/robot_01/health` 의 `localization_state` 가 LOST) 경로 실행기가 이렇게 해요.

| 순서 | 일어나는 일 |
|---|---|
| 1 | 주행 중이면 **즉시 정지**하고 임무를 **보류**해요. 알람 `LOCALIZATION_LOST`. 15/30초 막힘 타이머와 배달 시간은 멈춰요 |
| 2 | 보류 중과 위치를 모를 때(LOST, UNINITIALIZED)는 **새 주문을 거절**해요 (`serve.py`/`go_to.py` 는 `REJECTED LOCALIZATION_...` 를 출력하고 끝나요) |
| 3 | 위치가 LOST 를 벗어나(OK 또는 DEGRADED) 2초 지나면 알람 `LOCALIZATION_RECOVERED` 를 내고 **기다려요**. 스스로 움직이지 않아요 (30초마다 다시 알려요). DEGRADED 면 "아직 거침" 문구가 붙어요: 멈춘 로봇은 위치 오차를 못 줄이고, `resume` 으로 달리면 좋아져요 |
| 4 | 사용자가 고르면 움직여요: `decide.py resume` = 같은 목적지로 이어가기, `decide.py home` = 임무 취소 후 홈(`standby_01`)으로 |

```bash
ros2 run serving_robot_navigation decide.py resume     # 이어가기
ros2 run serving_robot_navigation decide.py home       # 취소하고 홈으로
ros2 run serving_robot_navigation decide.py resume --robot robot_02   # 로봇 이름 지정
```

- `LOCALIZATION_RECOVERED` 전에 고르면 `REFUSED: the position is not OK yet` 로 거절돼요.
- 적재/수령 대기 중에는 로봇이 이미 서 있어서 보류하지 않고 알람만 내요.
- 위치가 DEGRADED(오차 0.25~1.0 m)일 때는 멈추지 않고 health 로만 알려요.
- 시험용 도구 (VM): `python3 ~/serving-robot-system/scripts/dev/amcl_kick.py lost` = 위치를 잃게 함, `... amcl_kick.py fix` = Gazebo 의 실제 위치를 AMCL 에 알려 복구.
- 비상 초기화: 이유를 모르게 막혔을 때 사람이 로봇을 바닥 노드(번호) 위에 올려놓고 이름을 알려줘요. 임무·정지·보류·타이머를 모두 비우고 그 노드 좌표를 AMCL 에 알린 뒤 IDLE 로 돌아가요 (알람 `ROBOT_RESET`). 로봇은 노드에서 약 0.3 m 안에 놓아야 해요 (실제 로봇에선 실행기가 확인할 수 없어요).

```bash
ros2 run serving_robot_navigation reset.py kitchen             # 로봇을 kitchen 노드에 올려놓은 뒤
ros2 run serving_robot_navigation reset.py kitchen --place     # 시뮬레이션: 먼저 초기화(정지), 그다음 Gazebo 에서 로봇을 그 노드로 옮김
ros2 run serving_robot_navigation reset.py kitchen --robot robot_02
python3 ~/serving-robot-system/scripts/dev/scan_min.py         # 진단: 로봇 둘레에서 가장 가까운 물체
```

- 값은 `runtime_limits.yaml` 의 `localization_hold` [제안]. 자세한 이유는 `docs/DECISIONS.md` B5.

### 사람 상태로 속도 제한 (person_speed_limiter)

`operation_mode.launch.py`(와 `navigation.launch.py`)는 기본으로 사람 감지기(`person_detector`, 카메라), 판단 노드(`person_state`), 속도 제한 노드를 같이 켜요(`person_limit:=true`).
**이때는 이 노드들을 터미널에서 따로 켜지 마세요** (같은 노드가 두 개가 돼요). 끄려면 `person_limit:=false`예요.

```
cmd_vel_smoothed → person_speed_limiter → cmd_vel_semantic → collision_monitor → … → /robot_01/cmd_vel
```

| 사람 상태 | 속도 |
|---|---|
| NORMAL | 그대로 |
| SLOW | 선속도 0.10 m/s(`slow_linear_mps`), 회전 0.40 rad/s 이하 |
| STOP | 앞으로 가는 속도 0. 후진(0.10 m/s 이하)과 회전(0.40 rad/s 이하)은 허용 (장애물 정지와 같은 규칙, 우회하려면 돌 수 있어야 해요) |
| SENSOR_STALE, 또는 감지기 메시지가 1초 넘게 안 옴 | SLOW와 같은 상한 + 경고 로그 `PERSON LIMIT ... no usable person state`. 단 마지막 상태가 STOP이었다면 LiDAR가 앞에 뭔가(1.0 m 안쪽)를 보거나 모를 때만 STOP을 유지하고, 앞이 비면 SLOW 상한으로 계속 움직여요 (우회 가능) |

값은 `runtime_limits.yaml`(`slow_linear_mps`, `person_safety`)에서 읽어요. 감지기가 모델을 불러오는 동안(몇 초)은 STALE이라 천천히 움직여요.
속도 제한 노드는 지금 적용 중인 결정을 `/robot_01/person_limit`(JSON: `state`, `silent`, `source`, `time`; 0.5초마다와 바뀔 때)로 내보내요. 경로 실행기와 UI는 감지기 대신 이 토픽을 읽어요(판단은 한 곳). `silent`는 감지기가 말이 없다는 뜻이에요.
감지기가 죽거나 말이 없으면 운영자에게 알람이 가요: `DETECTOR_SILENT`(warn, 30초마다 반복, 감지기가 한 번도 안 켜졌으면 30초 뒤부터), 다시 말하면 `DETECTOR_RECOVERED`(info).
감지기가 없어도 LiDAR와 Collision Monitor가 지켜 주니 로봇은 계속 움직여요(SLOW 상한).
이 노드는 입력 명령을 자르기만 해요. 최종 `/robot_01/cmd_vel`은 지금처럼 soft_stop_limiter 하나만 내보내요.

### 사람 감지 (카메라 + YOLO)

YOLO는 `~/yolo_venv`(별도 가상환경)에 설치돼 있어야 해요. 감지 노드는 이 안의 패키지를 시스템 파이썬 **뒤에** 붙여서 쓰므로,
venv의 numpy를 시스템 numpy와 **같은 버전**으로 맞춰요 (`python3 -c "import numpy; print(numpy.__version__)"`; Ubuntu 24.04 + Jazzy는 1.26.4).
감지기는 CPU로 돌아서 CPU용 torch를 써요.

처음 한 번 설치 (내려받는 용량이 1 GB 안팎이라 몇 분 걸려요):

```bash
sudo apt install -y python3-venv
python3 -m venv ~/yolo_venv
~/yolo_venv/bin/pip install --upgrade pip
~/yolo_venv/bin/pip install "numpy==1.26.4"            # 시스템 numpy 버전에 맞춰요
~/yolo_venv/bin/pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
~/yolo_venv/bin/pip install ultralytics "numpy==1.26.4"
mkdir -p ~/yolo_models
```

설치 확인 (모델 `yolov8n.pt` 가 없으면 이때 현재 폴더로 자동으로 내려와요):

```bash
cd ~/yolo_models && ~/yolo_venv/bin/python ~/serving-robot-system/scripts/dev/yolo_check.py   # persons: 3~4 가 나오면 정상
```

시험용 사람 모델(`spawn_person.py`)은 처음 한 번 받아 두세요: `gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Standing person"`

카메라는 3대예요(앞 `camera_front`, 좌후방 `camera_rear_left`, 우후방 `camera_rear_right`, 각 120°, 1.0 m 높이).
사람 감지는 두 노드예요: `person_detector`(카메라 3대를 YOLO 모델 하나로 처리: 앞은 매 주기, 뒤 두 대는 번갈아 → `/robot_01/person_observations`, 본 사람과 실제 거리와 방향)와
`person_state`(판단: 관측 + LiDAR → `/robot_01/person_detection`, 상태). 둘 다 launch가 자동으로 켜요.
손으로 띄워 보려면(디버그 영상 저장 등) `operation_mode.launch.py person_limit:=false`로 시작한 뒤 다른 터미널에서
(이때는 속도 제한 노드가 없어서 상태를 보기만 해요):

```bash
ros2 run serving_robot_navigation person_detector.py --ros-args -p use_sim_time:=true -p debug_dir:=~/person_debug
ros2 run serving_robot_navigation person_state.py --ros-args -p use_sim_time:=true
ros2 topic echo /robot_01/person_detection --field data      # JSON 한 줄 (state: NORMAL / SLOW / STOP / SENSOR_STALE)
python3 ~/serving-robot-system/scripts/dev/spawn_person.py 2.0   # 로봇 앞 2 m에 사람 세우기 (--remove 로 치우기)
```

방향별 규칙: **앞(±60°)** 은 아래 거리 규칙 그대로, **옆(60~120°)** 은 0.8 m 안이면 SLOW만(STOP 없음, 1초 비면 해제), **뒤(120°~)** 는 속도에 영향 없음(`rear_m`으로 보기만).
카메라 하나가 2초 넘게 영상이 없으면 `CAMERA_SILENT` 알람(옆/뒤 카메라는 알람만, LiDAR가 지켜요). 디버그 영상은 `debug_dir`에 `latest_<카메라>.jpg`로 저장돼요.
카메라 노드의 입력은 카메라마다 `/robot_01/<카메라>/image`(RGB), `/depth`(깊이), `/camera_info`이고, 판단 노드의 결과는 `/robot_01/person_detection`(JSON: state, distance_m, nearest_m, confidence, count, bbox, frame_age_s)이에요.
`STOP < 0.8 m`, `SLOW < 1.5 m`가 처음 값이고(`person_state`의 `-p stop_m:=0.8 -p slow_m:=1.5`, launch에서는 `runtime_limits.yaml`), 다시 풀릴 때는 +0.2 m 여유와 3프레임 연속 조건이 필요해요.
가까이 있던 사람(마지막 거리가 1.7 m 안쪽)이 갑자기 안 보이게 되면(너무 가까워 머리가 화면 밖으로 나간 경우 등) `SLOW`/`STOP`을 바로 풀지 않고, 최소 3초(`-p lost_hold_s:=3.0`) 동안 유지하고, 그 뒤에도 LiDAR로 본 전방 여유(`/robot_01/scan_dynamic`, 몸체 앞 기준)가 1.0 m 이상(`clear_m`)인 상태가 2초(`clear_hold_s`) 이어질 때까지 유지해요. 스캔이 없거나 0.5초 넘게 오래됐으면 "막힘"으로 보고, `/robot_01/obstacle_hold`가 true인 동안도 유지해요. 상태 JSON의 `front_clear_m`으로 지금 전방 여유를 볼 수 있어요.
판단은 하나로 모아요: 카메라 거리와 LiDAR 전방 거리를 한 목록에 넣고 가장 가까운 값으로 상태를 정해요. 둘 중 하나라도 가까우면 그쪽이 근거가 되고(최악 우선), JSON의 `source`(`camera`/`lidar`)가 어느 쪽인지 알려줘요. 사람을 못 알아봐도 LiDAR가 0.8 m 안쪽에서 뭔가 보면 `STOP`, 1.5 m 안쪽이면 `SLOW`예요(`reason`: `LiDAR: something 0.39 m ahead`). 상태의 `reason`에 `lost while close, holding ...`이 나와요. 멀리서 사라진 사람은 지금처럼 3프레임 뒤 풀려요.
`debug_dir`을 주면 1초에 한 번 감지 박스가 그려진 `latest.jpg`가 저장돼요. 감지기 자체는 판단만 하고, 속도 제한은 위의 person_speed_limiter가 해요 (LiDAR/Collision Monitor가 최종 안전 계층).
카메라는 지면에서 1.0 m 높이에 달려 있고(`sensors.xacro`의 `z`; 선반 약 0.75 m + 커버 0.3 m 가정), 가로 시야 86°로 넓게, 위로 8도 기울여서 가까이 선 사람도 가슴부터 머리까지 보이게 했어요. 낮고 좁은 시야로 달면 가까운 사람의 하체만 찍혀서 YOLO가 못 알아봐요. 지지대는 그리지 않았어요 (GPU LiDAR가 가짜 장애물로 봐요). 기울기 때문에 깊이 거리가 앞쪽 거리보다 위쪽 행에서 최대 약 5% 짧게 나올 수 있어요(VM에서 실제 오차를 확인해요).

---

## 3. 종료와 정리

터미널 1에서 `Ctrl + C`를 한 번 누르면 모두 종료돼요. 창이 남아 있으면 닫아 주세요.
그다음 남은 프로세스가 없는지 확인해요.

```bash
ps aux | grep -E "gz|ruby|parameter_bridge|rviz2|nav2|amcl|map_server" | grep -v grep
```

아무것도 안 나오면 깨끗해요. 남아 있으면 정리해요.

```bash
pkill -9 -f "gz sim"; pkill -9 -f ruby; pkill -9 -f parameter_bridge; pkill -9 -f rviz2
pkill -9 -f nav2; pkill -9 -f amcl; pkill -9 -f map_server; pkill -9 -f robot_state_publisher
ros2 daemon stop; ros2 daemon start
```

---

## 실물 로봇에서 (mode:=real)

```bash
ros2 launch serving_robot_bringup operation_mode.launch.py mode:=real
```

- Gazebo, 로봇 생성, 브리지는 실행되지 않고 실제 시간(wall clock)으로 동작해요.
- 로봇 드라이버는 아직 이 저장소에 없어요. 다음 규약대로 **같은 이름의 토픽**만 내보내면 위쪽 코드는 바꿀 필요가 없어요.

  | 방향 | 토픽 | 형식 |
  |---|---|---|
  | 발행 | `/robot_01/scan` | `sensor_msgs/LaserScan` (프레임 `laser_link`) |
  | 발행 | `/robot_01/odom` + TF | `nav_msgs/Odometry`, `odom -> base_footprint` |
  | 구독 | `/robot_01/cmd_vel` | `geometry_msgs/Twist` (m/s, rad/s) |

- 시작 위치를 모르면 RViz의 **2D Pose Estimate**로 로봇의 실제 위치를 찍어 주세요 (`mode:=real`은 기본적으로 시작 위치 자동 설정을 끄고 최대 10분 기다려요).
- 크기, 속도, 가속도 같은 값은 코드가 아니라 `config/` 파일에서 바꿔요.

---

## 문제 해결

| 증상 (터미널 1의 `FAILED:` 메시지 등) | 원인과 해결 |
|---|---|
| `Gazebo did not start` | Gazebo 창/로그 확인. 이전 `gz` 프로세스가 남았는지 확인하고 "정리" 후 재시도 |
| `robot did not publish /clock, ...` | 로봇 생성 실패 또는 브리지 미동작. 터미널 로그에서 `Entity creation successful`과 `Creating GZ->ROS Bridge` 줄을 확인 |
| `map->odom never appeared` | 위치 추정이 시작 위치를 못 받음. 대부분 이전 실행이 남아 시간이 뒤섞인 경우 → "정리" 후 재시도. 실물이면 RViz에서 2D Pose Estimate |
| `Nav2 nodes did not become active` | 바로 위의 Nav2 로그 확인 (`Failed to bring up all requested nodes` 앞의 에러 줄) |
| 지도가 RViz에서 안 보임 | 가상머신이면 `export LIBGL_ALWAYS_SOFTWARE=1` 후 재시작 |
| 각 단계 대기가 너무 길다 | `wait_timeout:=300`처럼 단계별 제한 시간(초)을 늘릴 수 있어요 |

---

## 부록: 단계별 수동 실행 (원인 추적용)

`operation_mode.launch.py`가 하는 일을 터미널 4개로 직접 나눠서 해 볼 수 있어요. 앞 단계의 성공 조건을 확인한 뒤 다음으로 가세요.

```bash
# 터미널 A: Gazebo 월드  (성공: 카페가 보이고 재생 중)
ros2 launch serving_robot_gazebo gazebo.launch.py

# 터미널 B: 로봇 + 브리지  (성공: 'Entity creation successful', 'Creating GZ->ROS Bridge: /robot_01/scan')
ros2 launch serving_robot_bringup robot_base.launch.py mode:=sim

# 터미널 C: 위치 추정  (성공: 'Managed nodes are active', 'Setting pose')
ros2 launch serving_robot_navigation localization.launch.py rviz:=false

# 터미널 D: Nav2 + collision_monitor  (성공: 'Managed nodes are active')
ros2 launch serving_robot_navigation navigation.launch.py localization:=false rviz:=false
```

---

관련 문서: 토픽, 서비스, JSON 필드, 알람 계약은 `docs/INTERFACES.md` 참고.

---

## 토픽 이름표와 검사 (interfaces.yaml)

토픽/서비스 이름은 `config/common/interfaces.yaml` 한 곳에 있고, 노드와 launch가 여기서 읽습니다 (`{robot}` 자리표시자 = 로봇 이름, 기본 `robot_01`).

| 하고 싶은 일 | 방법 |
|---|---|
| 로봇 이름 바꿔 실행 | `ros2 launch serving_robot_bringup operation_mode.launch.py robot_name:=robot_02` (Gazebo 쪽은 아직 `robot_01` 고정이라 실제 다중 로봇은 Fleet 설계 이후) |
| 명령줄 도구에서 이름 | `go_to.py --robot robot_02 ...`, `serve.py --robot robot_02 ...`, `decide.py resume --robot robot_02` (`--cmd-topic` 등으로 한 이름만 덮어쓰기도 가능) |
| 파일끼리 이름이 맞는지 (PC/VM 어디서나) | `python3 scripts/validate_interfaces.py` → `interfaces: consistent` |
| 실행 중인 로봇이 문서대로 말하는지 (VM) | `source ros_ws/install/setup.bash` 후 `python3 scripts/dev/check_interfaces_live.py` → 마지막 줄 `interfaces live: all good` (`person_limit:=false` 로 띄웠다면 `--no-person`) |


### 로봇 요약 상태 (`/robot_01/health`)

`robot_health` 노드(operation_mode 와 함께 켜져요)가 로봇 상태를 한 토픽에 모아요. 1초마다, 그리고 상태가 바뀌면 바로 보내요.
값은 팀 통합 가이드의 enum 그대로예요: `robot_state`(BOOTING/IDLE/NAVIGATING/PAUSED/BLOCKED/DEGRADED/ERROR), `safety_state`(NORMAL/SLOW/STOP), `localization_state`(UNINITIALIZED/OK/DEGRADED/LOST). 정상이 아닌 이유는 `reasons`.

```bash
ros2 topic echo /robot_01/health --field data          # 전체 JSON
```

터미널 1 로그에는 상태가 바뀔 때 `HEALTH <robot_state> safety <...> localization <...> [이유]` 한 줄이 나와요. 이 노드는 보고만 해요. 위치를 잃었을 때의 동작은 경로 실행기가 이 값을 읽어서 처리해요 (위의 "위치를 잃었을 때").

---

## 구획별 launch와 검사 (monitoring / decision / motion / command)

`operation_mode` 사용법은 그대로입니다. 안쪽에서 `navigation.launch.py` 가 Nav2 본체와 구획 launch 4개를 시작합니다.

| 파일 | 시작하는 것 |
|---|---|
| `launch/monitoring.launch.py` | `dynamic_scan_filter`, `person_detector` (카메라) |
| `launch/decision.launch.py` | `person_state` (사람 상태 판단), `robot_health` (로봇 요약 상태) |
| `launch/motion.launch.py` | `velocity_smoother`, `person_speed_limiter`, `collision_monitor`, `soft_stop_limiter` |
| `launch/command.launch.py` | `route_executor` |

| 하고 싶은 일 | 방법 |
|---|---|
| 구획 하나만 띄워 시험 (인자 목록 확인) | `ros2 launch serving_robot_navigation monitoring.launch.py --show-args` |
| 새 모듈을 꽂았는지 검사 (PC/VM 어디서나) | `python3 scripts/validate_layout.py` → `layout: all plugged in` |
| 새 모듈 꽂는 순서 | 구획 폴더의 `README.md` (`serving_robot_navigation/<구획>/README.md`) |
