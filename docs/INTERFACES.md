# 인터페이스 계약 (자율주행 파트)  v0.14

React 관리/조작 패널, Fleet(다중 로봇), 다른 팀원 노드가 **자율주행 파트와 주고받는 토픽, 서비스, 메시지 필드**를 한곳에 모은 문서입니다.
코드를 읽지 않아도 "무엇을 읽고, 무엇을 보내면 되는지" 알 수 있게 하는 것이 목적입니다.

- 이 문서의 모든 이름은 2026-10-15 기준 코드(`feature/autonomy-navigation`)와 대조한 값입니다.
- 코드와 다르면 **코드가 맞고 문서를 고칩니다.** 토픽, 필드를 바꾸는 PR은 이 문서도 같이 수정합니다.
- "(계획)"이라고 적힌 항목은 아직 코드에 없습니다.

---

## 1. 큰 그림: 네 구획

```
 [감시]  센서를 보고 "사실"만 내보냄
   scan / camera / contact ──► dynamic_scan_filter, person_detector(카메라), 접촉 센서 입력
                                    │ person_observations, scan_dynamic
 [판단]  사실을 보고 "결정"을 내림
   person_state(사람 상태), MissionCore(대기/알람/우회), Collision Monitor 규칙
                                    │ person_detection, person_limit, obstacle_hold, collision_latched
 [구동]  결정을 "속도"로 바꿈
   Nav2 → velocity_smoother → person_speed_limiter → collision_monitor → soft_stop_limiter
                                    │ /robot_01/cmd_vel   (최종 속도 명령, 주인은 soft_stop_limiter 하나)
 [제어]  사람/UI의 "명령"을 받아 임무를 진행하고 상태를 알림
   route_executor  ◄── mission_cmd, 서비스      ──► mission_status, alarm
```

| 구획 | 질문 | 하는 일 | 노드 |
|---|---|---|---|
| 감시 | 지금 무엇이 보이나? | 센서 값을 사실(거리, 사람 여부, 접촉)로 바꿔 내보냄. 결정은 하지 않음 | `dynamic_scan_filter`, `person_detector`, `soft_stop_limiter`(접촉 입력 부분) |
| 판단 | 그래서 어떻게 해야 하나? | 사실을 모아 상태(NORMAL/SLOW/STOP, 막힘 대기, 우회)를 결정 | `person_state`(사람 상태), `route_executor`(MissionCore), `collision_monitor` |
| 구동 | 그 결정대로 속도를 어떻게 줄까? | 속도 명령을 단계별로 제한, 부드럽게 정지 | `velocity_smoother`, `person_speed_limiter`, `collision_monitor`, `soft_stop_limiter` |
| 제어 | 누가 무엇을 시켰고 어디까지 했나? | 임무 명령 접수, 진행 상태와 알람 발행 | `route_executor`, `go_to.py`, `serve.py` |

> 사람 감지는 두 노드로 나뉩니다: `person_detector`(감시: 카메라 한 대가 본 사람과 거리 → `person_observations`)와 `person_state`(판단: 관측 + LiDAR → `person_detection`). 카메라가 늘면 `person_detector` 만 늘고 판단은 `person_state` 한 곳입니다.

### 코드 위치 (`ros_ws/src/serving_robot_navigation/`)

| 구획 | 순수 로직 (`serving_robot_navigation/<구획>/`, ROS 없이 테스트) | 노드 (`scripts/<구획>/`) |
|---|---|---|
| 감시 `monitoring` | `scan_filter.py`(지도 장애물 제거), `clearance.py`(앞쪽 여유 거리, 접촉 이름), `detector_watch.py`(감지기 생존 확인), `person_observation.py`(카메라 관측 JSON, 사람 거리) | `dynamic_scan_filter.py`, `person_detector.py` |
| 판단 `decision` | `person_safety.py`(사람 상태 NORMAL/SLOW/STOP/SENSOR_STALE), `person_judge.py`(관측 → `person_detection` 메시지) | `person_state.py` |
| 구동 `motion` | `person_limit.py`(사람 상태 → 속도 제한), `soft_stop.py`(부드러운 정지, 재출발 규칙) | `person_speed_limiter.py`, `soft_stop_limiter.py` |
| 제어 `command` | `mission.py`(임무 상태 기계: 대기 알람, 우회, 배달 시간) | `route_executor.py`, `go_to.py`, `serve.py` |
| 공통 (루트) | `site_config.py`(목적지, 경로 그래프, 구역), `interfaces.py`(토픽/서비스 이름) | |

`ros2 run serving_robot_navigation <노드>.py` 이름은 폴더와 상관없이 그대로입니다.

---

## 2. 공통 규칙

| 항목 | 규칙 |
|---|---|
| 이름의 기준 | **`config/common/interfaces.yaml`** 이 토픽/서비스 이름의 단일 출처. `{robot}` 자리표시자를 로봇 이름으로 바꿔 씀 (기본 `robot_01`). 노드와 launch는 여기서 이름을 읽고, launch 인자 `robot_name:=robot_02` 또는 노드 파라미터 `robot_name`으로 바꿈. 한 이름만 덮어쓸 때는 기존 노드 파라미터(`cmd_topic` 등) 사용 |
| 고정 YAML과의 일치 | `bridge.yaml`, `nav2.yaml`, `amcl.yaml`, `slam.yaml`, `collision_monitor.yaml`은 `{robot}`을 못 써서 `python3 scripts/validate_interfaces.py`가 이름 일치를 검사 (단위 테스트에도 포함). 실행 중인 로봇은 `scripts/dev/check_interfaces_live.py`로 이름, 타입, 메시지 수신을 확인 |
| Nav2 내부 토픽 | 접두어 없음 (`cmd_vel_nav`, `cmd_vel_smoothed` 등). UI가 직접 쓸 일 없음 |
| 단위 | m, m/s, rad/s, s (SI). 필드 이름에 `_m`, `_mps`, `_rps`, `_s` 접미사 |
| 시간 | `time` 필드 = 시뮬레이션 시계(초, 소수 1자리). 시뮬레이션은 실시간보다 느릴 수 있음 → **시간 차이는 반드시 같은 토픽의 `time`끼리 비교** |
| 메시지 형식 | UI가 읽는 상태, 알람, 명령은 `std_msgs/String` 안의 **JSON 한 줄** |
| QoS | 별도 표기가 없으면 기본(RELIABLE, VOLATILE, depth 10). `latched`로 표시한 토픽은 TRANSIENT_LOCAL(늦게 접속해도 마지막 값을 바로 받음) |
| 상태(state) vs 이벤트(event) | **상태**는 주기적으로 계속 발행(마지막 값이 현재 값). **이벤트**(알람)는 일이 생겼을 때 발행. 화면에 "지금 어떤지"를 보여줄 때는 상태 토픽을 쓰고, 알람은 상태를 설명하는 보조로 씁니다 |

---

## 3. 구획별 토픽

### 3.1 감시

| 토픽 | 타입 | 발행 | 구독 | 주기 / QoS | 내용 |
|---|---|---|---|---|---|
| `/robot_01/scan` | `sensor_msgs/LaserScan` | Gazebo 브리지 (실물: LiDAR 드라이버) | `dynamic_scan_filter`, `collision_monitor`(하드 스톱) | 센서 QoS | 원본 LiDAR |
| `/robot_01/scan_dynamic` | `sensor_msgs/LaserScan` | `dynamic_scan_filter` | `collision_monitor`(정지/감속 구역), `soft_stop_limiter`, `person_state`, `person_speed_limiter` | 센서 QoS | 지도의 벽, 고정 장애물을 뺀 스캔 (움직이는 것만 남김) |
| `/robot_01/camera_front/image` | `sensor_msgs/Image` | 브리지 | `person_detector` | 10 Hz (320x240) | 전방 RGB (120°) |
| `/robot_01/camera_front/depth` | `sensor_msgs/Image` | 브리지 | `person_detector` | 10 Hz | 전방 깊이 |
| `/robot_01/camera_front/camera_info` | `sensor_msgs/CameraInfo` | 브리지 | `person_detector` | | 초점거리(fx, cx)를 읽어 사람 방향 계산에 씀 |
| `/robot_01/camera_rear_left/image`, `/robot_01/camera_rear_left/depth`, `/robot_01/camera_rear_left/camera_info` | Image, Image, CameraInfo | 브리지 | `person_detector` | 5 Hz | 좌후방(+120°) 카메라 (120°) |
| `/robot_01/camera_rear_right/image`, `/robot_01/camera_rear_right/depth`, `/robot_01/camera_rear_right/camera_info` | Image, Image, CameraInfo | 브리지 | `person_detector` | 5 Hz | 우후방(−120°) 카메라 (120°) |
| `/robot_01/contact` | `ros_gz_interfaces/Contacts` | 브리지 | `soft_stop_limiter` | | 충돌(접촉) 센서 |
| `/robot_01/odom`, `/tf`, `/clock` | Odometry, TF, Clock | 브리지 | Nav2, 실행기 등 | | 위치/시간 |
| `/robot_01/person_observations` | `std_msgs/String` (JSON) | `person_detector` (카메라마다 하나) | `person_state` | 4 Hz (`rate_hz`), 새 영상이 없어도 발행 | 카메라가 본 사람과 거리 (아래 5.6). 내부용: UI가 읽을 필요 없음 |

### 3.2 판단

| 토픽 | 타입 | 발행 | 구독 | 주기 / QoS | 내용 |
|---|---|---|---|---|---|
| **`/robot_01/person_detection`** | `std_msgs/String` (JSON) | `person_state` | `person_speed_limiter`, `route_executor`(감지기 생존 확인, `silent_cameras`), `robot_health`(`silent_cameras`) | 관측마다 (4 Hz) | 사람 상태 (아래 5.1). 관측이 끊기면 이것도 끊김 = 감지기 응답 없음 |
| **`/robot_01/person_limit`** | `std_msgs/String` (JSON) | `person_speed_limiter` | `route_executor`, UI | 감지기 메시지마다 + 0.5 s 마다 | **지금 실제로 적용 중인 사람 제한**. 속도 제한과 실행기가 같은 결정을 보게 하는 단일 출처 (아래 5.2) |
| `/robot_01/obstacle_hold` | `std_msgs/Bool` | `soft_stop_limiter` | `route_executor`, `person_state`, `robot_health` | 값이 바뀔 때, latched | 장애물 때문에 멈춰서 길이 비기를 기다리는 중 |
| `/robot_01/collision_latched` | `std_msgs/Bool` | `soft_stop_limiter` | `route_executor`, `robot_health`, UI | 값이 바뀔 때, latched | 충돌 후 멈춤(잠금) 상태. 해제는 서비스 `clear_collision` |
| **`/robot_01/health`** | `std_msgs/String` (JSON) | `robot_health` | Fleet, UI, `route_executor` | 1 s 마다 + 상태가 바뀌면 바로 | **로봇 요약 상태**: 팀 enum 의 `robot_state`, `safety_state`, `localization_state` + 위치 + 이유 (아래 5.7). 끊기면 Fleet 이 OFFLINE 판정 |
| `collision_monitor_state` | `nav2_msgs/CollisionMonitorState` | `collision_monitor` | `soft_stop_limiter` | | 어느 구역(HardStop/StopZone/SlowZone)이 발동했나 |

### 3.3 구동 (속도 명령 사슬)

```
Nav2 controller → cmd_vel_nav → velocity_smoother → cmd_vel_smoothed
  → person_speed_limiter → cmd_vel_semantic        (person_limit:=false 이면 이 단계 없음)
  → collision_monitor    → cmd_vel_safe
  → soft_stop_limiter    → /robot_01/cmd_vel       (로봇 바퀴로 가는 최종 명령)
```

- 모두 `geometry_msgs/Twist` (m/s, rad/s). **최종 `/robot_01/cmd_vel`을 쓰는 노드는 `soft_stop_limiter` 하나뿐**입니다. 다른 노드(UI 포함)는 여기에 직접 발행하지 마세요 (사람이 조종해야 하면 사슬의 앞단에 끼워 넣는 방식으로 별도 설계).
- UI는 읽기 전용으로 `/robot_01/cmd_vel`(실제 속도)과 `cmd_vel_smoothed`(제한 전 의도) 비교용으로만 구독할 수 있습니다.

### 3.4 제어

| 토픽 | 타입 | 발행 | 구독 | 주기 | 내용 |
|---|---|---|---|---|---|
| **`/robot_01/mission_cmd`** | `std_msgs/String` (JSON) | UI, `go_to.py`, `serve.py`, `decide.py` | `route_executor` | 이벤트 | 임무 명령 (5.5) |
| **`/robot_01/mission_status`** | `std_msgs/String` (JSON) | `route_executor` | UI, `go_to.py`, `serve.py`, `decide.py` | 2 Hz | 임무 진행 상태 (5.3) |
| **`/robot_01/alarm`** | `std_msgs/String` (JSON) | `route_executor` | UI, `go_to.py`, `serve.py`, `decide.py` | 이벤트 | 운영자에게 알릴 일 (5.4, 6) |

---

## 4. 서비스와 액션

| 이름 | 타입 | 서버 | 하는 일 | 응답 |
|---|---|---|---|---|
| `/robot_01/request_detour` | `std_srvs/Trigger` | `route_executor` | 지금 다른 복도로 우회 요청 | success=false 면 message에 이유 (`no mission is running`) |
| `/robot_01/cancel_mission` | Trigger | `route_executor` | 임무 취소 | `canceling` / `no mission is running` / `staff must confirm the handover first` |
| `/robot_01/confirm_load` | Trigger | `route_executor` | 주방에서 적재 완료 확인 | success=false: `the robot is not waiting for a load` |
| `/robot_01/confirm_handover` | Trigger | `route_executor` | 테이블에서 수령 완료 확인 (실패 배달 후 직원 인계 확인에도 사용) | success=false: `the robot is not waiting for a handover` |
| `/robot_01/clear_collision` | Trigger | `soft_stop_limiter` | 충돌 잠금 해제 (마지막 접촉 후 `collision_release_s` 3 s 지나야 허용) | |
| `follow_path` | `nav2_msgs/FollowPath` (액션) | Nav2 `controller_server` | `route_executor`가 경로 그래프 구간을 **직선 경로**(5 cm 간격)로 보냄. 플래너를 거치지 않아 장애물을 피해 휘지 않음. UI가 직접 쓰지 않음 | |
| `navigate_to_pose` | `nav2_msgs/NavigateToPose` (액션) | Nav2 | `route_executor`가 노드에서 먼 직접 이동(`DIRECT_GOAL`)에만 사용. UI가 직접 쓰지 않음 | |

서비스와 `mission_cmd` 토픽은 같은 일을 하는 두 입구입니다. 패널은 **서비스**(성공/실패 응답을 바로 받음)를 쓰는 것을 권장합니다.

---

## 5. JSON 메시지 상세

### 5.1 `/robot_01/person_detection` (person_state → 관측마다, 4 Hz)

기존 필드는 분리 전(`person_detector` 한 노드)과 같고, 카메라 여러 대(4단계)용 필드 4개가 늘었습니다. 주 카메라(`camera_front`)의 관측마다 한 번 발행합니다.
`distance_m`/`nearest_m`/`confidence`/`bbox`는 **앞(±60°)** 사람만 봅니다. 옆 사람은 `side_m`, 뒤 사람은 `rear_m`.


| 필드 | 타입 | 뜻 |
|---|---|---|
| `state` | `"NORMAL"` `"SLOW"` `"STOP"` `"SENSOR_STALE"` | 사람/장애물 상태 (카메라 거리와 LiDAR 앞쪽 여유 중 가까운 쪽, 최악 기준) |
| `reason` | string | 이유 한 줄 (예: `LiDAR: something 0.62 m ahead`) |
| `source` | `"camera"` `"lidar"` `""` | 상태를 정한 센서 |
| `distance_m` | number/null | 안정화된 가장 가까운 거리 |
| `nearest_m` | number/null | 안정화 전 가장 가까운 거리 |
| `front_clear_m` | number/null | LiDAR 앞쪽 여유 거리 (없음/모름 = null) |
| `obstacle_hold` | bool | 외부(구동부)의 장애물 대기 신호 |
| `confidence`, `count`, `bbox` | number, int, [x1,y1,x2,y2]/null | 카메라 감지 정보 |
| `frame_age_s` | number/null | 마지막 카메라 프레임 이후 시간 |
| `camera` | string | 주 카메라 이름 (`camera_front`) |
| `time` | number | 시뮬레이션 초 |
| `side_m`, `side_camera` | number/null, string | 옆(60~120°)에서 가장 가까운 사람과 그 카메라. 0.8 m 안이면 NORMAL 을 **SLOW로만** 올림 (`reason`: `person beside ...`), 0.8+0.2 m 밖으로 1 s 지나면 해제 |
| `rear_m` | number/null | 뒤(120~180°)에서 가장 가까운 사람. **속도에 영향 없음** (정보) |
| `silent_cameras` | string[] | 새 영상이 2 s 넘게 없는 카메라. 실행기가 `CAMERA_SILENT` 알람 |

기준 거리: STOP < 0.8 m, SLOW < 1.5 m (앞), 옆 SLOW < 0.8 m. 방향 기준은 `runtime_limits.yaml` 의 `person_safety` (`runtime_limits.yaml`의 `person_stop_distance_m`, `person_slow_distance_m`).

### 5.2 `/robot_01/person_limit` (person_speed_limiter → 지금 적용 중인 결정)

| 필드 | 타입 | 뜻 |
|---|---|---|
| `state` | `"NORMAL"` `"SLOW"` `"STOP"` `"SENSOR_STALE"` | **실제로 적용되는** 상태. 감지기가 멈춘 경우 규칙에 따라 `person_detection`과 다를 수 있음 |
| `silent` | bool | 감지기 메시지가 1.0 s(`message_timeout_s`) 넘게 없음 = **감지기 응답 없음 상태** |
| `source` | `"camera"` `"lidar"` `""` | STOP일 때만 채워짐. 감지기가 멈춘 채 STOP이 유지되면 `lidar` |
| `time` | number | 시뮬레이션 초 |

속도 규칙: NORMAL 통과 / SLOW 선속도 0.10 m/s, 각속도 0.40 rad/s 이하 / STOP 전진 0, 후진과 회전은 SLOW 한도 / SENSOR_STALE 은 SLOW와 같음.

> 참고: STOP 의 "후진 허용"은 이 제한기의 규칙일 뿐입니다. 실제 속도 사슬에는 후진 명령이 나오지 않습니다(`velocity_smoother` 최소 속도 0, `allow_reversing: false`, backup 행동 없음). STOP 에서 실제로 쓰이는 것은 **제자리 회전**입니다.

> **패널이 "감지기 고장" 배지를 켤 때**는 `silent == true`를 쓰세요. 이 토픽 자체가 2초 넘게 안 오면 `person_speed_limiter`가 죽은 것이니 "알 수 없음"으로 표시합니다.

### 5.3 `/robot_01/mission_status` (route_executor → 2 Hz)

| 필드 | 타입 | 뜻 |
|---|---|---|
| `state` | `"IDLE"` `"DRIVING"` `"WAIT_LOAD"` `"WAIT_HANDOVER"` `"PAUSED"` | 임무 상태. PAUSED = 배달 실패 후 주방 복귀, 직원 인계 확인 대기 |
| `mission_id` | string | 현재(또는 마지막) 임무 번호 |
| `destination` | string | 현재 목적지 이름 |
| `leg`, `legs` | int | 현재 구간 / 전체 구간 |
| `stopped_s` | number | 멈춰 있는 시간: 정지 신호(`hold`, `semantic_stop`) 또는 실제로 안 움직임(`stalled`). 움직이면 0. 15 s에 BLOCKED, 30 s에 우회 |
| `stalled` | bool | 주행 중인데 3 s 동안 5 cm 이동도 0.15 rad 회전도 없음 (정지 신호가 없어도 멈춘 것으로 셈) |
| `localization` | `""` `UNINITIALIZED` `OK` `DEGRADED` `LOST` | 실행기가 health 에서 읽은 위치 상태 (`""` = 모름: health 가 3 s 넘게 없음) |
| `loc_hold` | `""` `"LOST"` `"RECOVERED"` | 위치를 잃어 임무를 보류 중. `LOST` = 위치 복구 대기, `RECOVERED` = 위치는 OK, 사용자의 `resume`/`home` 선택 대기. 이때 `state`는 `DRIVING` 그대로이고 타이머(15 s/30 s/배달 시간)는 멈춤 |
| `detours` | int | 사용한 우회 횟수 (최대 3) |
| `avoiding` | string[] | 피하는 중인 복도 |
| `serve`, `returning` | bool | 서빙 임무 여부 / 지연으로 주방 복귀 중 |
| `table` | string | 서빙 대상 테이블 |
| `delivery_s` | number | 배달 경과 시간 (제한 180 s) |
| `wait_s` | number | 적재/수령 대기 시간 |
| `node` | string | 로봇이 있는(또는 방금 떠난) 경로 그래프 노드. 대기 중이면 그 자리 노드, 그래프에서 벗어났거나 모르면 `""` |
| `next_edge` | string | 지금 달리는 복도(edge id). 달리지 않으면 `""` |
| `route` | string[] | 앞으로 지날 노드 (다음 노드부터, 남은 목적지까지 포함). 달리지 않거나 직접 목표(DIRECT_GOAL)면 `[]`. 로봇끼리 경로 조정(백엔드)용 읽기 전용 정보이며 로봇 동작은 바뀌지 않음 |
| `result` | `""` `"SUCCEEDED"` `"FAILED"` `"CANCELED"` `"RETURNED"` | 마지막 임무 결과 |
| `code`, `text` | string | 결과 사유 (아래) |
| `hold` | bool | 구동부의 장애물 대기 (`obstacle_hold`) |
| `semantic_stop` | bool | 사람/LiDAR 판단으로 STOP 적용 중 (`person_limit.state == STOP`) |
| `semantic_source` | `"camera"` `"lidar"` `""` | 그 STOP의 출처 |
| `latched` | bool | 충돌 잠금 중 |
| `time` | number | 시뮬레이션 초 |

`result`가 `FAILED`일 때 `code`: `BAD_DESTINATION`, `NO_POSE`, `NAV_UNAVAILABLE`, `NO_ROUTE`, `NAV_FAILED`. `RETURNED`일 때 `DELIVERY_OVERTIME`.

### 5.4 `/robot_01/alarm` (route_executor → 이벤트)

| 필드 | 타입 | 뜻 |
|---|---|---|
| `code` | string | 알람 종류 (6장 표) |
| `level` | `"info"` `"warn"` | 심각도 |
| `text` | string | 사람이 읽는 설명 (영어, 화면 표시용 한글 문구는 패널이 `code`로 매핑 권장) |
| `mission_id`, `destination` | string | 관련 임무, 목적지 |
| `zone` | string | 로봇이 있는 구역 (없으면 `""`) |
| `time` | number | 시뮬레이션 초 |
| (추가 필드) | | `BLOCKED`는 `cause`(`obstacle` 정지 신호 / `not_moving` 신호 없이 안 움직임). `OFF_RAIL`은 `off_m`, `x`, `y`. `DELIVERY_OVERTIME_STUCK`는 `x`, `y` 포함. 주문을 거절한 `BUSY`, `HANDOVER_REQUIRED`, `LOCALIZATION_HOLD`, `LOCALIZATION_LOST`, `LOCALIZATION_UNINITIALIZED`는 `rejected_id`(거절된 주문의 `id`) 포함: 보낸 쪽은 이 값이 자기 `id`면 기다리기를 멈춤 (`mission_id`는 실행 중인 임무라 다름). `LOCALIZATION_LOST`, `LOCALIZATION_RECOVERED`는 `destination`, `LOCALIZATION_RECOVERED`는 `choices`(`["resume","home"]`), `home`(홈 이름), `localization`(`OK`/`DEGRADED`) 포함 |

### 5.5 `/robot_01/mission_cmd` (UI → route_executor)

| 명령 | JSON |
|---|---|
| 이동 | `{"cmd":"go","id":"m1","names":["kitchen","table_01"]}` (`id`는 생략 가능) |
| 서빙 | `{"cmd":"serve","id":"m2","table":"table_01"}` (주방 적재 → 테이블 수령 → 홈). `"home":false` 를 더하면 수령 확인에서 임무가 `SUCCEEDED` 로 끝나고 로봇은 그 자리에 IDLE 로 남음 (홈 복귀는 따로 `go` 로). 생략하면 지금처럼 홈까지 |
| 적재 확인 | `{"cmd":"confirm_load"}` |
| 수령 확인 | `{"cmd":"confirm_handover"}` |
| 취소 | `{"cmd":"cancel","id":"m2"}`: `id`가 실행 중인 임무와 같을 때만 취소 (다르면 `BAD_COMMAND`). `id`를 빼면 실행 중인 임무를 취소 |
| 우회 요청 | `{"cmd":"detour"}` |
| 이어가기 | `{"cmd":"resume"}`: `LOCALIZATION_RECOVERED` 뒤에만. 지금 위치에서 같은 목적지로 경로를 다시 잡아 주행 (그 전에는 `BAD_COMMAND`) |
| 비상 초기화 | `{"cmd":"reset","node":"kitchen"}`: 아무 때나(주행·정지·보류·대기). 사람이 로봇을 그 노드 위에 올려놓은 뒤 보냄. 임무를 `CANCELED`(`code: RESET`)로 끝내고 모든 타이머·보류를 비운 뒤 그 노드 좌표를 AMCL `/initialpose`에 알리고 IDLE. 알 수 없는 노드면 `BAD_COMMAND`. 위치가 아직 LOST 면 새 주문은 계속 거절 |
| 홈으로 | `{"cmd":"home"}`: `LOCALIZATION_RECOVERED` 뒤에만. 임무를 `CANCELED`(`code: LOCALIZATION_LOST`)로 끝내고, `<임무id>_home` 이름의 새 임무로 홈(`standby_01`)까지 이동 |

읽을 수 없는 명령은 `BAD_COMMAND` 알람으로 돌아옵니다. 목적지 이름은 `config/common/waypoints.yaml`의 이름/별칭입니다.

### 5.6 `/robot_01/person_observations` (person_detector → person_state, 내부용)

| 필드 | 타입 | 뜻 |
|---|---|---|
| `camera` | string | 카메라 이름 (`camera_front`, `camera_rear_left`, `camera_rear_right`). 한 노드가 카메라마다 한 건씩, 주 카메라를 마지막에 보냄 |
| `time` | number | 카메라 노드의 시뮬레이션 초 |
| `new_frame` | bool | 이번 주기에 새 영상을 처리했나. `false`여도 발행 = 카메라 노드가 살아 있다는 신호 (판단 노드는 2초 넘게 새 영상이 없으면 `SENSOR_STALE`) |
| `count` | int | 찾은 사람 수 (거리를 못 잰 사람 포함) |
| `people` | list | `{distance_m: number/null, bearing_rad: number/null, confidence: number, bbox: [x1,y1,x2,y2]}`. `distance_m` = 카메라에서 **실제 거리**(깊이 영상은 광축 방향 거리라 픽셀 각도로 보정: 깊이 / cos(각)), null = 사람은 보였지만 깊이 값을 못 씀. `bearing_rad` = 로봇에서 본 방향(0 정면, + 왼쪽) |


### 5.7 `/robot_01/health` (robot_health → 1 s 마다 + 변화 즉시)

상태 값은 **시스템 통합 개발 가이드 Q3-3 (MUST)** 의 enum 입니다. 세 상태는 한 필드에 합치지 않습니다 (상태 분리 원칙).

| 필드 | 타입 | 뜻 |
|---|---|---|
| `schema_version` | int | 1 |
| `robot_id`, `time` | string, number | 로봇 이름, 시뮬레이션 초 (생존 신호: 끊기면 Fleet 이 `OFFLINE`) |
| `robot_state` | `BOOTING` `IDLE` `NAVIGATING` `PAUSED` `BLOCKED` `DEGRADED` `ERROR` | 아래 우선순위로 하나. `OFFLINE` 은 Fleet 이 정함 |
| `safety_state` | `NORMAL` `SLOW` `STOP` | 적용 중인 사람 제한(`person_limit`, SENSOR_STALE → SLOW), 장애물 대기/충돌 잠금/속도 제한 노드 침묵이면 STOP |
| `localization_state` | `UNINITIALIZED` `OK` `DEGRADED` `LOST` | AMCL 표준편차 0.25 m / 0.2 rad 이하 OK, 넘으면 DEGRADED, 1.0 m 이상 또는 map→base TF 2 s 없음 LOST |
| `pose` | object/null | `{frame_id, x_m, y_m, yaw_rad, map_revision}` (팀 공통 Pose DTO) |
| `localization` | object | `{std_xy_m, std_yaw_rad}` (null = 아직 모름) |
| `mission` | object | `{state, mission_id, destination, stopped_s, loc_hold}` (실행기 상태 그대로, 업무 상태는 따로. `loc_hold`: `""` `LOST` `RECOVERED`, 5.3) |
| `checks` | object | `{detector: ok/silent/unknown/off, silent_cameras, scan: ok/stale, odom: ok/stale, obstacle_hold, collision_latched}` |
| `reasons` | string[] | 정상이 아닌 이유 (`UPPER_SNAKE_CASE`): `COLLISION_LATCHED`, `LOCALIZATION_LOST`, `LOCALIZATION_UNINITIALIZED`, `SCAN_STALE`, `MISSION_STATUS_SILENT`, `BLOCKED`, `LOCALIZATION_DEGRADED`, `ODOM_STALE`, `PERSON_LIMIT_SILENT`, `DETECTOR_SILENT`, `CAMERA_SILENT`, `LOCALIZATION_HOLD`, `BOOTING` |

`robot_state` 우선순위 (먼저 걸리는 것):

| 순서 | 상태 | 조건 |
|---|---|---|
| 1 | `ERROR` | 충돌 잠금, 위치 LOST, LiDAR 2 s 끊김, 실행기 침묵, 시작 60 s 안에 준비 안 됨 |
| 2 | `BOOTING` | LiDAR, 위치, 실행기 상태를 아직 다 못 받음 |
| 3 | `BLOCKED` | 주행 중 15 s 넘게 정지 (`BLOCKED` 알람과 같은 시점). 위치 보류 중(`loc_hold`)은 제외 |
| 4 | `DEGRADED` | 감지기/카메라 침묵, 위치 DEGRADED, 속도 제한 노드 침묵, 오도메트리 끊김 |
| 5 | `PAUSED` | 적재/수령 대기, 배달 실패 후 대기, 위치 보류 중 (`LOCALIZATION_HOLD`, 위치가 OK 일 때만. 아래 참고) |
| 6 | `NAVIGATING` / `IDLE` | 주행 중 / 임무 없음 |

> 위치 보류 중의 `robot_state`: 위치가 LOST 인 동안은 `ERROR`(우선순위 1), 복구 뒤 DEGRADED 면 `DEGRADED`, OK 면 `PAUSED` 입니다. 어느 경우든 `reasons` 에 `LOCALIZATION_HOLD` 가 들어갑니다. 보류 여부는 `robot_state` 가 아니라 `mission.loc_hold` 로 판단하세요.

`robot_health` 노드는 **보고만** 합니다 (로봇을 직접 움직이지 않음). 다만 `route_executor` 가 `localization_state` 를 읽어 위치 LOST 때 임무를 보류합니다 (6장 `LOCALIZATION_*`). 기준값은 `runtime_limits.yaml` 의 `health`, `localization_hold` [제안].

---

## 6. 알람 계약

**발행 쪽(노드)이 하는 일**: 이벤트를 발행한다. 계속되는 문제는 주기적으로 다시 발행해서 "아직 안 고쳐졌다"를 알린다.
**표시 쪽(패널)이 해야 하는 일(권장 규칙)**:
1. 알람을 **`code`(+`destination`)별로 한 칸**만 보여준다. 같은 코드가 또 오면 새 줄을 만들지 않고 그 칸의 "마지막 수신 시각"만 갱신한다.
2. 짝이 되는 해제 이벤트나 상태 조건이 오면 그 칸을 지우거나 "복구됨"으로 바꾼다.
3. 반복 알람은 마지막 수신 후 **반복 주기의 3배** 동안 안 오면 "발행 노드 응답 없음"으로 본다.

| `code` | level | 언제 | 반복 | 해제 조건 |
|---|---|---|---|---|
| `DETECTOR_SILENT` | warn | 사람 감지기 응답 없음: `person_detection`이 끊김 (`person_detector` 또는 `person_state`가 멈춤). 로봇은 LiDAR + Collision Monitor로 계속 주행 | 30 s 마다 (`detector_alarm_repeat_s`) | `DETECTOR_RECOVERED` 수신 (또는 `person_limit.silent == false`) |
| `DETECTOR_RECOVERED` | info | 감지기 복구 | 1회 | (해제 이벤트) |
| `BLOCKED` | warn | 15 s 넘게 정지: 장애물/사람 정지 신호, 또는 신호 없이 안 움직임 (`cause`) | 1회 (다시 막히면 또 발행) | `mission_status.stopped_s == 0` |
| `CAMERA_SILENT` | warn | 카메라 하나가 2 s 넘게 영상이 없음 (`camera` 포함). 감지 노드를 다시 켠 직후에는 카메라마다 2 s 유예 (재시작 때 알람이 쏟아지지 않게). 옆/뒤 카메라는 알람만, 앞 카메라는 SENSOR_STALE 로 SLOW 상한도 걸림 | 30 s 마다 | `CAMERA_RECOVERED` (같은 `camera`) |
| `CAMERA_RECOVERED` | info | 그 카메라 영상이 다시 옴 | 1회 | (해제 이벤트) |
| `OFF_RAIL` | warn | 직선 레일에서 0.15 m 넘게 벗어남 → 정지, 3 s 뒤 그 자리에서 다음 노드로 직선 재출발 | 벗어날 때마다 | 정보성 |
| `DETOUR_STARTED` | warn | 30 s 정지 후 우회 시작 (또는 우회 요청) | 우회마다 1회 | 정보성, 칸을 유지하지 않고 로그로만 |
| `NO_DETOUR` | warn | 우회할 다른 복도 없음 / 경로 그래프를 따르는 중이 아님 | 1회 | `stopped_s == 0` |
| `DETOUR_LIMIT` | warn | 우회 3회 모두 사용 | 1회 | `stopped_s == 0` |
| `DELIVERY_OVERTIME_STUCK` | warn | 배달 제한(180 s) 초과인데 정지 중이라 복귀 못 함 (`x`,`y` 포함) | 1회 | 임무 종료 |
| `DELIVERY_OVERTIME` | warn | 배달 180 s 초과 → 주방 복귀 시작 | 1회 | 정보성 |
| `RETURN_STARTED` | info | 주방으로 복귀 주행 시작 | 1회 | 정보성 |
| `RETURNED_TO_KITCHEN` | warn | 실패한 배달 후 주방 도착, 신규 주문 보류 | 1회 | `HANDOVER_CONFIRMED` |
| `HANDOVER_REQUIRED` | warn | 보류 중에 새 주문이 들어옴 → 직원이 먼저 인계 확인 필요 | 요청마다 | `HANDOVER_CONFIRMED` |
| `HANDOVER_CONFIRMED` | info | 직원 인계 확인, 신규 주문 가능 | 1회 | (해제 이벤트) |
| `WAIT_LOAD` | info | 주방에서 적재 대기 (최초 + 반복) | 30 s 마다 (`wait_repeat_s`) | `confirm_load` 처리 |
| `WAIT_HANDOVER` | info | 테이블에서 수령 대기 (최초) | 1회 | `confirm_handover` 처리 |
| `HANDOVER_TIMEOUT` | warn | 60 s 동안 아무도 안 가져감, 계속 대기 | 30 s 마다 | `confirm_handover` 처리 |
| `DIRECT_GOAL` | info | 경로 노드에서 0.6 m 넘게 떨어져 있어 직접 이동(우회 불가) | 1회 | 정보성 |
| `LOCALIZATION_LOST` | warn | 주행 중 위치 LOST → 정지, 임무 보류. 대기 중(적재/수령)이면 알람만. 새 주문이 LOST 때문에 거절될 때도 이 코드 (`rejected_id` 포함) | 보류 시작 1회 + 30 s 마다 (`wait_repeat_s`) | `LOCALIZATION_RECOVERED` |
| `LOCALIZATION_UNINITIALIZED` | warn | 아직 위치가 없어 새 주문 거절 (`rejected_id` 포함) | 요청마다 | 정보성 |
| `LOCALIZATION_RECOVERED` | warn | 위치가 LOST 를 벗어나(OK 또는 DEGRADED) 2 s 연속: **사용자가 `resume` 또는 `home` 을 선택해야 함** (`choices`, `destination`, `home`, `localization`). DEGRADED 면 문구에 "아직 거침"이 붙음 (멈춘 로봇은 AMCL 오차를 못 줄이므로 OK 를 기다리지 않음) | 30 s 마다 (선택할 때까지 재알림) | `MISSION_RESUMED` 또는 `GO_HOME` |
| `LOCALIZATION_HOLD` | warn | 보류 중에 새 주문이 들어옴 → 거절 (`rejected_id` 포함) | 요청마다 | 정보성 |
| `MISSION_RESUMED` | info | 사용자가 `resume` 선택, 같은 목적지로 주행 | 1회 | (해제 이벤트) |
| `ROBOT_RESET` | info | 사용자가 `reset <노드>` 보냄: 임무·보류 비움, IDLE | 1회 | (해제 이벤트) |
| `GO_HOME` | info | 사용자가 `home` 선택, 임무 취소 후 홈으로 | 1회 | (해제 이벤트) |
| `BUSY` | warn | 임무 진행 중에 새 임무 요청 | 요청마다 | 정보성 |
| `BAD_COMMAND` | warn | 읽을 수 없거나 모르는 명령, 지금 상태에서 불가능한 확인/우회, 실행 중이 아닌 임무 `id`로 온 취소 | 요청마다 | 정보성 |

> 표의 "정보성"은 칸을 유지할 필요 없이 이벤트 로그/토스트로 한 번 보여주면 된다는 뜻입니다.

---

## 7. UI가 쓰는 것만 모은 요약

**읽기 (구독)**

| 무엇을 보여주나 | 토픽 | 쓰는 필드 |
|---|---|---|
| 임무 진행 | `mission_status` | `state`, `destination`, `leg/legs`, `delivery_s`, `result/code/text` |
| 멈춤/막힘 | `mission_status` | `stopped_s`, `hold`, `semantic_stop`, `semantic_source`, `detours`, `avoiding` |
| 사람 근접 상태 | `person_limit` | `state`, `source` |
| 로봇 전체 상태 (권장) | `health` | `robot_state`, `safety_state`, `localization_state`, `reasons`, `pose` |
| 감지기 고장 | `person_limit` | `silent` |
| 사람 거리(디버그) | `person_detection` | `distance_m`, `front_clear_m`, `reason` |
| 충돌 잠금 | `collision_latched` (또는 `mission_status.latched`) | Bool |
| 알람 | `alarm` | 6장 규칙 |
| 위치 보류 | `mission_status` | `loc_hold` (`LOST` 복구 대기 / `RECOVERED` 선택 대기), `localization` |
| 위치 | `/robot_01/odom`, `/tf` | |

**쓰기 (명령)**

| 하는 일 | 방법 |
|---|---|
| 이동 / 서빙 | `mission_cmd`의 `go` / `serve` |
| 적재 / 수령 확인 | 서비스 `confirm_load` / `confirm_handover` |
| 취소 / 우회 | 서비스 `cancel_mission` / `request_detour` |
| 충돌 해제 | 서비스 `clear_collision` |
| 위치 복구 후 선택 | `mission_cmd`의 `resume` (이어가기) / `home` (취소 후 홈으로). 서비스는 아직 없음 (`LOCALIZATION_RECOVERED` 알람 뒤에만 가능) |

---

## 8. 다른 파트와 맞춰야 할 것 (확인 필요)

1. `serving_robot_interfaces/msg/PersonSafetyEvent.msg`(`robot_id`, `state`, `confidence`, `distance_m`)가 있지만 **현재 어떤 노드도 쓰지 않습니다.** 패널/Fleet이 이 메시지를 기대하고 있는지 팀 확인이 필요합니다. 쓰기로 하면 `person_limit`을 이 형식으로 한 번 더 발행하는 식으로 맞출 수 있습니다.
2. `serving_robot_perception`의 `person_detector.py`, `semantic_safety.py`, `depth_fusion.py`는 **빈 파일**입니다. 실제 사람 감지는 `serving_robot_navigation`의 `person_detector`(카메라)와 `person_state`(판단) 노드가 합니다. 이름이 겹치므로 팀에서 정리 필요.
3. 이름은 `interfaces.yaml` 한곳에서 읽도록 정리됐습니다(2단계). 다만 다중 로봇으로 완전히 가려면 Gazebo 쪽(`robot_base.launch.py`, 스폰, URDF), Nav2 고정 YAML의 이름 공간 처리가 따로 필요합니다 (Fleet 설계 때 같이 결정).
4. 알람의 `text`는 영어입니다. 한글 문구는 패널이 `code`로 매핑하세요.
5. 로봇 전체 상태는 `/robot_01/health`(5.7, 팀 enum)로 모았습니다. Fleet 의 `RobotState`(02 설계안) 가 이 값을 쓸지, 정식 `.msg` 타입은 IC-0 계약 동결 때 팀과 확정합니다.

---

## 9. 새 모듈을 꽂을 때

구획마다 README 에 "꽂는 순서"가 있습니다 (`ros_ws/src/serving_robot_navigation/serving_robot_navigation/<구획>/README.md`).

| 하고 싶은 일 | 볼 곳 |
|---|---|
| 새 센서, 새 감지 | `monitoring/README.md` |
| 새 판단 규칙 | `decision/README.md` |
| 새 속도 제한 단계 | `motion/README.md` |
| 새 명령, 새 알람, 새 제어 노드 | `command/README.md` |

공통 규칙:
- 각 구획 launch(`launch/<구획>.launch.py`)의 **MODULES** 목록에 노드를 한 줄 추가하면 꽂힙니다. `navigation.launch.py`가 구획 launch를 포함하므로 `operation_mode`는 그대로입니다. 구획만 따로 띄워 시험할 수도 있습니다 (예: `ros2 launch serving_robot_navigation monitoring.launch.py`).
- 새 토픽/서비스 이름: `config/common/interfaces.yaml` 한 줄 → 이 문서 표 한 줄 → `scripts/dev/check_interfaces_live.py` 의 `TYPES` 한 줄.
- 끝나면 PC에서 `python3 scripts/validate_layout.py`(빠뜨린 연결), `python3 scripts/validate_interfaces.py`(이름 일치), `tests/robot/` 의 테스트를 모두 통과시킵니다.

---

변경 이력
- v0.14 (2026-10-22): `serve` 명령에 선택 옵션 `home`(기본 true). `false` 면 수령 확인에서 임무 종료. 기존 동작 그대로.
- v0.13 (2026-10-21): `mission_status` 에 `node`, `next_edge`, `route` 추가 (경로 정보 노출, 동작 변경 없음). 필드 이름은 자율주행 파트 제안이며 팀 합의 필요. 기존 필드/명령은 그대로.
- v0.12 (2026-10-20): 코드와 대조해 설명만 바로잡음(필드, 토픽, 동작 변경 없음). front `camera_info` 구독자, `robot_health` 구독 목록, STOP 후진은 실제로 쓰이지 않음, 위치 보류 중 `robot_state`(ERROR/DEGRADED/PAUSED).
- v0.11 (2026-10-16): 비상 초기화 명령 `reset`(`node`), 알람 `ROBOT_RESET`, 임무 결과 코드 `RESET`, nav2 이름 `initialpose`. 기존 필드/명령은 그대로.
- v0.10 (2026-10-16): 위치 LOST 처리 (복구 알람은 LOST 를 벗어나면: OK 또는 DEGRADED). `mission_status.localization`, `loc_hold`, 명령 `resume`/`home`, 알람 `LOCALIZATION_LOST`/`LOCALIZATION_UNINITIALIZED`/`LOCALIZATION_RECOVERED`/`LOCALIZATION_HOLD`/`MISSION_RESUMED`/`GO_HOME`, health `reasons`에 `LOCALIZATION_HOLD`. 실행기가 `health`를 구독. 기존 필드/명령은 그대로.
- v0.9 (2026-10-15): `/robot_01/health` 추가 (팀 enum: robot_state / safety_state / localization_state, Pose DTO, reasons). nav2 이름에 `amcl_pose`.
- v0.8 (2026-10-15): 카메라 3대(앞, 좌후방, 우후방, 120°). 관측에 실제 거리 + `bearing_rad`, `person_detection`에 `side_m`, `side_camera`, `rear_m`, `silent_cameras`, 알람 `CAMERA_SILENT`/`CAMERA_RECOVERED`. 방향별 규칙(앞 그대로, 옆 SLOW만, 뒤 영향 없음).
- v0.7 (2026-10-15): 경로 그래프 구간을 직선 레일(`follow_path` 액션)로 주행. `mission_status.stalled`, `BLOCKED.cause`, 알람 `OFF_RAIL` 추가. 멈춤 = 정지 신호 또는 실제로 안 움직임.
- v0.6 (2026-10-15): `person_detector`를 감시(`person_detector` → 새 토픽 `person_observations`)와 판단(`person_state` → `person_detection`)으로 분리. `person_detection`의 이름과 필드는 그대로. `DETECTOR_SILENT` 설명 보완.
- v0.5 (2026-10-15): 거절 알람(`BUSY`, `HANDOVER_REQUIRED`)에 `rejected_id` 추가, `cancel`에 선택 필드 `id` 추가 (다른 클라이언트의 임무를 취소하지 않게). 기존 형식은 그대로 동작. 제목 버전 표기 수정.
- v0.4 (2026-10-15): 구획 README, 구획별 launch(monitoring/motion/command), 레이아웃 검사 추가 (4단계). 토픽/서비스/필드는 변경 없음.
- v0.3 (2026-10-15): 코드를 구획 폴더로 정리 (3단계). "코드 위치" 표 추가. 토픽/서비스/필드는 변경 없음.
- v0.2 (2026-10-15): 이름의 기준을 `config/common/interfaces.yaml`로 이동 (2단계). 노드, launch가 여기서 이름을 읽음. 검사 도구 추가.
- v0.1 (2026-10-15): 최초 작성. 코드(`route_executor`, `person_detector`, `person_speed_limiter`, `soft_stop_limiter`, `dynamic_scan_filter`, `mission`, 브리지 설정)와 대조.
