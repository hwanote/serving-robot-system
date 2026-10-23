# 판단 (decision)

사실(거리, 여유 거리, 장애물 대기 신호)을 모아 **상태**(NORMAL / SLOW / STOP / SENSOR_STALE)를 정하는 구획입니다. 센서를 직접 읽지도, 속도를 직접 바꾸지도 않습니다.
토픽, 필드의 뜻은 `docs/INTERFACES.md` 3.2절과 5.1, 5.2절을 보세요.

## 모듈 (순수 로직, ROS 없이 테스트)

| 파일 | 하는 일 |
|---|---|
| `person_safety.py` | `PersonSafetyCore`: 카메라 거리와 LiDAR 앞쪽 여유 거리를 **하나의 후보 목록**에 넣고 가장 가까운 것으로 상태를 정함 (최악 기준) |
| `person_judge.py` | `PersonJudge`: 카메라 관측을 받아 주 카메라 앞(±60°) 사람으로 `PersonSafetyCore` 를 돌리고, 옆(60~120°) 0.8 m 안이면 SLOW 로 올리고(STOP 없음), 뒤는 정보만, 카메라별 생존(`silent_cameras`)을 붙여 `person_detection` JSON 을 만듦. 카메라 1대일 때는 분리 전과 같은 메시지 |
| `health.py` | `HealthCore`: 로봇 요약 상태. 팀 enum(통합 가이드 Q3-3) `robot_state` / `safety_state` / `localization_state` 를 따로 정하고 `checks`, `reasons` 를 붙임 (보고만) |

규칙 요약: 중앙값 3개, 히스테리시스 0.2 m, 해제까지 3프레임, 사람이 사라져도 최소 3초 유지 후 앞이 1.0 m 이상 2.0초 비어야 해제, 정보가 없으면(모름) 막힌 것으로 취급. 기준 거리 STOP 0.8 m, SLOW 1.5 m 는 `config/common/runtime_limits.yaml`.

## 노드 (`scripts/decision/`, `launch/decision.launch.py` 가 시작)

| 노드 | 읽는 것 | 내는 것 |
|---|---|---|
| `person_state.py` | `person_observations`(카메라 노드), `scan_dynamic`(앞쪽 여유 거리), `obstacle_hold` | `person_detection` (관측이 올 때마다 JSON). 관측이 끊기면 같이 조용해져서 "감지기 응답 없음" 규칙이 그대로 적용됨 |
| `robot_health.py` | `scan`, `odom`, `mission_status`, `person_limit`, `person_detection`, `obstacle_hold`, `collision_latched`, `amcl_pose`, TF | `health` (1초마다 + 상태가 바뀌면 바로, JSON). `person_limit:=false` 여도 항상 실행 |

카메라가 여러 대여도 `person_state` 는 하나입니다 (결정은 한 곳에서). 주 카메라 관측 때만 발행해서 "감지기 응답 없음" 규칙이 그대로입니다. 옆/뒤 카메라 고장은 `CAMERA_SILENT` 알람만 (속도 영향 없음, LiDAR 가 지킴). 카메라 목록과 방향 기준은 `runtime_limits.yaml` 의 `person_safety`.

## 이 구획의 규칙
- **적용 중인 결정은 한 곳에서만** 발행하고 모두 그것을 읽습니다. 예: 속도 제한 노드가 `person_limit` 을 발행하고 경로 실행기는 감지기를 다시 읽지 않고 `person_limit` 만 읽습니다 (같은 사실을 서로 다르게 해석해 생긴 버그를 막기 위해서).
- 정보가 없거나 낡았으면 보수적으로(SLOW 이하) 판단합니다.

## 새 규칙 꽂는 순서 (예: 새 종류의 위험 판단)
1. `serving_robot_navigation/decision/<이름>.py` 에 순수 로직을 만들고 `tests/robot/` 에 테스트를 추가합니다 (시간은 인자로 받기: 테스트에서 시계를 흉내 낼 수 있게).
2. 사실은 감시 구획의 토픽에서 받고, 결정은 새 토픽(JSON)으로 내보내도록 노드를 만듭니다. `scripts/decision/` 을 처음 쓰면 `CMakeLists.txt` 와 `launch/decision.launch.py` 도 추가합니다.
3. `config/common/interfaces.yaml`, `docs/INTERFACES.md`, `scripts/dev/check_interfaces_live.py` 의 `TYPES` 에 새 토픽을 추가합니다.
4. 이 README 의 표를 갱신하고, `python3 scripts/validate_layout.py`, `python3 scripts/validate_interfaces.py`, `tests/robot/` 를 통과시킵니다.

## 테스트
- `tests/robot/test_person_safety.py`
- `tests/robot/test_health.py` : 상태 우선순위, 위치 추정 기준, 끊김 판정
- `tests/robot/test_person_judge.py` : 분리 전 노드와 같은 메시지가 나오는지 (같은 시나리오를 옛 로직과 새 로직에 넣어 매 순간 비교)
