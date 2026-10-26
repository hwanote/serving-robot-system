# 감시 (monitoring)

센서를 보고 **사실만** 내보내는 구획입니다. 사실 = 거리(m), 있다/없다(bool), 센서가 살아 있는가. "그래서 멈춰라" 같은 결정은 하지 않습니다 (결정은 `decision/`).
토픽, 필드의 정확한 뜻은 `docs/INTERFACES.md` 3.1절과 5.1절을 보세요.

## 모듈 (순수 로직, ROS 없이 테스트)

| 파일 | 하는 일 |
|---|---|
| `scan_filter.py` | 지도에 있는 고정 장애물(벽, 카운터, 기둥)에 맞은 LiDAR 점을 제거 (`StaticMask`, `filter_ranges`) |
| `clearance.py` | `forward_clearance`: 몸 앞 복도에서 가장 가까운 물체까지 거리, `touched_name`: 부딪힌 바깥 물체 이름 |
| `detector_watch.py` | `DetectorWatch`: 사람 감지기가 살아 있는지 (조용해지면 SILENT, 돌아오면 RECOVERED 이벤트), `CameraWatch`: 카메라별 SILENT / 반복 / RECOVERED |
| `person_observation.py` | 카메라 한 대가 본 것: `box_distance`(사람 상자의 깊이 중앙값), `pixel_angle` / `true_range`(광축 깊이 → 실제 거리, 방향), `make_observation` / `parse_observation`(관측 JSON), `measured`(거리가 있는 사람만) |

## 노드 (`scripts/monitoring/`)

| 노드 | 읽는 것 | 내는 것 |
|---|---|---|
| `dynamic_scan_filter.py` | `scan`, `/map`, TF | `scan_dynamic` (움직이는 것만 남은 스캔) |
| `person_detector.py` | `cameras` 목록의 카메라마다 영상/깊이/camera_info, TF(카메라 방향) | `person_observations` (카메라마다 4 Hz JSON: 본 사람, 실제 거리, 방향). 새 영상이 없어도 매 주기 보냄(살아 있다는 신호). YOLO 모델 하나로 주 카메라는 매 주기, 나머지는 돌아가며 (VM 성능). 상태는 판단 구획의 `person_state` 가 정함 |

토픽 이름은 `config/common/interfaces.yaml` 에서 읽습니다 (노드에 `/robot_01/...`을 직접 쓰지 않기).

## 이 구획의 규칙
- 값이 없으면 `null`(모름)로 내보냅니다. 0이나 "멀다"로 채우지 않습니다 (모름은 보수적으로 취급되어야 하므로).
- 시간 필드 `time`은 노드의 시계(시뮬레이션 시계)입니다.
- 판단/구동 구획의 노드를 직접 부르지 않습니다. 토픽으로만 주고받습니다.

## 새 모듈 꽂는 순서 (예: 새 센서 하나)
1. `serving_robot_navigation/monitoring/<이름>.py` 에 순수 로직을 만들고 `tests/robot/` 에 테스트를 추가합니다.
2. `scripts/monitoring/<노드>.py` 에 노드를 만듭니다 (토픽 이름은 `names_for(...)` 로 읽기).
3. `CMakeLists.txt` 의 `install(PROGRAMS ...)` 에 `scripts/monitoring/<노드>.py` 를 한 줄 추가합니다.
4. `launch/monitoring.launch.py` 의 MODULES 목록에 노드를 한 줄 추가합니다.
5. 새 토픽이면 `config/common/interfaces.yaml`, `docs/INTERFACES.md` 표, `scripts/dev/check_interfaces_live.py` 의 `TYPES` 에 한 줄씩 추가합니다.
6. 이 README 의 표에 모듈/노드를 한 줄 추가합니다.
7. `python3 scripts/validate_layout.py`, `python3 scripts/validate_interfaces.py`, `tests/robot/` 의 테스트가 모두 통과하면 끝입니다.

## 테스트
- `tests/robot/test_soft_stop.py` : `forward_clearance`, `touched_name`
- `tests/robot/test_person_limit.py` : `DetectorWatch`
- `tests/robot/test_scan_filter.py` : `dilate`, `StaticMask`, `filter_ranges` (실제 데모 지도 + 노드 yaml 의 `tolerance_m` 포함. 지도를 바꾸면 이 테스트부터 돌리기)
- `tests/robot/test_person_judge.py` : `person_observation` (관측 JSON, `box_distance`)
