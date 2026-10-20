# 자율주행 파트 설계 결정 기록 (DECISIONS)

아래 항목은 **그 시점에 논의를 거쳐 채택/반려된 기록**입니다. 새 세션이 같은 논의를 처음부터 다시 하지 않도록 남긴 참고용이고, 영구히 봉인한 것은 아닙니다. 상황이 바뀌었거나 더 나은 근거가 생기면 다시 꺼내 제안해도 됩니다. (뚜렷한 근거 없이 매번 다시 검토하자고 하지는 않기.)

"뭘 했는지"는 `CHECKPOINT_demo_map.md` 와 `docs/INTERFACES.md`, 여기에는 "왜 그렇게 했는지"만 둡니다.
표시: **[합의]** = 팀 계약으로 합의된 값, **[제안]** = 사용자가 승인했지만 팀과는 아직 확정하지 않은 값.

---

## A. 속도 제어와 안전

### A1. 속도 사슬과 소유권
- **채택**: `velocity_smoother → person_speed_limiter → collision_monitor → soft_stop_limiter → /robot_01/cmd_vel`. 최종 `cmd_vel`을 쓰는 노드는 `soft_stop_limiter` 하나.
- **이유**: 의미 기반 안전 필터와 Collision Monitor 는 직렬 계층이고, 최종 명령의 주인이 둘이면 서로 덮어씁니다. LiDAR/Collision Monitor 는 마지막 방어선으로 남겨야 하므로 사람 제한은 그 앞에 둠.
- **`person_limit:=false`**: 감지기와 제한 노드를 빼고 `cmd_vel_smoothed` 를 monitor 로 직결 (카메라 없이도 돌릴 수 있게).

### A2. 카메라와 LiDAR를 하나의 판단으로 합침
- **채택**: 카메라 거리와 LiDAR 앞쪽 여유 거리(`front_clear_m`)를 **하나의 후보 목록**에 넣고 가장 가까운 것으로 상태 결정(최악 기준). 이후 중앙값 3개, 히스테리시스 0.2 m, 해제 3프레임.
- **반려한 대안**: LiDAR 를 보조 "hold" 신호로만 쓰기. 실제로 (1) 사람이 0.6 m 에서 사라지면 NORMAL 로 풀렸고 (2) 0.39 m 앞에 물체가 있는데 SLOW 가 유지되는 문제가 났음.
- 사람이 사라진 뒤: 최소 3초 유지(`lost_hold_s`) 후, 앞이 1.0 m 이상 2.0초 비어야 해제. 앞 여유를 모르면 "막힌 것"으로 취급.

### A3. SLOW / STOP 속도 규칙
- **SLOW**: 선속도 ≤ 0.10 m/s **[합의]**, 각속도 ≤ 0.40 rad/s **[제안]**.
- **STOP**: 전진만 0. **후진과 회전은 SLOW 한도로 허용.** (제한기 규칙이 그렇다는 뜻. 실제 속도 사슬에는 후진 명령이 나오지 않아 쓰이는 것은 회전이다: `velocity_smoother` 최소 0, `allow_reversing: false`, backup 행동 없음. 2026-10-20 코드 대조로 추가)
  - 반려한 대안: STOP 에서 회전도 0. 우회하려고 회전해야 하는데 로봇이 얼어붙어 우회가 불가능했음(실제 시험에서 발생).
- **SENSOR_STALE / 메시지 없음 / 알 수 없는 상태 문자열**: SLOW 한도 + 경고 로그. 정지가 아니라 "느리게 갈 수 있게": 센서 고장이 곧 영구 정지가 되면 안 되고, 나머지 방어선(LiDAR, Collision Monitor)이 있음.

### A4. 감지기가 죽었을 때
- **채택**: 계속 주행(LiDAR + Collision Monitor 가 지킴) + 운영자에게 알림. 직전 상태가 STOP 이었다면 **LiDAR 가 앞에 무언가 보는 동안(또는 모를 때)만 STOP 유지**, 앞이 비면 SLOW 한도. 살아 있는 카메라 STOP 은 LiDAR 와 무관하게 유지.
- **반려한 대안**: 마지막 STOP 을 영구 유지. 감지기가 죽으면 로봇이 얼어붙어 우회도 못 했음(실제 시험).
- 사용자 의견: 문제는 알려야 하지만 다른 쪽이 커버하니 계속 움직여도 된다.

### A5. 결정은 한 곳에서만 (단일 출처)
- **채택**: `person_speed_limiter` 가 "지금 적용 중인 결정"을 `/robot_01/person_limit`(state, silent, source, time)으로 발행. 경로 실행기와 UI 는 **이것만** 읽음. 감지기의 `person_detection` 은 디버그용과 감지기 생존 확인용.
- **반려한 대안**: 실행기가 감지기의 마지막 STOP 을 따로 기억. 속도 제한은 "앞이 비어 SLOW"로 풀렸는데 실행기는 STOP 이라고 믿어, 로봇이 달리는 중에 가짜 BLOCKED / NO_DETOUR / 두 번째 우회가 나왔음.
- 이름: `person_stop` → `semantic_stop` + `semantic_source` 로 변경. 이 값은 카메라뿐 아니라 LiDAR 까지 합친 STOP 이라 이름이 오해를 줬음.

### A6. `obstacle_hold` 는 그대로 둠
- limiter 의 `holding` 은 Collision Monitor 의 STOP 이벤트로만 켜짐. 합친 의미 기반 STOP(0.8 m)이 더 일찍 로봇을 세우므로 hold 는 가까운 구역(예: 0.3 m)에 무언가 들어올 때만 켜짐. 의도된 동작이고, 막힘 판단은 `hold OR semantic_stop` 으로 함.

### A7. 감지기 실행 방식
- `person_limit:=true` 이면 launch 가 감지기와 제한 노드를 자동 시작. 수동으로 따로 띄우지 않음(죽인 뒤 다시 띄울 때만 수동). 중복 실행을 막기 위해서.

---

## B. 임무와 알람

### B1. 막힘 대기 타이머
- 규칙 **[제안]**: 멈춘 지 15초 → `BLOCKED` 알람, 30초 → 우회, 우회 최대 3회, 막힌 복도 60초 기억.
- `stuck_s` 는 `hold OR semantic_stop` 일 때 증가. **사람 → 장애물, 장애물 → 사람으로 신호가 넘어가도 타이머는 이어짐**(제3자가 보기에 "계속 멈출 이유가 있으면 30초 후 우회").
- **신호가 둘 다 꺼지는 틈이 생기면 0부터 다시.** 틈을 메우는 유예 로직(내가 제안했던 10초/1 m 유예)은 **반려**: 신호 지연으로 타이머가 다시 시작하는 쪽이 오히려 안전 측면에서 낫다는 사용자 판단. `mission.py` 는 이 이유로 바꾸지 않음.

### B2. 알람은 이벤트, 화면은 상태
- 계속되는 문제(`DETECTOR_SILENT`)는 30초마다 다시 발행(하트비트), 복구 시 `DETECTOR_RECOVERED`. 시작 후 30초는 모델 로딩 유예.
- 표시 규칙: **코드(+목적지)별로 한 칸**, 같은 코드가 또 오면 마지막 수신 시각만 갱신, 반복 주기의 3배 동안 소식이 없으면 "발행 노드 응답 없음". 해제 조건은 `docs/INTERFACES.md` 6장.
- 사용자 의견: 고치기 전까지는 계속 알려야 맞다. 쌓지 말고 마지막 메시지 기준으로 보이게.
- 반려/보류: 반복을 없애거나 줄이기. 지금 값 유지(필요하면 `detector_alarm_repeat_s` 를 yaml 에서 120 등으로 변경 가능).
- 패널의 "감지기 고장" 배지는 알람 로그가 아니라 `person_limit.silent` 를 사용.
- 알람 `text` 는 영어. 한글 문구는 패널이 `code` 로 매핑.


### B3. 직선 레일: 플래너 대신 직선 경로를 제어기에 직접 (2026-10-15)
- **문제(사용자 지적)**: 노드 사이에 사람이 있으면 로봇이 곡선을 그림. 실행기가 `NavigateToPose` 만 주고, 행동 트리가 1초마다 전역 costmap(LiDAR 장애물 포함)으로 다시 계획해 사람을 피해 휘었음 → 옆구리 접근 → HardStop(사방 0.10 m)이 회전까지 막아 굳음.
- **채택**: 경로 그래프 구간은 실행기가 직선(로봇 위치 → 노드, 5 cm 간격 "서브노드")을 만들어 `follow_path`(controller_server)로 직접 보냄. 장애물이 있어도 다시 그리지 않음 → 정면에서 멈춤(사람 STOP, StopZone). 돌아가기는 30초 우회 규칙만. 노드 회전은 RotationShim.
- 사용자 아이디어(노드 사이 서브노드를 따라가기)를 소프트웨어로 구현한 것. 서브노드를 "목표"로 하나씩 주면 플래너가 그 사이에서 또 휘므로, 핵심은 플래너를 빼는 것.
- **레일 이탈 감시**: 0.15 m 넘게 벗어나면 정지 + `OFF_RAIL`, 3초 뒤 그 자리에서 직선 재출발 [제안].
- RPP `use_collision_detection` 끔: 켜 두면 제어기가 앞 장애물에서 경로를 포기(실패)함. 멈추는 일은 사람 STOP / Collision Monitor 가 함.
- 직접 이동(노드에서 0.6 m 초과)은 여전히 `navigate_to_pose`(플래너). 
- 후보(실물 단계): 바닥 마커(QR, 자기 테이프)로 위치 보정.

### B5. 위치를 잃으면 멈추고, 복구 후 사용자가 이어갈지 홈으로 갈지 고름 (2026-10-16, 사용자 결정)
- **문제**: 위치가 LOST 여도 health 가 `ERROR` 로 알릴 뿐 로봇은 계속 달렸고 새 임무도 받았음. 위치가 틀린 채 엉뚱한 곳으로 갈 수 있음.
- **채택(사용자 지시 + 제안 3번 확정)**: 주행 중 LOST 면 즉시 정지(`CANCEL_NAV`)하고 임무를 **보류**(`LOCALIZATION_LOST`). LOST 를 벗어나(OK 또는 DEGRADED) 2초 지나면 `LOCALIZATION_RECOVERED` 알람으로 알리고 **사용자가** `resume`(같은 목적지로 이어가기) 또는 `home`(임무 취소 후 `standby_01` 로 이동)을 고름. 코어는 스스로 다시 움직이지 않음.
- 보류 중: 15/30 s 막힘 타이머, 배달 제한 시간, 정지 감지(stalled)를 멈춤(재개하자마자 시간 초과로 홈 복귀하는 것 방지). 새 주문은 거절(`LOCALIZATION_HOLD`, `rejected_id` 포함). 대기 중(적재/수령 대기)이 아닌 IDLE 에서도 LOST/UNINITIALIZED 면 새 주문 거절.
- 적재/수령 대기 중 LOST: 이미 서 있으므로 보류하지 않고 알람만.
- DEGRADED(0.25~1.0 m)는 멈추지 않음(health 보고만).
- **위치 판단은 health 한 곳**: 실행기는 `/robot_01/health` 의 `localization_state` 만 읽음(AMCL 을 따로 보지 않음). health 가 3초 넘게 없으면 "모름": 그것만으로 멈추지 않음(시작 직후 불필요한 정지를 피함). 보류 중에 health 가 사라지면 보류 유지(`cancel` 로 해제 가능).
- `resume` 은 지금 위치에서 같은 목적지로 경로를 다시 잡음(위치가 점프했을 수 있음). 이미 쓴 배달 시간은 유지.
- health: 보류 중 이유에 `LOCALIZATION_HOLD` 가 들어가고 BLOCKED 로 오인하지 않음. `robot_state` 는 LOST 동안 `ERROR`, 복구 뒤 DEGRADED 면 `DEGRADED`, OK 면 `PAUSED` (2026-10-20 코드 대조로 정정: 이전에는 "보류 중 PAUSED" 로만 적혀 있었음).
- 로봇 이름은 `robot_01`, `robot_02` … 로 늘어남: 새 코드(`decide.py --robot`, 실행기 `health_topic`)는 `interfaces.yaml` 의 `{robot}` 으로만 이름을 만듦.
- **복구 조건을 "OK"가 아니라 "LOST 를 벗어남"으로 (VM 시험 L-2 에서 발견, 사용자 승인)**: 멈춘 로봇은 AMCL 이 갱신되지 않아(움직여야 갱신) 한 번 DEGRADED(0.25~1.0 m)가 되면 영원히 OK 가 되지 못함. 실제로 RViz 2D Pose Estimate(약 0.5 m)로 위치를 알려 줘도 같음. 그래서 OK 또는 DEGRADED 가 2초 이어지면 알람을 내고, 알람에 현재 상태(`localization`)와 "아직 거침" 문구를 붙임. 선택은 사용자가 하고, `resume` 으로 달리면 AMCL 이 수렴함. 필요하면 상한(`recover_max_std_m`)을 둘 수 있음(팀 확인 항목).
- 값 [제안]: `localization_hold.recover_stable_s` 2.0, `health_max_age_s` 3.0.
- 서비스(Trigger)는 아직 없음: `mission_cmd` 의 `resume`/`home` 과 `decide.py` 로만. 패널이 서비스를 원하면 추가.

### B6. 비상 초기화 `reset <노드>` (2026-10-16, 사용자 승인)
- 이유를 모르는 막힘(L-3b 에서 주방 앞 정지)이 또 나와도 사람이 로봇을 근처 노드에 올려놓고 버튼 하나로 복구. 현실에서는 바닥 노드에 번호를 붙이고 사람이 그 위로 옮겨 노드 번호를 알려주는 방식(시뮬레이션은 `--place`).
- 동작: 임무·Nav2 목표 취소, 정지/우회/위치보류/타이머 모두 비움, 노드 좌표를 `/initialpose`(std 0.1)로 AMCL 에 알림, IDLE. 위치 상태(LOST 등)는 지우지 않음: health 가 LOST 를 벗어나야 새 주문 수락.
- 노드에서 얼마나 가까운지는 실행기가 확인하지 못함(정답 위치가 없음): 사람 책임, 문서에 약 0.3 m [제안].

### B4. "멈춤" = 정지 신호 또는 실제로 안 움직임 (2026-10-15)
- **문제**: 대기 타이머가 `hold`/`semantic_stop` 신호만 봐서, 신호 없이 무언가(옆구리 HardStop 등)가 로봇을 붙잡으면 알람도 우회도 없이 영원히 서 있었음.
- **채택**: 주행 중 3초 동안 5 cm 이동도 0.15 rad 회전도 없으면 `stalled` → 타이머 조건 `hold OR semantic_stop OR stalled`. 정지가 시작된 순간부터 셈. 우회/새 목적지 때 기준점 초기화(5초마다 우회 반복 방지). 충돌 잠금 시간은 계속 제외. [제안]
- B1(신호 틈이 생기면 0부터)과 충돌하지 않음: 틈을 억지로 메우는 유예가 아니라 "실제로 안 움직였다"는 사실을 하나 더 보는 것.
- 단위 시험 harness 는 로봇을 움직이지 않으므로 `MissionParams` 기본값은 꺼짐(0), 노드가 `runtime_limits.yaml` 값으로 켬.

---

## C. 이름, 구조, 문서

### C1. 토픽 이름: `{robot}` 자리표시자
- **채택**: `config/common/interfaces.yaml` 에 `/{robot}/...` 형식 (기본 `robot_01`), 노드는 `names_for(robot)` 로 읽고 개별 파라미터 덮어쓰기는 유지. Nav2 내부 이름(`navigate_to_pose`, `collision_monitor_state`)은 접두어 없음.
- **반려한 대안**: 로봇마다 이름을 전부 따로 적는 목록 (로봇이 늘면 파일이 늘어남).
- **의도적으로 템플릿화하지 않은 것**: `bridge.yaml`, `nav2.yaml`, `amcl.yaml`, `slam.yaml`, `collision_monitor.yaml`, URDF, 월드. Gazebo/Nav2 가 그대로 읽는 파일이라 `{robot}` 을 못 씀 → `scripts/validate_interfaces.py` 로 일치만 검사. 다중 로봇 이름 공간(Gazebo 쪽, Nav2 YAML)은 Fleet 설계와 함께 따로 결정.

### C2. 코드 구획 (`monitoring / decision / motion / command`)
- 구획 이름과 의미: 감시(사실만) / 판단(상태 결정) / 구동(속도로 변환) / 제어(명령과 임무). 
- **`soft_stop.py` 분리**: 측정 함수(`forward_clearance`, `touched_name`)는 `monitoring/clearance.py`, `LimiterCore` 는 `motion/soft_stop.py`. **`person_limit.py` 분리**: `DetectorWatch` 는 `monitoring/detector_watch.py`. 이유: 측정(감시)과 속도 적용(구동)이 한 파일에 섞여 있어 감시 모듈만 따로 꽂을 수 없었음.
- **`mission.py` 는 `command/`** (`decision/` 아님): 임무 상태 기계라서. 15초/30초 규칙은 그 안의 한 부분.
- **옛 import 경로 호환용 shim 은 만들지 않음**: 쓰는 곳이 우리 노드와 테스트뿐이고, 남기면 정리의 의미가 흐려짐.
- 공통 파일(`site_config.py`, `interfaces.py`, `launch_params.py`)은 루트에 둠. 모든 구획이 씀.
- ~~`person_detector` 노드가 감시와 판단을 한 노드에서 함~~ → 2026-10-15 분리함 (C4).
- `go_to.py`, `serve.py` 는 명령줄 도구라 `command/` 에 두되 launch 에서는 시작하지 않음 (`validate_layout.py` 의 `CLI_TOOLS`).

### C3. launch 분리
- **Nav2 본체(controller, planner, behaviors, bt_navigator, lifecycle manager)는 `navigation.launch.py` 에 남김**: 우리 구획이 아니라 건드릴 이유가 없음. 구획 launch 4개(monitoring, decision, motion, command)를 include.
- 각 구획 launch 에 `MODULES` 목록: 노드 한 줄 추가로 꽂힘. `velocity_smoother`, `collision_monitor` 는 Nav2 노드지만 속도 사슬이라 `motion.launch.py` 에서 시작 (lifecycle manager 가 이름으로 관리).
- 구획 README 는 **구획 폴더 안**(코드와 같이 보고 고치기 위해). `docs/` 한곳에 모으자는 안은 반려.
- `scripts/validate_layout.py`: 노드, 모듈을 꽂을 때 CMake, README, launch 중 빠뜨린 곳을 잡음.


### C4. 사람 감지를 감시 노드와 판단 노드로 분리 (2026-10-15)
- **채택**: `person_detector`(카메라 한 대: YOLO + 깊이 → `person_observations`) + `person_state`(판단: 관측 + `scan_dynamic` + `obstacle_hold` → `PersonSafetyCore` → `person_detection`).
- **이유**: 카메라를 3대로 늘리면 감지기 3개가 각자 판단하게 되어 "결정은 한 곳"(A5)이 깨짐. 카메라는 늘어도 판단 노드는 하나.
- **출력 토픽 이름 `person_detection` 유지** (사용자 결정): 이제 판단 결과라 이름이 조금 어색하지만, 바꾸면 속도 제한, 실행기, 도구, 문서, UI 계약이 모두 바뀜. 이름 변경은 팀과 맞출 때.
- **동작은 바꾸지 않음**: 테스트(`test_person_judge.py`)가 분리 전 노드 로직과 매 순간 같은 JSON 을 내는지 비교.
- **고장 동작 유지 방법**: 카메라 노드는 새 영상이 없어도 매 주기 관측을 보냄(생존 신호, 판단 노드는 2초 후 `SENSOR_STALE`). 판단 노드는 관측이 올 때만 발행 → 카메라 노드가 죽으면 `person_detection` 도 끊김 → 기존의 "감지기 응답 없음" 규칙(A4, `DETECTOR_SILENT`)이 그대로 적용.
- **보류**: 카메라 여러 대를 합치는 규칙, 일부 카메라만 고장일 때의 규칙 → 카메라를 늘릴 때(4단계) 결정.


### C5. 카메라 3대와 방향별 규칙 (2026-10-15, 사용자 승인: 추천대로)
- **배치**: 120° RGB-D 3대, 앞 0° / 좌후방 +120° / 우후방 -120°, 1.0 m. 앞 카메라 위치(x 0.15)는 유지(거리 기준 유지), 뒤 두 대는 5 Hz(GPU 없는 VM 렌더링 부담).
- **실제 거리 보정**: 깊이 영상은 광축 방향 거리 → 120° 가장자리에서 실제의 절반. 픽셀 각으로 `깊이 / cos(각)`. (Gazebo 깊이가 광축 거리인지 VM 에서 측정해 확인할 것)
- **방향별 규칙** [제안]: 앞(±60°) 지금 규칙, 옆(60~120°) 0.8 m 안 SLOW 만, 뒤 영향 없음. 이유: 가장 가까운 사람으로 STOP 을 정하면 테이블 옆 손님 때문에 로봇이 못 떠남. 로봇은 후진하지 않음.
- **판단은 하나**: 주 카메라(앞) 관측 때만 `person_state` 가 발행 → 감지기 응답 없음 규칙(A4) 그대로. `PersonSafetyCore` 는 그대로(앞 사람만 넣음).
- **일부 고장**: 옆/뒤 카메라 = `CAMERA_SILENT` 알람만(LiDAR 가 지킴), 앞 카메라 = SENSOR_STALE(지금 규칙) + 알람.
- **노드 하나 + 모델 하나**: VM 메모리/CPU 때문에 카메라 3대를 한 감지 노드가 처리(앞 매 주기, 뒤 번갈아, 한 번에 묶어 추론). `cameras` 파라미터로 카메라별 노드도 가능.
- 카메라 목록은 `runtime_limits.yaml` `person_safety.cameras` 한 곳 → launch 가 두 노드에 같은 값.


### C6. 로봇 요약 상태 `/robot_01/health` (2026-10-15, 사용자 승인: 추천대로)
- **팀 기준을 그대로**: 통합 가이드 Q3-3 의 enum(robot_state / safety_state / localization_state), 상태 분리 원칙, `schema_version`, Pose DTO, `UPPER_SNAKE_CASE` 이유 코드. `OFFLINE` 은 Fleet 이 생존 신호(이 토픽) 끊김으로 정함.
- **형식**: JSON(String) + `schema_version`. 정식 `.msg` 는 IC-0 계약 동결 때 팀과 (지금 만들면 팀 RobotState 와 두 번 맞춰야 함).
- **robot_state 우선순위**: ERROR > BOOTING > BLOCKED > DEGRADED > PAUSED > NAVIGATING/IDLE. 여러 개가 걸려도 하나만, 나머지는 `reasons`.
- **safety_state**: 속도 제한 노드가 조용하면 STOP (속도 사슬의 일부라 실제로 명령이 바퀴까지 안 감).
- **health 노드는 보고만**: 동작/알람은 바꾸지 않음. 위치를 잃었을 때의 동작은 실행기가 이 토픽을 읽어 처리함 → B5 (2026-10-16).

---

## D. 값의 상태 (팀 확정 전 주의)

| 값 | 위치 | 상태 |
|---|---|---|
| `max_linear_mps` 0.25, `max_angular_rps` 0.80, `slow_linear_mps` 0.10, 사람 SLOW 1.5 m / STOP 0.8 m | `runtime_limits.yaml` | **[합의]** (설계안 8장) |
| SLOW 각속도 0.40, 감지기 타임아웃 1.0 s, 시작 유예 30 s, 알람 반복 30 s | `person_safety` 블록 | **[제안]** |
| 막힘 알람 15 s / 우회 30 s / 우회 3회 / 복도 기억 60 s, 지연 180 s / 수령 대기 60 s | `blocked_response`, `delivery` | **[제안]** |
| 위치 추정 OK 0.25 m / 0.2 rad, LOST 1.0 m / TF 2 s, 시작 60 s, 끊김 2 s | `health` | **[제안]** |
| 방향 기준 앞 ±60° / 옆 120° / 옆 SLOW 0.8 m / 해제 1 s, 뒤 카메라 5 Hz | `person_safety`, `sensors.xacro` | **[제안]** |
| 위치가 LOST 를 벗어나 2 s, health 없음 3 s 면 "모름" | `localization_hold` | **[제안]** |
| 안 움직임 3 s / 5 cm / 0.15 rad, 레일 5 cm / 이탈 0.15 m / 재출발 3 s, RPP 충돌 예측 끔 | `blocked_response`, `rail`, `nav2.yaml` | **[제안]** |
| 가감속 한도, 재출발 규칙(1.0 m 비어 2.0 s), 충돌 해제 3 s | `motion_comfort`, `obstacle_reaction` | **[제안]** |

---

## E. 의도적으로 하지 않은 것 / 보류

- ~~`/robot_01/health` 계획만~~ → 2026-10-15 구현 (C6). 위치를 잃었을 때 주행 중단 / 새 임무 거절 → 2026-10-16 구현 (B5).
- `PersonSafetyEvent.msg`: 쓰는 노드 없음. 패널/Fleet 이 이 형식을 기대하는지 팀 확인 전에는 맞추지 않음.
- `serving_robot_perception` 의 빈 파일 3개: 건드리지 않음 (이름이 `person_detector` 와 겹쳐 팀 정리 필요).
- 푸시와 팀 공유: 하지 않음 (제안값 확정 전).
- 속도 제한 노드 자체가 죽는 경우: 실행기는 2초(`person_limit_max_age_s`) 넘게 메시지가 없으면 "STOP 아님"으로 봄. 패널은 `person_limit` 이 2초 넘게 안 오면 "알 수 없음" 표시. 이 경계는 아직 따로 막지 않음.
- 사람을 치운 뒤 LiDAR 가 몇 초 동안 앞 ~1.19 m 에 무언가 보았다고 한 현상: 원인 미확인 (SLOW 만 걸림). 재현되면 확인.

---

## F. 작업 중 겪은 함정 (같은 실수 방지)

- rclpy: 로그 호출 위치(call site)가 심각도를 바꾸면 에러 → 심각도별로 호출 위치를 분리. `Node.handle` 은 읽기 전용.
- rclpy(Jazzy): 기본 SIGINT 처리기가 Ctrl-C 때 context 를 먼저 닫아서, `except KeyboardInterrupt` 안의 publish 가 `publisher's context is invalid` 로 실패함 → Ctrl-C 때 무언가 보내야 하는 명령줄 도구(`serve.py`, `go_to.py`)는 `rclpy.init(signal_handler_options=SignalHandlerOptions.NO)`. launch 가 관리하는 노드는 해당 없음.
- 시험의 시간 기준은 시뮬레이션 시계. VM 은 실시간보다 느림 → 점검 도구의 메시지 대기는 20초(6초는 짧아서 가짜 FAIL 이 났음).
- `vm_sync` 는 옛 파일을 지우지 않음 → 파일을 옮기면 VM 의 옛 파일과 `build/`, `install/` 의 해당 패키지를 직접 지우기. (옛 파일이 남으면 import 실수가 가려짐)
- 지도 `map.yaml` 의 `free_thresh` 0.25 이면 알 수 없음 픽셀 205 (0.196)가 빈칸으로 읽힘(VM 에서 `/map` 에 -1 이 0개로 확인) → 0.196 으로. 팀 실제 지도도 같은 확인 필요 (`test_scan_filter.py` 가 검사).
- `comm` 은 정렬 로케일이 다르면 경고를 냄 → `LC_ALL=C` 를 쓰거나 경로를 직접 적기.
- PowerShell 로 커밋할 때는 내용이 바뀐 파일만 `git add` (줄바꿈/권한만 다른 파일 17개는 제외).
