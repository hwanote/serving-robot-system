# 제어 (command)

사람/UI의 **명령**을 받아 임무를 진행하고, 진행 상태와 알람을 내보내는 구획입니다.
토픽, 서비스, 알람 코드의 정확한 뜻은 `docs/INTERFACES.md` 3.4절, 4장, 5.3~5.5절, 6장을 보세요.

## 모듈 (순수 로직, ROS 없이 테스트)

| 파일 | 하는 일 |
|---|---|
| `mission.py` | `MissionCore`: 임무 상태 기계. 이동/서빙, 적재/수령 대기, 막힘 대기 알람(15초) 과 우회(30초), 배달 시간 제한(180초), 알람 코드 발행 규칙. "멈춤" = 정지 신호(`hold`, 사람 STOP) **또는 실제로 안 움직임**(3초 동안 5 cm / 0.15 rad 미만). 직선 레일 이탈 감시(`OFF_RAIL`). **위치 보류**: 주행 중 위치가 LOST 면 즉시 정지하고 임무를 보류(`LOCALIZATION_LOST`), LOST 를 벗어나(OK 또는 DEGRADED) 2초 지나면 `LOCALIZATION_RECOVERED` 로 알리고 사용자의 `resume`(이어가기) / `home`(취소 후 홈으로) 선택을 기다림. 보류 중에는 타이머가 멈추고 새 주문은 거절. LOST/UNINITIALIZED 일 때는 대기 중인 임무가 없어도 새 주문을 거절 |
| `rail.py` | `rail_poses`: 구간을 로봇 위치 → 노드 직선 위 5 cm 간격 자세 목록(서브노드)으로 만듦. 마지막 자세에 도착 방향 |

## 노드 (`scripts/command/`)

| 노드 | 하는 일 |
|---|---|
| `route_executor.py` | 명령(`mission_cmd` 토픽, 서비스 4개)을 받아 `MissionCore` 를 돌림. 경로 그래프의 구간은 **직선 경로로 Nav2 제어기에 직접**(`follow_path` 액션, 플래너 없음 → 장애물을 피해 휘지 않고 정면에서 멈춤), 노드에서 먼 직접 이동만 `navigate_to_pose`. `mission_status`, `alarm` 발행. 정지 신호는 `obstacle_hold` 와 `person_limit` 만 읽고, 위치 상태는 `health` 의 `localization_state` 만 읽고(판단은 health 노드 한 곳), TF 로 위치/방향을 봄(실제로 움직이는지) |
| `go_to.py` | 명령줄 도구: 목적지 이름으로 이동 (`--executor` 로 경로 실행기 사용) |
| `serve.py` | 명령줄 도구: 테이블 서빙, 적재/수령 확인은 `--confirm load` 또는 `--confirm handover` |
| `decide.py` | 명령줄 도구: `LOCALIZATION_RECOVERED` 에 답함. `decide.py resume`(같은 목적지로 이어가기) / `decide.py home`(취소하고 홈으로). 로봇 이름은 `--robot robot_02` 처럼 지정 |
| `reset.py` | 명령줄 도구: 비상 초기화. 사람이 로봇을 바닥 노드 위에 옮겨 놓고 `reset.py <노드이름>` (예: `reset.py kitchen`). 임무·정지·위치상실 보류·타이머를 모두 비우고 그 노드 좌표를 AMCL 에 알린 뒤 IDLE 로 돌아감(알람 `ROBOT_RESET`). 시뮬레이션은 `--place`: 먼저 초기화(정지)한 뒤 Gazebo 에서 로봇을 그 노드로 옮김 |

## 이 구획의 규칙
- 센서나 감지기를 직접 읽지 않습니다. 판단 구획이 내보낸 **결정 토픽**(`person_limit`, `obstacle_hold`, `collision_latched`)만 읽습니다.
- 시간은 노드 시계로 재고, 규칙(`mission.py`)은 시간을 인자로 받습니다 (테스트에서 시계를 흉내 내기 위해).
- 알람은 **이벤트**입니다. 계속되는 문제는 주기적으로 다시 내고, 화면은 코드별 한 칸으로 보여줍니다 (`docs/INTERFACES.md` 6장).

## 새 기능 꽂는 순서

**새 명령 (예: "일시정지")**
1. `mission.py` 의 `MissionCore` 에 메서드를 추가하고 `tests/robot/test_mission.py` 에 테스트를 추가합니다.
2. `route_executor.py` 의 `_on_cmd` 에 `cmd` 분기 하나(필요하면 서비스 하나)를 추가합니다. 서비스 이름은 `interfaces.yaml` 의 `services` 에 추가합니다.
3. `docs/INTERFACES.md` 의 4장, 5.5절 표와 `scripts/dev/check_interfaces_live.py` 를 갱신합니다.

**새 알람**
1. `mission.py` 에서 `Action(ALARM, code='...', level=..., text=...)` 로 발행합니다.
2. `docs/INTERFACES.md` 6장 표에 코드, 언제, 반복 여부, 해제 조건을 한 줄 추가합니다 (패널이 이 표로 화면을 만듭니다).

**새 노드**
1. `scripts/command/<노드>.py` 를 만들고 `CMakeLists.txt` 에 한 줄, `launch/command.launch.py` 의 MODULES 목록에 한 줄 추가합니다 (명령줄 도구는 launch 에 넣지 않고 `scripts/validate_layout.py` 의 `CLI_TOOLS` 에 이름을 추가).
2. 이 README 의 표를 갱신합니다.

마지막으로 `python3 scripts/validate_layout.py`, `python3 scripts/validate_interfaces.py`, `tests/robot/` 를 통과시킵니다.

## 테스트
- `tests/robot/test_mission.py` (레일, 안 움직임 감지 포함)
