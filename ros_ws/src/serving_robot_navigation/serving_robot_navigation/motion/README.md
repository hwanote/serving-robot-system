# 구동 (motion)

결정을 **속도 명령**으로 바꾸는 구획입니다. 속도 명령 사슬의 각 단계는 입력 토픽 하나, 출력 토픽 하나만 가집니다.
토픽, 필드의 뜻은 `docs/INTERFACES.md` 3.3절을 보세요.

```
Nav2 controller → cmd_vel_nav → velocity_smoother → cmd_vel_smoothed
  → person_speed_limiter → cmd_vel_semantic        (person_limit:=false 이면 이 단계 없음)
  → collision_monitor    → cmd_vel_safe
  → soft_stop_limiter    → /robot_01/cmd_vel       (바퀴로 가는 최종 명령)
```

## 모듈 (순수 로직, ROS 없이 테스트)

| 파일 | 하는 일 |
|---|---|
| `person_limit.py` | `PersonLimitCore`: 사람 상태 → 속도 제한 (SLOW 선속도 0.10 m/s, 각속도 0.40 rad/s 이하, STOP 은 전진 0, 후진/회전은 SLOW 한도). `parse_message`, `parse_state` |
| `soft_stop.py` | `LimiterCore`: 부드러운 정지(가감속 제한), 하드 스톱 즉시 0, 장애물 정지 후 재출발 규칙, 충돌 잠금. `rate_limit` |
| `hold_recovery.py` | `HoldRecoveryCore`: 장애물 정지(hold)가 5 s 이어지고 주행 목표가 없으면 **제자리 회전만**으로 트인 방향을 찾아 빠져나옴(후진·전진 없음). `heading_options` |

## 노드 (`scripts/motion/`)

| 노드 | 읽는 것 | 내는 것 |
|---|---|---|
| `person_speed_limiter.py` | `cmd_vel_smoothed`, `person_detection`, `scan_dynamic` | `cmd_vel_semantic`, `person_limit` (적용 중인 사람 제한) |
| `soft_stop_limiter.py` | `cmd_vel_safe`, `collision_monitor_state`, `scan_dynamic`, `contact` | `/robot_01/cmd_vel`, `obstacle_hold`, `collision_latched`, 서비스 `clear_collision` |
| `hold_recovery.py` | `obstacle_hold`, `scan_dynamic`, `/navigate_to_pose/_action/status` | `cmd_vel_nav` (회전 명령만, 사슬 맨 앞). `hold_recovery:=true` 일 때만 시작(맵핑 모드) |

Nav2 의 `velocity_smoother`, `collision_monitor` 도 이 사슬에 속하며 `launch/motion.launch.py` 에서 같이 시작합니다.

## 이 구획의 규칙
- **최종 `/robot_01/cmd_vel` 을 쓰는 노드는 `soft_stop_limiter` 하나뿐**입니다. 다른 노드는 사슬 중간에 끼워 넣는 방식으로만 속도에 영향을 줍니다.
- 마지막 방어선은 LiDAR/Collision Monitor 입니다. 새 단계는 이 방어선을 우회하지 않도록 `collision_monitor` 앞에 둡니다.
- 한도 값은 코드에 쓰지 않고 `config/common/runtime_limits.yaml` 에서 launch 가 읽어 파라미터로 넘깁니다 (`serving_robot_navigation/launch_params.py`).

## 새 단계 꽂는 순서 (예: 속도 제한 단계 하나 추가)
1. `serving_robot_navigation/motion/<이름>.py` 에 순수 로직(입력 속도, 상태, 시간 → 출력 속도)을 만들고 `tests/robot/` 에 테스트를 추가합니다.
2. `scripts/motion/<노드>.py` 를 만듭니다 (`in_topic`, `out_topic` 파라미터 하나씩).
3. `CMakeLists.txt` 의 `install(PROGRAMS ...)` 에 한 줄 추가합니다.
4. `launch/motion.launch.py` 의 MODULES 목록에 추가하고, 앞뒤 단계의 토픽 이름이 이어지도록 `collision_monitor` 의 `cmd_vel_in_topic` 등을 맞춥니다 (사슬 그림을 위 README 에도 갱신).
5. 새 토픽이 외부에 보이면 `interfaces.yaml`, `docs/INTERFACES.md`, `scripts/dev/check_interfaces_live.py` 를 갱신합니다.
6. `python3 scripts/validate_layout.py`, `python3 scripts/validate_interfaces.py`, `tests/robot/` 가 통과하면 끝입니다. VM 에서는 `go_to` 한 번과 `scripts/dev/watch_person_limit.py` 로 속도 변화를 확인하세요.

## 테스트
- `tests/robot/test_person_limit.py` : `PersonLimitCore`, `parse_*`
- `tests/robot/test_soft_stop.py` : `LimiterCore`
- `tests/robot/test_hold_recovery.py` : `HoldRecoveryCore`, `heading_options`
