# Backend API (v0.1, web/backend)

프론트엔드 계약(`web/frontend/docs/CONTRACT_NOTES.md`)을 그대로 따르고, 아래 항목이 추가되었습니다.
오류는 `{code, message}` 입니다 (`message`는 사람용, `code`는 프로그램용).

## 주문
| 메서드 | 경로 | 설명 |
|---|---|---|
| GET | `/api/tasks` | 최신순 목록 |
| POST | `/api/tasks` | `{type:"SERVE", destination_id, client_request_id?}` → 201. 같은 `client_request_id` 로 다시 보내면 **200** 으로 처음 Task 를 돌려줌(중복 생성 안 함). 같은 id 에 다른 목적지는 409 `REQUEST_ID_CONFLICT` |
| POST | `/api/tasks/{id}/cancel` | 404 `TASK_NOT_FOUND`, 409 `TASK_FINISHED` |

Task 필드: `task_id type destination_id status robot_id created_at updated_at` + (있을 때) `client_request_id assigned_at started_at finished_at detail`.
`detail` 은 실패 이유 코드 (`ROBOT_OFFLINE`, `COMMAND_NOT_ACKED`, `SERVER_RESTART`, 로봇이 준 코드).

## 로봇·운영
| 메서드 | 경로 | 설명 |
|---|---|---|
| GET | `/api/robots` | 로봇별 상태 (`online`, `accepting_orders`, `mission_state`, `loc_hold`, `home`) |
| POST | `/api/robots/{id}/pause` · `/resume` | 새 주문만 받지 않음/다시 받음 (진행 중인 주문은 끝까지) |
| POST | `/api/robots/{id}/recover` | 위치 복구 뒤 `{action:"resume"|"home"}`. 아니면 409 `NOT_RECOVERABLE` |
| POST | `/api/dispatch/pause` · `/resume` | 전체 배차 멈춤/재개 (주문은 접수되어 대기) |
| GET | `/api/snapshot` | WebSocket 재연결 뒤 상태 맞추기용 (`robots`, `tasks`, `queue`) |
| GET | `/api/metrics` | 건수·대기시간·소요시간·처리량·로봇별 완료 수 |
| GET | `/api/debug` | 대기 이유, 로봇 원본 상태, 최근 로그 |

## 지도 / 매핑 / WebSocket
`/api/stores/{id}/map|map.png`, `/api/mapping/*`(아직 501), `WS /ws/state` — 임시 브리지와 동일.

## 지도 설정(Semantic Map) 저장과 자동 노드 적용
저장본은 두 단계예요. **초안**은 편집 화면에서 저장한 그대로(`data/semantics/<store>.json`), **적용**은 그 초안에서
`tools/site_builder` 가 자동 노드·복도를 만들어 통과한 결과(`data/generated/<store>/`, 임시 `provisional`)예요.
**어느 쪽도 `config/common` 에 쓰지 않으므로 로봇 경로와 배차는 바뀌지 않아요.** 적용 결과는 사람이 확인한 뒤 옮겨요.

| 메서드 | 경로 | 설명 |
|---|---|---|
| GET | `/api/stores/{id}/semantics` | 저장한 초안, 없으면 `config/common` 에서 만든 값 |
| PUT | `/api/stores/{id}/semantics` | **초안 저장**. 프론트 계약 그대로의 본문. 400 `INVALID_SEMANTICS`(`details.problems`), 400 `INVALID_JSON`, **409 `MAP_REVISION_MISMATCH`**(지도 revision 이 다름) |
| PUT | `/api/stores/{id}/semantics?apply=true` | 초안 저장 + **자동 노드 생성·적용**. 성공하면 저장본에 `apply_result` 를 덧붙여 돌려줘요 |
| POST | `/api/stores/{id}/semantics/apply` | 저장해 둔 초안을 다시 보내지 않고 적용 (초안이 없으면 409 `NO_DRAFT`) |
| GET | `/api/stores/{id}/semantics/status` | `{draft, applied, in_sync, require_drive_test}`. `in_sync` 는 적용 결과가 지금 초안과 같은지 |

적용이 거절되면 **초안은 저장된 채로** 아래 오류가 나요 (`message` 는 사람용, `details` 는 프로그램용).
| 상태 / code | 뜻 |
|---|---|
| 422 `GENERATION_REJECTED` | 통과하지 못한 곳이 있어요. `details.fails`(노드·통로·정지 위치 FAIL, `near`=가까운 점)와 `details.excluded`(복도에 놓지 못해 빠진 점, `reason_ko`). 해당 점을 지우거나 옮기고 다시 적용 |
| 422 `INVALID_LABELS` / `GENERATION_FAILED` / `MISSING_POSE` | 대기 위치가 없음 등 라벨 문제 / 복도를 못 찾음 / 위치가 없는 객체 |
| 422 `MISSING_TOP_SIZE` | 테이블 상단 크기(`top_size_m`)가 없는 테이블이 있음 (`details.ids`). 초안 저장(`PUT` 만)은 막지 않고, 적용(`?apply=true`)만 거절해요 |
| 409 `DRIVE_TEST_REQUIRED` | `--require-drive-test` 인데 시험 주행 통과 기록이 없음(시험 도구는 아직 연결 전이라 항상 거절) |
| 503 `SITE_BUILDER_MISSING` | `tools/site_builder` 가 없음 (WSL 에서 `git pull`) |

적용 기준: **FAIL 이 하나라도 있거나, 복도에 놓지 못해 빠진 점이 하나라도 있으면 거절**해요 (WARN 은 통과, 개수는 `summary`).
적용 결과 폴더에는 `waypoints.yaml entities.yaml zones.yaml preview.json labels.json applied.json` 이 있고
`applied.json` 에 `source_sha`, `summary`, `warnings`, `notes`, `drive_test: {status: not_run}` 이 기록돼요.

객체 확장 필드(선택, 저장은 그대로 보관): 테이블 `top_size_m: {x, y}` (m, map 축 정렬, x = 가로 / y = 세로), `chairs: [{dx, dy, size_m}]` — 생성기가 장애물·정지 위치 계산에 써요. 적용 시 테이블마다 `top_size_m` 이 **필수**예요(프론트는 mm 로 입력받아 m 로 보내요). 데모 지도의 `GET semantics` 는 `entities.yaml` 의 실제 값을 실어 줘요.
변환 규칙: `table→table`, `kitchen→pickup`, `robot_wait→standby`(`robot_wait_NN` 은 `standby_NN` 으로 이름을 바꿔 생성), `counter` 는 `kitchen` 이 있으면 건너뜀.
사용자 지정 구역(`zones`, 꼭짓점 3개 이상 다각형)과 수동 waypoint 는 **저장만** 하고 아직 생성에 쓰지 않아요(waypoint 는 나중에 덮어쓰기 용도).

## 안전 규칙
- 로봇이 `offline_after_s` 동안 조용하면 OFFLINE: 새 주문을 받지 않고, 진행 중 주문은 `offline_fail_s` 뒤 `FAILED(ROBOT_OFFLINE)`. 프론트 enum 에 OFFLINE 이 없어 `robot_state` 는 `ERROR` 로 보내고 `online:false` 를 덧붙임.
- 로봇이 `ack_timeout_s` 안에 주문을 받았다고 알리지 않으면 `FAILED(COMMAND_NOT_ACKED)`.
- 서버 재시작: 대기 중이던 주문은 다시 대기열로, 로봇에 넘어간 주문은 `FAILED(SERVER_RESTART)` (결과를 알 수 없으므로 성공으로 처리하지 않음).
