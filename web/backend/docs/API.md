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
| GET | `/api/robots` | 로봇별 상태 (`online`, `accepting_orders`, `mission_state`, `loc_hold`, `home`, `localization`, `map_current`, `controls`) — 같은 필드가 WebSocket `robot_state` 에도 실려요 |
| POST | `/api/robots/{id}/pause` · `/resume` | 새 주문만 받지 않음/다시 받음 (진행 중인 주문은 끝까지) |
| POST | `/api/robots/{id}/recover` | 위치 복구 뒤 `{action:"resume"|"home"}`. 아니면 409 `NOT_RECOVERABLE` |
| POST | `/api/robots/{id}/reset` | **비상 리셋**: 로봇을 사람이 `node` 위에 올려놓았다는 뜻. 본문 `{node?}` (기본: 그 로봇의 대기 위치). 진행 중이던 주문은 `CANCELED`(detail `RESET`). 404 `ROBOT_NOT_FOUND`, 400 `BAD_NODE`, 409 `ROBOT_OFFLINE` |
| POST | `/api/robots/{id}/home` | **대기 위치로 보냄**. 쉬고 있고 위치가 `OK` 일 때만 (위치를 잃었으면 409 `NOT_LOCALIZED` → 먼저 리셋). 409 `ROBOT_BUSY`, `ALREADY_HOME`, `ROBOT_OFFLINE`. 위치를 잃었다가 복구된 직후에는 로봇의 `home` 선택으로 처리 |
| POST | `/api/robots/{id}/reload-map` | 로봇이 `config/common` 을 **다시 읽음** (로봇 노드는 시작할 때만 읽으므로). 쉬고 있을 때만, 아니면 409 `ROBOT_BUSY`. 결과는 `map_current` 로 확인 |
| POST | `/api/robots/reload-map` | 쉬고 있는 모든 로봇에 위 명령. `{robots:{id:"sent"|"current"|"busy"|"offline"}}` (이미 최신이면 `current`) |
| POST | `/api/dispatch/pause` · `/resume` | 전체 배차 멈춤/재개 (주문은 접수되어 대기) |
| GET | `/api/snapshot` | WebSocket 재연결 뒤 상태 맞추기용 (`robots`, `tasks`, `queue`) |
| GET | `/api/metrics` | 건수·대기시간·소요시간·처리량·로봇별 완료 수 |
| GET | `/api/debug` | 대기 이유, 로봇 원본 상태, 최근 로그 |

`controls` (카드 버튼 규칙, 화면은 이 값만 따라요): `reset` 연결돼 있으면 true · `reset_nodes` 리셋 때 고를 수 있는 위치(대기 위치, 주방) ·
`home` 위 규칙을 만족할 때 · `reload_map` 쉬고 있고 `map_current` 가 true 가 아닐 때.
`map_current`: 로봇이 쓰는 지도가 지금 `config/common` 과 같은지 (`true`/`false`, 로봇 노드가 옛 버전이라 알려 주지 않으면 `null`).
로봇이 보고하는 값은 `mission_status.config_revision` (= 세 yaml 파일의 짧은 해시)이에요.

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
| GET | `/api/stores/{id}/semantics/live` | 로봇이 읽는 지도(`config/common`) 상태: `{live_sha, matches_applied, last, backup_count}`. `last` = 마지막 반영/복구 기록 |
| POST | `/api/stores/{id}/semantics/promote` | **적용 결과를 `config/common` 에 반영.** 먼저 지금 지도를 `data/backups/<store>/<YYYYMMDD-HHMMSS>/` 에 백업(여러 개 계속 보관). 응답 `{backup, live}`, `live.needs_robot_restart=true` — **로봇은 시작할 때와 `reload-map` 명령을 받을 때만 읽음 — 반영 뒤 로봇 카드의 "지도 다시 읽기"** |
| GET | `/api/stores/{id}/semantics/backups` | 백업 목록(최신순): `{id, created_at, kind(promote/before_restore), note, has_semantics, summary{nodes,edges,tables}}` — `has_semantics`: 그 지도를 만든 편집 화면 저장본(draft)이 백업에 같이 있는지 |
| POST | `/api/stores/{id}/semantics/backups/{backup_id}/restore` | 그 백업으로 되돌림. 되돌리기 전 지도도 `before_restore` 백업으로 남아서 다시 취소 가능. **편집 화면 저장본(draft)도 같이 되돌림** — 응답 `{backup, live, semantics, semantics_restored, semantics_note}`. 백업에 저장본이 없거나(옛 백업) 지금 지도 revision 과 안 맞으면 `semantics_restored=false`, 로봇 지도만 되돌리고 이유를 `semantics_note` 에 담음. 되돌린 뒤에는 "적용 결과"가 저장본보다 새것이라 `in_sync=false` (다시 적용해야 반영 가능) |

반영·복구가 거절되면 (초안·적용 결과·`config/common` 은 그대로):
| 상태 / code | 뜻 |
|---|---|
| 409 `ORDERS_ACTIVE` | 주문이 대기 중이거나 로봇이 수행/복귀 중 |
| 409 `NOT_APPLIED` / `NOT_IN_SYNC` / `ALREADY_LIVE` | 적용 결과가 없음 / 초안이 적용 결과보다 새로움 / 이미 같은 지도 |
| 422 `ROBOT_LOADER_REJECTED` / `MISSING_STANDBY` / `MISSING_KITCHEN` / `INCOMPLETE` | 로봇 로더가 거절 / 로봇 번호에 맞는 `standby_NN` 이 없음(`details.robots`) / 주방 위치 없음 / 파일이 빠짐 |
| 404 `BACKUP_NOT_FOUND` | 없는 백업 id |
| 500 `PROMOTE_FAILED` | 바꾸다 실패 — 원래 지도로 되돌려 둠 |

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
