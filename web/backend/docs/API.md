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
`/api/stores/{id}/map|map.png|semantics`, `/api/mapping/*`(501), `WS /ws/state` — 임시 브리지와 동일.

## 안전 규칙
- 로봇이 `offline_after_s` 동안 조용하면 OFFLINE: 새 주문을 받지 않고, 진행 중 주문은 `offline_fail_s` 뒤 `FAILED(ROBOT_OFFLINE)`. 프론트 enum 에 OFFLINE 이 없어 `robot_state` 는 `ERROR` 로 보내고 `online:false` 를 덧붙임.
- 로봇이 `ack_timeout_s` 안에 주문을 받았다고 알리지 않으면 `FAILED(COMMAND_NOT_ACKED)`.
- 서버 재시작: 대기 중이던 주문은 다시 대기열로, 로봇에 넘어간 주문은 `FAILED(SERVER_RESTART)` (결과를 알 수 없으므로 성공으로 처리하지 않음).
