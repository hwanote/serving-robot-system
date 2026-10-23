# Frontend Contract Notes

프론트엔드가 따르는 공통 Contract 와, 구현 중 **임시로 결정했거나 다른 파트와 합의가 필요한 항목**을 기록합니다.
(기준: 「Claude Code 개발 지시서 — Serving Robot System Frontend UI」, 저장소 내 `config/common/*.yaml`·`README.md` 는 작성 시점에 비어 있어 충돌 없음)

## 1. 준수하는 공통 Contract (변경 없음)

| 항목 | 값 | 위치 |
|---|---|---|
| 좌표계 | `map` / `odom` / `base_footprint` / `base_link`, Semantic 좌표는 `map` | `types/contracts.ts` |
| Robot 상태 | `IDLE NAVIGATING SERVING RETURNING CHARGING PAUSED ERROR` | 그 외 문자열 수신 시 무시(카운트) |
| Task 상태 | `QUEUED ASSIGNED RUNNING SUCCEEDED FAILED CANCELED` | 동일 |
| Mapping | `POST /api/mapping/{start,stop,save}` body `{ store_id }` | `api/mappingApi.ts` |
| Raw Map | `GET /api/stores/{store_id}/map` | `api/semanticApi.ts` |
| Semantic | `GET/PUT /api/stores/{store_id}/semantics` — `entities` / `waypoints` / `zones` | `api/semanticApi.ts` |
| Serve | `POST /api/tasks` body `{ type: "SERVE", destination_id }` (좌표·robot_id 미전송) | `api/taskApi.ts` |
| Cancel | `POST /api/tasks/{task_id}/cancel` | `api/taskApi.ts` |
| WebSocket | `/ws/state`, `robot_state` 이벤트 | `api/stateSocket.ts` |

## 2. 임시 계약 (Backend/Fleet 확정 필요) — `types/provisional.ts`

| # | 항목 | 프론트엔드 임시 형식 | 비고 |
|---|---|---|---|
| P1 | `POST /api/tasks` 응답 | `Task { task_id, type, destination_id, status, robot_id\|null, created_at, updated_at }` (201) | 응답 수신 전에는 Task 를 표시하지 않음 |
| P2 | `POST /api/tasks/{id}/cancel` 응답 | 갱신된 `Task` (본문 없어도 동작) | 이미 종료된 Task → 409 가정 |
| P3 | `GET /api/tasks` | `Task[]` (또는 `{ tasks: Task[] }`) | Dashboard 현재/최근 Task, 재연결 후 동기화 |
| P4 | WS `task_state` 이벤트 | `{ type: "task_state", task_id, status, robot_id, destination_id, updated_at }` | Task 상태 전이 표시용. 없으면 REST 재조회로 대체 필요 |
| P5 | `GET /api/mapping/status?store_id=` 및 매핑 명령 응답 | `{ store_id, phase: "idle"\|"mapping"\|"stopped", started_at, last_saved_revision }` | Robot/Task 상태와 구분하려고 소문자 사용 |
| P6 | 오류 응답 본문 | `{ message }` (또는 `detail`/`error`) | 화면에 그대로 표시 |
| P7 | 지도/semantic 없음 | `404` | "지도 없음" Empty 상태로 처리 |
| P8 | Semantic revision 불일치 PUT | `409` | 프론트에서도 저장 전 차단 |
| P9 | `robot_state.task_id` / `destination_id` | 작업이 없으면 `null` | 예시에는 문자열만 있음 |

## 3. Contract 확장/해석 — Mapping 파트 확인 필요

### 3-1. `robot_wait` 엔티티 타입 (제안)
지시서상 UI 제안 타입. Semantic API 확정 전 타입명 확인 필요. 로봇 대기(복귀) 위치로 사용하며 Waypoint pose = 대기 pose.

### 3-2. 엔티티 표시 위치 `entities[].pose` (임시 확장 필드, optional)
Contract 예시의 entity 는 `id/type/label` 만 있어 테이블 위치를 지도에 그릴 수 없음.
`pose?: {x, y, yaw}` 를 **선택 필드**로 추가 저장한다. 필드가 없으면 Waypoint 위치에 표시하므로 기존 데이터와 호환.
→ 합의 안 되면 `toSemanticPayload()` 에서 제거하면 됨.

### 3-2b. 테이블 상단 크기 `entities[].top_size_m` 와 의자 `entities[].chairs` (임시 확장 필드, optional)
자동 노드 생성(site_builder)이 테이블에 붙는 접근 거리를 계산하려면 테이블 상단 크기가 필요하다.
- `top_size_m?: {x, y}` (m, map frame 축 정렬, x = 가로 / y = 세로). 화면에서는 **mm 로 입력**하고 1 mm 단위로 저장한다.
  `table` 타입은 값이 없거나 0 이하·10 m 초과이면 저장 오류로 막는다. **기본값은 채우지 않는다** (생각 없이 승인하는 것을 막기 위해).
  비스듬한 테이블은 전체를 감싸는 x·y축 정렬 사각형 크기를 입력하도록 안내한다 (회전은 지원하지 않음).
- `chairs?: [{dx, dy, size_m}]` 는 편집 UI 없이 **읽고 그대로 저장**만 한다. 예전에는 파서·직렬화가 두 필드를 버려서
  프론트에서 저장하면 서버 초안의 값이 사라졌다 (`parsers.ts`, `toSemanticPayload()`).
- Backend `PUT /api/stores/{id}/semantics` 는 두 필드를 그대로 보관하고, `?apply=true` 때 자동 노드 생성에 쓴다.

### 3-3. ID 생성 규칙
- Entity: `{type}_{NN}` (예: `table_03`, `robot_wait_01`), 로드된 데이터의 최대 번호 + 1 (삭제된 번호 재사용 안 함)
- Waypoint: `wp_{entity_id}` (엔티티당 1개 기준, 2개 이상이면 경고)
- Zone: `zone_{type}_{NN}` (예: `zone_no_go_01`). 지시서 예시 `zone_service_aisle` 처럼 의미 있는 이름이 필요하면 label 필드 추가 협의 필요 (현재 zone 스키마에 label 없음 → 추가하지 않음)
- label 변경·zone 타입 변경 시에도 ID 불변

### 3-4. 기본 Waypoint
엔티티 추가 시 기본 접근 위치를 엔티티 −y 방향 0.6 m, 엔티티를 바라보는 yaw(+π/2)로 생성 → 사용자가 드래그/방향 핸들로 조정.
(로봇이 테이블 중심으로 직접 주행하지 않도록 함)

### 3-5. 서빙 목적지
`type === "table"` 이고 Waypoint 가 있는 엔티티만 선택 가능. Backend 가 counter/kitchen 목적지도 허용하는지 확인 필요.

## 4. 좌표 변환 규칙 (Mapping 파트 확인 권장)

- 지시서 픽셀 인덱스 공식 `y = origin_y + (height − 1 − v)·res` 는 **정수 픽셀 (u, v) 의 좌하단 모서리** 좌표로 해석했다.
  필수 사례 pixel (100, 70) → (4.25, 1.63) 통과.
- 마우스 클릭처럼 연속 좌표에는 ROS map_server 규칙(`origin` = 좌하단 픽셀의 좌하단 모서리)에 맞춘
  `y = origin_y + (height − v)·res` (v ∈ [0, height]) 를 사용한다. 두 식은 "인덱스 v 의 좌하단 = 연속 v+1" 로 정확히 일치.
  ⚠ 지시서 식을 연속 좌표에 그대로 쓰면 y 가 1픽셀(0.05 m) 어긋나므로 이렇게 분리했다.
- `origin.yaw ≠ 0`: map_server 와 동일하게 지도 로컬 좌표를 origin.yaw 만큼 회전 (테스트 포함). 화면에 안내 표시.
- 저장 정밀도: 위치 소수 3자리(1 mm), yaw 4자리, yaw 는 (−π, π] 로 정규화.
- Map revision 이 바뀌면(재매핑) 기존 Semantic 은 저장을 차단하고, 사용자가 위치를 확인한 뒤 "현재 지도 기준으로 갱신" 해야 저장 가능.
  Robot Control 지도는 revision 이 다른 Semantic 을 표시하지 않고, **서빙 명령도 차단**한다
  (이전 지도 기준 Waypoint 로 로봇이 이동하는 것을 방지). Dashboard·Map Setup 에 "갱신 필요" 경고 표시.

## 5. 기타 합의 필요

- `image_url` 의 이미지 형식: PNG 가정 (`.pgm` 직접 사용 안 함). Backend 가 PGM → PNG 변환 제공 필요.
- 매장 목록/이름 API 없음 → `VITE_STORE_ID`, `VITE_STORE_NAME` 환경 변수 사용.
- 로봇 목록 API 없음 → WebSocket 으로 수신한 로봇만 표시. 연결 직후 전체 로봇 스냅샷 전송을 Backend 에 요청.
- 인증/권한: 미구현 (범위 외).
- 매핑 상태는 앱 로드 시와 이 UI 에서 명령 시에만 갱신 (다른 클라이언트에서 매핑을 시작하면 반영 안 됨).
  실시간 반영이 필요하면 WS `mapping_state` 이벤트 또는 polling 을 Backend 와 협의.

## 3-2c. 지도 반영(promote)·백업 — 우리 Backend 전용 (2026-10-23)
`GET .../semantics/status`, `POST .../semantics/apply`, `GET .../semantics/live`, `POST .../semantics/promote`, `GET .../semantics/backups`, `POST .../semantics/backups/{id}/restore`. 응답·거절 코드는 `web/backend/docs/API.md`. 백업 id 는 서버 시각 `YYYYMMDD-HHMMSS`(같은 초면 `-2`)이고, 화면에는 `created_at` 을 PC 시간대로 바꿔 보여 준다. Mock 은 `src/mocks/liveMapHandlers.ts`.

## 3-2d. 로봇 카드 버튼 — 우리 Backend 전용 (2026-10-23)
`robot_state` 이벤트(와 `GET /api/robots`)에 **선택 필드**가 붙는다: `localization`(`OK|DEGRADED|LOST|UNINITIALIZED`), `map_current`(`true|false|null`), `controls`(`{reset, reset_nodes[], home, reload_map}`). 없어도 화면은 동작하고(버튼 줄만 숨김), 모양이 틀리면 그 필드만 버린다(`parseRobotExtras`). 버튼 켜짐/꺼짐은 화면이 판단하지 않고 `controls` 를 따른다.
`POST /api/robots/{id}/reset|home|reload-map`, `POST /api/robots/reload-map` — 규칙·거절 코드는 `web/backend/docs/API.md`. Mock 은 `src/mocks/simulator.ts`(규칙)와 `handlers.ts`(거절 코드), 위치 잃음 흉내는 Mock 제어 패널의 "위치 잃음".
