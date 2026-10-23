# Serving Robot Frontend (MVP)

ROS 2 Jazzy + Gazebo 다중 서빙로봇 시스템의 운영 UI입니다.
실제 Backend/로봇 없이 **Mock REST + Mock WebSocket**으로 전체 흐름을 재현하고, 환경 변수 하나로 실제 Backend에 연결합니다.

- React 19 · TypeScript(strict) · Vite · React Router · Axios
- Mock: MSW 3 (REST + WebSocket 가로채기 → 앱 코드는 mock/real 동일)
- 테스트: Vitest · React Testing Library

## 실행

```bash
cd web/frontend
npm install
npm run dev          # Mock 모드 (기본) → http://localhost:5173
npm run dev:real     # Real 모드: /api, /ws 를 VITE_BACKEND_URL(기본 http://localhost:8000)로 프록시
```

| 스크립트 | 설명 |
|---|---|
| `npm run dev` / `dev:real` | 개발 서버 (Mock / Real) |
| `npm run build` / `build:real` | 타입 검사 + 프로덕션 빌드 (Mock 포함 / Mock 제외) |
| `npm test` | 단위·UI 테스트 (Vitest) |
| `npm run typecheck` | `tsc -b` |
| `npm run lint` | oxlint |
| `npm run mock:map` | Mock 지도 PNG 재생성 (`src/mocks/assets/demo_store_map.png`) |

## 화면

| 경로 | 화면 | 주요 기능 |
|---|---|---|
| `/` | Dashboard | 매장·매핑 상태·Map Revision·로봇 수(작업/대기)·현재/최근 Task·연결 상태 |
| `/map` | Map Setup | Start/Stop/Save Mapping, Raw Map Viewer(줌/팬), Table·Counter·Kitchen·Robot Wait 추가, Waypoint(위치+방향), Exclusive/No-Go/Slow Zone 다각형, Save Store Setup |
| `/robots` | Robot Control | 로봇 상태·배터리·목적지·Task, 목적지 테이블 선택 → Serve, Task 취소, 실시간 위치 지도 |

### Map Setup 조작

- 휠: 커서 기준 확대/축소 · 빈 곳 드래그(선택 도구) 또는 휠 클릭 드래그: 이동 · `+` `-` `0`: 확대/축소/맞춤
- 추가 도구 선택 후 지도 클릭 → `table_01`, `table_02` … 순서로 생성 (기존 ID 와 중복 없음)
- 선택 도구: 객체/Waypoint/구역 드래그 이동, 흰 원 핸들로 Waypoint 방향(yaw) 조절
- 구역: 꼭짓점 클릭 → 첫 점 클릭 또는 `Enter` 로 완료 (3점 미만 불가), `Backspace` 마지막 점 취소, `Esc` 취소
- 구역 선택 시 꼭짓점 드래그 / 변 중간점 클릭으로 꼭짓점 추가 / 꼭짓점 선택 후 `Delete`
- `Ctrl+Z` 되돌리기 · `Delete` 선택 삭제 · 저장하지 않고 이동하면 경고

## Mock / Real 전환

| 변수 | 기본값 | 설명 |
|---|---|---|
| `VITE_API_MODE` | `mock` | `mock` 또는 `real` |
| `VITE_STORE_ID` | `demo_store` | 매장 ID |
| `VITE_STORE_NAME` | `데모 매장` | 상단 표시 이름 |
| `VITE_BACKEND_URL` | `http://localhost:8000` | dev 서버 프록시 대상 (real) |
| `VITE_API_BASE_URL` | (빈 값) | 프록시 없이 직접 호출 시 REST base URL |
| `VITE_WS_URL` | (빈 값) | 직접 지정할 WebSocket URL (기본: 같은 origin `/ws/state`) |

`.env.example` 을 `.env.local` 로 복사해 수정합니다. `npm run dev:real` 은 `.env.real`(`VITE_API_MODE=real`)을 사용합니다.
Real 빌드(`build:real`)에는 Mock 코드와 Service Worker 가 포함되지 않습니다.

### Mock 모드 사용법

- 처음에는 지도가 없는 상태입니다. `/map` 에서 **Start Mapping → Stop Mapping → Save Map** 으로 지도(r1)를 만듭니다.
- 지도/Semantic Map 은 브라우저 localStorage 에 보존됩니다 (Task 기록은 새로고침 시 초기화).
- 좌측 하단 **Mock 제어** 패널
  - API 실패 시뮬레이션(503): 매핑 / 지도 조회 / Semantic 저장 / Task 생성 / Task 취소
  - 응답 지연, WebSocket 서버 다운 / 현재 연결 끊기
  - 로봇 오류 상태 토글, 로봇 위치 초기화
  - 데모 데이터 채우기(지도+테이블 3개+구역) / 초기화(지도 없음)
- 시연 시나리오(로봇 1대 기준): Task `QUEUED → ASSIGNED → RUNNING → SUCCEEDED`, Robot `IDLE → NAVIGATING → SERVING → RETURNING → IDLE` (약 25초)

### 실제 Backend 연결 시 확인할 것

1. `npm run dev:real` (또는 `VITE_API_MODE=real`) + `VITE_BACKEND_URL` 지정
2. 임시 계약(`src/types/provisional.ts`)을 Backend 와 확정하고, 다르면 `src/api/*.ts` / `src/api/parsers.ts` 만 수정
3. 화면 코드는 API Adapter 만 사용하므로 수정 불필요

## 구조

```
src/
├── api/            # API Adapter (client, mappingApi, semanticApi, taskApi, stateSocket, parsers)
├── components/
│   ├── common/     # Icon, StatusBadge, Loading/Error/Empty
│   ├── layout/     # AppLayout, 연결 상태 표시/배너
│   ├── map/        # MapViewport(줌/팬/좌표), SemanticLayer, EditorToolbar, Inspector, MappingPanel
│   └── robots/     # RobotCard, ServePanel, TaskTable, FleetMap, RobotLayer
├── config/env.ts   # 환경 변수
├── hooks/          # useMappingControl, useSemanticEditor, useDestinations, useNow
├── mocks/          # MSW handlers, Mock DB, Fleet 시뮬레이터, Mock 제어 패널 (개발 전용)
├── pages/          # DashboardPage, MapSetupPage, RobotControlPage
├── state/          # FleetProvider(로봇/Task/WS), StoreDataProvider(지도/Semantic/매핑), Toast
├── types/          # contracts.ts(공통 Contract), provisional.ts(임시 계약)
└── utils/          # coordinates(픽셀↔ROS), semantic(검증/ID), semanticEditor(편집 reducer), labels
```

데이터 흐름: **Page → Hook/State → API Adapter → (MSW Mock | 실제 Backend)**
화면 컴포넌트는 HTTP/Mock 데이터를 직접 다루지 않습니다.

## 좌표 변환

`src/utils/coordinates.ts` — 상세 규칙은 파일 상단 주석과 `docs/CONTRACT_NOTES.md` 참고.

- 픽셀 인덱스 공식(팀 지시서): `x = origin_x + u·res`, `y = origin_y + (height − 1 − v)·res` — pixel (100, 70) → (4.25, 1.63)
- 클릭/렌더링은 연속 이미지 좌표(map_server 규칙) 사용, 줌/팬은 `screen = image × scale + offset`
- origin yaw ≠ 0 회전 변환 지원, 저장 정밀도 1 mm / 0.0001 rad

## 테스트

`npm test` — 좌표 변환(픽셀↔ROS, 줌/팬 불변, 회전, 경계), ID 중복 방지, Waypoint 참조, Polygon 저장, Semantic 저장/복원,
Map revision 불일치, Task 생성 본문, WebSocket 상태 반영/재연결, API 오류·연결 끊김 UI.
