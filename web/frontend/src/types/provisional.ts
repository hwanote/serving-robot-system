/**
 * 임시 계약 (Provisional Contract)
 *
 * 아래 형식은 Backend/Fleet 파트와 아직 확정되지 않았다.
 * Mock 서버와 프론트엔드가 같은 형식을 쓰도록 여기 한 곳에 모아 두고,
 * 실제 Backend 확정 시 이 파일과 src/api/* 파서만 수정한다.
 * 상태 문자열은 contracts.ts 의 공통 값만 사용한다.
 */
import type { SemanticMap, TaskStatus } from './contracts'

/** POST /api/tasks 응답 및 GET /api/tasks 항목 */
export interface Task {
  task_id: string
  type: 'SERVE'
  destination_id: string
  status: TaskStatus
  /** 배정 전에는 null */
  robot_id: string | null
  /** ISO 8601 */
  created_at: string
  /** ISO 8601 */
  updated_at: string
}

/** WebSocket /ws/state 의 Task 상태 이벤트 (임시) */
export interface TaskStateEvent {
  type: 'task_state'
  task_id: string
  status: TaskStatus
  robot_id: string | null
  destination_id: string
  updated_at: string
}

/** 매핑 세션 상태 (Robot/Task 상태와 무관한 UI용 값, 소문자로 구분) */
export type MappingPhase = 'idle' | 'mapping' | 'stopped'

/** 자동 매핑 진행 단계 (백엔드 session.py). 옛 백엔드/mock 은 보내지 않을 수 있다. */
export type MappingStage = 'idle' | 'starting' | 'exploring' | 'finishing' | 'converting' | 'ready' | 'failed'

export interface MappingProgress {
  /** 추정 진행률 (0~100). 줄어들지 않고, 초안이 준비되기 전에는 99 를 넘지 않는다 */
  percent: number
  /** true 면 "추정" 이라고 표기한다 */
  estimated: boolean
  elapsed_s: number | null
  goals_sent: number
  goals_arrived: number
  goals_failed: number
}

/** GET /api/mapping/status 및 mapping 명령 응답 (임시) */
export interface MappingStatus {
  store_id: string
  phase: MappingPhase
  started_at: string | null
  /** 현재 매장 지도의 revision. 없으면 null */
  last_saved_revision: number | null
  /** 보정할 임시 지도(초안)가 준비됨 */
  draft_ready?: boolean
  stage?: MappingStage
  progress?: MappingProgress
  message?: string | null
  error?: string | null
  stop_requested?: boolean
  /** false 면 백엔드에서 자동 매핑이 꺼져 있다 (mapping_enabled) */
  enabled?: boolean
  /** 보정 저장은 됐지만 지도 반영(채택)이 남아 있다 */
  saved?: boolean
}

/** POST /api/mapping/save · /api/mapping/adopt 응답 */
export interface MapSaveResult extends MappingStatus {
  review_result?: { saved: boolean; status?: string }
  adopted: boolean
  adopt_error?: { code: string; message: string } | null
  needs_robot_restart?: boolean
  backup?: { id: string; has_semantics: boolean } | null
  map_revision?: number
}

/** GET /api/stores/{id}/map-review */
export interface ReviewSuggestion {
  id: string
  x1: number
  y1: number
  x2: number
  y2: number
  gap_m: number
}

export interface MapReviewData {
  width: number
  height: number
  resolution: number
  origin: { x: number; y: number; yaw: number }
  source_hash: string
  image_url: string
  suggestions: ReviewSuggestion[]
}

export interface MapSaveBody {
  source_hash: string
  confirmed: true
  strokes: Array<{ x1: number; y1: number; x2: number; y2: number; value: number; points?: Array<{ x: number; y: number }> }>
}

// ---------- 지도 반영 (Backend semantics 적용 / config/common 반영 / 백업) ----------

/** GET /api/stores/{id}/semantics/status */
export interface SemanticStatus {
  /** 저장해 둔 초안. 없으면 null */
  draft: { saved_at: string; sha: string; map_revision: number | null } | null
  /** 자동 노드 생성 결과. 없으면 null */
  applied: { applied_at: string; source_sha: string; summary: { OK: number; WARN: number; FAIL: number }; notes: string[]; warnings: string[] } | null
  /** 적용 결과가 지금 초안과 같은지 */
  in_sync: boolean
}

/** GET /api/stores/{id}/semantics/live — 로봇이 읽는 지도(config/common) 상태 */
export interface LiveStatus {
  /** 적용 결과가 이미 로봇 지도와 같은지 */
  matches_applied: boolean
  backup_count: number
  last: { at: string; kind: 'promote' | 'restore'; backup_id: string; note: string; needs_robot_restart: boolean } | null
}

/** 지도 백업 한 건 (id 는 날짜-시각: 20261023-153012) */
export interface MapBackup {
  id: string
  created_at: string
  kind: 'promote' | 'before_restore'
  note: string
  summary: { nodes: number; edges: number; tables: number } | null
  /** 그 지도를 만든 편집 화면 저장본이 백업에 같이 있는지 (옛 백엔드면 null) */
  has_semantics: boolean | null
}

/** POST promote / restore 응답 */
export interface LiveChangeResult {
  backup: MapBackup
  live: NonNullable<LiveStatus['last']>
  /** 복구 때 함께 되돌려진 편집 화면 저장본 (없으면 null) */
  semantics: SemanticMap | null
  semantics_restored: boolean
  /** 편집 화면 저장본을 되돌리지 못했을 때의 이유 */
  semantics_note: string | null
}

// ---------- 로봇 카드 버튼 (리셋 / 대기 위치로 / 지도 다시 읽기) ----------

/** 백엔드가 계산해서 robot_state 이벤트에 싣는 카드 버튼 규칙. 화면은 이 값만 따른다. */
export interface RobotControls {
  /** 연결돼 있으면 true */
  reset: boolean
  /** 리셋할 때 고를 수 있는 위치 (대기 위치, 주방). 첫 번째가 기본값 */
  reset_nodes: string[]
  /** 쉬고 있고 위치가 OK 일 때만 true (위치를 잃었으면 먼저 리셋) */
  home: boolean
  /** 쉬고 있고 로봇 지도가 최신이 아닐 때만 true */
  reload_map: boolean
}

/** robot_state 이벤트의 임시 추가 필드 (없어도 동작) */
export interface RobotExtras {
  /** OK | DEGRADED | LOST | UNINITIALIZED. 모르면 없음 */
  localization?: string | null
  /** 로봇이 쓰는 지도가 지금 config/common 과 같은지. 모르면 null */
  map_current?: boolean | null
  /** 로봇이 시작할 때 읽은 지도 그림이 지금 매장 지도와 같은지. false 면 로봇 재시작이 필요하다. 모르면 null */
  map_image_current?: boolean | null
  controls?: RobotControls
}

// ---------- 자동 생성 통로 미리보기 (GET /api/stores/{id}/semantics/graph) ----------

export type CheckLevel = 'OK' | 'WARN' | 'FAIL'

export interface GraphNode {
  id: string
  /** map frame (m) */
  x: number
  y: number
  level: CheckLevel
}

export interface GraphEdge {
  id: string
  from: string
  to: string
  length_m: number
  level: CheckLevel
}

/** 자동 생성이 정한 정차 위치와 방향. 로봇이 이 자리에 서서 yaw 방향을 바라본다 */
export interface GraphStop {
  /** 생성기가 쓰는 이름 (robot_wait_NN 은 standby_NN) */
  id: string
  /** 편집 화면 객체 id. 모르면 null */
  entity_id: string | null
  node: string
  x: number
  y: number
  /** rad (map frame, 0 = +x 동쪽, π/2 = +y 북쪽) */
  yaw: number
  yaw_deg: number
  kind: 'table_service' | 'pickup' | 'standby'
  level: CheckLevel
  /** 사람이 손으로 지정한 정차 위치인지 (waypoint.manual) */
  pinned: boolean
}

/** 통로 위에 놓지 못한 점 */
export interface GraphExcluded {
  id: string
  entity_id: string | null
  reason: string
  /** 점의 위치. 모르면 null */
  x: number | null
  y: number | null
  /** 손으로 지정한 정차 위치가 검사에 실패한 경우, 지정한 자리와 방향 */
  pin: { x: number; y: number; yaw_deg: number | null } | null
}

export interface GeneratedGraph {
  /** draft = 저장한 초안으로 계산, config = 저장한 초안이 없어 로봇 지도(config/common)로 계산 */
  source: 'draft' | 'config'
  draft_sha: string
  /** 지금 적용 결과가 이 초안으로 만든 것인지 */
  matches_applied: boolean
  /** false 면 "노드 자동 생성"이 거절될 상황 (그래도 그래프는 돌려준다) */
  ok: boolean
  nodes: GraphNode[]
  edges: GraphEdge[]
  stops: GraphStop[]
  excluded: GraphExcluded[]
  /** 문제 한 줄씩 (한국어) */
  problems: string[]
  warnings: string[]
  notes: string[]
  summary: { OK: number; WARN: number; FAIL: number }
}
