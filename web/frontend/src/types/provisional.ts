/**
 * 임시 계약 (Provisional Contract)
 *
 * 아래 형식은 Backend/Fleet 파트와 아직 확정되지 않았다.
 * Mock 서버와 프론트엔드가 같은 형식을 쓰도록 여기 한 곳에 모아 두고,
 * 실제 Backend 확정 시 이 파일과 src/api/* 파서만 수정한다.
 * 상태 문자열은 contracts.ts 의 공통 값만 사용한다.
 */
import type { TaskStatus } from './contracts'

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

/** GET /api/mapping/status 및 mapping 명령 응답 (임시) */
export interface MappingStatus {
  store_id: string
  phase: MappingPhase
  started_at: string | null
  /** 마지막으로 저장된 Raw Map revision. 없으면 null */
  last_saved_revision: number | null
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
}

/** POST promote / restore 응답 */
export interface LiveChangeResult {
  backup: MapBackup
  live: NonNullable<LiveStatus['last']>
}

// ---------- 로봇 카드 버튼 (리셋 / 홈 / 지도 다시 읽기) ----------

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
  controls?: RobotControls
}
