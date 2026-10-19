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
