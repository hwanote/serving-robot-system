/**
 * 팀 공통 Contract 타입.
 * 여기 정의된 이름/문자열은 Backend·Mapping·Fleet 파트와 합의된 값이므로 임의로 바꾸지 않는다.
 * (아직 합의되지 않은 임시 계약은 ./provisional.ts 에 분리)
 */

/** ROS 좌표계 (Semantic Map 좌표는 항상 map frame 기준) */
export const ROS_FRAMES = ['map', 'odom', 'base_footprint', 'base_link'] as const
export type RosFrame = (typeof ROS_FRAMES)[number]

export const ROBOT_STATES = [
  'IDLE',
  'NAVIGATING',
  'SERVING',
  'RETURNING',
  'CHARGING',
  'PAUSED',
  'ERROR',
] as const
export type RobotState = (typeof ROBOT_STATES)[number]

export const TASK_STATUSES = ['QUEUED', 'ASSIGNED', 'RUNNING', 'SUCCEEDED', 'FAILED', 'CANCELED'] as const
export type TaskStatus = (typeof TASK_STATUSES)[number]

export function isRobotState(value: unknown): value is RobotState {
  return typeof value === 'string' && (ROBOT_STATES as readonly string[]).includes(value)
}

export function isTaskStatus(value: unknown): value is TaskStatus {
  return typeof value === 'string' && (TASK_STATUSES as readonly string[]).includes(value)
}

/** x, y: m / yaw: rad (map frame) */
export interface Pose {
  x: number
  y: number
  yaw: number
}

// ---------- Mapping ----------

export interface MappingCommandBody {
  store_id: string
}

// ---------- Raw Map ----------

export interface RawMapInfo {
  store_id: string
  map_revision: number
  image_url: string
  /** m / pixel */
  resolution: number
  /** pixel */
  width: number
  /** pixel */
  height: number
  /** 지도 좌하단 픽셀의 map frame 기준 pose */
  origin: Pose
  frame_id: 'map'
}

// ---------- Semantic Map ----------

/**
 * robot_wait 는 UI 제안 타입이다. Semantic API 확정 전 Mapping 파트와 타입명 확인 필요.
 * (docs/CONTRACT_NOTES.md 참고)
 */
export const ENTITY_TYPES = ['table', 'counter', 'kitchen', 'robot_wait'] as const
export type EntityType = (typeof ENTITY_TYPES)[number]

export interface SemanticEntity {
  /** 시스템 생성 ID (예: table_01). label 변경과 무관하게 불변 */
  id: string
  type: EntityType
  /** 사용자 표시 이름 */
  label: string
  /**
   * [임시 확장 필드] 지도 위 객체 표시 위치 (map frame).
   * Contract 예시에는 없으므로 Mapping 파트와 합의 필요. 없으면 Waypoint 위치에 표시한다.
   */
  pose?: Pose
}

export interface Waypoint {
  /** 예: wp_table_01 */
  id: string
  /** 참조하는 SemanticEntity.id */
  entity_id: string
  /** 로봇이 접근할 위치와 방향 (map frame) */
  pose: Pose
}

export const ZONE_TYPES = ['exclusive', 'no_go', 'slow'] as const
export type ZoneType = (typeof ZONE_TYPES)[number]

/** [x, y] (m, map frame) */
export type MapPoint = [number, number]

export interface Zone {
  id: string
  type: ZoneType
  /** 최소 3개 꼭짓점, map frame 좌표 */
  polygon: MapPoint[]
}

export interface SemanticMap {
  store_id: string
  map_revision: number
  entities: SemanticEntity[]
  waypoints: Waypoint[]
  zones: Zone[]
}

// ---------- Task ----------

/** 사용자는 좌표가 아닌 목적지 ID만 전달한다. 로봇 배정/좌표는 Backend(Fleet)가 결정. */
export interface ServeTaskRequest {
  type: 'SERVE'
  destination_id: string
}

// ---------- WebSocket /ws/state ----------

export interface RobotStateEvent {
  type: 'robot_state'
  robot_id: string
  state: RobotState
  task_id: string | null
  destination_id: string | null
  battery_percent: number
  pose: Pose
}
