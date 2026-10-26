import type { SocketStatus } from '../api/stateSocket'
import type { EntityType, RobotState, TaskStatus, ZoneType } from '../types/contracts'
import type { MappingPhase, MappingStage } from '../types/provisional'

export type Tone = 'gray' | 'blue' | 'teal' | 'indigo' | 'amber' | 'slate' | 'red' | 'green'

export const ROBOT_STATE_META: Record<RobotState, { label: string; tone: Tone }> = {
  IDLE: { label: '대기', tone: 'green' },
  NAVIGATING: { label: '이동 중', tone: 'blue' },
  SERVING: { label: '서빙 중', tone: 'teal' },
  RETURNING: { label: '복귀 중', tone: 'indigo' },
  CHARGING: { label: '충전 중', tone: 'amber' },
  PAUSED: { label: '일시정지', tone: 'slate' },
  ERROR: { label: '오류', tone: 'red' },
}

export const TASK_STATUS_META: Record<TaskStatus, { label: string; tone: Tone }> = {
  QUEUED: { label: '대기열', tone: 'gray' },
  ASSIGNED: { label: '배정됨', tone: 'indigo' },
  RUNNING: { label: '수행 중', tone: 'blue' },
  SUCCEEDED: { label: '완료', tone: 'green' },
  FAILED: { label: '실패', tone: 'red' },
  CANCELED: { label: '취소됨', tone: 'slate' },
}

export const SOCKET_STATUS_META: Record<SocketStatus, { label: string; tone: Tone }> = {
  connecting: { label: '연결 중', tone: 'amber' },
  open: { label: '실시간 연결됨', tone: 'green' },
  reconnecting: { label: '재연결 중', tone: 'red' },
  closed: { label: '연결 종료', tone: 'slate' },
}

export const MAPPING_PHASE_META: Record<MappingPhase, { label: string; tone: Tone }> = {
  idle: { label: '대기', tone: 'gray' },
  mapping: { label: '매핑 진행 중', tone: 'blue' },
  stopped: { label: '중지됨 (미저장)', tone: 'amber' },
}

export const MAPPING_STAGE_LABEL: Record<MappingStage, string> = {
  idle: '대기',
  starting: '시작하는 중',
  exploring: '탐색 중',
  finishing: '마무리하는 중',
  converting: '지도 초안 만드는 중',
  ready: '지도 초안 준비됨',
  failed: '지도 초안을 만들지 못함',
}

/** 단계 막대에 보여 주는 순서 (failed 는 따로 표시) */
export const MAPPING_STAGE_ORDER: MappingStage[] = ['starting', 'exploring', 'finishing', 'converting', 'ready']

export const ENTITY_TYPE_LABEL: Record<EntityType, string> = {
  table: '테이블',
  counter: '카운터',
  kitchen: '주방',
  robot_wait: '로봇 대기 위치',
}

/** 구역(전용·진입 금지·감속)은 편집 초안에만 저장되고 로봇 동작에는 아직 반영되지 않는다. */
export const ZONE_NOT_APPLIED_NOTE = '구역은 지금 저장만 되고 로봇 동작에는 아직 반영되지 않아요.'

export const ZONE_TYPE_LABEL: Record<ZoneType, string> = {
  exclusive: '전용 구역',
  no_go: '진입 금지',
  slow: '감속 구역',
}

/** 로봇이 작업(이동/서빙/복귀) 중인지 */
export const isWorkingState = (s: RobotState) => s === 'NAVIGATING' || s === 'SERVING' || s === 'RETURNING'

export function formatTime(iso: string | null | undefined): string {
  if (!iso) return '-'
  const d = new Date(iso)
  if (Number.isNaN(d.getTime())) return '-'
  return d.toLocaleTimeString('ko-KR', { hour: '2-digit', minute: '2-digit', second: '2-digit' })
}

export function formatAgo(ms: number, now = Date.now()): string {
  const sec = Math.max(0, Math.round((now - ms) / 1000))
  if (sec < 5) return '방금'
  if (sec < 60) return `${sec}초 전`
  const min = Math.floor(sec / 60)
  if (min < 60) return `${min}분 전`
  return `${Math.floor(min / 60)}시간 전`
}
