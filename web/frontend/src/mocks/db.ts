/**
 * Mock 서버의 상태 저장소 (개발용 데이터 계층).
 * 프론트엔드 화면 상태가 아니라 "가짜 Backend" 의 Source of Truth 역할만 한다.
 * 지도/Semantic 은 새로고침 후에도 유지되도록 localStorage 에 저장한다.
 */
import type { RawMapInfo, SemanticMap } from '../types/contracts'
import type { MappingPhase, Task } from '../types/provisional'
import { buildRawMap, buildSeedSemantics, MOCK_STORE_ID } from './fixtures'

export const FAILURE_KEYS = ['mapping', 'map_load', 'semantics_save', 'task_create', 'task_cancel'] as const
export type FailureKey = (typeof FAILURE_KEYS)[number]

export const FAILURE_LABELS: Record<FailureKey, string> = {
  mapping: '매핑 명령 실패',
  map_load: '지도/Semantic 조회 실패',
  semantics_save: 'Semantic 저장 실패',
  task_create: 'Task 생성 실패',
  task_cancel: 'Task 취소 실패',
}

export interface MockDbState {
  mappingPhase: MappingPhase
  mappingStartedAt: string | null
  /** 보정할 임시 지도(초안)가 준비됨 */
  mappingDraft: boolean
  /** 탐색이 이 시간(ms) 뒤에 스스로 끝난다 (Stop 없이 초안 준비). 0 이하면 끝나지 않음 */
  mappingAutoEndMs: number
  /** true 면 지도 반영(채택)이 주문 때문에 보류된다 (테스트·개발용) */
  adoptBlocked: boolean
  rawMap: RawMapInfo | null
  semantics: SemanticMap | null
  tasks: Task[]
  taskSeq: number
  failures: Record<FailureKey, boolean>
  /** 응답 지연 (ms) */
  latencyMs: number
  /** WebSocket 서버 다운 시뮬레이션 */
  wsDown: boolean
}

const STORAGE_KEY = 'serving-robot-mock-db/v1'

function noFailures(): Record<FailureKey, boolean> {
  return Object.fromEntries(FAILURE_KEYS.map((k) => [k, false])) as Record<FailureKey, boolean>
}

function initialState(): MockDbState {
  return {
    mappingPhase: 'idle',
    mappingStartedAt: null,
    mappingDraft: false,
    mappingAutoEndMs: 8000,
    adoptBlocked: false,
    rawMap: null,
    semantics: null,
    tasks: [],
    taskSeq: 0,
    failures: noFailures(),
    latencyMs: 300,
    wsDown: false,
  }
}

function loadPersisted(): Partial<MockDbState> {
  try {
    if (typeof localStorage === 'undefined') return {}
    const raw = localStorage.getItem(STORAGE_KEY)
    if (!raw) return {}
    const data = JSON.parse(raw) as Pick<MockDbState, 'rawMap' | 'semantics'>
    return { rawMap: data.rawMap ?? null, semantics: data.semantics ?? null }
  } catch {
    return {}
  }
}

export const mockDb: MockDbState = { ...initialState(), ...loadPersisted() }

const listeners = new Set<() => void>()

/** Mock 개발 패널이 변경을 구독하기 위한 용도 */
export function subscribeMockDb(listener: () => void): () => void {
  listeners.add(listener)
  return () => listeners.delete(listener)
}

export function commitMockDb() {
  try {
    if (typeof localStorage !== 'undefined') {
      localStorage.setItem(STORAGE_KEY, JSON.stringify({ rawMap: mockDb.rawMap, semantics: mockDb.semantics }))
    }
  } catch {
    // 저장소 접근 불가 (사생활 보호 모드 등) — 메모리 상태만 유지
  }
  listeners.forEach((l) => l())
}

/** 매장 초기 상태 (지도 없음) */
export function resetMockDb() {
  Object.assign(mockDb, initialState())
  commitMockDb()
}

/** 지도 저장 + Semantic 데모 데이터가 채워진 상태 */
export function seedMockDb() {
  const revision = (mockDb.rawMap?.map_revision ?? 0) + 1
  mockDb.mappingPhase = 'idle'
  mockDb.mappingDraft = false
  mockDb.rawMap = buildRawMap(MOCK_STORE_ID, revision)
  mockDb.semantics = buildSeedSemantics(MOCK_STORE_ID, revision)
  commitMockDb()
}

export function nextTaskId(): string {
  mockDb.taskSeq += 1
  return `task_${String(mockDb.taskSeq).padStart(3, '0')}`
}
