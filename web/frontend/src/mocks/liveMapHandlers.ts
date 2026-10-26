/**
 * 지도 반영(적용 → 로봇 지도 반영 → 백업/복구) Mock.
 * 실제 Backend 의 web/backend/app/promote.py 와 같은 규칙(거절 코드 포함)을 흉내 낸다.
 * 상태는 메모리에만 두고 새로고침하면 초기화된다.
 */
import { delay, http, HttpResponse } from 'msw'
import { parseSemanticMap } from '../api/parsers'
import { API_BASE_URL } from '../config/env'
import type { SemanticMap } from '../types/contracts'
import { hasErrors, validateSemanticMap } from '../utils/semantic'
import { mockDb } from './db'
import { MOCK_STORE_ID } from './fixtures'
import { buildMockGraph } from './graphFixture'

const api = (path: string) => `${API_BASE_URL}/api/stores/:storeId/semantics${path}`

interface Snapshot {
  sha: string
  summary: { nodes: number; edges: number; tables: number }
  /** 그 지도를 만든 편집 화면 저장본 (없을 수 있음) */
  semantics?: SemanticMap | null
}
interface Backup extends Snapshot {
  id: string
  created_at: string
  kind: 'promote' | 'before_restore'
  note: string
}

const state = {
  applied: null as null | { applied_at: string; source_sha: string; summary: { OK: number; WARN: number; FAIL: number } },
  live: { sha: 'initial', summary: { nodes: 20, edges: 19, tables: 4 } } as Snapshot,
  backups: [] as Backup[],
  last: null as null | { at: string; kind: 'promote' | 'restore'; backup_id: string; note: string; needs_robot_restart: boolean },
}

/** 로봇 지도 비교용: 지금 "config/common" 의 지문 */
export const liveSha = () => state.live.sha

/** 테스트·개발 패널에서 처음 상태로 */
export function resetLiveMapMock() {
  state.applied = null
  state.live = { sha: 'initial', summary: { nodes: 20, edges: 19, tables: 4 } }
  state.backups = []
  state.last = null
}

function hash(text: string): string {
  let h = 5381
  for (let i = 0; i < text.length; i++) h = ((h << 5) + h + text.charCodeAt(i)) | 0
  return (h >>> 0).toString(16)
}

const draftSha = () => (mockDb.semantics ? hash(JSON.stringify(mockDb.semantics)) : null)

function currentSummary() {
  const tables = mockDb.semantics?.entities.filter((e) => e.type === 'table').length ?? 0
  return { nodes: 16 + tables * 2, edges: 15 + tables * 2, tables }
}

const pad = (n: number) => String(n).padStart(2, '0')

function newId(): string {
  const d = new Date()
  const stamp = `${d.getFullYear()}${pad(d.getMonth() + 1)}${pad(d.getDate())}-${pad(d.getHours())}${pad(d.getMinutes())}${pad(d.getSeconds())}`
  let id = stamp
  for (let n = 2; state.backups.some((b) => b.id === id); n++) id = `${stamp}-${n}`
  return id
}

const fail = (status: number, code: string, message: string) => HttpResponse.json({ code, message }, { status })
const busy = () => mockDb.tasks.some((t) => t.status === 'QUEUED' || t.status === 'ASSIGNED' || t.status === 'RUNNING')

async function pause() {
  if (mockDb.latencyMs > 0) await delay(mockDb.latencyMs)
}

function publicBackup(b: Backup) {
  return { id: b.id, created_at: b.created_at, kind: b.kind, note: b.note, summary: b.summary, has_semantics: !!b.semantics }
}

const clone = <T,>(v: T): T => JSON.parse(JSON.stringify(v)) as T

function swap(kind: 'promote' | 'restore', target: Snapshot, note: string, backupKind: Backup['kind'], backupNote: string) {
  // 복구 직전 백업에는 지금 편집 화면에 보이는 저장본을 담는다 (실제 백엔드와 같은 규칙)
  const keep = backupKind === 'before_restore' ? (mockDb.semantics ? clone(mockDb.semantics) : null) : (state.live.semantics ?? null)
  const backup: Backup = { id: newId(), created_at: new Date().toISOString(), kind: backupKind, note: backupNote, ...state.live, semantics: keep }
  state.backups.unshift(backup)
  state.live = { ...target }
  state.last = { at: new Date().toISOString(), kind, backup_id: backup.id, note, needs_robot_restart: true }
  // 저장본이 없는 옛 백업: 서버는 지도 파일에서 편집 화면을 다시 만든다(직접 그린 구역은 빠짐) — Mock 은 지금 초안에서 구역만 비워 흉내
  const rebuilt = kind === 'restore' && !target.semantics && mockDb.semantics ? { ...clone(mockDb.semantics), zones: [] } : null
  const restored = kind === 'restore' ? (target.semantics ? clone(target.semantics) : rebuilt) : null
  if (restored) mockDb.semantics = restored
  return HttpResponse.json({
    backup: publicBackup(backup),
    live: state.last,
    semantics: restored,
    semantics_restored: !!restored,
    semantics_note: kind === 'restore' && !target.semantics ? '이 백업에는 편집 화면 저장본이 없어서 지도 파일에서 다시 만들었어요. 직접 그린 구역은 빠져요.' : null,
  })
}

export const liveMapHandlers = [
  http.get(api('/status'), async ({ params }) => {
    await pause()
    if (params.storeId !== MOCK_STORE_ID || !mockDb.semantics) return fail(404, 'NO_SEMANTICS', 'Semantic Map이 없습니다.')
    const sha = draftSha()!
    return HttpResponse.json({
      draft: { saved_at: new Date().toISOString(), sha, map_revision: mockDb.semantics.map_revision },
      applied: state.applied && { ...state.applied, notes: [], warnings: [] },
      in_sync: state.applied?.source_sha === sha,
    })
  }),

  http.get(api('/graph'), async ({ params }) => {
    await pause()
    if (params.storeId !== MOCK_STORE_ID || !mockDb.semantics) return fail(404, 'NO_SEMANTICS', 'Semantic Map이 없습니다.')
    const sha = draftSha()!
    const graph = buildMockGraph(mockDb.semantics, sha, state.applied?.source_sha === sha)
    if (!graph) return fail(422, 'GENERATION_FAILED', '자동 노드 생성에 실패했어요: 통로를 만들 Waypoint가 없어요.')
    return HttpResponse.json(graph)
  }),

  // 저장하지 않은 편집 내용으로 같은 계산 (본문 = Semantic Map)
  http.post(api('/graph'), async ({ params, request }) => {
    await pause()
    if (params.storeId !== MOCK_STORE_ID) return fail(404, 'NO_SEMANTICS', 'Semantic Map이 없습니다.')
    let doc: SemanticMap
    try {
      doc = parseSemanticMap(await request.json())
    } catch (e) {
      return fail(400, 'INVALID_SEMANTICS', `Semantic Map 형식 오류: ${e instanceof Error ? e.message : String(e)}`)
    }
    const graph = buildMockGraph(doc, hash(JSON.stringify(doc)), false)
    if (!graph) return fail(422, 'GENERATION_FAILED', '자동 노드 생성에 실패했어요: 통로를 만들 Waypoint가 없어요.')
    return HttpResponse.json(graph)
  }),

  http.post(api('/apply'), async ({ params }) => {
    await pause()
    if (params.storeId !== MOCK_STORE_ID || !mockDb.semantics) return fail(404, 'NO_SEMANTICS', 'Semantic Map이 없습니다.')
    const problems = validateSemanticMap(mockDb.semantics).filter((i) => i.level === 'error')
    if (hasErrors(problems)) return fail(422, 'MISSING_TOP_SIZE', problems.map((p) => p.message).join('\n'))
    state.applied = { applied_at: new Date().toISOString(), source_sha: draftSha()!, summary: { OK: 21, WARN: 3, FAIL: 0 } }
    return HttpResponse.json({ ...state.applied })
  }),

  http.get(api('/live'), async () => {
    await pause()
    return HttpResponse.json({
      live_sha: state.live.sha,
      matches_applied: !!state.applied && state.applied.source_sha === state.live.sha,
      backup_count: state.backups.length,
      last: state.last,
    })
  }),

  http.post(api('/promote'), async () => {
    await pause()
    if (!state.applied) return fail(409, 'NOT_APPLIED', '적용 결과가 없어요. 먼저 적용해 주세요.')
    if (state.applied.source_sha !== draftSha()) return fail(409, 'NOT_IN_SYNC', '저장한 초안이 적용 결과보다 새로워요. 다시 적용한 뒤 반영해 주세요.')
    if (busy()) return fail(409, 'ORDERS_ACTIVE', '주문이 진행 중이거나 대기 중이라 지도를 바꿀 수 없어요. 주문이 끝난 뒤에 다시 시도해 주세요.')
    if (state.applied.source_sha === state.live.sha) return fail(409, 'ALREADY_LIVE', '이미 로봇이 읽는 지도와 같아요.')
    return swap('promote', { sha: state.applied.source_sha, summary: currentSummary(), semantics: mockDb.semantics ? clone(mockDb.semantics) : null }, '적용 결과를 로봇 지도로 반영', 'promote', '적용 결과를 반영하기 전 지도')
  }),

  http.get(api('/backups'), async () => {
    await pause()
    return HttpResponse.json(state.backups.map(publicBackup))
  }),

  http.post(api('/backups/:backupId/restore'), async ({ params }) => {
    await pause()
    const target = state.backups.find((b) => b.id === params.backupId)
    if (!target) return fail(404, 'BACKUP_NOT_FOUND', `그 백업을 찾지 못했어요: ${String(params.backupId)}`)
    if (busy()) return fail(409, 'ORDERS_ACTIVE', '주문이 진행 중이거나 대기 중이라 지도를 바꿀 수 없어요. 주문이 끝난 뒤에 다시 시도해 주세요.')
    if (target.sha === state.live.sha) return fail(409, 'ALREADY_LIVE', '이미 그 백업과 같은 지도예요.')
    return swap('restore', target, `${target.id} 백업으로 되돌림`, 'before_restore', `${target.id} 백업으로 되돌리기 전 지도`)
  }),
]
