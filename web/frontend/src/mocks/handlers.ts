/**
 * Mock REST 핸들러 (MSW). API Contract 경로/본문을 그대로 재현한다.
 * 화면 코드에서는 이 파일을 import 하지 않는다.
 */
import { delay, http, HttpResponse, type HttpResponseResolver } from 'msw'
import { parseSemanticMap } from '../api/parsers'
import { API_BASE_URL } from '../config/env'
import type { MappingCommandBody, ServeTaskRequest } from '../types/contracts'
import type { MappingStatus, Task } from '../types/provisional'
import { checkRevision, hasErrors, toSemanticPayload, validateSemanticMap } from '../utils/semantic'
import mapPngDataUrl from './assets/demo_store_map.png?inline'
import { commitMockDb, mockDb, nextTaskId, type FailureKey } from './db'
import { buildRawMap, MOCK_STORE_ID } from './fixtures'
import { liveMapHandlers } from './liveMapHandlers'
import { cancelSimTask, enqueueTask, simHome, simMapImageChanged, simReloadAll, simReloadMap, simReset, type RobotCommandProblem } from './simulator'

const api = (path: string) => `${API_BASE_URL}/api${path}`

const error = (status: number, message: string) => HttpResponse.json({ message }, { status })
const refuse = (status: number, code: string, message: string) => HttpResponse.json({ code, message }, { status })

/** 백엔드 routers/robots.py 와 같은 거절 코드·문구 */
function robotRefusal(robotId: string, problem: RobotCommandProblem) {
  switch (problem) {
    case 'not_found':
      return refuse(404, 'ROBOT_NOT_FOUND', `로봇을 찾을 수 없습니다: ${robotId}`)
    case 'bad_node':
      return refuse(400, 'BAD_NODE', '지도에 없는 위치예요.')
    case 'busy':
      return refuse(409, 'ROBOT_BUSY', '로봇이 임무 중입니다. 끝난 뒤에 눌러 주세요.')
    case 'not_localized':
      return refuse(409, 'NOT_LOCALIZED', '로봇이 자기 위치를 아직 모릅니다. 먼저 리셋해서 위치를 잡아 주세요.')
    case 'at_home':
      return refuse(409, 'ALREADY_HOME', '이미 대기 위치에 있습니다.')
  }
}

async function respondDelay() {
  if (mockDb.latencyMs > 0) await delay(mockDb.latencyMs)
}

/** Dev 패널에서 켠 실패 시뮬레이션 */
function injectedFailure(key: FailureKey) {
  return mockDb.failures[key] ? error(503, `[Mock] ${key} 실패 시뮬레이션이 켜져 있습니다.`) : null
}

/** 시작한 뒤 흐른 시간(ms). 탐색이 스스로 끝날 시간이 지났으면 초안을 준비한다 (실제 백엔드와 같은 동작) */
function elapsedMs(): number {
  return mockDb.mappingStartedAt ? Date.now() - Date.parse(mockDb.mappingStartedAt) : 0
}

function settleMapping() {
  if (mockDb.mappingPhase === 'mapping' && mockDb.mappingAutoEndMs > 0 && elapsedMs() >= mockDb.mappingAutoEndMs) {
    mockDb.mappingPhase = 'stopped'
    mockDb.mappingDraft = true
  }
}

function mappingStatus(): MappingStatus {
  settleMapping()
  const ms = elapsedMs()
  const mapping = mockDb.mappingPhase === 'mapping'
  const arrived = Math.floor(ms / 1500)
  return {
    store_id: MOCK_STORE_ID,
    phase: mockDb.mappingPhase,
    started_at: mockDb.mappingStartedAt,
    last_saved_revision: mockDb.rawMap?.map_revision ?? null,
    draft_ready: mockDb.mappingDraft,
    stage: mapping ? (arrived > 0 ? 'exploring' : 'starting') : mockDb.mappingDraft ? 'ready' : 'idle',
    progress: {
      percent: mockDb.mappingDraft ? 100 : mapping ? Math.min(99, Math.floor(95 * (1 - Math.exp(-arrived / 12)))) : 0,
      estimated: !mockDb.mappingDraft,
      elapsed_s: mockDb.mappingStartedAt ? Math.floor(ms / 1000) : null,
      goals_sent: mapping || mockDb.mappingDraft ? arrived + 1 : 0,
      goals_arrived: mapping || mockDb.mappingDraft ? arrived : 0,
      goals_failed: 0,
    },
    message: mockDb.mappingDraft ? '자동 탐색이 끝났어요. 지도를 검토해 주세요.' : null,
    error: null,
    stop_requested: false,
    enabled: true,
    saved: false,
  }
}

async function readJson(request: Request): Promise<Record<string, unknown> | null> {
  try {
    const body: unknown = await request.json()
    return body && typeof body === 'object' && !Array.isArray(body) ? (body as Record<string, unknown>) : null
  } catch {
    return null
  }
}

function mappingHandler(command: 'start' | 'stop' | 'discard'): HttpResponseResolver {
  return async ({ request }) => {
    await respondDelay()
    const failure = injectedFailure('mapping')
    if (failure) return failure
    const body = (await readJson(request)) as Partial<MappingCommandBody> | null
    if (!body || typeof body.store_id !== 'string') return error(400, 'store_id 가 필요합니다.')
    if (body.store_id !== MOCK_STORE_ID) return error(404, `알 수 없는 매장입니다: ${body.store_id}`)
    settleMapping()

    if (command === 'start') {
      if (mockDb.mappingPhase === 'mapping') return refuse(409, 'ALREADY_MAPPING', '이미 매핑 중이에요.')
      if (mockDb.mappingDraft) return refuse(409, 'DRAFT_PENDING', '아직 반영하지 않은 임시 지도가 있어요. 보정해서 저장하거나 버린 뒤에 다시 시작해 주세요.')
      mockDb.mappingPhase = 'mapping'
      mockDb.mappingStartedAt = new Date().toISOString()
    } else if (command === 'stop') {
      if (mockDb.mappingPhase !== 'mapping') return refuse(409, 'NOT_MAPPING', '중지할 매핑이 없어요.')
      mockDb.mappingPhase = 'stopped'
      mockDb.mappingDraft = true
    } else {
      if (mockDb.mappingPhase === 'mapping') return refuse(409, 'ALREADY_MAPPING', '매핑 중에는 버릴 수 없어요. 먼저 중지해 주세요.')
      mockDb.mappingPhase = 'idle'
      mockDb.mappingDraft = false
      mockDb.mappingStartedAt = null
    }
    commitMockDb()
    return HttpResponse.json(mappingStatus())
  }
}

/** 새 지도를 매장 지도로 반영: revision +1, 편집기는 새 지도에서 빈 상태로 시작 */
function adoptMockMap() {
  const revision = (mockDb.rawMap?.map_revision ?? 0) + 1
  mockDb.rawMap = buildRawMap(MOCK_STORE_ID, revision)
  mockDb.semantics = { store_id: MOCK_STORE_ID, map_revision: revision, entities: [], waypoints: [], zones: [] }
  mockDb.mappingPhase = 'idle'
  mockDb.mappingDraft = false
  mockDb.mappingStartedAt = null
  commitMockDb()
  simMapImageChanged()
  return revision
}

const MOCK_SOURCE_HASH = 'mock-draft-hash'

function adoptResult(review: boolean) {
  if (mockDb.adoptBlocked) {
    return HttpResponse.json({
      ...mappingStatus(), saved: true, adopted: false,
      ...(review ? { review_result: { saved: true } } : {}),
      adopt_error: { code: 'ORDERS_ACTIVE', message: '주문이 진행 중이거나 대기 중이라 지도를 바꿀 수 없어요. 주문이 끝난 뒤에 다시 시도해 주세요.' },
    })
  }
  const revision = adoptMockMap()
  return HttpResponse.json({
    ...mappingStatus(), adopted: true, adopt_error: null, needs_robot_restart: true, map_revision: revision,
    ...(review ? { review_result: { saved: true } } : {}),
    message: '새 지도를 적용했어요. 로봇을 재시작해 주세요. (새로운 지도가 적용되었습니다.)',
  })
}

function pngBytes(): Uint8Array<ArrayBuffer> {
  const base64 = mapPngDataUrl.slice(mapPngDataUrl.indexOf(',') + 1)
  const binary = atob(base64)
  const bytes = new Uint8Array(new ArrayBuffer(binary.length))
  for (let i = 0; i < binary.length; i++) bytes[i] = binary.charCodeAt(i)
  return bytes
}

const FORBIDDEN_TASK_KEYS = ['x', 'y', 'pose', 'robot_id', 'position', 'goal']

export const httpHandlers = [
  ...liveMapHandlers,
  http.post(api('/mapping/start'), mappingHandler('start')),
  http.post(api('/mapping/stop'), mappingHandler('stop')),
  http.post(api('/mapping/discard'), mappingHandler('discard')),

  http.post(api('/mapping/save'), async ({ request }) => {
    await respondDelay()
    const failure = injectedFailure('mapping')
    if (failure) return failure
    settleMapping()
    if (!mockDb.mappingDraft) return refuse(409, 'NO_DRAFT', '보정할 수 있는 임시 지도가 아직 없어요.')
    const body = await readJson(request)
    if (!body || body.confirmed !== true) return refuse(422, 'REVIEW_REJECTED', '지도 수정 내용의 최종 확인이 필요합니다.')
    if (body.source_hash !== MOCK_SOURCE_HASH) return refuse(409, 'REVIEW_REJECTED', '지도가 변경되었습니다. 최신 지도를 다시 확인해 주세요.')
    return adoptResult(true)
  }),

  http.post(api('/mapping/adopt'), async () => {
    await respondDelay()
    if (mockDb.mappingDraft && mockDb.mappingPhase !== 'mapping') return adoptResult(false)
    return refuse(409, 'NOT_SAVED', '저장한 보정 지도가 없어요. 먼저 지도를 보정해서 저장해 주세요.')
  }),

  http.get(api('/stores/:storeId/map-review'), async ({ params }) => {
    await respondDelay()
    settleMapping()
    if (params.storeId !== MOCK_STORE_ID) return refuse(404, 'NO_MAP', '매장을 찾지 못했어요.')
    if (!mockDb.mappingDraft) return refuse(409, 'NO_DRAFT', '보정할 수 있는 임시 지도가 아직 없어요.')
    const geo = buildRawMap(MOCK_STORE_ID, 1)
    return HttpResponse.json({
      width: geo.width, height: geo.height, resolution: geo.resolution, origin: geo.origin,
      source_hash: MOCK_SOURCE_HASH,
      image_url: `/api/stores/${MOCK_STORE_ID}/map-review.png`,
      suggestions: [{ id: 'wall-gap-001', x1: 20, y1: 20, x2: 24, y2: 20, gap_m: 0.15 }],
    })
  }),

  http.get(api('/stores/:storeId/map-review.png'), async ({ params }) => {
    if (params.storeId !== MOCK_STORE_ID || !mockDb.mappingDraft) return refuse(409, 'NO_DRAFT', '보정할 수 있는 임시 지도가 아직 없어요.')
    return HttpResponse.arrayBuffer(pngBytes().buffer, { headers: { 'Content-Type': 'image/png' } })
  }),

  http.get(api('/mapping/status'), async () => {
    await respondDelay()
    return HttpResponse.json(mappingStatus())
  }),

  http.get(api('/stores/:storeId/map'), async ({ params }) => {
    await respondDelay()
    const failure = injectedFailure('map_load')
    if (failure) return failure
    if (params.storeId !== MOCK_STORE_ID || !mockDb.rawMap) return error(404, '저장된 지도가 없습니다.')
    return HttpResponse.json(mockDb.rawMap)
  }),

  http.get(api('/stores/:storeId/map.png'), async ({ params }) => {
    if (params.storeId !== MOCK_STORE_ID || !mockDb.rawMap) return error(404, '저장된 지도가 없습니다.')
    return HttpResponse.arrayBuffer(pngBytes().buffer, { headers: { 'Content-Type': 'image/png' } })
  }),

  http.get(api('/stores/:storeId/semantics'), async ({ params }) => {
    await respondDelay()
    const failure = injectedFailure('map_load')
    if (failure) return failure
    if (params.storeId !== MOCK_STORE_ID || !mockDb.semantics) return error(404, 'Semantic Map이 없습니다.')
    return HttpResponse.json(mockDb.semantics)
  }),

  http.put(api('/stores/:storeId/semantics'), async ({ params, request }) => {
    await respondDelay()
    const failure = injectedFailure('semantics_save')
    if (failure) return failure
    if (params.storeId !== MOCK_STORE_ID) return error(404, `알 수 없는 매장입니다: ${String(params.storeId)}`)
    let doc
    try {
      doc = parseSemanticMap(await request.json())
    } catch (e) {
      return error(400, `Semantic Map 형식 오류: ${e instanceof Error ? e.message : String(e)}`)
    }
    if (doc.store_id !== params.storeId) return error(400, 'store_id 가 경로와 다릅니다.')
    const revision = checkRevision(doc, mockDb.rawMap)
    if (!revision.ok) return error(409, revision.message ?? 'Map revision 불일치')
    const issues = validateSemanticMap(doc)
    if (hasErrors(issues)) {
      return error(400, issues.filter((i) => i.level === 'error').map((i) => i.message).join(' / '))
    }
    mockDb.semantics = toSemanticPayload(doc)
    commitMockDb()
    return HttpResponse.json(mockDb.semantics)
  }),

  http.post(api('/robots/reload-map'), async () => {
    await respondDelay()
    return HttpResponse.json({ robots: simReloadAll() })
  }),

  http.post(api('/robots/:robotId/reset'), async ({ params, request }) => {
    await respondDelay()
    const body = (await readJson(request)) as { node?: unknown } | null
    const robotId = String(params.robotId)
    const problem = simReset(robotId, typeof body?.node === 'string' ? body.node : undefined)
    return problem ? robotRefusal(robotId, problem) : HttpResponse.json({ robot_id: robotId })
  }),

  http.post(api('/robots/:robotId/home'), async ({ params }) => {
    await respondDelay()
    const robotId = String(params.robotId)
    const problem = simHome(robotId)
    return problem ? robotRefusal(robotId, problem) : HttpResponse.json({ robot_id: robotId })
  }),

  http.post(api('/robots/:robotId/reload-map'), async ({ params }) => {
    await respondDelay()
    const robotId = String(params.robotId)
    const problem = simReloadMap(robotId)
    return problem ? robotRefusal(robotId, problem) : HttpResponse.json({ robot_id: robotId })
  }),

  http.get(api('/tasks'), async () => {
    await respondDelay()
    const tasks = [...mockDb.tasks].sort((a, b) => b.created_at.localeCompare(a.created_at))
    return HttpResponse.json(tasks)
  }),

  http.post(api('/tasks'), async ({ request }) => {
    await respondDelay()
    const failure = injectedFailure('task_create')
    if (failure) return failure
    const body = (await readJson(request)) as Partial<ServeTaskRequest> | null
    if (!body) return error(400, '요청 본문이 올바르지 않습니다.')
    if (body.type !== 'SERVE') return error(400, `지원하지 않는 Task 타입입니다: ${String(body.type)}`)
    const forbidden = FORBIDDEN_TASK_KEYS.filter((k) => k in body)
    if (forbidden.length) return error(400, `좌표나 로봇을 직접 지정할 수 없습니다: ${forbidden.join(', ')}`)
    if (typeof body.destination_id !== 'string') return error(400, 'destination_id 가 필요합니다.')
    const destinationId = body.destination_id
    const hasWaypoint = mockDb.semantics?.waypoints.some((w) => w.entity_id === destinationId)
    if (!hasWaypoint) return error(422, `유효하지 않은 목적지입니다: ${destinationId}`)

    const now = new Date().toISOString()
    const task: Task = {
      task_id: nextTaskId(),
      type: 'SERVE',
      destination_id: destinationId,
      status: 'QUEUED',
      robot_id: null,
      created_at: now,
      updated_at: now,
    }
    mockDb.tasks.push(task)
    commitMockDb()
    enqueueTask(task)
    return HttpResponse.json(task, { status: 201 })
  }),

  http.post(api('/tasks/:taskId/cancel'), async ({ params }) => {
    await respondDelay()
    const failure = injectedFailure('task_cancel')
    if (failure) return failure
    const task = mockDb.tasks.find((t) => t.task_id === params.taskId)
    if (!task) return error(404, `Task를 찾을 수 없습니다: ${String(params.taskId)}`)
    if (!cancelSimTask(task)) return error(409, `이미 종료된 Task입니다. (${task.status})`)
    return HttpResponse.json(task)
  }),
]
