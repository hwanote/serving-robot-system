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
import { cancelSimTask, enqueueTask } from './simulator'

const api = (path: string) => `${API_BASE_URL}/api${path}`

const error = (status: number, message: string) => HttpResponse.json({ message }, { status })

async function respondDelay() {
  if (mockDb.latencyMs > 0) await delay(mockDb.latencyMs)
}

/** Dev 패널에서 켠 실패 시뮬레이션 */
function injectedFailure(key: FailureKey) {
  return mockDb.failures[key] ? error(503, `[Mock] ${key} 실패 시뮬레이션이 켜져 있습니다.`) : null
}

function mappingStatus(): MappingStatus {
  return {
    store_id: MOCK_STORE_ID,
    phase: mockDb.mappingPhase,
    started_at: mockDb.mappingStartedAt,
    last_saved_revision: mockDb.rawMap?.map_revision ?? null,
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

function mappingHandler(command: 'start' | 'stop' | 'save'): HttpResponseResolver {
  return async ({ request }) => {
    await respondDelay()
    const failure = injectedFailure('mapping')
    if (failure) return failure
    const body = (await readJson(request)) as Partial<MappingCommandBody> | null
    if (!body || typeof body.store_id !== 'string') return error(400, 'store_id 가 필요합니다.')
    if (body.store_id !== MOCK_STORE_ID) return error(404, `알 수 없는 매장입니다: ${body.store_id}`)

    if (command === 'start') {
      if (mockDb.mappingPhase === 'mapping') return error(409, '이미 매핑이 진행 중입니다.')
      mockDb.mappingPhase = 'mapping'
      mockDb.mappingStartedAt = new Date().toISOString()
    } else if (command === 'stop') {
      if (mockDb.mappingPhase !== 'mapping') return error(409, '진행 중인 매핑이 없습니다.')
      mockDb.mappingPhase = 'stopped'
    } else {
      if (mockDb.mappingPhase === 'idle') return error(409, '저장할 매핑 결과가 없습니다. 먼저 매핑을 시작하세요.')
      const revision = (mockDb.rawMap?.map_revision ?? 0) + 1
      mockDb.rawMap = buildRawMap(MOCK_STORE_ID, revision)
      if (mockDb.mappingPhase === 'stopped') {
        mockDb.mappingPhase = 'idle'
        mockDb.mappingStartedAt = null
      }
    }
    commitMockDb()
    return HttpResponse.json(mappingStatus())
  }
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
  http.post(api('/mapping/start'), mappingHandler('start')),
  http.post(api('/mapping/stop'), mappingHandler('stop')),
  http.post(api('/mapping/save'), mappingHandler('save')),

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
