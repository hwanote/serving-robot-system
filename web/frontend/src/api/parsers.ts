/**
 * 서버 응답/이벤트 런타임 검증.
 * Contract와 다른 데이터가 들어오면 화면에 반영하지 않고 오류로 처리한다.
 */
import {
  ENTITY_TYPES,
  isRobotState,
  isTaskStatus,
  ZONE_TYPES,
  type Chair,
  type EntityType,
  type MapPoint,
  type Pose,
  type RawMapInfo,
  type RobotStateEvent,
  type SemanticEntity,
  type SemanticMap,
  type Waypoint,
  type Zone,
  type ZoneType,
} from '../types/contracts'
import type {
  CheckLevel,
  GeneratedGraph,
  GraphEdge,
  GraphExcluded,
  GraphNode,
  GraphStop,
  LiveChangeResult,
  LiveStatus,
  MapBackup,
  RobotControls,
  RobotExtras,
  SemanticStatus,
  Task,
  TaskStateEvent,
} from '../types/provisional'

export class ContractError extends Error {
  constructor(message: string) {
    super(message)
    this.name = 'ContractError'
  }
}

type Obj = Record<string, unknown>

function obj(value: unknown, where: string): Obj {
  if (!value || typeof value !== 'object' || Array.isArray(value)) throw new ContractError(`${where}: 객체가 아닙니다.`)
  return value as Obj
}
function num(value: unknown, where: string): number {
  if (typeof value !== 'number' || !Number.isFinite(value)) throw new ContractError(`${where}: 숫자가 아닙니다.`)
  return value
}
function str(value: unknown, where: string): string {
  if (typeof value !== 'string' || value.length === 0) throw new ContractError(`${where}: 문자열이 아닙니다.`)
  return value
}
function optStr(value: unknown, where: string): string | null {
  if (value === null || value === undefined || value === '') return null
  return str(value, where)
}
function arr(value: unknown, where: string): unknown[] {
  if (!Array.isArray(value)) throw new ContractError(`${where}: 배열이 아닙니다.`)
  return value
}

export function parsePose(value: unknown, where = 'pose'): Pose {
  const o = obj(value, where)
  return { x: num(o.x, `${where}.x`), y: num(o.y, `${where}.y`), yaw: num(o.yaw ?? 0, `${where}.yaw`) }
}

export function parseRawMap(value: unknown): RawMapInfo {
  const o = obj(value, 'map')
  const frame = str(o.frame_id, 'map.frame_id')
  if (frame !== 'map') throw new ContractError(`map.frame_id 는 'map' 이어야 합니다. (받은 값: ${frame})`)
  const info: RawMapInfo = {
    store_id: str(o.store_id, 'map.store_id'),
    map_revision: num(o.map_revision, 'map.map_revision'),
    image_url: str(o.image_url, 'map.image_url'),
    resolution: num(o.resolution, 'map.resolution'),
    width: num(o.width, 'map.width'),
    height: num(o.height, 'map.height'),
    origin: parsePose(o.origin, 'map.origin'),
    frame_id: 'map',
  }
  if (info.resolution <= 0 || info.width <= 0 || info.height <= 0) {
    throw new ContractError('map 의 resolution/width/height 는 0보다 커야 합니다.')
  }
  return info
}

function parseEntity(value: unknown, i: number): SemanticEntity {
  const o = obj(value, `entities[${i}]`)
  const type = str(o.type, `entities[${i}].type`)
  if (!(ENTITY_TYPES as readonly string[]).includes(type)) {
    throw new ContractError(`entities[${i}].type 알 수 없는 타입: ${type}`)
  }
  const entity: SemanticEntity = {
    id: str(o.id, `entities[${i}].id`),
    type: type as EntityType,
    label: typeof o.label === 'string' ? o.label : '',
  }
  if (o.pose !== undefined && o.pose !== null) entity.pose = parsePose(o.pose, `entities[${i}].pose`)
  if (o.top_size_m !== undefined && o.top_size_m !== null) {
    const t = obj(o.top_size_m, `entities[${i}].top_size_m`)
    entity.top_size_m = { x: num(t.x, `entities[${i}].top_size_m.x`), y: num(t.y, `entities[${i}].top_size_m.y`) }
  }
  if (o.chairs !== undefined && o.chairs !== null) {
    entity.chairs = arr(o.chairs, `entities[${i}].chairs`).map((c, j): Chair => {
      const co = obj(c, `entities[${i}].chairs[${j}]`)
      return {
        dx: num(co.dx, `entities[${i}].chairs[${j}].dx`),
        dy: num(co.dy, `entities[${i}].chairs[${j}].dy`),
        size_m: num(co.size_m, `entities[${i}].chairs[${j}].size_m`),
      }
    })
  }
  return entity
}

function parseWaypoint(value: unknown, i: number): Waypoint {
  const o = obj(value, `waypoints[${i}]`)
  return {
    id: str(o.id, `waypoints[${i}].id`),
    entity_id: str(o.entity_id, `waypoints[${i}].entity_id`),
    pose: parsePose(o.pose, `waypoints[${i}].pose`),
    ...(o.manual === true ? { manual: true } : {}), // 손으로 지정한 정차 위치 (임시 확장)
  }
}

function parseZone(value: unknown, i: number): Zone {
  const o = obj(value, `zones[${i}]`)
  const type = str(o.type, `zones[${i}].type`)
  if (!(ZONE_TYPES as readonly string[]).includes(type)) throw new ContractError(`zones[${i}].type 알 수 없는 타입: ${type}`)
  const polygon = arr(o.polygon, `zones[${i}].polygon`).map((p, j): MapPoint => {
    const pair = arr(p, `zones[${i}].polygon[${j}]`)
    return [num(pair[0], `zones[${i}].polygon[${j}][0]`), num(pair[1], `zones[${i}].polygon[${j}][1]`)]
  })
  return { id: str(o.id, `zones[${i}].id`), type: type as ZoneType, polygon }
}

export function parseSemanticMap(value: unknown): SemanticMap {
  const o = obj(value, 'semantics')
  return {
    store_id: str(o.store_id, 'semantics.store_id'),
    map_revision: num(o.map_revision, 'semantics.map_revision'),
    entities: arr(o.entities, 'semantics.entities').map(parseEntity),
    waypoints: arr(o.waypoints, 'semantics.waypoints').map(parseWaypoint),
    zones: arr(o.zones, 'semantics.zones').map(parseZone),
  }
}

export function parseTask(value: unknown): Task {
  const o = obj(value, 'task')
  const status = o.status
  if (!isTaskStatus(status)) throw new ContractError(`task.status 알 수 없는 상태: ${String(status)}`)
  const now = new Date().toISOString()
  return {
    task_id: str(o.task_id, 'task.task_id'),
    type: 'SERVE',
    destination_id: str(o.destination_id, 'task.destination_id'),
    status,
    robot_id: optStr(o.robot_id, 'task.robot_id'),
    created_at: typeof o.created_at === 'string' ? o.created_at : now,
    updated_at: typeof o.updated_at === 'string' ? o.updated_at : now,
  }
}

export type StateSocketEvent = (RobotStateEvent & RobotExtras) | TaskStateEvent

function parseControls(value: unknown): RobotControls | undefined {
  if (value === undefined || value === null) return undefined
  const o = obj(value, 'robot_state.controls')
  const nodes = o.reset_nodes === undefined ? [] : arr(o.reset_nodes, 'robot_state.controls.reset_nodes').map((n, i) => str(n, `robot_state.controls.reset_nodes[${i}]`))
  return { reset: o.reset === true, reset_nodes: nodes, home: o.home === true, reload_map: o.reload_map === true }
}

/** 없어도 되는 추가 필드. 모양이 틀리면 그 필드만 버리고 나머지(Contract)는 그대로 받는다. */
function parseRobotExtras(o: Obj): RobotExtras {
  const out: RobotExtras = {}
  if (typeof o.localization === 'string') out.localization = o.localization
  if (typeof o.map_current === 'boolean') out.map_current = o.map_current
  else if (o.map_current === null) out.map_current = null
  if (typeof o.map_image_current === 'boolean') out.map_image_current = o.map_image_current
  else if (o.map_image_current === null) out.map_image_current = null
  try {
    const controls = parseControls(o.controls)
    if (controls) out.controls = controls
  } catch {
    // 버튼 규칙을 읽지 못하면 버튼을 숨길 뿐, 로봇 상태 표시는 막지 않는다
  }
  return out
}

/** /ws/state 메시지 파싱. 알 수 없는 type 은 null (무시), Contract 위반은 ContractError. */
export function parseStateEvent(raw: unknown): StateSocketEvent | null {
  const o = obj(typeof raw === 'string' ? JSON.parse(raw) : raw, 'event')
  if (o.type === 'robot_state') {
    if (!isRobotState(o.state)) throw new ContractError(`robot_state.state 알 수 없는 상태: ${String(o.state)}`)
    const battery = num(o.battery_percent, 'robot_state.battery_percent')
    return {
      type: 'robot_state',
      robot_id: str(o.robot_id, 'robot_state.robot_id'),
      state: o.state,
      task_id: optStr(o.task_id, 'robot_state.task_id'),
      destination_id: optStr(o.destination_id, 'robot_state.destination_id'),
      battery_percent: Math.max(0, Math.min(100, battery)),
      pose: parsePose(o.pose, 'robot_state.pose'),
      ...parseRobotExtras(o),
    }
  }
  if (o.type === 'task_state') {
    if (!isTaskStatus(o.status)) throw new ContractError(`task_state.status 알 수 없는 상태: ${String(o.status)}`)
    return {
      type: 'task_state',
      task_id: str(o.task_id, 'task_state.task_id'),
      status: o.status,
      robot_id: optStr(o.robot_id, 'task_state.robot_id'),
      destination_id: str(o.destination_id, 'task_state.destination_id'),
      updated_at: typeof o.updated_at === 'string' ? o.updated_at : new Date().toISOString(),
    }
  }
  return null
}


// ---------- 지도 반영 ----------

function parseBackup(value: unknown, where = 'backup'): MapBackup {
  const o = obj(value, where)
  const kind = str(o.kind, `${where}.kind`)
  if (kind !== 'promote' && kind !== 'before_restore') throw new ContractError(`${where}.kind 알 수 없는 값: ${kind}`)
  let summary: MapBackup['summary'] = null
  if (o.summary !== undefined && o.summary !== null) {
    const s = obj(o.summary, `${where}.summary`)
    summary = { nodes: num(s.nodes, `${where}.summary.nodes`), edges: num(s.edges, `${where}.summary.edges`), tables: num(s.tables, `${where}.summary.tables`) }
  }
  return { id: str(o.id, `${where}.id`), created_at: str(o.created_at, `${where}.created_at`), kind,
    note: typeof o.note === 'string' ? o.note : '',
    summary,
    has_semantics: typeof o.has_semantics === 'boolean' ? o.has_semantics : null,
  }
}

export function parseBackups(value: unknown): MapBackup[] {
  return arr(value, 'backups').map((b, i) => parseBackup(b, `backups[${i}]`))
}

function parseLast(value: unknown): LiveStatus['last'] {
  if (value === null || value === undefined) return null
  const o = obj(value, 'live.last')
  const kind = str(o.kind, 'live.last.kind')
  if (kind !== 'promote' && kind !== 'restore') throw new ContractError(`live.last.kind 알 수 없는 값: ${kind}`)
  return {
    at: str(o.at, 'live.last.at'),
    kind,
    backup_id: str(o.backup_id, 'live.last.backup_id'),
    note: typeof o.note === 'string' ? o.note : '',
    needs_robot_restart: o.needs_robot_restart === true,
  }
}

export function parseLiveStatus(value: unknown): LiveStatus {
  const o = obj(value, 'live')
  return { matches_applied: o.matches_applied === true, backup_count: num(o.backup_count ?? 0, 'live.backup_count'), last: parseLast(o.last) }
}

export function parseLiveChange(value: unknown): LiveChangeResult {
  const o = obj(value, 'result')
  const last = parseLast(o.live)
  if (!last) throw new ContractError('result.live: 값이 없습니다.')
  const hasDoc = o.semantics !== undefined && o.semantics !== null
  return {
    backup: parseBackup(o.backup, 'result.backup'),
    live: last,
    semantics: hasDoc ? parseSemanticMap(o.semantics) : null,
    semantics_restored: o.semantics_restored === true && hasDoc,
    semantics_note: typeof o.semantics_note === 'string' ? o.semantics_note : null,
  }
}

export function parseSemanticStatus(value: unknown): SemanticStatus {
  const o = obj(value, 'status')
  let draft: SemanticStatus['draft'] = null
  if (o.draft) {
    const d = obj(o.draft, 'status.draft')
    draft = {
      saved_at: typeof d.saved_at === 'string' ? d.saved_at : '',
      sha: str(d.sha, 'status.draft.sha'),
      map_revision: typeof d.map_revision === 'number' ? d.map_revision : null,
    }
  }
  let applied: SemanticStatus['applied'] = null
  if (o.applied) {
    const a = obj(o.applied, 'status.applied')
    const sm = a.summary ? obj(a.summary, 'status.applied.summary') : {}
    const strings = (v: unknown) => (Array.isArray(v) ? v.filter((x): x is string => typeof x === 'string') : [])
    applied = {
      applied_at: typeof a.applied_at === 'string' ? a.applied_at : '',
      source_sha: typeof a.source_sha === 'string' ? a.source_sha : '',
      summary: { OK: Number(sm.OK ?? 0), WARN: Number(sm.WARN ?? 0), FAIL: Number(sm.FAIL ?? 0) },
      notes: strings(a.notes),
      warnings: strings(a.warnings),
    }
  }
  return { draft, applied, in_sync: o.in_sync === true }
}

// ---------- 자동 생성 통로 미리보기 ----------

const CHECK_LEVELS: readonly string[] = ['OK', 'WARN', 'FAIL']
const STOP_KINDS: readonly string[] = ['table_service', 'pickup', 'standby']

function checkLevel(value: unknown, where: string): CheckLevel {
  if (typeof value === 'string' && CHECK_LEVELS.includes(value)) return value as CheckLevel
  throw new ContractError(`${where}: OK/WARN/FAIL 이 아닙니다.`)
}
function strings(value: unknown): string[] {
  return Array.isArray(value) ? value.filter((x): x is string => typeof x === 'string') : []
}
function optNum(value: unknown): number | null {
  return typeof value === 'number' && Number.isFinite(value) ? value : null
}

export function parseGeneratedGraph(value: unknown): GeneratedGraph {
  const o = obj(value, 'graph')
  const nodes = arr(o.nodes, 'graph.nodes').map((n, i): GraphNode => {
    const w = `graph.nodes[${i}]`
    const x = obj(n, w)
    return { id: str(x.id, `${w}.id`), x: num(x.x, `${w}.x`), y: num(x.y, `${w}.y`), level: checkLevel(x.level, `${w}.level`) }
  })
  const nodeIds = new Set(nodes.map((n) => n.id))
  const edges = arr(o.edges, 'graph.edges').map((e, i): GraphEdge => {
    const w = `graph.edges[${i}]`
    const x = obj(e, w)
    const from = str(x.from, `${w}.from`)
    const to = str(x.to, `${w}.to`)
    if (!nodeIds.has(from) || !nodeIds.has(to)) throw new ContractError(`${w}: 없는 노드를 가리킵니다.`)
    return { id: str(x.id, `${w}.id`), from, to, length_m: num(x.length_m, `${w}.length_m`), level: checkLevel(x.level, `${w}.level`) }
  })
  const stops = arr(o.stops, 'graph.stops').map((s, i): GraphStop => {
    const w = `graph.stops[${i}]`
    const x = obj(s, w)
    const kind = str(x.kind, `${w}.kind`)
    if (!STOP_KINDS.includes(kind)) throw new ContractError(`${w}.kind: 알 수 없는 종류입니다.`)
    return {
      id: str(x.id, `${w}.id`),
      entity_id: optStr(x.entity_id, `${w}.entity_id`),
      node: str(x.node, `${w}.node`),
      x: num(x.x, `${w}.x`),
      y: num(x.y, `${w}.y`),
      yaw: num(x.yaw, `${w}.yaw`),
      yaw_deg: num(x.yaw_deg, `${w}.yaw_deg`),
      kind: kind as GraphStop['kind'],
      level: checkLevel(x.level, `${w}.level`),
      pinned: x.pinned === true,
    }
  })
  const excluded = arr(o.excluded ?? [], 'graph.excluded').map((e, i): GraphExcluded => {
    const w = `graph.excluded[${i}]`
    const x = obj(e, w)
    const reason = typeof x.reason_ko === 'string' ? x.reason_ko : typeof x.reason === 'string' ? x.reason : ''
    const pin = x.pin ? obj(x.pin, `${w}.pin`) : null
    return {
      id: str(x.id, `${w}.id`),
      entity_id: optStr(x.entity_id, `${w}.entity_id`),
      reason,
      x: optNum(x.x),
      y: optNum(x.y),
      pin: pin ? { x: num(pin.x, `${w}.pin.x`), y: num(pin.y, `${w}.pin.y`), yaw_deg: optNum(pin.yaw_deg) } : null,
    }
  })
  const sm = o.summary ? obj(o.summary, 'graph.summary') : {}
  return {
    source: o.source === 'config' ? 'config' : 'draft',
    draft_sha: typeof o.draft_sha === 'string' ? o.draft_sha : '',
    matches_applied: o.matches_applied === true,
    ok: o.ok === true,
    nodes,
    edges,
    stops,
    excluded,
    problems: strings(o.problems),
    warnings: strings(o.warnings),
    notes: strings(o.notes),
    summary: { OK: Number(sm.OK ?? 0), WARN: Number(sm.WARN ?? 0), FAIL: Number(sm.FAIL ?? 0) },
  }
}
