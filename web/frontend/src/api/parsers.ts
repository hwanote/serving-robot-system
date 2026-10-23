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
import type { Task, TaskStateEvent } from '../types/provisional'

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

export type StateSocketEvent = RobotStateEvent | TaskStateEvent

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
