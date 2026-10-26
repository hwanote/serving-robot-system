import type {
  EntityType,
  MapPoint,
  Pose,
  RawMapInfo,
  SemanticEntity,
  SemanticMap,
  Waypoint,
  Zone,
  ZoneType,
} from '../types/contracts'
import { isInsideMap, normalizeYaw, roundTo, type MapGeometry } from './coordinates'
import { isManualWaypoint, markManual } from './manualStop'

// ---------- ID / label ----------

const pad2 = (n: number) => String(n).padStart(2, '0')

// ---------- 테이블 상단 크기 ----------

/** 테이블 상단 한 변의 허용 최댓값 (m). Backend 검증(0 초과 10 m 이하)과 같다. */
export const TOP_SIZE_MAX_M = 10

const validSide = (v: number) => Number.isFinite(v) && roundTo(v, 3) > 0 && v <= TOP_SIZE_MAX_M

export const isValidTopSize = (t: { x: number; y: number }) => validSide(t.x) && validSide(t.y)

export const mmToM = (mm: number) => roundTo(mm / 1000, 3)
export const mToMm = (m: number) => Math.round(m * 1000)

function maxSuffix(ids: Iterable<string>, prefix: string): number {
  const re = new RegExp(`^${prefix}_(\\d+)$`)
  let max = 0
  for (const id of ids) {
    const m = re.exec(id)
    if (m) max = Math.max(max, Number(m[1]))
  }
  return max
}

function allIds(doc: Pick<SemanticMap, 'entities' | 'waypoints' | 'zones'>): Set<string> {
  return new Set([...doc.entities.map((e) => e.id), ...doc.waypoints.map((w) => w.id), ...doc.zones.map((z) => z.id)])
}

/** table → table_01, table_02 ... 기존(로드된) ID와 겹치지 않도록 최대 번호 + 1 */
export function nextEntityId(doc: Pick<SemanticMap, 'entities' | 'waypoints' | 'zones'>, type: EntityType): string {
  const taken = allIds(doc)
  let n = maxSuffix(taken, type) + 1
  while (taken.has(`${type}_${pad2(n)}`) || taken.has(waypointIdFor(`${type}_${pad2(n)}`))) n++
  return `${type}_${pad2(n)}`
}

export function nextZoneId(doc: Pick<SemanticMap, 'entities' | 'waypoints' | 'zones'>, type: ZoneType): string {
  const taken = allIds(doc)
  const prefix = `zone_${type}`
  let n = maxSuffix(taken, prefix) + 1
  while (taken.has(`${prefix}_${pad2(n)}`)) n++
  return `${prefix}_${pad2(n)}`
}

export function waypointIdFor(entityId: string): string {
  return `wp_${entityId}`
}

const LABEL_BASE: Record<EntityType, string> = {
  table: 'Table',
  counter: 'Counter',
  kitchen: 'Kitchen',
  robot_wait: 'Robot Wait',
}

/** table_03 → "Table 3" */
export function defaultLabel(type: EntityType, id: string): string {
  const m = /_(\d+)$/.exec(id)
  return m ? `${LABEL_BASE[type]} ${Number(m[1])}` : LABEL_BASE[type]
}

export function createEmptySemanticMap(storeId: string, mapRevision: number): SemanticMap {
  return { store_id: storeId, map_revision: mapRevision, entities: [], waypoints: [], zones: [] }
}

/** 엔티티 위치에서 서빙 접근 위치(Waypoint)의 기본값. 테이블 중심이 아닌 앞쪽(-y) 0.6m, 테이블을 바라봄 */
export function defaultWaypointPose(type: EntityType, entityPose: Pose): Pose {
  if (type === 'robot_wait') return { ...entityPose }
  const distance = 0.6
  return { x: entityPose.x, y: entityPose.y - distance, yaw: Math.PI / 2 }
}

// ---------- 조회 ----------

export function waypointForEntity(doc: SemanticMap, entityId: string): Waypoint | undefined {
  return doc.waypoints.find((w) => w.entity_id === entityId)
}

/** 엔티티 표시 위치: pose(임시 확장 필드) → 없으면 Waypoint 위치 */
export function entityDisplayPose(doc: SemanticMap, entity: SemanticEntity): Pose | null {
  return entity.pose ?? waypointForEntity(doc, entity.id)?.pose ?? null
}

export interface Destination {
  id: string
  label: string
  type: EntityType
}

/** 서빙 목적지로 선택 가능한 엔티티: 테이블이면서 Waypoint 가 지정된 것 */
export function servableDestinations(doc: SemanticMap | null): Destination[] {
  if (!doc) return []
  const withWaypoint = new Set(doc.waypoints.map((w) => w.entity_id))
  return doc.entities
    .filter((e) => e.type === 'table' && withWaypoint.has(e.id))
    .map((e) => ({ id: e.id, label: e.label || e.id, type: e.type }))
    .sort((a, b) => a.id.localeCompare(b.id))
}

// ---------- 검증 ----------

export interface ValidationIssue {
  level: 'error' | 'warning'
  message: string
  /** 관련 객체 ID */
  ref?: string
}

const finitePose = (p: Pose) => [p.x, p.y, p.yaw].every(Number.isFinite)

export function validateSemanticMap(doc: SemanticMap, geo?: MapGeometry | null): ValidationIssue[] {
  const issues: ValidationIssue[] = []
  const seen = new Map<string, string>()
  const checkId = (id: string, kind: string) => {
    if (!id) issues.push({ level: 'error', message: `${kind} ID가 비어 있습니다.` })
    else if (seen.has(id)) issues.push({ level: 'error', message: `ID가 중복되었습니다: ${id}`, ref: id })
    else seen.set(id, kind)
  }

  for (const e of doc.entities) {
    checkId(e.id, 'Entity')
    if (!e.label.trim()) issues.push({ level: 'warning', message: `${e.id}: 표시 이름(label)이 비어 있습니다.`, ref: e.id })
    if (e.pose && !finitePose(e.pose)) issues.push({ level: 'error', message: `${e.id}: 좌표가 올바르지 않습니다.`, ref: e.id })
    if (e.type === 'table') {
      if (!e.top_size_m) {
        issues.push({ level: 'error', message: `${e.id}: 테이블 상단 크기(가로·세로 mm)를 입력하세요.`, ref: e.id })
      } else if (!isValidTopSize(e.top_size_m)) {
        issues.push({
          level: 'error',
          message: `${e.id}: 테이블 상단 크기는 가로·세로 모두 1 mm 이상 ${TOP_SIZE_MAX_M * 1000} mm 이하여야 합니다.`,
          ref: e.id,
        })
      }
    }
  }

  const entityIds = new Set(doc.entities.map((e) => e.id))
  const waypointCount = new Map<string, number>()
  for (const w of doc.waypoints) {
    checkId(w.id, 'Waypoint')
    if (!entityIds.has(w.entity_id)) {
      issues.push({ level: 'error', message: `${w.id}: 존재하지 않는 entity_id(${w.entity_id})를 참조합니다.`, ref: w.id })
    }
    waypointCount.set(w.entity_id, (waypointCount.get(w.entity_id) ?? 0) + 1)
    if (!finitePose(w.pose)) issues.push({ level: 'error', message: `${w.id}: 좌표가 올바르지 않습니다.`, ref: w.id })
    else if (geo && !isInsideMap(geo, w.pose.x, w.pose.y)) {
      issues.push({ level: 'warning', message: `${w.id}: 지도 범위 밖에 있습니다.`, ref: w.id })
    }
  }

  for (const e of doc.entities) {
    const count = waypointCount.get(e.id) ?? 0
    if (count === 0 && e.type !== 'robot_wait') {
      issues.push({ level: 'warning', message: `${e.id}: Waypoint가 없어 로봇 목적지로 사용할 수 없습니다.`, ref: e.id })
    }
    if (count === 0 && e.type === 'robot_wait') {
      issues.push({ level: 'warning', message: `${e.id}: 대기 위치 Waypoint가 없습니다.`, ref: e.id })
    }
    if (count > 1) issues.push({ level: 'warning', message: `${e.id}: Waypoint가 ${count}개입니다.`, ref: e.id })
  }

  for (const z of doc.zones) {
    checkId(z.id, 'Zone')
    if (z.polygon.length < 3) {
      issues.push({ level: 'error', message: `${z.id}: 꼭짓점이 3개 이상이어야 합니다. (현재 ${z.polygon.length}개)`, ref: z.id })
    }
    if (!z.polygon.every((p) => p.length === 2 && p.every(Number.isFinite))) {
      issues.push({ level: 'error', message: `${z.id}: 좌표가 올바르지 않습니다.`, ref: z.id })
    } else if (geo && z.polygon.some(([x, y]) => !isInsideMap(geo, x, y))) {
      issues.push({ level: 'warning', message: `${z.id}: 일부 꼭짓점이 지도 범위 밖에 있습니다.`, ref: z.id })
    }
  }

  return issues
}

export const hasErrors = (issues: ValidationIssue[]) => issues.some((i) => i.level === 'error')

export interface RevisionCheck {
  ok: boolean
  message: string | null
}

/** Semantic 문서가 현재 Raw Map revision 기준인지 확인 */
export function checkRevision(doc: Pick<SemanticMap, 'map_revision'>, rawMap: Pick<RawMapInfo, 'map_revision'> | null): RevisionCheck {
  if (!rawMap) return { ok: false, message: '저장된 지도가 없어 Semantic Map을 저장할 수 없습니다.' }
  if (doc.map_revision !== rawMap.map_revision) {
    return {
      ok: false,
      message: `지도 revision이 다릅니다. (Semantic r${doc.map_revision} / 현재 지도 r${rawMap.map_revision}) 객체 위치를 확인한 뒤 현재 지도 기준으로 갱신하세요.`,
    }
  }
  return { ok: true, message: null }
}

// ---------- 저장 직렬화 ----------

const roundPose = (p: Pose): Pose => ({ x: roundTo(p.x, 3), y: roundTo(p.y, 3), yaw: roundTo(normalizeYaw(p.yaw), 4) })

/** 저장용 정규화: Contract 필드만 남기고 좌표 정밀도를 맞춘다 (재로드 시 동일 결과 보장) */
export function toSemanticPayload(doc: SemanticMap): SemanticMap {
  return {
    store_id: doc.store_id,
    map_revision: doc.map_revision,
    entities: doc.entities.map((e): SemanticEntity => {
      const out: SemanticEntity = { id: e.id, type: e.type, label: e.label.trim() }
      if (e.pose) out.pose = roundPose(e.pose)
      if (e.top_size_m) out.top_size_m = { x: roundTo(e.top_size_m.x, 3), y: roundTo(e.top_size_m.y, 3) }
      if (e.chairs) out.chairs = e.chairs.map((c) => ({ dx: c.dx, dy: c.dy, size_m: c.size_m }))
      return out
    }),
    waypoints: doc.waypoints.map((w): Waypoint => markManual({ id: w.id, entity_id: w.entity_id, pose: roundPose(w.pose) }, isManualWaypoint(w))),
    zones: doc.zones.map(
      (z): Zone => ({
        id: z.id,
        type: z.type,
        polygon: z.polygon.map(([x, y]): MapPoint => [roundTo(x, 3), roundTo(y, 3)]),
      }),
    ),
  }
}
