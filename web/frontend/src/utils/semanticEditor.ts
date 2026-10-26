/**
 * Semantic Map 편집기 상태 (순수 reducer, UI 와 분리되어 테스트 가능)
 * 모든 좌표는 ROS map frame (m, rad)
 */
import type { EntityType, MapPoint, Pose, SemanticMap, ZoneType } from '../types/contracts'
import { normalizeYaw } from './coordinates'
import { isManualWaypoint, markManual, snapAxisYaw } from './manualStop'
import {
  defaultLabel,
  defaultWaypointPose,
  entityDisplayPose,
  nextEntityId,
  nextZoneId,
  toSemanticPayload,
  waypointIdFor,
} from './semantic'

export type AddEntityTool = `add_${EntityType}`
export type ZoneTool = `zone_${ZoneType}`
export type EditorTool = 'select' | 'waypoint' | AddEntityTool | ZoneTool

export const toolEntityType = (tool: EditorTool): EntityType | null =>
  tool.startsWith('add_') ? (tool.slice(4) as EntityType) : null
export const toolZoneType = (tool: EditorTool): ZoneType | null =>
  tool.startsWith('zone_') ? (tool.slice(5) as ZoneType) : null

export type Selection =
  | { kind: 'entity'; id: string }
  | { kind: 'waypoint'; id: string }
  | { kind: 'zone'; id: string; vertex?: number }
  | null

export interface EditorState {
  doc: SemanticMap
  /** 마지막으로 로드/저장한 문서 (변경 여부 판단용, 직렬화 문자열) */
  baseline: string
  selection: Selection
  tool: EditorTool
  /** 작성 중인 Zone 꼭짓점 */
  zoneDraft: MapPoint[]
  past: SemanticMap[]
  /** 사용자에게 보여줄 마지막 편집 안내/오류 */
  notice: string | null
}

export type EditorAction =
  | { type: 'load'; doc: SemanticMap }
  | { type: 'mark_saved'; doc: SemanticMap }
  | { type: 'set_tool'; tool: EditorTool }
  | { type: 'select'; selection: Selection }
  | { type: 'checkpoint' }
  | { type: 'undo' }
  | { type: 'add_entity'; entityType: EntityType; x: number; y: number }
  | { type: 'move_entity'; id: string; x: number; y: number; record?: boolean }
  | { type: 'set_label'; id: string; label: string }
  | { type: 'set_top_size'; id: string; x?: number; y?: number }
  | { type: 'set_waypoint'; entityId: string; pose: Pose; record?: boolean }
  | { type: 'move_waypoint'; id: string; x: number; y: number; record?: boolean }
  | { type: 'set_waypoint_yaw'; id: string; yaw: number; record?: boolean }
  /** 자동 정차 위치를 손으로 옮김/방향 지정: 그 객체의 waypoint 를 수동(manual)으로 만든다 (방향은 동·서·남·북으로 맞춤) */
  | { type: 'set_manual_stop'; entityId: string; x: number; y: number; yaw: number; record?: boolean }
  /** 수동 지정을 지우고 waypoint 를 기본 위치로 되돌림 (정차 위치는 다시 생성기가 정함) */
  | { type: 'clear_manual_stop'; entityId: string }
  | { type: 'zone_add_point'; point: MapPoint }
  | { type: 'zone_undo_point' }
  | { type: 'zone_finish' }
  | { type: 'zone_cancel' }
  | { type: 'move_zone_vertex'; id: string; index: number; point: MapPoint; record?: boolean }
  | { type: 'delete_zone_vertex'; id: string; index: number }
  | { type: 'insert_zone_vertex'; id: string; afterIndex: number; point: MapPoint }
  | { type: 'translate_zone'; id: string; dx: number; dy: number; record?: boolean }
  | { type: 'set_zone_type'; id: string; zoneType: ZoneType }
  | { type: 'delete_selection' }
  | { type: 'rebase_revision'; revision: number }

const UNDO_LIMIT = 50

export const serializeDoc = (doc: SemanticMap) => JSON.stringify(toSemanticPayload(doc))

export function createEditorState(doc: SemanticMap): EditorState {
  return { doc, baseline: serializeDoc(doc), selection: null, tool: 'select', zoneDraft: [], past: [], notice: null }
}

export const isDirty = (s: EditorState) => serializeDoc(s.doc) !== s.baseline

/** 문서 변경 + undo 기록 */
function commit(s: EditorState, doc: SemanticMap, record = true, patch: Partial<EditorState> = {}): EditorState {
  const past = record ? [...s.past, s.doc].slice(-UNDO_LIMIT) : s.past
  return { ...s, ...patch, doc, past, notice: patch.notice ?? null }
}

export function editorReducer(s: EditorState, a: EditorAction): EditorState {
  const doc = s.doc
  switch (a.type) {
    case 'load':
      return createEditorState(a.doc)

    case 'mark_saved':
      return { ...s, doc: a.doc, baseline: serializeDoc(a.doc), notice: null }

    case 'set_tool':
      return { ...s, tool: a.tool, zoneDraft: [], notice: null }

    case 'select':
      return { ...s, selection: a.selection, notice: null }

    case 'checkpoint':
      return { ...s, past: [...s.past, doc].slice(-UNDO_LIMIT) }

    case 'undo': {
      if (s.zoneDraft.length) return { ...s, zoneDraft: s.zoneDraft.slice(0, -1) }
      // 선택만 하고 변경하지 않은 checkpoint(같은 문서 참조)는 건너뜀
      let past = s.past
      while (past.length && past[past.length - 1] === doc) past = past.slice(0, -1)
      const prev = past.at(-1)
      if (!prev) return { ...s, past }
      return { ...s, doc: prev, past: past.slice(0, -1), selection: validSelection(prev, s.selection), notice: null }
    }

    case 'add_entity': {
      const id = nextEntityId(doc, a.entityType)
      const pose: Pose = { x: a.x, y: a.y, yaw: 0 }
      const wpPose = defaultWaypointPose(a.entityType, pose)
      return commit(
        s,
        {
          ...doc,
          entities: [...doc.entities, { id, type: a.entityType, label: defaultLabel(a.entityType, id), pose }],
          waypoints: [...doc.waypoints, { id: waypointIdFor(id), entity_id: id, pose: wpPose }],
        },
        true,
        { selection: { kind: 'entity', id } },
      )
    }

    case 'move_entity': {
      const entity = doc.entities.find((e) => e.id === a.id)
      if (!entity) return s
      const from = entity.pose ?? doc.waypoints.find((w) => w.entity_id === a.id)?.pose
      const dx = from ? a.x - from.x : 0
      const dy = from ? a.y - from.y : 0
      return commit(s, {
        ...doc,
        entities: doc.entities.map((e) => (e.id === a.id ? { ...e, pose: { x: a.x, y: a.y, yaw: e.pose?.yaw ?? 0 } } : e)),
        // 엔티티를 옮기면 접근 위치(Waypoint)도 상대 위치를 유지하며 함께 이동
        waypoints: doc.waypoints.map((w) =>
          w.entity_id === a.id ? { ...w, pose: { ...w.pose, x: w.pose.x + dx, y: w.pose.y + dy } } : w,
        ),
      }, a.record)
    }

    case 'set_label':
      return commit(s, { ...doc, entities: doc.entities.map((e) => (e.id === a.id ? { ...e, label: a.label } : e)) })

    case 'set_top_size': {
      const entity = doc.entities.find((e) => e.id === a.id)
      if (!entity || entity.type !== 'table') return s
      // 한 칸만 입력된 상태(예: 가로만)도 허용: 비어 있는 쪽은 0 으로 두어 검증에서 막힌다.
      const cur = entity.top_size_m ?? { x: 0, y: 0 }
      const top = { x: a.x ?? cur.x, y: a.y ?? cur.y }
      return commit(s, { ...doc, entities: doc.entities.map((e) => (e.id === a.id ? { ...e, top_size_m: top } : e)) })
    }

    case 'set_waypoint': {
      if (!doc.entities.some((e) => e.id === a.entityId)) return { ...s, notice: '존재하지 않는 객체입니다.' }
      const pose = { ...a.pose, yaw: normalizeYaw(a.pose.yaw) }
      const existing = doc.waypoints.find((w) => w.entity_id === a.entityId)
      const waypoints = existing
        ? doc.waypoints.map((w) => (w.id === existing.id ? { ...w, pose } : w))
        : [...doc.waypoints, { id: waypointIdFor(a.entityId), entity_id: a.entityId, pose }]
      const id = existing?.id ?? waypointIdFor(a.entityId)
      return commit(s, { ...doc, waypoints }, a.record, { selection: { kind: 'waypoint', id } })
    }

    case 'move_waypoint':
      return commit(
        s,
        { ...doc, waypoints: doc.waypoints.map((w) => (w.id === a.id ? { ...w, pose: { ...w.pose, x: a.x, y: a.y } } : w)) },
        a.record,
      )

    case 'set_waypoint_yaw':
      return commit(
        s,
        {
          ...doc,
          waypoints: doc.waypoints.map((w) => (w.id === a.id ? { ...w, pose: { ...w.pose, yaw: normalizeYaw(a.yaw) } } : w)),
        },
        a.record,
      )

    case 'set_manual_stop': {
      if (!doc.entities.some((e) => e.id === a.entityId)) return { ...s, notice: '존재하지 않는 객체입니다.' }
      const pose: Pose = { x: a.x, y: a.y, yaw: snapAxisYaw(a.yaw) }
      const existing = doc.waypoints.find((w) => w.entity_id === a.entityId)
      const waypoints = existing
        ? doc.waypoints.map((w) => (w.id === existing.id ? markManual({ ...w, pose }, true) : w))
        : [...doc.waypoints, markManual({ id: waypointIdFor(a.entityId), entity_id: a.entityId, pose }, true)]
      return commit(s, { ...doc, waypoints }, a.record)
    }

    case 'clear_manual_stop': {
      const entity = doc.entities.find((e) => e.id === a.entityId)
      const existing = doc.waypoints.find((w) => w.entity_id === a.entityId)
      if (!entity || !existing || !isManualWaypoint(existing)) return s
      const at = entityDisplayPose(doc, entity)
      const pose = at ? defaultWaypointPose(entity.type, at) : existing.pose
      return commit(s, { ...doc, waypoints: doc.waypoints.map((w) => (w.id === existing.id ? markManual({ ...w, pose }, false) : w)) })
    }

    case 'zone_add_point':
      if (!toolZoneType(s.tool)) return s
      return { ...s, zoneDraft: [...s.zoneDraft, a.point], notice: null }

    case 'zone_undo_point':
      return { ...s, zoneDraft: s.zoneDraft.slice(0, -1) }

    case 'zone_cancel':
      return { ...s, zoneDraft: [], notice: null }

    case 'zone_finish': {
      const zoneType = toolZoneType(s.tool)
      if (!zoneType) return s
      if (s.zoneDraft.length < 3) return { ...s, notice: '구역은 꼭짓점이 3개 이상이어야 합니다.' }
      const id = nextZoneId(doc, zoneType)
      return commit(s, { ...doc, zones: [...doc.zones, { id, type: zoneType, polygon: s.zoneDraft }] }, true, {
        zoneDraft: [],
        selection: { kind: 'zone', id },
      })
    }

    case 'move_zone_vertex':
      return commit(
        s,
        {
          ...doc,
          zones: doc.zones.map((z) =>
            z.id === a.id ? { ...z, polygon: z.polygon.map((p, i) => (i === a.index ? a.point : p)) } : z,
          ),
        },
        a.record,
      )

    case 'delete_zone_vertex': {
      const zone = doc.zones.find((z) => z.id === a.id)
      if (!zone) return s
      if (zone.polygon.length <= 3) return { ...s, notice: '꼭짓점이 3개 미만이 되므로 삭제할 수 없습니다.' }
      return commit(
        s,
        { ...doc, zones: doc.zones.map((z) => (z.id === a.id ? { ...z, polygon: z.polygon.filter((_, i) => i !== a.index) } : z)) },
        true,
        { selection: { kind: 'zone', id: a.id } },
      )
    }

    case 'insert_zone_vertex':
      return commit(
        s,
        {
          ...doc,
          zones: doc.zones.map((z) =>
            z.id === a.id
              ? { ...z, polygon: [...z.polygon.slice(0, a.afterIndex + 1), a.point, ...z.polygon.slice(a.afterIndex + 1)] }
              : z,
          ),
        },
        true,
        { selection: { kind: 'zone', id: a.id, vertex: a.afterIndex + 1 } },
      )

    case 'translate_zone':
      return commit(
        s,
        {
          ...doc,
          zones: doc.zones.map((z) =>
            z.id === a.id ? { ...z, polygon: z.polygon.map(([x, y]): MapPoint => [x + a.dx, y + a.dy]) } : z,
          ),
        },
        a.record,
      )

    case 'set_zone_type':
      // ID 는 시스템 생성값이므로 타입을 바꿔도 유지
      return commit(s, { ...doc, zones: doc.zones.map((z) => (z.id === a.id ? { ...z, type: a.zoneType } : z)) })

    case 'delete_selection': {
      const sel = s.selection
      if (!sel) return s
      if (sel.kind === 'entity') {
        return commit(
          s,
          {
            ...doc,
            entities: doc.entities.filter((e) => e.id !== sel.id),
            // 참조 무결성: 엔티티를 지우면 그 Waypoint 도 삭제
            waypoints: doc.waypoints.filter((w) => w.entity_id !== sel.id),
          },
          true,
          { selection: null },
        )
      }
      if (sel.kind === 'waypoint') {
        return commit(s, { ...doc, waypoints: doc.waypoints.filter((w) => w.id !== sel.id) }, true, { selection: null })
      }
      if (sel.vertex !== undefined) return editorReducer(s, { type: 'delete_zone_vertex', id: sel.id, index: sel.vertex })
      return commit(s, { ...doc, zones: doc.zones.filter((z) => z.id !== sel.id) }, true, { selection: null })
    }

    case 'rebase_revision':
      return commit(s, { ...doc, map_revision: a.revision })
  }
}

function validSelection(doc: SemanticMap, sel: Selection): Selection {
  if (!sel) return null
  const exists =
    sel.kind === 'entity'
      ? doc.entities.some((e) => e.id === sel.id)
      : sel.kind === 'waypoint'
        ? doc.waypoints.some((w) => w.id === sel.id)
        : doc.zones.some((z) => z.id === sel.id)
  return exists ? sel : null
}
