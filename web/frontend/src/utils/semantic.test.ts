import { describe, expect, it } from 'vitest'
import { getSemantics, putSemantics } from '../api/semanticApi'
import { mockDb, seedMockDb } from '../mocks/db'
import { buildRawMap, buildSeedSemantics } from '../mocks/fixtures'
import { ContractError, parseSemanticMap } from '../api/parsers'
import type { SemanticMap } from '../types/contracts'
import {
  checkRevision,
  createEmptySemanticMap,
  hasErrors,
  isValidTopSize,
  mmToM,
  mToMm,
  nextEntityId,
  nextZoneId,
  servableDestinations,
  toSemanticPayload,
  validateSemanticMap,
} from './semantic'
import { createEditorState, editorReducer, isDirty, type EditorAction, type EditorState } from './semanticEditor'

const run = (s: EditorState, ...actions: EditorAction[]) => actions.reduce(editorReducer, s)

describe('Semantic Entity ID', () => {
  it('table_01, table_02, table_03 순서로 생성한다', () => {
    let s = createEditorState(createEmptySemanticMap('demo_store', 1))
    s = run(
      s,
      { type: 'add_entity', entityType: 'table', x: 1, y: 1 },
      { type: 'add_entity', entityType: 'table', x: 2, y: 1 },
      { type: 'add_entity', entityType: 'table', x: 3, y: 1 },
      { type: 'add_entity', entityType: 'counter', x: 3, y: 3 },
    )
    expect(s.doc.entities.map((e) => e.id)).toEqual(['table_01', 'table_02', 'table_03', 'counter_01'])
    expect(s.doc.entities.map((e) => e.label)).toEqual(['Table 1', 'Table 2', 'Table 3', 'Counter 1'])
  })

  it('기존 데이터를 로드한 경우 ID 가 중복되지 않는다 (중간 번호 삭제 포함)', () => {
    const doc: SemanticMap = {
      ...createEmptySemanticMap('demo_store', 1),
      entities: [
        { id: 'table_01', type: 'table', label: 'A' },
        { id: 'table_07', type: 'table', label: 'B' },
      ],
    }
    expect(nextEntityId(doc, 'table')).toBe('table_08')
    expect(nextEntityId(doc, 'kitchen')).toBe('kitchen_01')
  })

  it('label 을 수정해도 ID 는 변하지 않는다', () => {
    let s = createEditorState(createEmptySemanticMap('demo_store', 1))
    s = run(s, { type: 'add_entity', entityType: 'table', x: 1, y: 1 }, { type: 'set_label', id: 'table_01', label: '창가 자리' })
    expect(s.doc.entities[0]).toMatchObject({ id: 'table_01', label: '창가 자리' })
    expect(s.doc.waypoints[0].id).toBe('wp_table_01')
  })

  it('Zone ID 도 타입별로 중복 없이 생성한다', () => {
    const doc = buildSeedSemantics('demo_store', 1)
    expect(nextZoneId(doc, 'no_go')).toBe('zone_no_go_02')
  })

  it('중복 ID 는 검증 오류다', () => {
    const doc: SemanticMap = {
      ...createEmptySemanticMap('demo_store', 1),
      entities: [
        { id: 'table_01', type: 'table', label: 'A' },
        { id: 'table_01', type: 'table', label: 'B' },
      ],
    }
    expect(hasErrors(validateSemanticMap(doc))).toBe(true)
  })
})

describe('Waypoint', () => {
  it('Entity 추가 시 wp_<entity_id> 를 생성하고 테이블 중심이 아닌 위치에 둔다', () => {
    const s = run(createEditorState(createEmptySemanticMap('demo_store', 1)), { type: 'add_entity', entityType: 'table', x: 2, y: 1 })
    const wp = s.doc.waypoints[0]
    expect(wp).toMatchObject({ id: 'wp_table_01', entity_id: 'table_01' })
    expect(wp.pose.x !== 2 || wp.pose.y !== 1).toBe(true)
  })

  it('존재하지 않는 entity_id 참조는 검증 오류다', () => {
    const doc: SemanticMap = {
      ...createEmptySemanticMap('demo_store', 1),
      waypoints: [{ id: 'wp_table_09', entity_id: 'table_09', pose: { x: 0, y: 0, yaw: 0 } }],
    }
    const issues = validateSemanticMap(doc)
    expect(issues.some((i) => i.level === 'error' && i.ref === 'wp_table_09')).toBe(true)
  })

  it('Entity 를 삭제하면 참조하던 Waypoint 도 함께 삭제한다', () => {
    let s = run(createEditorState(createEmptySemanticMap('demo_store', 1)), { type: 'add_entity', entityType: 'table', x: 2, y: 1 })
    s = run(s, { type: 'select', selection: { kind: 'entity', id: 'table_01' } }, { type: 'delete_selection' })
    expect(s.doc.entities).toHaveLength(0)
    expect(s.doc.waypoints).toHaveLength(0)
    expect(hasErrors(validateSemanticMap(s.doc))).toBe(false)
  })

  it('Waypoint pose(x, y, yaw) 지정 및 yaw 정규화', () => {
    let s = run(createEditorState(createEmptySemanticMap('demo_store', 1)), { type: 'add_entity', entityType: 'table', x: 2, y: 1 })
    s = run(s, { type: 'set_waypoint', entityId: 'table_01', pose: { x: 2, y: 1.7, yaw: (3 * Math.PI) / 2 } })
    expect(s.doc.waypoints[0].pose.x).toBe(2)
    expect(s.doc.waypoints[0].pose.y).toBe(1.7)
    expect(s.doc.waypoints[0].pose.yaw).toBeCloseTo(-Math.PI / 2, 9)
  })

  it('서빙 목적지는 Waypoint 가 있는 테이블만', () => {
    const doc = buildSeedSemantics('demo_store', 1)
    doc.waypoints = doc.waypoints.filter((w) => w.entity_id !== 'table_02')
    expect(servableDestinations(doc).map((d) => d.id)).toEqual(['table_01', 'table_03'])
  })
})

describe('Zone Polygon', () => {
  it('꼭짓점 3개 미만이면 생성하지 않는다', () => {
    let s = run(createEditorState(createEmptySemanticMap('demo_store', 1)), { type: 'set_tool', tool: 'zone_no_go' })
    s = run(s, { type: 'zone_add_point', point: [0, 0] }, { type: 'zone_add_point', point: [1, 0] }, { type: 'zone_finish' })
    expect(s.doc.zones).toHaveLength(0)
    expect(s.notice).toMatch(/3개/)
  })

  it('map 좌표 그대로 polygon 에 저장한다', () => {
    let s = run(createEditorState(createEmptySemanticMap('demo_store', 1)), { type: 'set_tool', tool: 'zone_exclusive' })
    s = run(
      s,
      { type: 'zone_add_point', point: [-1.0, 0.5] },
      { type: 'zone_add_point', point: [0.2, 0.5] },
      { type: 'zone_add_point', point: [0.2, 2.0] },
      { type: 'zone_add_point', point: [-1.0, 2.0] },
      { type: 'zone_finish' },
    )
    expect(s.doc.zones).toEqual([
      {
        id: 'zone_exclusive_01',
        type: 'exclusive',
        polygon: [
          [-1.0, 0.5],
          [0.2, 0.5],
          [0.2, 2.0],
          [-1.0, 2.0],
        ],
      },
    ])
  })

  it('꼭짓점 삭제로 3개 미만이 되지 않게 막는다', () => {
    const doc = buildSeedSemantics('demo_store', 1)
    doc.zones[0].polygon = doc.zones[0].polygon.slice(0, 3)
    const s = run(createEditorState(doc), { type: 'delete_zone_vertex', id: doc.zones[0].id, index: 0 })
    expect(s.doc.zones[0].polygon).toHaveLength(3)
  })

  it('3개 미만 polygon 은 검증 오류', () => {
    const doc = createEmptySemanticMap('demo_store', 1)
    doc.zones = [{ id: 'zone_slow_01', type: 'slow', polygon: [[0, 0], [1, 1]] }]
    expect(hasErrors(validateSemanticMap(doc))).toBe(true)
  })
})

describe('편집 이력 / dirty', () => {
  it('되돌리기 및 저장 상태 판단', () => {
    let s = createEditorState(createEmptySemanticMap('demo_store', 1))
    expect(isDirty(s)).toBe(false)
    s = run(s, { type: 'add_entity', entityType: 'kitchen', x: 0, y: 0 })
    expect(isDirty(s)).toBe(true)
    s = run(s, { type: 'undo' })
    expect(s.doc.entities).toHaveLength(0)
    expect(isDirty(s)).toBe(false)
  })

  it('엔티티 이동 시 Waypoint 도 상대 위치를 유지하며 이동한다', () => {
    let s = run(createEditorState(createEmptySemanticMap('demo_store', 1)), { type: 'add_entity', entityType: 'table', x: 2, y: 1 })
    const before = s.doc.waypoints[0].pose
    s = run(s, { type: 'move_entity', id: 'table_01', x: 3, y: 1.5 })
    expect(s.doc.waypoints[0].pose.x).toBeCloseTo(before.x + 1, 9)
    expect(s.doc.waypoints[0].pose.y).toBeCloseTo(before.y + 0.5, 9)
  })
})

describe('Semantic 저장 후 복원 (Mock API)', () => {
  it('PUT 후 GET 하면 동일한 문서가 복원된다', async () => {
    seedMockDb()
    const revision = mockDb.rawMap!.map_revision
    let s = createEditorState(createEmptySemanticMap('demo_store', revision))
    s = run(
      s,
      { type: 'add_entity', entityType: 'table', x: 1.23456, y: -0.5 },
      { type: 'add_entity', entityType: 'robot_wait', x: 0.1, y: 0.2 },
      { type: 'set_tool', tool: 'zone_slow' },
      { type: 'zone_add_point', point: [0, 0] },
      { type: 'zone_add_point', point: [1, 0] },
      { type: 'zone_add_point', point: [1, 1] },
      { type: 'zone_finish' },
      { type: 'set_label', id: 'table_01', label: '창가' },
      { type: 'set_top_size', id: 'table_01', x: 0.9, y: 0.6 },
    )
    const payload = toSemanticPayload(s.doc)
    const saved = await putSemantics(payload)
    const loaded = await getSemantics('demo_store')

    expect(saved).toEqual(payload)
    expect(loaded).toEqual(payload)
    // 다시 편집기에 로드해도 변경 없음 (dirty=false)
    expect(isDirty(createEditorState(loaded!))).toBe(false)
    expect(loaded!.entities[0].pose!.x).toBe(1.235)
  })

  it('필수 구조(entities / waypoints / zones)를 유지한다', () => {
    const payload = toSemanticPayload(buildSeedSemantics('demo_store', 1))
    expect(Object.keys(payload).sort()).toEqual(['entities', 'map_revision', 'store_id', 'waypoints', 'zones'])
  })
})

describe('Map revision 불일치 방지', () => {
  it('checkRevision: revision 이 다르면 저장 불가', () => {
    expect(checkRevision({ map_revision: 1 }, buildRawMap('demo_store', 2)).ok).toBe(false)
    expect(checkRevision({ map_revision: 2 }, buildRawMap('demo_store', 2)).ok).toBe(true)
    expect(checkRevision({ map_revision: 1 }, null).ok).toBe(false)
  })

  it('Mock 서버도 revision 불일치 PUT 을 409 로 거부한다', async () => {
    seedMockDb()
    const stale = { ...buildSeedSemantics('demo_store', mockDb.rawMap!.map_revision - 1) }
    await expect(putSemantics(stale)).rejects.toMatchObject({ status: 409 })
  })

  it('편집기에서 현재 revision 으로 갱신(rebase) 후 저장 가능', () => {
    const s = run(createEditorState(buildSeedSemantics('demo_store', 1)), { type: 'rebase_revision', revision: 2 })
    expect(checkRevision(s.doc, buildRawMap('demo_store', 2)).ok).toBe(true)
  })
})

describe('테이블 상단 크기 (top_size_m)', () => {
  const withTable = () => run(createEditorState(createEmptySemanticMap('demo_store', 1)), { type: 'add_entity', entityType: 'table', x: 1, y: 1 })
  const topErrors = (s: EditorState) => validateSemanticMap(s.doc).filter((i) => i.level === 'error' && i.ref === 'table_01')

  it('새 테이블은 상단 크기가 비어 있고 저장 오류로 막힌다 (기본값 없음)', () => {
    const s = withTable()
    expect(s.doc.entities[0].top_size_m).toBeUndefined()
    expect(topErrors(s)).toHaveLength(1)
    expect(topErrors(s)[0].message).toContain('상단 크기')
  })

  it('가로·세로를 모두 입력하면 오류가 사라진다', () => {
    const s = run(withTable(), { type: 'set_top_size', id: 'table_01', x: 0.8, y: 0.8 })
    expect(topErrors(s)).toHaveLength(0)
    expect(hasErrors(validateSemanticMap(s.doc))).toBe(false)
  })

  it('한 변만 입력하면 아직 오류다', () => {
    const s = run(withTable(), { type: 'set_top_size', id: 'table_01', x: 0.8 })
    expect(s.doc.entities[0].top_size_m).toEqual({ x: 0.8, y: 0 })
    expect(topErrors(s)).toHaveLength(1)
  })

  it('범위를 벗어나면 오류다 (0 이하, 10 m 초과)', () => {
    expect(isValidTopSize({ x: 0.8, y: 0 })).toBe(false)
    expect(isValidTopSize({ x: -1, y: 0.8 })).toBe(false)
    expect(isValidTopSize({ x: 10.001, y: 0.8 })).toBe(false)
    expect(isValidTopSize({ x: 10, y: 0.001 })).toBe(true)
  })

  it('테이블이 아닌 객체에는 적용되지 않는다', () => {
    const s0 = run(createEditorState(createEmptySemanticMap('demo_store', 1)), { type: 'add_entity', entityType: 'kitchen', x: 0, y: 0 })
    const s = run(s0, { type: 'set_top_size', id: 'kitchen_01', x: 1, y: 1 })
    expect(s.doc.entities[0].top_size_m).toBeUndefined()
  })

  it('되돌리기와 변경 감지가 동작한다', () => {
    const s0 = withTable()
    const s1 = run(s0, { type: 'set_top_size', id: 'table_01', x: 0.8, y: 0.8 })
    expect(isDirty({ ...s1, baseline: s0.baseline })).toBe(true)
    const s2 = run(s1, { type: 'undo' })
    expect(s2.doc.entities[0].top_size_m).toBeUndefined()
  })

  it('mm ↔ m 변환은 1 mm 단위로 맞는다', () => {
    expect(mmToM(800)).toBe(0.8)
    expect(mmToM(1234)).toBe(1.234)
    expect(mToMm(0.8)).toBe(800)
    expect(mToMm(1.234)).toBe(1234)
  })

  it('저장 직렬화는 top_size_m 을 1 mm 로 맞추고 chairs 를 보존한다', () => {
    const doc: SemanticMap = {
      ...createEmptySemanticMap('demo_store', 1),
      entities: [
        {
          id: 'table_01',
          type: 'table',
          label: 'T',
          top_size_m: { x: 0.80004, y: 0.6 },
          chairs: [{ dx: -0.65, dy: 0, size_m: 0.45 }],
        },
      ],
    }
    const out = toSemanticPayload(doc).entities[0]
    expect(out.top_size_m).toEqual({ x: 0.8, y: 0.6 })
    expect(out.chairs).toEqual([{ dx: -0.65, dy: 0, size_m: 0.45 }])
  })

  it('서버 문서를 파싱 → 직렬화해도 top_size_m 과 chairs 가 유지된다 (예전에는 버려졌다)', () => {
    const raw = {
      store_id: 'demo_store',
      map_revision: 1,
      entities: [{ id: 'table_01', type: 'table', label: 'T', top_size_m: { x: 0.8, y: 0.8 }, chairs: [{ dx: 0.65, dy: 0, size_m: 0.45 }] }],
      waypoints: [],
      zones: [],
    }
    const parsed = parseSemanticMap(raw)
    expect(toSemanticPayload(parsed).entities[0]).toMatchObject({ top_size_m: { x: 0.8, y: 0.8 }, chairs: [{ dx: 0.65, dy: 0, size_m: 0.45 }] })
  })

  it('필드가 없는 문서도 그대로 파싱된다', () => {
    const parsed = parseSemanticMap({ store_id: 's', map_revision: 1, entities: [{ id: 'table_01', type: 'table', label: 'T' }], waypoints: [], zones: [] })
    expect(parsed.entities[0].top_size_m).toBeUndefined()
    expect(parsed.entities[0].chairs).toBeUndefined()
  })

  it('형식이 틀린 top_size_m / chairs 는 ContractError 다', () => {
    const base = { store_id: 's', map_revision: 1, waypoints: [], zones: [] }
    expect(() => parseSemanticMap({ ...base, entities: [{ id: 'table_01', type: 'table', label: 'T', top_size_m: { x: 'a', y: 1 } }] })).toThrow(ContractError)
    expect(() => parseSemanticMap({ ...base, entities: [{ id: 'table_01', type: 'table', label: 'T', chairs: [{ dx: 1 }] }] })).toThrow(ContractError)
  })
})
