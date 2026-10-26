import { describe, expect, it } from 'vitest'
import type { SemanticMap } from '../types/contracts'
import { AXIS_DIRECTIONS, isManualWaypoint, markManual, snapAxisYaw } from './manualStop'
import { toSemanticPayload } from './semantic'
import { createEditorState, editorReducer, serializeDoc } from './semanticEditor'

const doc = (): SemanticMap => ({
  store_id: 's',
  map_revision: 1,
  entities: [
    { id: 'table_01', type: 'table', label: 'T1', pose: { x: 2, y: 2, yaw: 0 }, top_size_m: { x: 0.8, y: 0.8 } },
    { id: 'kitchen_01', type: 'kitchen', label: 'K', pose: { x: 1, y: 3, yaw: 0 } },
  ],
  waypoints: [{ id: 'wp_table_01', entity_id: 'table_01', pose: { x: 2, y: 1.4, yaw: Math.PI / 2 } }],
  zones: [],
})

describe('방향 맞추기', () => {
  it('가장 가까운 동·북·서·남으로 맞춘다', () => {
    expect(snapAxisYaw(0.2)).toBe(0)
    expect(snapAxisYaw(1.4)).toBeCloseTo(Math.PI / 2)
    expect(snapAxisYaw(-1.5)).toBeCloseTo(-Math.PI / 2)
    expect(snapAxisYaw(3.0)).toBeCloseTo(Math.PI)
    expect(snapAxisYaw(-Math.PI)).toBeCloseTo(Math.PI)
    expect(Object.is(snapAxisYaw(-0.1), -0)).toBe(false)
  })
  it('방향 버튼은 동·서·남·북 네 개', () => {
    expect(AXIS_DIRECTIONS.map((d) => d.name)).toEqual(['동', '서', '남', '북'])
  })
})

describe('수동 waypoint 표식', () => {
  it('켜고 끌 수 있고 원본은 바뀌지 않는다', () => {
    const w = doc().waypoints[0]
    const m = markManual(w, true)
    expect(isManualWaypoint(m)).toBe(true)
    expect(isManualWaypoint(w)).toBe(false)
    expect('manual' in markManual(m, false)).toBe(false)
  })
  it('저장용 정리(toSemanticPayload)를 거쳐도 남는다. 없는 waypoint 에는 붙지 않는다', () => {
    const d = doc()
    d.waypoints = [markManual(d.waypoints[0], true), { id: 'wp_k', entity_id: 'kitchen_01', pose: { x: 1, y: 2.4, yaw: 1 } }]
    const out = toSemanticPayload(d)
    expect(isManualWaypoint(out.waypoints[0])).toBe(true)
    expect('manual' in out.waypoints[1]).toBe(false)
  })
})

describe('편집기: 정차 위치 직접 지정', () => {
  it('waypoint 를 수동으로 만들고 방향을 동·서·남·북으로 맞춘다 (undo 한 번에 되돌아간다)', () => {
    const s0 = createEditorState(doc())
    const s1 = editorReducer(s0, { type: 'set_manual_stop', entityId: 'table_01', x: 2.3, y: 0.9, yaw: 1.4 })
    const w = s1.doc.waypoints[0]
    expect(w.pose.x).toBe(2.3)
    expect(w.pose.y).toBe(0.9)
    expect(w.pose.yaw).toBeCloseTo(Math.PI / 2)
    expect(isManualWaypoint(w)).toBe(true)
    expect(serializeDoc(s1.doc)).not.toBe(serializeDoc(s0.doc)) // 저장 안 됨(dirty)으로 잡힌다
    const back = editorReducer(s1, { type: 'undo' })
    expect(back.doc.waypoints[0]).toEqual(s0.doc.waypoints[0])
  })

  it('waypoint 가 없는 객체도 새로 만든다', () => {
    const s1 = editorReducer(createEditorState(doc()), { type: 'set_manual_stop', entityId: 'kitchen_01', x: 1, y: 2.4, yaw: Math.PI / 2 })
    const w = s1.doc.waypoints.find((x) => x.entity_id === 'kitchen_01')!
    expect(w.id).toBe('wp_kitchen_01')
    expect(isManualWaypoint(w)).toBe(true)
  })

  it('없는 객체는 안내만 하고 바꾸지 않는다', () => {
    const s0 = createEditorState(doc())
    const s1 = editorReducer(s0, { type: 'set_manual_stop', entityId: 'nope', x: 0, y: 0, yaw: 0 })
    expect(s1.doc).toBe(s0.doc)
    expect(s1.notice).toBeTruthy()
  })

  it('되돌리기: 수동 표식을 지우고 waypoint 를 기본 위치(객체 앞 0.6 m)로 돌린다', () => {
    const s1 = editorReducer(createEditorState(doc()), { type: 'set_manual_stop', entityId: 'table_01', x: 2.3, y: 0.9, yaw: 0 })
    const s2 = editorReducer(s1, { type: 'clear_manual_stop', entityId: 'table_01' })
    const w = s2.doc.waypoints[0]
    expect(isManualWaypoint(w)).toBe(false)
    expect(w.pose).toEqual({ x: 2, y: 1.4, yaw: Math.PI / 2 })
  })

  it('수동이 아닌 waypoint 의 되돌리기는 아무 일도 하지 않는다', () => {
    const s0 = createEditorState(doc())
    expect(editorReducer(s0, { type: 'clear_manual_stop', entityId: 'table_01' })).toBe(s0)
  })
})
