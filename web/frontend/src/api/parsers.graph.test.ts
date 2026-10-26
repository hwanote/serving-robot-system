import { describe, expect, it } from 'vitest'
import { parseGeneratedGraph, parseSemanticMap } from './parsers'

const base = () => ({
  source: 'draft',
  draft_sha: 'abc',
  matches_applied: false,
  ok: true,
  nodes: [
    { id: 'n01', x: 1, y: 2, level: 'OK' },
    { id: 'n02', x: 3, y: 2, level: 'WARN' },
  ],
  edges: [{ id: 'e01', from: 'n01', to: 'n02', length_m: 2, level: 'OK' }],
  stops: [{ id: 'table_01', entity_id: 'table_01', node: 'n01', x: 1, y: 2, yaw: 1.5708, yaw_deg: 90, kind: 'table_service', level: 'OK', pinned: true }],
  excluded: [{ id: 'kitchen_01', entity_id: 'kitchen_01', reason: 'x', reason_ko: '통로 밖', x: 3.9, y: 3 }],
  problems: ['문제'],
  warnings: [],
  notes: [],
  summary: { OK: 3, WARN: 1, FAIL: 0 },
})

describe('parseGeneratedGraph', () => {
  it('정상 응답을 읽는다', () => {
    const g = parseGeneratedGraph(base())
    expect(g.nodes).toHaveLength(2)
    expect(g.edges[0].length_m).toBe(2)
    expect(g.stops[0].kind).toBe('table_service')
    expect(g.excluded[0]).toMatchObject({ id: 'kitchen_01', reason: '통로 밖', x: 3.9, y: 3 })
    expect(g.summary).toEqual({ OK: 3, WARN: 1, FAIL: 0 })
  })
  it('수동 지정 표식과 실패한 지정(pin)을 읽는다', () => {
    const v = base()
    ;(v.excluded[0] as Record<string, unknown>).pin = { x: 1.5, y: 0.5, yaw_deg: 90 }
    const g = parseGeneratedGraph(v)
    expect(g.stops[0].pinned).toBe(true)
    expect(g.excluded[0].pin).toEqual({ x: 1.5, y: 0.5, yaw_deg: 90 })
    const none = parseGeneratedGraph({ ...base(), stops: [{ ...base().stops[0], pinned: undefined }] })
    expect(none.stops[0].pinned).toBe(false)
  })

  it('수동 waypoint 표식(manual)은 Semantic Map 을 읽어도 남고, 없는 것은 그대로다', () => {
    const doc = (extra: object) =>
      parseSemanticMap({
        store_id: 's',
        map_revision: 1,
        entities: [{ id: 'table_01', type: 'table', label: 'T' }],
        waypoints: [{ id: 'wp_table_01', entity_id: 'table_01', pose: { x: 1, y: 2, yaw: 0 }, ...extra }],
        zones: [],
      })
    expect(doc({ manual: true }).waypoints[0]).toMatchObject({ manual: true })
    expect('manual' in doc({}).waypoints[0]).toBe(false)
    expect('manual' in doc({ manual: false }).waypoints[0]).toBe(false)
  })

  it('없는 노드를 가리키는 통로는 거절한다', () => {
    const v = base()
    v.edges[0].to = 'n99'
    expect(() => parseGeneratedGraph(v)).toThrow(/없는 노드/)
  })
  it('알 수 없는 정차 종류와 검사 등급은 거절한다', () => {
    const a = base()
    a.stops[0].kind = 'weird'
    expect(() => parseGeneratedGraph(a)).toThrow()
    const b = base()
    b.nodes[0].level = 'BAD'
    expect(() => parseGeneratedGraph(b)).toThrow()
  })
  it('배치 불가 점의 좌표가 없어도 읽는다', () => {
    const v = base()
    delete (v.excluded[0] as Record<string, unknown>).x
    delete (v.excluded[0] as Record<string, unknown>).y
    const g = parseGeneratedGraph(v)
    expect(g.excluded[0].x).toBeNull()
  })
})
