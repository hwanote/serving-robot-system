import { fireEvent, render } from '@testing-library/react'
import { describe, expect, it, vi } from 'vitest'
import type { GeneratedGraph } from '../../types/provisional'
import { DEFAULT_GRAPH_LAYERS, type GraphLayers } from '../../utils/graph'
import { GeneratedGraphLayer } from './GeneratedGraphLayer'
import type { MapOverlayContext } from './MapViewport'

const RES = 0.05
const ctx: MapOverlayContext = {
  geo: { resolution: RES, width: 200, height: 200, origin: { x: 0, y: 0, yaw: 0 } },
  view: { scale: 1, offsetX: 0, offsetY: 0 },
  toImage: (x, y) => ({ u: x / RES, v: 200 - y / RES }),
  px: (n) => n,
  beginDrag: () => {},
}

const graph: GeneratedGraph = {
  source: 'draft',
  draft_sha: 's',
  matches_applied: false,
  ok: false,
  nodes: [
    { id: 'n01', x: 1, y: 2, level: 'OK' },
    { id: 'n02', x: 3.5, y: 2, level: 'OK' },
  ],
  edges: [{ id: 'e01', from: 'n01', to: 'n02', length_m: 2.5, level: 'OK' }],
  stops: [{ id: 'table_01', entity_id: 'table_01', node: 'n01', x: 1, y: 2, yaw: 0, yaw_deg: 0, kind: 'table_service', level: 'WARN', pinned: false }],
  excluded: [{ id: 'kitchen_01', entity_id: 'kitchen_01', reason: '통로 밖', x: 3.9, y: 3, pin: null }],
  problems: [],
  warnings: [],
  notes: [],
  summary: { OK: 2, WARN: 1, FAIL: 0 },
}

const draw = (layers: GraphLayers = DEFAULT_GRAPH_LAYERS, stale = false, g = graph) =>
  render(
    <svg>
      <GeneratedGraphLayer ctx={ctx} graph={g} layers={layers} stale={stale} />
    </svg>,
  )

describe('자동 생성 통로 레이어', () => {
  it('통로를 선으로 그리고 길이(m)를 가운데에 적는다', () => {
    const { getByTestId } = draw()
    const line = getByTestId('graph-edge-e01').querySelector('line')!
    expect(Number(line.getAttribute('x1'))).toBeCloseTo(20)
    expect(Number(line.getAttribute('x2'))).toBeCloseTo(70)
    expect(getByTestId('graph-length-e01')).toHaveTextContent('2.5 m')
    expect(Number(getByTestId('graph-length-e01').getAttribute('x'))).toBeCloseTo(45)
  })

  it('정차 위치는 화살표로, 배치 불가 점은 ✕ 와 이름으로 표시한다', () => {
    const { getByTestId } = draw()
    expect(getByTestId('graph-stop-table_01')).toHaveAttribute('data-level', 'WARN')
    expect(getByTestId('graph-excluded-kitchen_01')).toHaveTextContent('kitchen_01 배치 불가')
  })

  it('길이 표시를 끄면 숫자만 사라지고 선은 남는다', () => {
    const { queryByTestId } = draw({ ...DEFAULT_GRAPH_LAYERS, lengths: false })
    expect(queryByTestId('graph-length-e01')).toBeNull()
    expect(queryByTestId('graph-edge-e01')).not.toBeNull()
  })

  it('통로·정차 위치를 각각 끌 수 있다', () => {
    const { queryByTestId } = draw({ ...DEFAULT_GRAPH_LAYERS, edges: false, stops: false })
    expect(queryByTestId('graph-edge-e01')).toBeNull()
    expect(queryByTestId('graph-node-n01')).toBeNull()
    expect(queryByTestId('graph-stop-table_01')).toBeNull()
  })

  it('옛 결과(stale)는 흐리게 표시한다', () => {
    const { getByTestId } = draw(DEFAULT_GRAPH_LAYERS, true)
    expect(getByTestId('graph-layer')).toHaveAttribute('data-stale', 'true')
    expect(Number(getByTestId('graph-layer').getAttribute('opacity'))).toBeLessThan(1)
  })

  it('좌표를 모르는 배치 불가 점은 그리지 않는다', () => {
    const { queryByTestId } = draw(DEFAULT_GRAPH_LAYERS, false, { ...graph, excluded: [{ id: 'k', entity_id: null, reason: '', x: null, y: null, pin: null }] })
    expect(queryByTestId('graph-excluded-k')).toBeNull()
  })

  it('이 레이어는 마우스 입력을 가로채지 않는다', () => {
    const { getByTestId } = draw()
    expect(getByTestId('graph-layer')).toHaveAttribute('pointer-events', 'none')
    expect(getByTestId('graph-stop-table_01')).toHaveAttribute('pointer-events', 'none')
  })
})

const withStop = (over: Partial<GeneratedGraph['stops'][number]>): GeneratedGraph => ({ ...graph, stops: [{ ...graph.stops[0], ...over }] })
const drawInteractive = (g: GeneratedGraph, onStopDragStart = vi.fn(), dragging = null as Parameters<typeof GeneratedGraphLayer>[0]['dragging']) =>
  render(
    <svg>
      <GeneratedGraphLayer ctx={ctx} graph={g} layers={DEFAULT_GRAPH_LAYERS} interactive dragging={dragging} onStopDragStart={onStopDragStart} />
    </svg>,
  )

describe('정차 위치를 손으로 옮기기', () => {
  it('수동 정차 위치는 "(수동)" 이름표와 표식이 붙는다', () => {
    const { getByTestId } = drawInteractive(withStop({ pinned: true, level: 'OK' }))
    expect(getByTestId('graph-stop-table_01')).toHaveAttribute('data-pinned', 'true')
    expect(getByTestId('graph-stop-table_01')).toHaveTextContent('table_01 (수동)')
  })

  it('선택 도구에서는 정차 화살표를 눌러 끌기를 시작할 수 있다 (객체 id 와 현재 방향 전달)', () => {
    const onStart = vi.fn()
    const { getByTestId } = drawInteractive(withStop({ yaw: Math.PI / 2, yaw_deg: 90 }), onStart)
    const stop = getByTestId('graph-stop-table_01')
    expect(stop).toHaveAttribute('pointer-events', 'auto')
    fireEvent.pointerDown(stop, { button: 0 })
    expect(onStart).toHaveBeenCalledTimes(1)
    expect(onStart.mock.calls[0][0]).toBe('table_01')
    expect(onStart.mock.calls[0][1]).toBeCloseTo(Math.PI / 2)
  })

  it('객체와 연결되지 않은 정차 위치는 끌 수 없다', () => {
    const onStart = vi.fn()
    const { getByTestId } = drawInteractive(withStop({ entity_id: null }), onStart)
    fireEvent.pointerDown(getByTestId('graph-stop-table_01'), { button: 0 })
    expect(onStart).not.toHaveBeenCalled()
  })

  it('끄는 동안에는 원래 자리를 흐리게 하고 끌고 있는 미리보기를 그린다', () => {
    const { getByTestId } = drawInteractive(graph, vi.fn(), { entityId: 'table_01', x: 2, y: 1, yaw: 0 })
    expect(Number(getByTestId('graph-stop-table_01').getAttribute('opacity'))).toBeLessThan(1)
    expect(getByTestId('graph-drag-ghost')).toBeInTheDocument()
  })

  it('손으로 지정한 자리가 검사에 실패하면 그 자리에 빨간 화살표가 남고 다시 끌 수 있다', () => {
    const failed: GeneratedGraph = {
      ...graph,
      stops: [],
      excluded: [{ id: 'table_01', entity_id: 'table_01', reason: '앞 간격 부족', x: 1, y: 2, pin: { x: 2, y: 1, yaw_deg: 90 } }],
    }
    const onStart = vi.fn()
    const { getByTestId } = drawInteractive(failed, onStart)
    const el = getByTestId('graph-excluded-table_01')
    expect(el).toHaveTextContent('table_01 배치 불가 (수동)')
    fireEvent.pointerDown(el, { button: 0 })
    expect(onStart).toHaveBeenCalledWith('table_01', Math.PI / 2, expect.anything())
  })
})
