import { render } from '@testing-library/react'
import { describe, expect, it } from 'vitest'
import type { SemanticMap } from '../../types/contracts'
import type { MapOverlayContext } from './MapViewport'
import { SemanticLayer } from './SemanticLayer'

const RES = 0.05
const ctx: MapOverlayContext = {
  geo: { resolution: RES, width: 200, height: 200, origin: { x: 0, y: 0, yaw: 0 } },
  view: { scale: 1, offsetX: 0, offsetY: 0 },
  toImage: (x, y) => ({ u: x / RES, v: 200 - y / RES }),
  px: (n) => n,
  beginDrag: () => {},
}

const doc = (entities: SemanticMap['entities']): SemanticMap => ({ store_id: 's', map_revision: 1, entities, waypoints: [], zones: [] })
const draw = (d: SemanticMap, selectedId?: string) =>
  render(
    <svg>
      <SemanticLayer ctx={ctx} doc={d} selection={selectedId ? { kind: 'entity', id: selectedId } : null} />
    </svg>,
  )

describe('테이블 상단 사각형 표시', () => {
  it('입력한 가로×세로(m)를 이미지 픽셀로 환산해 객체 중심에 그린다', () => {
    const { container } = draw(doc([{ id: 'table_01', type: 'table', label: 'T', pose: { x: 2, y: 1, yaw: 0 }, top_size_m: { x: 0.8, y: 0.6 } }]))
    const rect = container.querySelector('[data-top-of="table_01"]')!
    expect(rect).not.toBeNull()
    // 0.8 m / 0.05 = 16 px, 0.6 m / 0.05 = 12 px, 중심 (40, 180)
    expect(Number(rect.getAttribute('width'))).toBeCloseTo(16)
    expect(Number(rect.getAttribute('height'))).toBeCloseTo(12)
    expect(Number(rect.getAttribute('x'))).toBeCloseTo(40 - 8)
    expect(Number(rect.getAttribute('y'))).toBeCloseTo(180 - 6)
  })

  it('상단 크기가 없거나 한 변이 0 이면 그리지 않는다', () => {
    const { container } = draw(
      doc([
        { id: 'table_01', type: 'table', label: 'A', pose: { x: 1, y: 1, yaw: 0 } },
        { id: 'table_02', type: 'table', label: 'B', pose: { x: 2, y: 1, yaw: 0 }, top_size_m: { x: 0.8, y: 0 } },
      ]),
    )
    expect(container.querySelector('[data-top-of]')).toBeNull()
  })

  it('테이블이 아니면 크기가 있어도 그리지 않는다', () => {
    const { container } = draw(doc([{ id: 'kitchen_01', type: 'kitchen', label: 'K', pose: { x: 1, y: 1, yaw: 0 }, top_size_m: { x: 1, y: 1 } }]))
    expect(container.querySelector('[data-top-of]')).toBeNull()
  })

  it('선택된 테이블은 더 진하게 표시한다', () => {
    const entities: SemanticMap['entities'] = [{ id: 'table_01', type: 'table', label: 'T', pose: { x: 2, y: 1, yaw: 0 }, top_size_m: { x: 0.8, y: 0.8 } }]
    const normal = draw(doc(entities)).container.querySelector('[data-top-of]')!.getAttribute('fill-opacity')
    const selected = draw(doc(entities), 'table_01').container.querySelector('[data-top-of]')!.getAttribute('fill-opacity')
    expect(Number(selected)).toBeGreaterThan(Number(normal))
  })
})
