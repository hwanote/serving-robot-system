import { describe, expect, it } from 'vitest'
import {
  fitView,
  imageToMap,
  imageToScreen,
  imageVectorToYaw,
  isInsideImage,
  mapToImage,
  mapToPixelIndex,
  mapToScreen,
  normalizeYaw,
  panBy,
  pixelIndexToMap,
  screenToImage,
  screenToMap,
  yawToScreenDeg,
  zoomAt,
  type MapGeometry,
  type ViewTransform,
} from './coordinates'

const geo: MapGeometry = {
  resolution: 0.05,
  width: 158,
  height: 120,
  origin: { x: -0.75, y: -0.82, yaw: 0 },
}

const LIMITS = { min: 0.01, max: 100 }

describe('Pixel → ROS map 좌표 (지시서 필수 사례)', () => {
  it('pixel (100, 70) → (4.25, 1.63)', () => {
    const p = pixelIndexToMap(geo, 100, 70)
    expect(p.x).toBeCloseTo(4.25, 9)
    expect(p.y).toBeCloseTo(1.63, 9)
  })

  it('pixel (0, height-1) 은 origin', () => {
    const p = pixelIndexToMap(geo, 0, geo.height - 1)
    expect(p.x).toBeCloseTo(-0.75, 9)
    expect(p.y).toBeCloseTo(-0.82, 9)
  })
})

describe('ROS map 좌표 → Pixel', () => {
  it('(4.25, 1.63) → pixel (100, 70)', () => {
    const p = mapToPixelIndex(geo, 4.25, 1.63)
    expect(p.u).toBeCloseTo(100, 9)
    expect(p.v).toBeCloseTo(70, 9)
  })

  it('픽셀 인덱스 왕복 변환', () => {
    for (const [u, v] of [
      [0, 0],
      [157, 119],
      [33, 81],
    ]) {
      const m = pixelIndexToMap(geo, u, v)
      const back = mapToPixelIndex(geo, m.x, m.y)
      expect(back.u).toBeCloseTo(u, 9)
      expect(back.v).toBeCloseTo(v, 9)
    }
  })

  it('연속 이미지 좌표는 픽셀 인덱스 공식과 일치한다 (인덱스 (u,v) 좌하단 = 연속 (u, v+1))', () => {
    const a = pixelIndexToMap(geo, 100, 70)
    const b = imageToMap(geo, { u: 100, v: 71 })
    expect(b.x).toBeCloseTo(a.x, 9)
    expect(b.y).toBeCloseTo(a.y, 9)
  })

  it('연속 좌표 왕복 변환', () => {
    const m = imageToMap(geo, { u: 12.34, v: 56.78 })
    const back = mapToImage(geo, m.x, m.y)
    expect(back.u).toBeCloseTo(12.34, 9)
    expect(back.v).toBeCloseTo(56.78, 9)
  })
})

describe('origin yaw ≠ 0 (회전 변환)', () => {
  const rotated: MapGeometry = { ...geo, origin: { x: 1, y: 2, yaw: Math.PI / 2 } }

  it('지도 로컬 +x 방향이 map +y 방향이 된다', () => {
    // pixel (20, height-1): 로컬 (1.0 m, 0) → map (1, 3)
    const p = pixelIndexToMap(rotated, 20, rotated.height - 1)
    expect(p.x).toBeCloseTo(1, 9)
    expect(p.y).toBeCloseTo(3, 9)
  })

  it('왕복 변환이 유지된다', () => {
    const m = imageToMap(rotated, { u: 40.5, v: 10.25 })
    const back = mapToImage(rotated, m.x, m.y)
    expect(back.u).toBeCloseTo(40.5, 9)
    expect(back.v).toBeCloseTo(10.25, 9)
  })

  it('yaw ↔ 화면 방향 변환이 origin yaw 를 반영한다', () => {
    expect(yawToScreenDeg(rotated, Math.PI / 2)).toBeCloseTo(0, 9)
    expect(imageVectorToYaw(rotated, 1, 0)).toBeCloseTo(Math.PI / 2, 9)
  })
})

describe('화면 좌표 (Zoom / Pan)', () => {
  const view0: ViewTransform = fitView(geo, 800, 600)

  it('fitView 는 지도 전체를 뷰포트 안에 중앙 정렬한다', () => {
    const tl = imageToScreen(view0, { u: 0, v: 0 })
    const br = imageToScreen(view0, { u: geo.width, v: geo.height })
    expect(tl.sx).toBeGreaterThanOrEqual(0)
    expect(br.sx).toBeLessThanOrEqual(800)
    expect(tl.sy).toBeGreaterThanOrEqual(0)
    expect(br.sy).toBeLessThanOrEqual(600)
    expect((tl.sx + br.sx) / 2).toBeCloseTo(400, 6)
  })

  it('screen ↔ image 왕복', () => {
    const s = { sx: 123.4, sy: 456.7 }
    const back = imageToScreen(view0, screenToImage(view0, s))
    expect(back.sx).toBeCloseTo(s.sx, 9)
    expect(back.sy).toBeCloseTo(s.sy, 9)
  })

  it('줌/팬 후에도 같은 실제 위치를 클릭하면 같은 map 좌표가 나온다', () => {
    const target = { x: 4.25, y: 1.63 }
    let view = view0
    const before = screenToMap(geo, view, mapToScreen(geo, view, target.x, target.y))

    view = zoomAt(view, 3.7, { sx: 210, sy: 95 }, LIMITS)
    view = panBy(view, -140, 62)
    view = zoomAt(view, 0.6, { sx: 500, sy: 300 }, LIMITS)

    const screen = mapToScreen(geo, view, target.x, target.y)
    const after = screenToMap(geo, view, screen)
    expect(after.x).toBeCloseTo(before.x, 9)
    expect(after.y).toBeCloseTo(before.y, 9)
    expect(after.x).toBeCloseTo(4.25, 9)
    expect(after.y).toBeCloseTo(1.63, 9)
  })

  it('zoomAt 은 커서 아래 지도 지점을 고정한다', () => {
    const anchor = { sx: 321, sy: 222 }
    const before = screenToMap(geo, view0, anchor)
    const zoomed = zoomAt(view0, 2.5, anchor, LIMITS)
    const after = screenToMap(geo, zoomed, anchor)
    expect(zoomed.scale).toBeCloseTo(view0.scale * 2.5, 9)
    expect(after.x).toBeCloseTo(before.x, 9)
    expect(after.y).toBeCloseTo(before.y, 9)
  })

  it('zoomAt 은 배율 한계를 지킨다', () => {
    const v = zoomAt(view0, 1000, { sx: 0, sy: 0 }, { min: 0.5, max: 20 })
    expect(v.scale).toBe(20)
  })

  it('Canvas(CSS) 크기가 바뀌어도 같은 이미지 픽셀은 같은 map 좌표다', () => {
    const small = fitView(geo, 400, 300)
    const big = fitView(geo, 1600, 1200)
    const img = { u: 100, v: 71 }
    const a = screenToMap(geo, small, imageToScreen(small, img))
    const b = screenToMap(geo, big, imageToScreen(big, img))
    expect(a.x).toBeCloseTo(b.x, 9)
    expect(a.y).toBeCloseTo(b.y, 9)
  })
})

describe('경계 / 해상도', () => {
  it('지도 경계 밖 클릭을 판별한다', () => {
    expect(isInsideImage(geo, { u: -0.1, v: 10 })).toBe(false)
    expect(isInsideImage(geo, { u: 10, v: 120.1 })).toBe(false)
    expect(isInsideImage(geo, { u: 0, v: 0 })).toBe(true)
    expect(isInsideImage(geo, { u: 158, v: 120 })).toBe(true)
  })

  it('해상도가 바뀌면 같은 픽셀의 map 좌표도 바뀐다 (저장 좌표는 map 기준이므로 재계산 필요)', () => {
    const hiRes: MapGeometry = { ...geo, resolution: 0.025, width: 316, height: 240 }
    const p = mapToPixelIndex(hiRes, 4.25, 1.63)
    expect(p.u).toBeCloseTo(200, 9)
    expect(p.v).toBeCloseTo(141, 9)
  })

  it('yaw 정규화 (-π, π]', () => {
    expect(normalizeYaw(3 * Math.PI)).toBeCloseTo(Math.PI, 9)
    expect(normalizeYaw(-Math.PI)).toBeCloseTo(Math.PI, 9)
    expect(normalizeYaw(-Math.PI / 2)).toBeCloseTo(-Math.PI / 2, 9)
  })
})
