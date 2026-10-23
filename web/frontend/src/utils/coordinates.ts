/**
 * Pixel ↔ ROS map 좌표 변환
 *
 * 좌표 공간
 *  1) map      : ROS map frame (m). x 오른쪽, y 위쪽.
 *  2) image    : 원본 지도 이미지 픽셀 좌표. u 오른쪽 증가, v 아래쪽 증가.
 *                연속 좌표로, 이미지 좌상단 모서리가 (0,0), 우하단 모서리가 (width,height).
 *                정수 픽셀 인덱스 (i,j) 는 [i,i+1) × [j,j+1) 영역을 차지한다.
 *  3) screen   : 지도 뷰포트(SVG) 기준 CSS px. 줌(scale)·팬(offset) 적용.
 *                screen = image × scale + offset
 *
 * 픽셀 인덱스 공식 (팀 지시서, origin yaw = 0 일 때)
 *    x = origin_x + u × resolution
 *    y = origin_y + (height − 1 − v) × resolution
 *  → 픽셀 (u, v) 의 "좌하단 모서리" map 좌표. pixelIndexToMap / mapToPixelIndex 로 제공.
 *
 * 연속 좌표 공식 (클릭/렌더링용, ROS map_server 규칙: origin = 좌하단 픽셀의 좌하단 모서리)
 *    x = origin_x + u × resolution
 *    y = origin_y + (height − v) × resolution
 *  → 정수 픽셀 인덱스 공식과 정확히 일치: 인덱스 (u, v) 의 좌하단 모서리 = 연속 좌표 (u, v + 1)
 *
 * origin.yaw ≠ 0 인 경우 map_server 와 동일하게 지도 로컬 좌표를 origin.yaw 만큼 회전한다.
 */
import type { RawMapInfo } from '../types/contracts'

export type MapGeometry = Pick<RawMapInfo, 'resolution' | 'width' | 'height' | 'origin'>

export interface Vec2 {
  x: number
  y: number
}

/** 이미지(원본 픽셀) 좌표 */
export interface ImagePoint {
  u: number
  v: number
}

/** 화면(뷰포트 CSS px) 좌표 */
export interface ScreenPoint {
  sx: number
  sy: number
}

export interface ViewTransform {
  /** 화면 px / 이미지 px */
  scale: number
  offsetX: number
  offsetY: number
}

export function isValidGeometry(geo: MapGeometry): boolean {
  return (
    geo.resolution > 0 &&
    geo.width > 0 &&
    geo.height > 0 &&
    [geo.origin.x, geo.origin.y, geo.origin.yaw, geo.resolution].every(Number.isFinite)
  )
}

// ---------- map ↔ 지도 로컬(회전 전) ----------

function localToMap(geo: MapGeometry, lx: number, ly: number): Vec2 {
  const { x, y, yaw } = geo.origin
  if (yaw === 0) return { x: x + lx, y: y + ly }
  const c = Math.cos(yaw)
  const s = Math.sin(yaw)
  return { x: x + c * lx - s * ly, y: y + s * lx + c * ly }
}

function mapToLocal(geo: MapGeometry, mx: number, my: number): { lx: number; ly: number } {
  const dx = mx - geo.origin.x
  const dy = my - geo.origin.y
  const yaw = geo.origin.yaw
  if (yaw === 0) return { lx: dx, ly: dy }
  const c = Math.cos(yaw)
  const s = Math.sin(yaw)
  return { lx: c * dx + s * dy, ly: -s * dx + c * dy }
}

// ---------- 픽셀 인덱스 (지시서 공식) ----------

/** 원본 지도 픽셀 인덱스 (u, v) → map 좌표 (픽셀 좌하단 모서리) */
export function pixelIndexToMap(geo: MapGeometry, u: number, v: number): Vec2 {
  return localToMap(geo, u * geo.resolution, (geo.height - 1 - v) * geo.resolution)
}

/** map 좌표 → 픽셀 인덱스 (실수). 정수 인덱스가 필요하면 호출 측에서 floor/round */
export function mapToPixelIndex(geo: MapGeometry, x: number, y: number): ImagePoint {
  const { lx, ly } = mapToLocal(geo, x, y)
  return { u: lx / geo.resolution, v: geo.height - 1 - ly / geo.resolution }
}

// ---------- 연속 이미지 좌표 (렌더링/클릭) ----------

export function imageToMap(geo: MapGeometry, p: ImagePoint): Vec2 {
  return localToMap(geo, p.u * geo.resolution, (geo.height - p.v) * geo.resolution)
}

export function mapToImage(geo: MapGeometry, x: number, y: number): ImagePoint {
  const { lx, ly } = mapToLocal(geo, x, y)
  return { u: lx / geo.resolution, v: geo.height - ly / geo.resolution }
}

export function isInsideImage(geo: MapGeometry, p: ImagePoint): boolean {
  return p.u >= 0 && p.v >= 0 && p.u <= geo.width && p.v <= geo.height
}

export function isInsideMap(geo: MapGeometry, x: number, y: number): boolean {
  return isInsideImage(geo, mapToImage(geo, x, y))
}

export function clampToImage(geo: MapGeometry, p: ImagePoint): ImagePoint {
  return { u: Math.min(geo.width, Math.max(0, p.u)), v: Math.min(geo.height, Math.max(0, p.v)) }
}

// ---------- 방향 (yaw) ----------

/** map yaw(rad) → 화면 회전각(deg, 시계방향 +). 이미지 v축이 아래 방향이므로 부호가 반전된다. */
export function yawToScreenDeg(geo: MapGeometry, yaw: number): number {
  return (-(yaw - geo.origin.yaw) * 180) / Math.PI
}

/** 이미지 공간의 방향 벡터 (du, dv) → map yaw(rad) */
export function imageVectorToYaw(geo: MapGeometry, du: number, dv: number): number {
  return normalizeYaw(Math.atan2(-dv, du) + geo.origin.yaw)
}

/** (-π, π] 로 정규화 */
export function normalizeYaw(yaw: number): number {
  let a = yaw % (2 * Math.PI)
  if (a <= -Math.PI) a += 2 * Math.PI
  if (a > Math.PI) a -= 2 * Math.PI
  return a
}

// ---------- 화면 ↔ 이미지 (줌/팬) ----------

export function screenToImage(view: ViewTransform, s: ScreenPoint): ImagePoint {
  return { u: (s.sx - view.offsetX) / view.scale, v: (s.sy - view.offsetY) / view.scale }
}

export function imageToScreen(view: ViewTransform, p: ImagePoint): ScreenPoint {
  return { sx: p.u * view.scale + view.offsetX, sy: p.v * view.scale + view.offsetY }
}

export function screenToMap(geo: MapGeometry, view: ViewTransform, s: ScreenPoint): Vec2 {
  return imageToMap(geo, screenToImage(view, s))
}

export function mapToScreen(geo: MapGeometry, view: ViewTransform, x: number, y: number): ScreenPoint {
  return imageToScreen(view, mapToImage(geo, x, y))
}

export interface ZoomLimits {
  min: number
  max: number
}

/** 화면 점 (sx, sy) 아래의 지도 위치를 고정한 채 확대/축소 */
export function zoomAt(view: ViewTransform, factor: number, anchor: ScreenPoint, limits: ZoomLimits): ViewTransform {
  const scale = Math.min(limits.max, Math.max(limits.min, view.scale * factor))
  const p = screenToImage(view, anchor)
  return { scale, offsetX: anchor.sx - p.u * scale, offsetY: anchor.sy - p.v * scale }
}

export function panBy(view: ViewTransform, dx: number, dy: number): ViewTransform {
  return { ...view, offsetX: view.offsetX + dx, offsetY: view.offsetY + dy }
}

/** 지도 전체가 뷰포트에 들어오도록 맞춤 */
export function fitView(geo: MapGeometry, viewportW: number, viewportH: number, padding = 24): ViewTransform {
  const availW = Math.max(1, viewportW - padding * 2)
  const availH = Math.max(1, viewportH - padding * 2)
  const scale = Math.max(0.01, Math.min(availW / geo.width, availH / geo.height))
  return {
    scale,
    offsetX: (viewportW - geo.width * scale) / 2,
    offsetY: (viewportH - geo.height * scale) / 2,
  }
}

// ---------- 표시/저장용 ----------

/** 저장 정밀도: 위치 1mm, 각도 0.0001rad */
export function roundTo(value: number, digits = 3): number {
  const f = 10 ** digits
  return Math.round(value * f) / f
}

export function formatMeters(value: number): string {
  return `${value.toFixed(2)} m`
}

export function formatYawDeg(yaw: number): string {
  return `${Math.round((yaw * 180) / Math.PI)}°`
}
