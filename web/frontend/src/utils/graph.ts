import type { CheckLevel, GraphStop } from '../types/provisional'

export interface GraphLayers {
  edges: boolean
  lengths: boolean
  stops: boolean
  /** 편집 화면에서 직접 지정한 Waypoint (파란 화살표) */
  myWaypoints: boolean
}

export const DEFAULT_GRAPH_LAYERS: GraphLayers = { edges: true, lengths: true, stops: true, myWaypoints: true }

export const GRAPH_EDGE_COLOR: Record<CheckLevel, string> = { OK: '#5b6b82', WARN: '#d99a00', FAIL: '#d93a32' }
/** 손으로 지정한 정차 위치 */
export const GRAPH_PINNED_COLOR = '#7c4dff'
export const GRAPH_STOP_COLOR: Record<CheckLevel, string> = { OK: '#0f8a7e', WARN: '#d99a00', FAIL: '#d93a32' }

/** map frame 방위: 0° = +x(동), 90° = +y(북) */
const COMPASS = [
  { deg: 0, name: '동' },
  { deg: 90, name: '북' },
  { deg: 180, name: '서' },
  { deg: -90, name: '남' },
] as const

const OPPOSITE: Record<string, string> = { 동: '서', 서: '동', 남: '북', 북: '남' }

/** 동·서·남·북 중 tol 도 안이면 그 이름, 아니면 null */
export function compassOf(yawDeg: number, tol = 12): string | null {
  for (const c of COMPASS) {
    const d = Math.abs((((yawDeg - c.deg) % 360) + 540) % 360 - 180)
    if (d <= tol) return c.name
  }
  return null
}

/** 정차 위치를 한 문장으로: 어느 쪽에 서서 어느 쪽을 보는지 */
export function stopSideText(stop: Pick<GraphStop, 'kind' | 'yaw_deg'>): string {
  const c = compassOf(stop.yaw_deg)
  if (!c) return `${Math.round(stop.yaw_deg)}° 방향을 바라봄`
  if (stop.kind === 'standby') return `${c}쪽을 바라봄`
  return `대상의 ${OPPOSITE[c]}쪽에 서서 ${c}쪽을 바라봄`
}
