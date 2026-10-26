/**
 * 손으로 지정한 정차 위치("수동" waypoint). [임시 확장: types/contracts.ts 는 건드리지 않는다]
 * waypoint 에 manual: true 가 있으면 자동 생성기가 그 자리를 정차 위치로 쓴다.
 * 프론트가 자동으로 만든 기본 waypoint(엔티티 앞 0.6 m)에는 붙지 않는다.
 */
import type { Waypoint } from '../types/contracts'
import { normalizeYaw } from './coordinates'

export type WaypointWithManual = Waypoint & { manual?: boolean }

export const isManualWaypoint = (w: Waypoint): boolean => (w as WaypointWithManual).manual === true

/** manual 표식을 켜거나(true) 지운 사본 */
export function markManual(w: Waypoint, manual: boolean): Waypoint {
  const out = { ...w } as WaypointWithManual
  delete out.manual
  if (manual) out.manual = true
  return out
}

/** 가장 가까운 동·북·서·남(0, π/2, π, -π/2)으로 맞춘다 */
export const snapAxisYaw = (yaw: number): number => {
  const q = (Math.PI / 2) * Math.round(yaw / (Math.PI / 2))
  return normalizeYaw(Object.is(q, -0) ? 0 : q)
}

/** 방향 버튼 (map frame: 0 = +x 동쪽, π/2 = +y 북쪽) */
export const AXIS_DIRECTIONS = [
  { name: '동', yaw: 0 },
  { name: '서', yaw: Math.PI },
  { name: '남', yaw: -Math.PI / 2 },
  { name: '북', yaw: Math.PI / 2 },
] as const
