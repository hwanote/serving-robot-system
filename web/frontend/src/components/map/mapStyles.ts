import type { EntityType, ZoneType } from '../../types/contracts'

export const ENTITY_STYLE: Record<EntityType, { color: string; letter: string }> = {
  table: { color: '#b86b1f', letter: 'T' },
  counter: { color: '#0f8a7e', letter: 'C' },
  kitchen: { color: '#c2410c', letter: 'K' },
  robot_wait: { color: '#2f8a4c', letter: 'W' },
}

export const ZONE_STYLE: Record<ZoneType, { color: string; dash?: string }> = {
  exclusive: { color: '#4b5ff0' },
  no_go: { color: '#d93a32' },
  slow: { color: '#d99a00', dash: '6 4' },
}

export const WAYPOINT_COLOR = '#1f58b5'
export const ROBOT_COLORS = ['#0f8a7e', '#7a3fd1', '#c2410c', '#2a6fdb']

/** robot_01 → 첫 번째 색 */
export const robotColor = (robotId: string) => {
  const n = Number(/(\d+)$/.exec(robotId)?.[1] ?? 1)
  return ROBOT_COLORS[(Math.max(1, n) - 1) % ROBOT_COLORS.length]
}
