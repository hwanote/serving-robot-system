import type { RobotView } from '../state/fleetReducer'

/** 리셋 위치 이름 → 사람이 읽는 말 (standby_01 → 대기 위치) */
export function nodeLabel(node: string): string {
  if (node === 'kitchen') return '주방'
  if (/^standby_\d+$/.test(node)) return '대기 위치'
  return node
}

export type MapBadge = { text: string; tone: 'ok' | 'warn' | 'muted' }

export function mapBadge(robot: Pick<RobotView, 'map_current'>): MapBadge {
  if (robot.map_current === true) return { text: '최신 지도', tone: 'ok' }
  if (robot.map_current === false) return { text: '옛 지도 — 다시 읽기 필요', tone: 'warn' }
  return { text: '지도 버전 확인 불가', tone: 'muted' }
}

/** 버튼이 꺼진 이유 (꺼져 있을 때만 안내) */
export function homeHint(robot: Pick<RobotView, 'localization' | 'controls'>): string {
  if (robot.controls?.home) return ''
  if (robot.localization && robot.localization !== 'OK') return '위치를 아직 몰라요. 로봇을 대기 위치 등에 올려놓고 먼저 리셋하세요.'
  return '쉬고 있고 대기 위치에서 떨어져 있을 때만 쓸 수 있어요.'
}

export function reloadHint(robot: Pick<RobotView, 'map_current' | 'controls'>): string {
  if (robot.controls?.reload_map) return ''
  if (robot.map_current === true) return '이미 최신 지도예요.'
  return '로봇이 쉬고 있을 때만 다시 읽을 수 있어요.'
}
