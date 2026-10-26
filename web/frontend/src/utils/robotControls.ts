import type { RobotView } from '../state/fleetReducer'

/** 리셋 위치 이름 → 사람이 읽는 말 (standby_01 → 대기 위치) */
export function nodeLabel(node: string): string {
  if (node === 'kitchen') return '주방'
  if (/^standby_\d+$/.test(node)) return '대기 위치'
  return node
}

/** 새 지도(그림)가 매장에 적용됐는데 로봇이 옛 지도로 돌고 있을 때 사람에게 보여 주는 말 */
export const RESTART_NOTICE = '로봇을 재시작해 주세요. (새로운 지도가 적용되었습니다.)'
export const RESTART_CAUTION = '영업이 끝났거나 주문이 없을 때 로봇을 껐다 켜 주세요. 재시작하기 전까지 이 로봇은 주문을 받지 않아요.'

/** 지도 그림이 바뀌어서 "지도 다시 읽기"로는 안 되고 재시작이 필요한 로봇 */
export function needsRestart(robot: Pick<RobotView, 'map_image_current'>): boolean {
  return robot.map_image_current === false
}

export type MapBadge = { text: string; tone: 'ok' | 'warn' | 'muted' }

export function mapBadge(robot: Pick<RobotView, 'map_current' | 'map_image_current'>): MapBadge {
  if (needsRestart(robot)) return { text: '지도 그림이 바뀜 — 로봇 재시작 필요', tone: 'warn' }
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

export function reloadHint(robot: Pick<RobotView, 'map_current' | 'map_image_current' | 'controls'>): string {
  if (needsRestart(robot)) return '지도 그림이 바뀌어서 다시 읽기로는 안 돼요. 로봇을 재시작해 주세요.'
  if (robot.controls?.reload_map) return ''
  if (robot.map_current === true) return '이미 최신 지도예요.'
  return '로봇이 쉬고 있을 때만 다시 읽을 수 있어요.'
}
