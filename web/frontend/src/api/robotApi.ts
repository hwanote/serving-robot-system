import { apiClient, request } from './client'

const path = (robotId: string, action: string) => `/api/robots/${encodeURIComponent(robotId)}/${action}`

/** POST /api/robots/{id}/reset — 로봇을 `node` 위에 직접 올려놓았다는 뜻. 진행 중이던 주문은 취소된다. */
export async function resetRobot(robotId: string, node?: string): Promise<void> {
  await request(apiClient.post<unknown>(path(robotId, 'reset'), node ? { node } : {}))
}

/** POST /api/robots/{id}/home — 대기 위치로 보냄 (위치가 OK 이고 쉬고 있을 때만) */
export async function sendRobotHome(robotId: string): Promise<void> {
  await request(apiClient.post<unknown>(path(robotId, 'home')))
}

/** POST /api/robots/{id}/reload-map — 로봇이 config/common 을 다시 읽음 (쉬고 있을 때만) */
export async function reloadRobotMap(robotId: string): Promise<void> {
  await request(apiClient.post<unknown>(path(robotId, 'reload-map')))
}

export type ReloadOutcome = 'sent' | 'current' | 'busy' | 'offline'

/** POST /api/robots/reload-map — 쉬고 있는 모든 로봇 */
export async function reloadAllRobotMaps(): Promise<Record<string, ReloadOutcome | string>> {
  const data = (await request(apiClient.post<unknown>('/api/robots/reload-map'))) as { robots?: Record<string, string> } | null
  return data?.robots ?? {}
}
