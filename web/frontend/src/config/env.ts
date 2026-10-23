export type ApiMode = 'mock' | 'real'

const env = import.meta.env

export const API_MODE: ApiMode = env.VITE_API_MODE === 'real' ? 'real' : 'mock'

export const STORE_ID = env.VITE_STORE_ID || 'demo_store'
export const STORE_NAME = env.VITE_STORE_NAME || '데모 매장'

/** REST base URL. 빈 문자열이면 같은 origin 기준 상대 경로(/api/...)로 요청한다. */
export const API_BASE_URL = env.VITE_API_BASE_URL ?? ''

export const STATE_SOCKET_PATH = '/ws/state'

/** WebSocket URL. 지정하지 않으면 현재 origin의 /ws/state로 연결한다. */
export function resolveStateSocketUrl(): string {
  if (env.VITE_WS_URL) return env.VITE_WS_URL
  const { protocol, host } = window.location
  return `${protocol === 'https:' ? 'wss' : 'ws'}://${host}${STATE_SOCKET_PATH}`
}

/** image_url 같은 서버 상대 경로를 실제 요청 URL로 변환한다. */
export function resolveApiUrl(path: string): string {
  if (/^https?:\/\//.test(path)) return path
  return `${API_BASE_URL}${path}`
}
