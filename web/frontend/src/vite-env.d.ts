/// <reference types="vite/client" />

interface ImportMetaEnv {
  /** 'mock' (기본) | 'real' */
  readonly VITE_API_MODE?: string
  /** 매장 ID (기본 demo_store) */
  readonly VITE_STORE_ID?: string
  /** 상단에 표시할 매장 이름 */
  readonly VITE_STORE_NAME?: string
  /** REST base URL. 비우면 같은 origin(Vite 프록시) 사용 */
  readonly VITE_API_BASE_URL?: string
  /** WebSocket 전체 URL. 비우면 같은 origin의 /ws/state 사용 */
  readonly VITE_WS_URL?: string
  /** dev 서버 프록시 대상 Backend 주소 (vite.config.ts 전용) */
  readonly VITE_BACKEND_URL?: string
}

interface ImportMeta {
  readonly env: ImportMetaEnv
}
