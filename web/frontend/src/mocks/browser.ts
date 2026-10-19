/**
 * 브라우저 Mock 서버 시작 (VITE_API_MODE=mock 에서만 동적 import 됨)
 * - REST : Service Worker 가 /api/* 요청을 가로챔
 * - WS   : MSW ws.link 가 /ws/state WebSocket 을 가로챔 → 앱의 StateSocket 코드는 real 모드와 동일
 */
import { ws } from 'msw'
import { setupWorker } from 'msw/browser'
import { resolveStateSocketUrl } from '../config/env'
import { mockDb } from './db'
import { httpHandlers } from './handlers'
import { snapshotMessages, startSimulator } from './simulator'

export const stateLink = ws.link(resolveStateSocketUrl())

const wsHandler = stateLink.addEventListener('connection', ({ client }) => {
  if (mockDb.wsDown) {
    client.close(4000, '[Mock] WebSocket 서버 다운 시뮬레이션')
    return
  }
  for (const message of snapshotMessages()) client.send(message)
})

/** Dev 패널: 현재 연결을 모두 끊음 (wsDown 이 켜져 있으면 재연결도 거부됨) */
export function dropAllSocketClients() {
  stateLink.clients.forEach((client) => client.close(4001, '[Mock] 연결 끊김 시뮬레이션'))
}

export async function startMockWorker() {
  const worker = setupWorker(...httpHandlers, wsHandler)
  await worker.start({
    quiet: true,
    // 지도 이미지 외의 정적 리소스/HMR 은 그대로 통과
    onUnhandledFrame: 'bypass',
  })
  startSimulator((message) => {
    if (!mockDb.wsDown) stateLink.broadcast(message)
  })
}
