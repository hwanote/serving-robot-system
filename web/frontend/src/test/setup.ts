import '@testing-library/jest-dom/vitest'
import { cleanup } from '@testing-library/react'
import { afterAll, afterEach, beforeAll, beforeEach, vi } from 'vitest'
import { mockDb, resetMockDb } from '../mocks/db'
import { resetLiveMapMock } from '../mocks/liveMapHandlers'
import { server } from './server'

// jsdom 에 없는 브라우저 API
class ResizeObserverStub {
  observe() {}
  unobserve() {}
  disconnect() {}
}
globalThis.ResizeObserver ??= ResizeObserverStub as unknown as typeof ResizeObserver

// Mock 개발 패널은 브라우저 Service Worker 전용이므로 테스트에서 제외
vi.mock('../mocks/MockDevPanel', () => ({ default: () => null }))

beforeAll(() => server.listen({ onUnhandledFrame: 'error' }))
beforeEach(() => {
  resetMockDb()
  resetLiveMapMock()
  mockDb.latencyMs = 0
})
afterEach(() => {
  cleanup()
  server.resetHandlers()
})
afterAll(() => server.close())
