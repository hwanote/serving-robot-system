import { setupServer } from 'msw/node'
import { httpHandlers } from '../mocks/handlers'

/** 테스트용 Mock REST 서버 (브라우저 Mock 과 동일한 handler 사용) */
export const server = setupServer(...httpHandlers)
