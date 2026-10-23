/// <reference types="vitest/config" />
import react from '@vitejs/plugin-react'
import { msw } from 'msw/vite'
import { defineConfig, loadEnv } from 'vite'

// https://vite.dev/config/
export default defineConfig(({ mode }) => {
  const env = loadEnv(mode, process.cwd(), '')
  // real 모드에서 /api, /ws 요청을 전달할 Backend 주소 (dev 서버 프록시용)
  const backendUrl = env.VITE_BACKEND_URL || 'http://localhost:8000'
  const isMock = env.VITE_API_MODE !== 'real'

  return {
    // msw: mock 모드에서만 mockServiceWorker.js 를 제공/출력 (public/ 복사 불필요)
    plugins: [react(), ...(isMock ? [msw({ mode: 'worker-only' })] : [])],
    server: {
      proxy: {
        '/api': { target: backendUrl, changeOrigin: true },
        '/ws': { target: backendUrl.replace(/^http/, 'ws'), ws: true, changeOrigin: true },
      },
    },
    test: {
      environment: 'jsdom',
      setupFiles: ['./src/test/setup.ts'],
      css: false,
    },
  }
})
