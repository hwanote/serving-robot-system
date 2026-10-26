import { StrictMode } from 'react'
import { createRoot } from 'react-dom/client'
import App from './App'
import './styles/global.css'
import './styles/map.css'

async function prepareMocks() {
  // import.meta.env 를 직접 비교해야 real 빌드에서 Mock 코드가 번들에서 제거된다
  if (import.meta.env.VITE_API_MODE === 'real') return
  const { startMockWorker } = await import('./mocks/browser')
  await startMockWorker()
}

const root = createRoot(document.getElementById('root')!)

prepareMocks()
  .catch((error: unknown) => {
    // Mock 서버를 시작하지 못해도 앱은 띄우고, 연결 상태 UI 로 문제를 드러낸다.
    console.error('[mock] Mock Service Worker 시작 실패', error)
  })
  .finally(() => {
    root.render(
      <StrictMode>
        <App />
      </StrictMode>,
    )
  })
