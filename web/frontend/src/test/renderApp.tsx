import { render } from '@testing-library/react'
import { createMemoryRouter, RouterProvider } from 'react-router-dom'
import { FakeSocket } from './FakeSocket'
import { routes } from '../routes'
import { FleetProvider } from '../state/FleetProvider'
import { StoreDataProvider } from '../state/StoreDataProvider'
import { ToastProvider } from '../state/ToastProvider'

const createSocket = (url: string) => new FakeSocket(url)

/** 실제 앱과 같은 Provider/라우트 구성으로 렌더링. WebSocket 은 FakeSocket 으로 주입 */
export function renderApp(path = '/') {
  FakeSocket.instances = []
  const router = createMemoryRouter(routes, { initialEntries: [path] })
  const utils = render(
    <ToastProvider>
      <StoreDataProvider>
        <FleetProvider createSocket={createSocket} socketUrl="ws://test/ws/state">
          <RouterProvider router={router} />
        </FleetProvider>
      </StoreDataProvider>
    </ToastProvider>,
  )
  return { ...utils, router, socket: () => FakeSocket.instances.at(-1)! }
}
