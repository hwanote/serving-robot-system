import { createBrowserRouter, RouterProvider } from 'react-router-dom'
import { routes } from './routes'
import { FleetProvider } from './state/FleetProvider'
import { StoreDataProvider } from './state/StoreDataProvider'
import { ToastProvider } from './state/ToastProvider'

const router = createBrowserRouter(routes)

export default function App() {
  return (
    <ToastProvider>
      <StoreDataProvider>
        <FleetProvider>
          <RouterProvider router={router} />
        </FleetProvider>
      </StoreDataProvider>
    </ToastProvider>
  )
}
