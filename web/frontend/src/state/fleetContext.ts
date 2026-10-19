import { createContext, useContext } from 'react'
import type { Task } from '../types/provisional'
import type { FleetState } from './fleetReducer'

export interface FleetContextValue {
  state: FleetState
  /** POST /api/tasks — Backend 응답을 받은 뒤에만 Task 를 상태에 추가한다 */
  createServeTask: (destinationId: string) => Promise<Task>
  cancelTask: (taskId: string) => Promise<void>
  refreshTasks: () => Promise<void>
  reconnectSocket: () => void
}

export const FleetContext = createContext<FleetContextValue | null>(null)

export function useFleet(): FleetContextValue {
  const value = useContext(FleetContext)
  if (!value) throw new Error('useFleet 는 FleetProvider 안에서 사용해야 합니다.')
  return value
}
