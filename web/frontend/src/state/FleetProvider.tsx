import { useCallback, useEffect, useMemo, useReducer, useRef, type ReactNode } from 'react'
import { toApiError } from '../api/client'
import { StateSocket, type MinimalWebSocket } from '../api/stateSocket'
import * as taskApi from '../api/taskApi'
import { resolveStateSocketUrl } from '../config/env'
import { FleetContext, type FleetContextValue } from './fleetContext'
import { fleetReducer, initialFleetState } from './fleetReducer'

interface Props {
  children: ReactNode
  /** 테스트용 WebSocket 주입 */
  createSocket?: (url: string) => MinimalWebSocket
  socketUrl?: string
}

export function FleetProvider({ children, createSocket, socketUrl }: Props) {
  const [state, dispatch] = useReducer(fleetReducer, initialFleetState)
  const socketRef = useRef<StateSocket | null>(null)

  useEffect(() => {
    const socket = new StateSocket({
      url: socketUrl ?? resolveStateSocketUrl(),
      createSocket,
      onEvent: (event) => {
        if (event.type === 'robot_state') dispatch({ type: 'robot_state', event, receivedAt: Date.now() })
        else dispatch({ type: 'task_state', event })
      },
      onStatus: (info) => dispatch({ type: 'socket_status', info }),
      onInvalidMessage: (reason) => dispatch({ type: 'invalid_message', reason }),
    })
    socketRef.current = socket
    socket.start()
    return () => {
      socket.stop()
      socketRef.current = null
    }
  }, [createSocket, socketUrl])

  const refreshTasks = useCallback(async () => {
    dispatch({ type: 'tasks_loading' })
    try {
      dispatch({ type: 'tasks_loaded', tasks: await taskApi.listTasks() })
    } catch (error) {
      dispatch({ type: 'tasks_failed', message: toApiError(error).message })
    }
  }, [])

  useEffect(() => {
    void refreshTasks()
  }, [refreshTasks])

  // 실시간 연결이 복구되면 끊긴 동안 놓친 Task 변경을 다시 조회
  const prevStatus = useRef(state.socket.status)
  useEffect(() => {
    if (prevStatus.current === 'reconnecting' && state.socket.status === 'open') void refreshTasks()
    prevStatus.current = state.socket.status
  }, [state.socket.status, refreshTasks])

  const createServeTask = useCallback(async (destinationId: string) => {
    const task = await taskApi.createServeTask(destinationId)
    dispatch({ type: 'task_upsert', task })
    return task
  }, [])

  const cancelTask = useCallback(async (taskId: string) => {
    const task = await taskApi.cancelTask(taskId)
    if (task) dispatch({ type: 'task_upsert', task })
  }, [])

  const reconnectSocket = useCallback(() => socketRef.current?.reconnectNow(), [])

  const value = useMemo<FleetContextValue>(
    () => ({ state, createServeTask, cancelTask, refreshTasks, reconnectSocket }),
    [state, createServeTask, cancelTask, refreshTasks, reconnectSocket],
  )

  return <FleetContext.Provider value={value}>{children}</FleetContext.Provider>
}
