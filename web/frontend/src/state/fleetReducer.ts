import type { SocketStatusInfo } from '../api/stateSocket'
import type { RobotStateEvent, TaskStatus } from '../types/contracts'
import type { RobotExtras, Task, TaskStateEvent } from '../types/provisional'

export interface RobotView extends RobotStateEvent, RobotExtras {
  /** 마지막 수신 시각 (ms epoch) */
  receivedAt: number
}

export interface FleetState {
  robots: Record<string, RobotView>
  tasks: Record<string, Task>
  socket: SocketStatusInfo
  /** 최초 연결 이후 한 번이라도 open 되었는지 */
  everConnected: boolean
  tasksLoading: boolean
  tasksError: string | null
  /** Contract 에 맞지 않아 무시한 메시지 수 */
  invalidMessages: number
  lastInvalidMessage: string | null
}

export type FleetAction =
  | { type: 'socket_status'; info: SocketStatusInfo }
  | { type: 'robot_state'; event: RobotStateEvent & RobotExtras; receivedAt: number }
  | { type: 'task_state'; event: TaskStateEvent }
  | { type: 'tasks_loading' }
  | { type: 'tasks_loaded'; tasks: Task[] }
  | { type: 'tasks_failed'; message: string }
  | { type: 'task_upsert'; task: Task }
  | { type: 'invalid_message'; reason: string }

export const initialFleetState: FleetState = {
  robots: {},
  tasks: {},
  socket: { status: 'connecting', attempt: 0, nextRetryAt: null, retryDelayMs: null, lastError: null },
  everConnected: false,
  tasksLoading: false,
  tasksError: null,
  invalidMessages: 0,
  lastInvalidMessage: null,
}

/** 같은 Task 의 두 버전 중 최신(updated_at) 을 선택. REST 응답과 WS 이벤트 순서가 뒤바뀌어도 상태가 되돌아가지 않게 한다. */
export function mergeTask(existing: Task | undefined, incoming: Task): Task {
  if (!existing) return incoming
  if (Date.parse(incoming.updated_at) < Date.parse(existing.updated_at)) return existing
  return { ...existing, ...incoming, created_at: existing.created_at }
}

export function fleetReducer(state: FleetState, action: FleetAction): FleetState {
  switch (action.type) {
    case 'socket_status':
      // 연결이 끊겨도 robots 는 그대로 둔다 (IDLE 로 간주하지 않음)
      return {
        ...state,
        socket: action.info,
        everConnected: state.everConnected || action.info.status === 'open',
      }
    case 'robot_state':
      return {
        ...state,
        robots: { ...state.robots, [action.event.robot_id]: { ...action.event, receivedAt: action.receivedAt } },
      }
    case 'task_state': {
      const e = action.event
      const existing = state.tasks[e.task_id]
      const incoming: Task = {
        task_id: e.task_id,
        type: 'SERVE',
        destination_id: e.destination_id,
        status: e.status,
        robot_id: e.robot_id,
        created_at: existing?.created_at ?? e.updated_at,
        updated_at: e.updated_at,
      }
      return { ...state, tasks: { ...state.tasks, [e.task_id]: mergeTask(existing, incoming) } }
    }
    case 'tasks_loading':
      return { ...state, tasksLoading: true, tasksError: null }
    case 'tasks_loaded': {
      const tasks = { ...state.tasks }
      for (const t of action.tasks) tasks[t.task_id] = mergeTask(tasks[t.task_id], t)
      return { ...state, tasks, tasksLoading: false, tasksError: null }
    }
    case 'tasks_failed':
      return { ...state, tasksLoading: false, tasksError: action.message }
    case 'task_upsert':
      return { ...state, tasks: { ...state.tasks, [action.task.task_id]: mergeTask(state.tasks[action.task.task_id], action.task) } }
    case 'invalid_message':
      return { ...state, invalidMessages: state.invalidMessages + 1, lastInvalidMessage: action.reason }
  }
}

// ---------- selectors ----------

export const ACTIVE_TASK_STATUSES: readonly TaskStatus[] = ['QUEUED', 'ASSIGNED', 'RUNNING']
export const FINISHED_TASK_STATUSES: readonly TaskStatus[] = ['SUCCEEDED', 'FAILED', 'CANCELED']

export const isActiveTask = (t: Task) => ACTIVE_TASK_STATUSES.includes(t.status)

export function sortedRobots(state: FleetState): RobotView[] {
  return Object.values(state.robots).sort((a, b) => a.robot_id.localeCompare(b.robot_id))
}

export function activeTasks(state: FleetState): Task[] {
  return Object.values(state.tasks)
    .filter(isActiveTask)
    .sort((a, b) => a.created_at.localeCompare(b.created_at))
}

export function recentFinishedTasks(state: FleetState, limit = 8): Task[] {
  return Object.values(state.tasks)
    .filter((t) => FINISHED_TASK_STATUSES.includes(t.status))
    .sort((a, b) => b.updated_at.localeCompare(a.updated_at))
    .slice(0, limit)
}
