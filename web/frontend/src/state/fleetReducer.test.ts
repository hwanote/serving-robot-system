import { describe, expect, it } from 'vitest'
import type { RobotStateEvent } from '../types/contracts'
import type { Task } from '../types/provisional'
import { activeTasks, fleetReducer, initialFleetState, recentFinishedTasks } from './fleetReducer'

const robot = (state: RobotStateEvent['state']): RobotStateEvent => ({
  type: 'robot_state',
  robot_id: 'robot_01',
  state,
  task_id: 'task_001',
  destination_id: 'table_03',
  battery_percent: 82,
  pose: { x: 1.23, y: -0.52, yaw: 0.7 },
})

const task = (status: Task['status'], updated_at: string): Task => ({
  task_id: 'task_001',
  type: 'SERVE',
  destination_id: 'table_03',
  status,
  robot_id: null,
  created_at: '2026-01-01T00:00:00.000Z',
  updated_at,
})

describe('fleetReducer', () => {
  it('robot_state 이벤트를 로봇 상태에 반영한다', () => {
    const s = fleetReducer(initialFleetState, { type: 'robot_state', event: robot('NAVIGATING'), receivedAt: 1 })
    expect(s.robots.robot_01).toMatchObject({ state: 'NAVIGATING', battery_percent: 82, destination_id: 'table_03' })
  })

  it('WebSocket 이 끊겨도 로봇 상태를 IDLE 로 바꾸지 않는다', () => {
    let s = fleetReducer(initialFleetState, { type: 'robot_state', event: robot('SERVING'), receivedAt: 1 })
    s = fleetReducer(s, { type: 'socket_status', info: { status: 'reconnecting', attempt: 1, nextRetryAt: null, retryDelayMs: null, lastError: 'x' } })
    expect(s.robots.robot_01.state).toBe('SERVING')
    expect(s.socket.status).toBe('reconnecting')
  })

  it('늦게 도착한 오래된 Task 정보가 최신 상태를 덮어쓰지 않는다', () => {
    let s = fleetReducer(initialFleetState, {
      type: 'task_state',
      event: { type: 'task_state', task_id: 'task_001', status: 'ASSIGNED', robot_id: 'robot_01', destination_id: 'table_03', updated_at: '2026-01-01T00:00:02.000Z' },
    })
    // POST 응답(QUEUED)이 WS 이벤트보다 늦게 도착한 경우
    s = fleetReducer(s, { type: 'task_upsert', task: task('QUEUED', '2026-01-01T00:00:01.000Z') })
    expect(s.tasks.task_001.status).toBe('ASSIGNED')
    expect(s.tasks.task_001.robot_id).toBe('robot_01')
  })

  it('진행 중 / 완료 Task 를 구분한다', () => {
    let s = fleetReducer(initialFleetState, { type: 'task_upsert', task: task('RUNNING', '2026-01-01T00:00:01.000Z') })
    expect(activeTasks(s)).toHaveLength(1)
    s = fleetReducer(s, { type: 'task_upsert', task: task('SUCCEEDED', '2026-01-01T00:00:05.000Z') })
    expect(activeTasks(s)).toHaveLength(0)
    expect(recentFinishedTasks(s)[0].status).toBe('SUCCEEDED')
  })
})
