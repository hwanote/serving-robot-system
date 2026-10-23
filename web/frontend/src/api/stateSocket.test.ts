import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { FakeSocket } from '../test/FakeSocket'
import { StateSocket, type SocketStatusInfo } from './stateSocket'
import type { StateSocketEvent } from './parsers'

const robotEvent = {
  type: 'robot_state',
  robot_id: 'robot_01',
  state: 'NAVIGATING',
  task_id: 'task_001',
  destination_id: 'table_03',
  battery_percent: 82,
  pose: { x: 1.23, y: -0.52, yaw: 0.7 },
}

describe('StateSocket', () => {
  let events: StateSocketEvent[]
  let statuses: SocketStatusInfo[]
  let invalid: string[]
  let socket: StateSocket

  beforeEach(() => {
    vi.useFakeTimers()
    FakeSocket.instances = []
    events = []
    statuses = []
    invalid = []
    socket = new StateSocket({
      url: 'ws://test/ws/state',
      createSocket: (url) => new FakeSocket(url),
      onEvent: (e) => events.push(e),
      onStatus: (s) => statuses.push(s),
      onInvalidMessage: (r) => invalid.push(r),
      baseDelayMs: 1000,
      maxDelayMs: 4000,
    })
  })
  afterEach(() => {
    socket.stop()
    vi.useRealTimers()
  })

  it('연결 → robot_state 이벤트를 Contract 형식 그대로 전달한다', () => {
    socket.start()
    expect(statuses.at(-1)?.status).toBe('connecting')
    FakeSocket.instances[0].open()
    expect(statuses.at(-1)?.status).toBe('open')
    FakeSocket.instances[0].message(robotEvent)
    expect(events).toEqual([robotEvent])
  })

  it('Contract 에 없는 로봇 상태 문자열은 반영하지 않고 invalid 로 보고한다', () => {
    socket.start()
    FakeSocket.instances[0].open()
    FakeSocket.instances[0].message({ ...robotEvent, state: 'MOVING' })
    FakeSocket.instances[0].message('not json')
    expect(events).toHaveLength(0)
    expect(invalid).toHaveLength(2)
  })

  it('알 수 없는 type 의 이벤트는 조용히 무시한다', () => {
    socket.start()
    FakeSocket.instances[0].open()
    FakeSocket.instances[0].message({ type: 'heartbeat' })
    expect(events).toHaveLength(0)
    expect(invalid).toHaveLength(0)
  })

  it('연결이 끊기면 reconnecting 으로 전환하고 지수 백오프로 재연결한다', () => {
    socket.start()
    FakeSocket.instances[0].open()
    FakeSocket.instances[0].serverClose(1006)
    expect(statuses.at(-1)).toMatchObject({ status: 'reconnecting', attempt: 1 })

    vi.advanceTimersByTime(999)
    expect(FakeSocket.instances).toHaveLength(1)
    vi.advanceTimersByTime(1)
    expect(FakeSocket.instances).toHaveLength(2)

    FakeSocket.instances[1].serverClose(1006)
    expect(statuses.at(-1)).toMatchObject({ status: 'reconnecting', attempt: 2 })
    vi.advanceTimersByTime(2000)
    expect(FakeSocket.instances).toHaveLength(3)

    FakeSocket.instances[2].open()
    expect(statuses.at(-1)).toMatchObject({ status: 'open', attempt: 0 })
  })

  it('stop() 후에는 재연결하지 않는다', () => {
    socket.start()
    FakeSocket.instances[0].open()
    socket.stop()
    expect(FakeSocket.instances[0].closed).toBe(true)
    expect(statuses.at(-1)?.status).toBe('closed')
    vi.advanceTimersByTime(60_000)
    expect(FakeSocket.instances).toHaveLength(1)
  })
})
