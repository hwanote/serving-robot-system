import { act, screen, waitFor } from '@testing-library/react'
import { describe, expect, it } from 'vitest'
import { mockDb, seedMockDb } from '../mocks/db'
import { renderApp } from '../test/renderApp'

describe('Dashboard 화면', () => {
  it('매장·Map Revision·로봇 수(작업/대기)를 공용 상태로 표시한다', async () => {
    seedMockDb()
    const { socket } = renderApp('/')
    await waitFor(() => expect(socket()).toBeDefined())
    const base = { type: 'robot_state', task_id: null, destination_id: null, battery_percent: 80, pose: { x: 0, y: 0, yaw: 0 } }
    act(() => {
      socket().open()
      socket().message({ ...base, robot_id: 'robot_01', state: 'NAVIGATING', task_id: 'task_001', destination_id: 'table_03' })
      socket().message({ ...base, robot_id: 'robot_02', state: 'IDLE' })
    })
    expect(await screen.findByTestId('stat-revision')).toHaveTextContent(`r${mockDb.rawMap!.map_revision}`)
    expect(screen.getByTestId('stat-robots')).toHaveTextContent('2대')
    expect(screen.getByTestId('stat-working')).toHaveTextContent('1대')
    expect(screen.getByTestId('stat-idle')).toHaveTextContent('1대')
    expect(screen.getByTestId('stat-connection')).toHaveTextContent('실시간 연결됨')
  })

  it('Semantic 과 지도 revision 불일치를 경고한다', async () => {
    seedMockDb()
    mockDb.rawMap = { ...mockDb.rawMap!, map_revision: mockDb.rawMap!.map_revision + 1 }
    renderApp('/')
    expect(await screen.findByTestId('dash-revision-mismatch')).toHaveTextContent('갱신 필요')
  })
})
