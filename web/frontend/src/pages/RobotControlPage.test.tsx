import { act, screen, waitFor, within } from '@testing-library/react'
import userEvent from '@testing-library/user-event'
import { http, HttpResponse } from 'msw'
import { afterEach, describe, expect, it, vi } from 'vitest'
import { mockDb, seedMockDb } from '../mocks/db'
import { renderApp } from '../test/renderApp'
import { server } from '../test/server'

const robotEvent = (state: string, extra: Record<string, unknown> = {}) => ({
  type: 'robot_state',
  robot_id: 'robot_01',
  state,
  task_id: 'task_001',
  destination_id: 'table_03',
  battery_percent: 82,
  pose: { x: 1.23, y: -0.52, yaw: 0.7 },
  ...extra,
})

describe('Robot Control 화면', () => {
  afterEach(() => {
    vi.useRealTimers()
  })

  it('Table 03 선택 → Serve → POST /api/tasks 에 destination_id=table_03 전달, 응답 후 QUEUED 표시', async () => {
    seedMockDb()
    const bodies: unknown[] = []
    server.events.on('request:start', async ({ request }) => {
      if (request.method === 'POST' && new URL(request.url).pathname === '/api/tasks') bodies.push(await request.clone().json())
    })
    const user = userEvent.setup()
    renderApp('/robots')

    const select = await screen.findByLabelText('목적지 테이블')
    // Semantic Map 의 유효 목적지만 선택지로 제공
    const options = within(select).getAllByRole('option').map((o) => o.getAttribute('value'))
    expect(options).toEqual(['', 'table_01', 'table_02', 'table_03'])

    await user.selectOptions(select, 'table_03')
    await user.click(screen.getByRole('button', { name: 'Serve' }))

    const last = await screen.findByTestId('last-task')
    expect(last).toHaveTextContent('task_001')
    expect(last).toHaveTextContent('QUEUED')
    expect(bodies).toEqual([{ type: 'SERVE', destination_id: 'table_03' }])
    server.events.removeAllListeners()
  })

  it('목적지가 없으면 Serve 를 비활성화하고 지도 설정을 안내한다', async () => {
    renderApp('/robots')
    expect(await screen.findByText(/서빙 가능한 테이블이 없습니다/)).toBeInTheDocument()
    expect(screen.queryByRole('button', { name: 'Serve' })).not.toBeInTheDocument()
  })

  it('Task 생성 API 오류 시 오류를 표시하고 Task 를 추가하지 않는다', async () => {
    seedMockDb()
    mockDb.failures.task_create = true
    const user = userEvent.setup()
    renderApp('/robots')
    await user.selectOptions(await screen.findByLabelText('목적지 테이블'), 'table_03')
    await user.click(screen.getByRole('button', { name: 'Serve' }))

    // 패널 인라인 오류(role=alert) + 토스트로 표시
    const alerts = await screen.findAllByRole('alert')
    expect(alerts.some((a) => /task_create 실패 시뮬레이션/.test(a.textContent ?? ''))).toBe(true)
    expect(screen.queryByTestId('last-task')).not.toBeInTheDocument()
    expect(screen.getByText('진행 중인 Task가 없습니다.')).toBeInTheDocument()
  })

  it('WebSocket robot_state 이벤트를 로봇 카드에 반영한다', async () => {
    seedMockDb()
    const { socket } = renderApp('/robots')
    await waitFor(() => expect(socket()).toBeDefined())
    act(() => {
      socket().open()
      socket().message(robotEvent('NAVIGATING'))
    })
    const card = await screen.findByTestId('robot-card-robot_01')
    expect(card).toHaveTextContent('NAVIGATING')
    expect(card).toHaveTextContent('82%')
    expect(card).toHaveTextContent('Table 3')
    expect(card).toHaveTextContent('task_001')

    act(() => socket().message(robotEvent('SERVING')))
    expect(card).toHaveTextContent('SERVING')
  })

  it('WebSocket 연결이 끊기면 경고를 표시하고 로봇 상태를 IDLE 로 바꾸지 않는다', async () => {
    seedMockDb()
    const { socket } = renderApp('/robots')
    await waitFor(() => expect(socket()).toBeDefined())
    act(() => {
      socket().open()
      socket().message(robotEvent('SERVING'))
    })
    expect(await screen.findByText('실시간 연결됨')).toBeInTheDocument()

    act(() => socket().serverClose(1006))

    expect(await screen.findByText(/실시간 연결이 끊겼습니다/)).toBeInTheDocument()
    expect(screen.getByTestId('socket-status')).toHaveTextContent('재연결 중')
    const card = screen.getByTestId('robot-card-robot_01')
    expect(card).toHaveTextContent('SERVING')
    expect(card).not.toHaveTextContent('IDLE')
    expect(card).toHaveTextContent(/연결 끊김/)
  })

  it('Contract 에 없는 상태 문자열은 화면에 반영하지 않는다', async () => {
    seedMockDb()
    const { socket } = renderApp('/robots')
    await waitFor(() => expect(socket()).toBeDefined())
    act(() => {
      socket().open()
      socket().message(robotEvent('IDLE', { task_id: null, destination_id: null }))
      socket().message(robotEvent('DELIVERING'))
    })
    const card = await screen.findByTestId('robot-card-robot_01')
    expect(card).toHaveTextContent('IDLE')
    expect(card).not.toHaveTextContent('DELIVERING')
  })

  it('Task 목록 조회 실패 시 오류와 다시 시도를 제공한다', async () => {
    server.use(http.get('/api/tasks', () => HttpResponse.json({ message: 'boom' }, { status: 500 })))
    renderApp('/robots')
    expect(await screen.findByText('Task 목록을 불러오지 못했습니다.')).toBeInTheDocument()
    expect(screen.getAllByRole('button', { name: /다시 시도/ }).length).toBeGreaterThan(0)
  })

  it('Semantic 과 지도의 revision 이 다르면 서빙 명령을 막고 갱신을 안내한다', async () => {
    seedMockDb()
    mockDb.rawMap = { ...mockDb.rawMap!, map_revision: mockDb.rawMap!.map_revision + 1 } // 재매핑 후 Semantic 미갱신
    renderApp('/robots')
    const warning = await screen.findByTestId('serve-revision-mismatch')
    expect(warning).toHaveTextContent(/현재 지도/)
    expect(screen.queryByLabelText('목적지 테이블')).not.toBeInTheDocument()
    expect(screen.queryByRole('button', { name: 'Serve' })).not.toBeInTheDocument()
  })

  it('재연결 카운트다운은 실제 재시도 대기 시간을 기준으로 표시한다 (마운트 후 시간이 흐른 경우)', async () => {
    vi.useFakeTimers({ toFake: ['Date'] })
    seedMockDb()
    const { socket } = renderApp('/robots')
    await waitFor(() => expect(socket()).toBeDefined())
    act(() => {
      socket().open()
      socket().message(robotEvent('IDLE', { task_id: null, destination_id: null }))
    })
    vi.setSystemTime(Date.now() + 60_000) // 연결된 채로 1분 경과
    act(() => socket().serverClose(1006))
    const banner = await screen.findByText(/실시간 연결이 끊겼습니다/)
    expect(banner.closest('.banner')).toHaveTextContent(/재시도 1회, 1초 후 재연결/)
  })
})
