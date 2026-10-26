import { act, screen, waitFor, within } from '@testing-library/react'
import userEvent from '@testing-library/user-event'
import { http, HttpResponse } from 'msw'
import { afterEach, describe, expect, it, vi } from 'vitest'
import { parseStateEvent } from '../api/parsers'
import { seedMockDb } from '../mocks/db'
import { renderApp } from '../test/renderApp'
import { server } from '../test/server'

const controls = (over: Record<string, unknown> = {}) => ({ reset: true, reset_nodes: ['standby_01', 'kitchen'], home: false, reload_map: false, ...over })

const event = (extra: Record<string, unknown> = {}, id = 'robot_01') => ({
  type: 'robot_state',
  robot_id: id,
  state: 'IDLE',
  task_id: null,
  destination_id: null,
  battery_percent: 90,
  pose: { x: 1, y: 1, yaw: 0 },
  localization: 'OK',
  map_current: true,
  controls: controls(),
  ...extra,
})

async function openPage(first: Record<string, unknown> = {}) {
  seedMockDb()
  const app = renderApp('/robots')
  await waitFor(() => expect(app.socket()).toBeDefined())
  act(() => {
    app.socket().open()
    app.socket().message(event(first))
  })
  return { ...app, card: await screen.findByTestId('robot-card-robot_01') }
}

/** 요청 본문을 모아 둔다 */
function capture(path: string, status = 200, body: Record<string, unknown> = { robot_id: 'robot_01' }) {
  const calls: unknown[] = []
  server.use(
    http.post(path, async ({ request }) => {
      calls.push(await request.text())
      return HttpResponse.json(body, { status })
    }),
  )
  return calls
}

describe('로봇 카드 버튼 (리셋 / 대기 위치로 / 지도 다시 읽기)', () => {
  afterEach(() => vi.restoreAllMocks())

  it('리셋 위치 기본값은 대기 위치이고, 확인 후 POST /reset 에 node 를 보낸다', async () => {
    const calls = capture('/api/robots/robot_01/reset')
    const confirm = vi.spyOn(window, 'confirm').mockReturnValue(true)
    const { card } = await openPage()
    const user = userEvent.setup()
    const select = within(card).getByLabelText('Robot 01 리셋할 위치') as HTMLSelectElement
    expect(select.value).toBe('standby_01')
    expect(within(select).getAllByRole('option').map((o) => o.textContent)).toEqual(['대기 위치', '주방'])

    await user.selectOptions(select, 'kitchen')
    await user.click(within(card).getByRole('button', { name: '리셋' }))
    expect(confirm).toHaveBeenCalledTimes(1)
    expect(confirm.mock.calls[0][0]).toMatch(/직접 올려놓으셨나요/)
    await waitFor(() => expect(calls).toEqual(['{"node":"kitchen"}']))
    expect(await screen.findByText(/위치가 잡히면 '대기 위치로' 버튼/)).toBeInTheDocument()
  })

  it('확인창에서 취소하면 아무것도 보내지 않는다', async () => {
    const calls = capture('/api/robots/robot_01/reset')
    vi.spyOn(window, 'confirm').mockReturnValue(false)
    const { card } = await openPage()
    await userEvent.setup().click(within(card).getByRole('button', { name: '리셋' }))
    await new Promise((r) => setTimeout(r, 50))
    expect(calls).toEqual([])
  })

  it('위치를 잃으면 안내가 뜨고 대기 위치로 버튼은 꺼져 있다 (리셋 먼저)', async () => {
    const { card } = await openPage({ localization: 'LOST', controls: controls({ home: false }) })
    expect(within(card).getByRole('status')).toHaveTextContent(/위치를 모릅니다/)
    const home = within(card).getByRole('button', { name: /대기 위치로/ })
    expect(home).toBeDisabled()
    expect(home).toHaveAttribute('title', expect.stringMatching(/먼저 리셋/))
  })

  it('대기 위치로 버튼이 켜져 있으면 POST /home, 서버가 거절하면 서버 문구를 보여 준다', async () => {
    const ok = capture('/api/robots/robot_01/home')
    const { card, socket } = await openPage({ controls: controls({ home: true }) })
    const user = userEvent.setup()
    await user.click(within(card).getByRole('button', { name: /대기 위치로/ }))
    await waitFor(() => expect(ok).toHaveLength(1))

    server.use(http.post('/api/robots/robot_01/home', () => HttpResponse.json({ code: 'NOT_LOCALIZED', message: '로봇이 자기 위치를 아직 모릅니다. 먼저 리셋해서 위치를 잡아 주세요.' }, { status: 409 })))
    act(() => socket().message(event({ controls: controls({ home: true }) })))
    await user.click(within(card).getByRole('button', { name: /대기 위치로/ }))
    expect(await screen.findByText(/먼저 리셋해서 위치를 잡아 주세요/)).toBeInTheDocument()
  })

  it('지도 상태 표시와 "지도 다시 읽기" 버튼 규칙', async () => {
    const calls = capture('/api/robots/robot_01/reload-map')
    const { card, socket } = await openPage()
    expect(screen.getByTestId('map-badge-robot_01')).toHaveTextContent('최신 지도')
    expect(within(card).getByRole('button', { name: '지도 다시 읽기' })).toBeDisabled()

    act(() => socket().message(event({ map_current: false, controls: controls({ reload_map: true }) })))
    expect(screen.getByTestId('map-badge-robot_01')).toHaveTextContent('옛 지도')
    const button = within(card).getByRole('button', { name: '지도 다시 읽기' })
    expect(button).toBeEnabled()
    await userEvent.setup().click(button)
    await waitFor(() => expect(calls).toHaveLength(1))

    act(() => socket().message(event({ map_current: null, controls: controls() })))
    expect(screen.getByTestId('map-badge-robot_01')).toHaveTextContent('확인 불가')
  })

  it('지도 그림이 바뀐 로봇에는 재시작 안내가 뜨고, 다시 읽기로는 안 된다고 알려 준다', async () => {
    const { card, socket } = await openPage({ map_image_current: true })
    expect(screen.queryByTestId('restart-notice-robot_01')).not.toBeInTheDocument()

    act(() => socket().message(event({ map_image_current: false, map_current: false, controls: controls({ reload_map: true }) })))
    const notice = screen.getByTestId('restart-notice-robot_01')
    expect(notice).toHaveTextContent('로봇을 재시작해 주세요. (새로운 지도가 적용되었습니다.)')
    expect(notice).toHaveTextContent('영업이 끝났거나 주문이 없을 때')
    expect(screen.getByTestId('map-badge-robot_01')).toHaveTextContent('로봇 재시작 필요')
    expect(within(card).getByRole('button', { name: '지도 다시 읽기' })).toBeDisabled()

    act(() => socket().message(event({ map_image_current: true })))
    expect(screen.queryByTestId('restart-notice-robot_01')).not.toBeInTheDocument()

    act(() => socket().message(event({ map_image_current: null })))
    expect(screen.queryByTestId('restart-notice-robot_01')).not.toBeInTheDocument()
  })

  it('"모든 로봇 지도 다시 읽기"는 결과를 로봇별로 요약한다', async () => {
    capture('/api/robots/reload-map', 200, { robots: { robot_01: 'sent', robot_02: 'busy' } })
    await openPage()
    await userEvent.setup().click(screen.getByRole('button', { name: /모든 로봇 지도 다시 읽기/ }))
    expect(await screen.findByText(/Robot 01 다시 읽는 중 · Robot 02 임무 중이라 건너뜀/)).toBeInTheDocument()
  })

  it('실시간 연결이 끊기면 모든 버튼이 꺼진다', async () => {
    const { card, socket } = await openPage({ controls: controls({ home: true, reload_map: true }), map_current: false })
    act(() => socket().serverClose(1006))
    await screen.findByText(/실시간 연결이 끊겼습니다/)
    for (const name of ['리셋', /대기 위치로/, '지도 다시 읽기']) expect(within(card).getByRole('button', { name })).toBeDisabled()
  })

  it('controls 가 없는 서버(옛 버전)면 버튼 줄을 그리지 않는다', async () => {
    const { card } = await openPage({ controls: undefined, localization: undefined, map_current: undefined })
    expect(within(card).queryByRole('button', { name: '리셋' })).not.toBeInTheDocument()
    expect(screen.queryByRole('button', { name: /모든 로봇 지도 다시 읽기/ })).not.toBeInTheDocument()
  })
})

describe('robot_state 추가 필드 파싱', () => {
  it('추가 필드의 모양이 틀려도 상태 이벤트 자체는 받는다 (그 필드만 버림)', () => {
    const e = parseStateEvent(event({ controls: { reset_nodes: 'nope' }, map_current: 'yes', localization: 5 }))
    expect(e && e.type === 'robot_state' && e.state).toBe('IDLE')
    if (e?.type === 'robot_state') {
      expect(e.controls).toBeUndefined()
      expect(e.map_current).toBeUndefined()
      expect(e.localization).toBeUndefined()
    }
  })

  it('정상 값은 그대로 읽는다', () => {
    const e = parseStateEvent(event({ map_current: null }))
    if (e?.type !== 'robot_state') throw new Error('robot_state 아님')
    expect(e.map_current).toBeNull()
    expect(e.controls).toEqual(controls())
  })
})
