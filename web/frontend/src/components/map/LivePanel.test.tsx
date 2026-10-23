import { screen, waitFor, within } from '@testing-library/react'
import userEvent from '@testing-library/user-event'
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { mockDb, seedMockDb } from '../../mocks/db'
import { renderApp } from '../../test/renderApp'
import { formatBackupId, formatBackupTime } from '../../utils/backup'

const openPanel = async () => {
  seedMockDb()
  const user = userEvent.setup()
  renderApp('/map')
  await screen.findByRole('heading', { name: '로봇 지도 반영' })
  return user
}
const button = (name: string | RegExp) => screen.getByRole('button', { name })
const openList = async (user: ReturnType<typeof userEvent.setup>) => {
  const toggle = screen.getByRole('button', { name: /지난 지도/ })
  if (toggle.getAttribute('aria-expanded') !== 'true') await user.click(toggle)
}

describe('백업 id 표시', () => {
  it('날짜·시간으로 읽기 좋게 바꾸고, 모르는 형태는 그대로 둔다', () => {
    expect(formatBackupId('20261023-153012')).toBe('2026-10-23 15:30:12')
    expect(formatBackupId('20261023-153012-2')).toBe('2026-10-23 15:30:12 (2)')
    expect(formatBackupId('weird')).toBe('weird')
  })

  it('표시 시각은 created_at 을 PC 시간대로 바꾼 값이고, 같은 초 백업은 번호가 붙는다', () => {
    const t = new Date(2026, 9, 23, 15, 30, 12).toISOString() // 로컬 시간 기준으로 만든 시각
    expect(formatBackupTime({ id: '20261023-063012', created_at: t })).toBe('2026-10-23 15:30:12')
    expect(formatBackupTime({ id: '20261023-063012-2', created_at: t })).toBe('2026-10-23 15:30:12 (2)')
    expect(formatBackupTime({ id: '20261023-063012', created_at: 'bad' })).toBe('2026-10-23 06:30:12')
  })
})

describe('로봇 지도 반영 패널', () => {
  beforeEach(() => {
    vi.spyOn(window, 'confirm').mockReturnValue(true)
  })
  afterEach(() => vi.restoreAllMocks())

  it('처음에는 적용 전이라 "로봇에 반영"이 막혀 있다', async () => {
    const user = await openPanel()
    expect(await screen.findByText('아직 안 함')).toBeInTheDocument()
    expect(button('로봇에 반영')).toBeDisabled()
    expect(button('노드 자동 생성')).toBeEnabled()
    expect(screen.queryByRole('list', { name: '지도 백업 목록' })).not.toBeInTheDocument() // 처음엔 접혀 있음
    await user.click(screen.getByRole('button', { name: /지난 지도 \(0\)/ }))
    expect(screen.getByText(/아직 백업이 없어요/)).toBeInTheDocument()
  })

  it('노드 자동 생성 → 로봇에 반영 → 백업이 쌓이고 재시작 안내가 나온다', async () => {
    const user = await openPanel()
    await user.click(await screen.findByRole('button', { name: '노드 자동 생성' }))
    expect(await screen.findByText(/완료 · OK 21/)).toBeInTheDocument()
    await user.click(button('로봇에 반영'))
    expect(await screen.findByText(/로봇 노드를 다시 켜야/)).toBeInTheDocument()
    expect(window.confirm).toHaveBeenCalled()
    await openList(user)
    const list = await screen.findByRole('list', { name: '지도 백업 목록' })
    expect(within(list).getAllByRole('listitem')).toHaveLength(1)
    expect(within(list).getByText(/반영 직전 지도/)).toBeInTheDocument()
    expect(screen.getByText('반영됨')).toBeInTheDocument()
    expect(button('로봇에 반영')).toBeDisabled()
  })

  it('저장하지 않은 변경이 있으면 적용·반영을 막는다', async () => {
    const user = await openPanel()
    await user.click(await screen.findByRole('button', { name: /Table 3/ }))
    await user.click(screen.getByRole('button', { name: /객체 삭제/ }))
    expect(await screen.findByText('변경 저장 필요')).toBeInTheDocument()
    expect(button('노드 자동 생성')).toBeDisabled()
    expect(button('로봇에 반영')).toBeDisabled()
  })

  it('주문이 진행 중이면 서버 거절 문구를 그대로 보여 준다', async () => {
    const user = await openPanel()
    await user.click(await screen.findByRole('button', { name: '노드 자동 생성' }))
    await screen.findByText(/완료 · OK 21/)
    mockDb.tasks.push({
      task_id: 't1', type: 'SERVE', destination_id: 'table_01', status: 'RUNNING', robot_id: 'robot_01',
      created_at: new Date().toISOString(), updated_at: new Date().toISOString(),
    })
    await user.click(button('로봇에 반영'))
    expect(await screen.findByText(/주문이 진행 중이거나 대기 중이라/)).toBeInTheDocument()
    expect(screen.queryByText(/로봇 노드를 다시 켜야/)).not.toBeInTheDocument()
  })

  it('여러 백업 중 오래된 것을 골라 복구하고, 복구 전 지도도 백업으로 남는다', async () => {
    const user = await openPanel()
    // 첫 반영
    await user.click(await screen.findByRole('button', { name: '노드 자동 생성' }))
    await screen.findByText(/완료 · OK 21/)
    await user.click(button('로봇에 반영'))
    await screen.findByText(/로봇 노드를 다시 켜야/)
    await openList(user)
    await screen.findByRole('list', { name: '지도 백업 목록' })
    // 초안을 바꿔(테이블 삭제) 다시 반영 → 백업 2개
    await user.click(await screen.findByRole('button', { name: /Table 3/ }))
    await user.click(screen.getByRole('button', { name: /객체 삭제/ }))
    await user.click(button('Save Store Setup'))
    await waitFor(() => expect(button('노드 자동 생성')).toBeEnabled())
    await user.click(button('노드 자동 생성'))
    await waitFor(() => expect(button('로봇에 반영')).toBeEnabled())
    await user.click(button('로봇에 반영'))
    await waitFor(() => expect(within(screen.getByRole('list', { name: '지도 백업 목록' })).getAllByRole('listitem')).toHaveLength(2))
    // 가장 오래된(맨 아래) 백업 복구
    const items = within(screen.getByRole('list', { name: '지도 백업 목록' })).getAllByRole('listitem')
    await user.click(within(items[items.length - 1]!).getByRole('button', { name: /백업으로 복구/ }))
    await waitFor(() => expect(within(screen.getByRole('list', { name: '지도 백업 목록' })).getAllByRole('listitem')).toHaveLength(3))
    expect(screen.getByText(/복구 직전 지도/)).toBeInTheDocument()
    expect(screen.getByText(/백업으로 되돌렸어요\. 로봇 노드를 다시 켜야/)).toBeInTheDocument()
  })

  it('확인창에서 취소하면 아무것도 바뀌지 않는다', async () => {
    const user = await openPanel()
    await user.click(await screen.findByRole('button', { name: '노드 자동 생성' }))
    await screen.findByText(/완료 · OK 21/)
    vi.mocked(window.confirm).mockReturnValue(false)
    await user.click(button('로봇에 반영'))
    await openList(user)
    expect(screen.getByText(/아직 백업이 없어요/)).toBeInTheDocument()
    expect(button('로봇에 반영')).toBeEnabled()
  })

  it('백업이 많아도 목록은 3~4개 높이의 스크롤 상자 안에 있다', async () => {
    const user = await openPanel()
    await user.click(await screen.findByRole('button', { name: '노드 자동 생성' }))
    await screen.findByText(/완료 · OK 21/)
    await user.click(button('로봇에 반영'))
    await screen.findByText(/로봇 노드를 다시 켜야/)
    await openList(user)
    const list = await screen.findByRole('list', { name: '지도 백업 목록' })
    expect(list.style.overflowY).toBe('auto')
    expect(parseInt(list.style.maxHeight, 10)).toBeGreaterThanOrEqual(3 * 48)
    expect(parseInt(list.style.maxHeight, 10)).toBeLessThanOrEqual(4 * 60)
  })
})
