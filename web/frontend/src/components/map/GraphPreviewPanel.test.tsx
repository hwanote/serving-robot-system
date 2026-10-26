import { render, screen, waitFor, within } from '@testing-library/react'
import userEvent from '@testing-library/user-event'
import { http, HttpResponse } from 'msw'
import { describe, expect, it, vi } from 'vitest'
import { API_BASE_URL } from '../../config/env'
import { mockDb, seedMockDb } from '../../mocks/db'
import { renderApp } from '../../test/renderApp'
import { server } from '../../test/server'
import type { GeneratedGraph } from '../../types/provisional'
import { DEFAULT_GRAPH_LAYERS } from '../../utils/graph'
import { GraphPreviewPanel } from './GraphPreviewPanel'

const open = async () => {
  seedMockDb()
  const user = userEvent.setup()
  renderApp('/map')
  await screen.findByRole('heading', { name: '자동 생성 통로 미리보기' })
  return user
}
const show = () => screen.getByRole('button', { name: '통로·정차 위치 보기' })
const editTableTop = async (user: ReturnType<typeof userEvent.setup>) => {
  await user.click(await screen.findByRole('button', { name: /Table 3/ }))
  const width = await screen.findByRole('spinbutton', { name: '상단 가로 (mm)' })
  await user.clear(width)
  await user.type(width, '1200{Enter}')
}

describe('자동 생성 통로 미리보기 패널', () => {
  it('처음에는 계산 결과가 없다', async () => {
    await open()
    expect(show()).toBeEnabled()
    expect(screen.queryByText('문제 없음')).not.toBeInTheDocument()
    expect(screen.queryByRole('list', { name: '자동 정차 위치 목록' })).not.toBeInTheDocument()
  })

  it('누르면 결과가 나오고 아무것도 저장하지 않는다', async () => {
    const user = await open()
    const before = JSON.stringify(mockDb.semantics)
    await user.click(show())
    expect(await screen.findByText('문제 없음')).toBeInTheDocument()
    expect(screen.getByRole('list', { name: '자동 정차 위치 목록' })).toBeInTheDocument()
    expect(JSON.stringify(mockDb.semantics)).toBe(before)
    expect(screen.getByRole('button', { name: '다시 계산' })).toBeEnabled()
  })

  it('정차 위치 목록에 어느 쪽에 서서 어디를 보는지 말해 준다', async () => {
    const user = await open()
    await user.click(show())
    const list = await screen.findByRole('list', { name: '자동 정차 위치 목록' })
    expect(within(list).getByText('table_01').parentElement).toHaveTextContent('대상의 북쪽에 서서 남쪽을 바라봄')
    expect(within(list).getByText('robot_wait_01').parentElement).toHaveTextContent('동쪽을 바라봄')
  })

  it('표시 항목 체크박스를 켜고 끌 수 있다', async () => {
    const user = await open()
    await user.click(show())
    const lengths = await screen.findByRole('checkbox', { name: '길이(m)' })
    expect(lengths).toBeChecked()
    await user.click(lengths)
    expect(lengths).not.toBeChecked()
    expect(screen.getByRole('checkbox', { name: '통로' })).toBeChecked()
    expect(screen.getByRole('checkbox', { name: '내 Waypoint' })).toBeChecked()
  })

  it('지우기 버튼으로 결과를 걷어낸다', async () => {
    const user = await open()
    await user.click(show())
    await screen.findByText('문제 없음')
    await user.click(screen.getByRole('button', { name: '미리보기 지우기' }))
    expect(screen.queryByText('문제 없음')).not.toBeInTheDocument()
    expect(show()).toBeInTheDocument()
  })

  it('저장하기 전의 변경도 바로 계산하고, 계산한 뒤에 또 바꾸면 "옛 결과"로 표시한다', async () => {
    const user = await open()
    await editTableTop(user)
    expect(screen.getByRole('button', { name: 'Save Store Setup' })).toBeEnabled() // 아직 저장 안 함
    expect(show()).toBeEnabled()
    await user.click(show())
    expect(await screen.findByText('문제 없음')).toBeInTheDocument()
    expect(screen.queryByText(/옛 결과/)).not.toBeInTheDocument()
    expect(mockDb.semantics?.entities.find((e) => e.id === 'table_03')?.top_size_m).toEqual({ x: 0.8, y: 0.8 }) // 저장되지 않음

    const width = await screen.findByRole('spinbutton', { name: '상단 가로 (mm)' })
    await user.clear(width)
    await user.type(width, '900{Enter}')
    expect(await screen.findByText(/옛 결과/)).toBeInTheDocument()
    await user.click(screen.getByRole('button', { name: '다시 계산' }))
    await waitFor(() => expect(screen.queryByText(/옛 결과/)).not.toBeInTheDocument())
  })

  it('서버가 통로를 만들 수 없다고 하면 그 메시지를 보여주고 결과는 비워 둔다', async () => {
    const user = await open()
    server.use(
      http.post(`${API_BASE_URL}/api/stores/:storeId/semantics/graph`, () =>
        HttpResponse.json({ code: 'GENERATION_FAILED', message: '자동 노드 생성에 실패했어요: 통로를 만들 Waypoint가 없어요.' }, { status: 422 }),
      ),
    )
    await user.click(show())
    expect(await screen.findByRole('alert')).toHaveTextContent(/Waypoint가 없어요/)
    expect(screen.queryByText('문제 없음')).not.toBeInTheDocument()
  })
})

describe('정차 방향 지정 · 되돌리기', () => {
  it('방향 버튼을 누르면 그 자리를 수동으로 고정하고 바로 다시 검사한다', async () => {
    const user = await open()
    await user.click(show())
    await screen.findByText('문제 없음')
    const list = screen.getByRole('list', { name: '자동 정차 위치 목록' })
    const row = within(list).getByText('table_01').closest('li')!
    expect(within(row).queryByText('수동')).not.toBeInTheDocument()
    expect(within(row).queryByRole('button', { name: /자동으로 되돌리기/ })).not.toBeInTheDocument()

    await user.click(within(row).getByRole('button', { name: 'table_01 방향 동쪽' }))
    await waitFor(() => expect(within(screen.getByRole('list', { name: '자동 정차 위치 목록' })).getByText('수동')).toBeInTheDocument())
    const row2 = within(screen.getByRole('list', { name: '자동 정차 위치 목록' })).getByText('table_01').closest('li')!
    expect(row2).toHaveTextContent('대상의 서쪽에 서서 동쪽을 바라봄')
    expect(within(row2).getByRole('button', { name: 'table_01 방향 동쪽' })).toHaveAttribute('aria-pressed', 'true')
    expect(screen.queryByText(/옛 결과/)).not.toBeInTheDocument() // 방금 검사한 결과
    expect(screen.getByRole('button', { name: 'Save Store Setup' })).toBeEnabled() // 편집 내용으로는 저장 대기

    await user.click(screen.getByRole('button', { name: 'Save Store Setup' }))
    await waitFor(() => expect(mockDb.semantics?.waypoints.find((w) => w.entity_id === 'table_01')).toMatchObject({ manual: true }))
    expect(mockDb.semantics?.waypoints.find((w) => w.entity_id === 'table_02')).not.toHaveProperty('manual') // 다른 것은 그대로
  })

  it('자동으로 되돌리면 수동 표식이 사라진다', async () => {
    const user = await open()
    await user.click(show())
    await screen.findByText('문제 없음')
    await user.click(screen.getByRole('button', { name: 'table_01 방향 동쪽' }))
    await user.click(await screen.findByRole('button', { name: 'table_01 자동으로 되돌리기' }))
    await waitFor(() => expect(screen.queryByText('수동')).not.toBeInTheDocument())
    expect(screen.queryByRole('button', { name: 'table_01 자동으로 되돌리기' })).not.toBeInTheDocument()
  })
})

describe('지정이 거절된 정차 위치 (패널만)', () => {
  const graph: GeneratedGraph = {
    source: 'draft',
    draft_sha: 's',
    matches_applied: false,
    ok: false,
    nodes: [],
    edges: [],
    stops: [],
    excluded: [{ id: 'table_01', entity_id: 'table_01', reason: '지정한 위치는 앞 간격이 기준을 벗어나요.', x: 1, y: 2, pin: { x: 3, y: 1, yaw_deg: 90 } }],
    problems: ['· table_01: 지정한 위치는 앞 간격이 기준을 벗어나요.'],
    warnings: [],
    notes: [],
    summary: { OK: 0, WARN: 0, FAIL: 0 },
  }
  const draw = () => {
    const h = { onCheck: vi.fn(), onClear: vi.fn(), onLayers: vi.fn(), onSetYaw: vi.fn(), onRevert: vi.fn() }
    render(<GraphPreviewPanel view={{ graph, loading: false, error: null, stale: false }} layers={DEFAULT_GRAPH_LAYERS} {...h} />)
    return h
  }

  it('거절 사유를 보여주고, 지정한 자리 기준으로 방향을 바꾸거나 되돌릴 수 있다', async () => {
    const h = draw()
    const user = userEvent.setup()
    const list = screen.getByRole('list', { name: '지정이 거절된 정차 위치' })
    expect(within(list).getByText('거절')).toBeInTheDocument()
    expect(within(list).getByText(/앞 간격이 기준을 벗어나요/)).toBeInTheDocument()
    expect(within(list).getByRole('button', { name: 'table_01 방향 북쪽' })).toHaveAttribute('aria-pressed', 'true')
    await user.click(within(list).getByRole('button', { name: 'table_01 방향 서쪽' }))
    expect(h.onSetYaw).toHaveBeenCalledWith('table_01', 3, 1, Math.PI)
    await user.click(within(list).getByRole('button', { name: 'table_01 자동으로 되돌리기' }))
    expect(h.onRevert).toHaveBeenCalledWith('table_01')
  })

  it('검사 중에는 버튼을 막는다', () => {
    render(<GraphPreviewPanel view={{ graph, loading: true, error: null, stale: false }} layers={DEFAULT_GRAPH_LAYERS} onCheck={vi.fn()} onClear={vi.fn()} onLayers={vi.fn()} onSetYaw={vi.fn()} onRevert={vi.fn()} />)
    expect(screen.getByRole('button', { name: 'table_01 방향 동쪽' })).toBeDisabled()
    expect(screen.getByRole('button', { name: 'table_01 자동으로 되돌리기' })).toBeDisabled()
  })
})
