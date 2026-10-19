import { screen, waitFor } from '@testing-library/react'
import userEvent from '@testing-library/user-event'
import { describe, expect, it } from 'vitest'
import { mockDb, seedMockDb } from '../mocks/db'
import { renderApp } from '../test/renderApp'

describe('Map Setup 화면', () => {
  it('저장된 지도가 없으면 Empty 상태와 매핑 안내를 표시한다', async () => {
    renderApp('/map')
    expect(await screen.findByText('저장된 지도가 없습니다.')).toBeInTheDocument()
    expect(screen.getByRole('button', { name: 'Start Mapping' })).toBeEnabled()
    expect(screen.getByRole('button', { name: 'Save Map' })).toBeDisabled()
  })

  it('Start → Stop → Save Map 흐름으로 Raw Map 이 표시된다', async () => {
    const user = userEvent.setup()
    renderApp('/map')
    await user.click(await screen.findByRole('button', { name: 'Start Mapping' }))
    expect(await screen.findByTestId('mapping-phase')).toHaveTextContent('매핑 진행 중')
    await user.click(screen.getByRole('button', { name: 'Stop Mapping' }))
    await waitFor(() => expect(screen.getByTestId('mapping-phase')).toHaveTextContent('중지됨'))
    await user.click(screen.getByRole('button', { name: 'Save Map' }))

    expect(await screen.findByTestId('map-svg')).toBeInTheDocument()
    expect(mockDb.rawMap?.map_revision).toBe(1)
    expect(screen.getByText('map r1')).toBeInTheDocument()
  })

  it('매핑 API 실패 시 오류를 표시하고 상태를 바꾸지 않는다', async () => {
    mockDb.failures.mapping = true
    const user = userEvent.setup()
    renderApp('/map')
    await user.click(await screen.findByRole('button', { name: 'Start Mapping' }))
    expect((await screen.findAllByText(/mapping 실패 시뮬레이션/)).length).toBeGreaterThan(0)
    expect(screen.getByTestId('mapping-phase')).toHaveTextContent('대기')
  })

  it('지도 조회 실패 시 오류 상태와 다시 시도를 제공한다', async () => {
    seedMockDb()
    mockDb.failures.map_load = true
    renderApp('/map')
    expect(await screen.findByText('데이터를 불러오지 못했습니다.')).toBeInTheDocument()
    expect(screen.getAllByRole('button', { name: /다시 시도/ }).length).toBeGreaterThan(0)
  })

  it('저장된 Semantic Map 을 불러와 객체 목록에 복원한다', async () => {
    seedMockDb()
    renderApp('/map')
    expect(await screen.findByRole('button', { name: /Table 3/ })).toBeInTheDocument()
    expect(screen.getByText('zone_no_go_01')).toBeInTheDocument()
    expect(screen.getByText('저장됨')).toBeInTheDocument()
  })

  it('지도 revision 이 바뀌면 "갱신 필요" 로 표시하고 저장을 막는다', async () => {
    seedMockDb()
    mockDb.rawMap = { ...mockDb.rawMap!, map_revision: mockDb.rawMap!.map_revision + 1 }
    renderApp('/map')
    expect(await screen.findByText('갱신 필요')).toBeInTheDocument()
    expect(screen.queryByText('저장됨')).not.toBeInTheDocument()
    expect(screen.getByText(/지도 revision이 다릅니다/)).toBeInTheDocument()
    expect(screen.getByRole('button', { name: 'Save Store Setup' })).toBeDisabled()
  })
})
