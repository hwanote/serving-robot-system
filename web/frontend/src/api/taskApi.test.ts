import { http, HttpResponse } from 'msw'
import { describe, expect, it } from 'vitest'
import { seedMockDb } from '../mocks/db'
import { server } from '../test/server'
import { ApiError } from './client'
import { cancelTask, createServeTask } from './taskApi'

describe('taskApi', () => {
  it('POST /api/tasks 에 { type: SERVE, destination_id } 만 전달한다', async () => {
    let captured: unknown = null
    server.use(
      http.post('/api/tasks', async ({ request }) => {
        captured = await request.json()
        const now = new Date().toISOString()
        return HttpResponse.json(
          { task_id: 'task_100', type: 'SERVE', destination_id: 'table_03', status: 'QUEUED', robot_id: null, created_at: now, updated_at: now },
          { status: 201 },
        )
      }),
    )

    const task = await createServeTask('table_03')

    expect(captured).toEqual({ type: 'SERVE', destination_id: 'table_03' })
    expect(task.status).toBe('QUEUED')
    expect(task.robot_id).toBeNull()
  })

  it('Mock 서버: 유효한 목적지면 QUEUED Task 를 생성한다', async () => {
    seedMockDb()
    const task = await createServeTask('table_03')
    expect(task).toMatchObject({ type: 'SERVE', destination_id: 'table_03', status: 'QUEUED' })
    expect(task.task_id).toMatch(/^task_\d+$/)
  })

  it('Mock 서버: Semantic Map 에 없는 목적지는 거부한다', async () => {
    seedMockDb()
    await expect(createServeTask('table_99')).rejects.toBeInstanceOf(ApiError)
  })

  it('Task 취소 후 CANCELED 상태를 반환하고, 재취소는 409 오류', async () => {
    seedMockDb()
    const task = await createServeTask('table_01')
    const canceled = await cancelTask(task.task_id)
    expect(canceled?.status).toBe('CANCELED')
    await expect(cancelTask(task.task_id)).rejects.toMatchObject({ status: 409 })
  })

  it('서버 응답의 Task 상태가 Contract 밖이면 오류로 처리한다', async () => {
    server.use(
      http.post('/api/tasks', () =>
        HttpResponse.json({ task_id: 't', type: 'SERVE', destination_id: 'table_01', status: 'DONE', robot_id: null }),
      ),
    )
    await expect(createServeTask('table_01')).rejects.toThrow(/task.status/)
  })
})
