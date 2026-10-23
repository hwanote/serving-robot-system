import type { ServeTaskRequest } from '../types/contracts'
import type { Task } from '../types/provisional'
import { apiClient, request } from './client'
import { parseTask } from './parsers'

/**
 * POST /api/tasks  body: { type: 'SERVE', destination_id }
 * 좌표나 robot_id 는 보내지 않는다. 배정/좌표 결정은 Backend(Fleet) 책임.
 */
export async function createServeTask(destinationId: string): Promise<Task> {
  const body: ServeTaskRequest = { type: 'SERVE', destination_id: destinationId }
  return parseTask(await request(apiClient.post<unknown>('/api/tasks', body)))
}

/** POST /api/tasks/{task_id}/cancel */
export async function cancelTask(taskId: string): Promise<Task | null> {
  const data = await request(apiClient.post<unknown>(`/api/tasks/${encodeURIComponent(taskId)}/cancel`))
  return data && typeof data === 'object' && 'task_id' in data ? parseTask(data) : null
}

/** [임시 계약] GET /api/tasks — 현재/최근 Task 목록 */
export async function listTasks(): Promise<Task[]> {
  const data = await request(apiClient.get<unknown>('/api/tasks'))
  const items = Array.isArray(data) ? data : ((data as { tasks?: unknown[] } | null)?.tasks ?? [])
  return items.map(parseTask)
}
