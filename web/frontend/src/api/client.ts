import axios, { AxiosError } from 'axios'
import { API_BASE_URL } from '../config/env'

export const apiClient = axios.create({
  baseURL: API_BASE_URL,
  timeout: 10_000,
  headers: { 'Content-Type': 'application/json' },
})

/** UI에서 그대로 표시할 수 있도록 정규화한 API 오류 */
export class ApiError extends Error {
  readonly status: number | null
  readonly code: string | null

  constructor(message: string, status: number | null, code: string | null = null) {
    super(message)
    this.name = 'ApiError'
    this.status = status
    this.code = code
  }

  get isNotFound() {
    return this.status === 404
  }
}

function serverMessage(data: unknown): string | null {
  if (data && typeof data === 'object') {
    const record = data as Record<string, unknown>
    for (const key of ['message', 'detail', 'error']) {
      if (typeof record[key] === 'string') return record[key]
    }
  }
  return null
}

export function toApiError(error: unknown): ApiError {
  if (error instanceof ApiError) return error
  if (error instanceof AxiosError) {
    const status = error.response?.status ?? null
    const fromServer = serverMessage(error.response?.data)
    if (status === null) {
      const timeout = error.code === 'ECONNABORTED' || error.code === 'ETIMEDOUT'
      return new ApiError(
        timeout ? '서버 응답 시간이 초과되었습니다.' : '서버에 연결할 수 없습니다. 네트워크 또는 Backend 상태를 확인하세요.',
        null,
        error.code ?? null,
      )
    }
    return new ApiError(fromServer ?? `요청이 실패했습니다. (HTTP ${status})`, status, error.code ?? null)
  }
  if (error instanceof Error) return new ApiError(error.message, null)
  return new ApiError('알 수 없는 오류가 발생했습니다.', null)
}

/** axios 호출을 감싸 응답 data만 반환하고 오류는 ApiError로 변환한다. */
export async function request<T>(promise: Promise<{ data: T }>): Promise<T> {
  try {
    const response = await promise
    return response.data
  } catch (error) {
    throw toApiError(error)
  }
}
