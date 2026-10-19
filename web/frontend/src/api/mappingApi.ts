import type { MappingCommandBody } from '../types/contracts'
import type { MappingStatus } from '../types/provisional'
import { apiClient, request } from './client'

export type MappingCommand = 'start' | 'stop' | 'save'

/**
 * POST /api/mapping/{start|stop|save}  body: { store_id }
 * 응답 형식은 임시 계약(MappingStatus). Backend 확정 시 여기서만 변환한다.
 */
export function sendMappingCommand(command: MappingCommand, storeId: string): Promise<MappingStatus> {
  const body: MappingCommandBody = { store_id: storeId }
  return request(apiClient.post<MappingStatus>(`/api/mapping/${command}`, body))
}

/** [임시 계약] GET /api/mapping/status?store_id= */
export function getMappingStatus(storeId: string): Promise<MappingStatus> {
  return request(apiClient.get<MappingStatus>('/api/mapping/status', { params: { store_id: storeId } }))
}
