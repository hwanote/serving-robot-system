import type { MappingCommandBody } from '../types/contracts'
import type { MapReviewData, MapSaveBody, MapSaveResult, MappingStatus } from '../types/provisional'
import { apiClient, request } from './client'

export type MappingCommand = 'start' | 'stop' | 'discard'

/**
 * POST /api/mapping/{start|stop|discard}  body: { store_id }
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

/** Save Map: 보정 저장 + 매장 지도로 반영(채택). 주문이 진행 중이면 반영만 보류되고 adopted=false */
export function saveMap(body: MapSaveBody): Promise<MapSaveResult> {
  return request(apiClient.post<MapSaveResult>('/api/mapping/save', body))
}

/** 저장은 됐지만 반영이 보류된 경우 다시 시도 */
export function adoptSavedMap(): Promise<MapSaveResult> {
  return request(apiClient.post<MapSaveResult>('/api/mapping/adopt', {}))
}

/** 보정 화면 정보 (초안이 준비된 뒤에만) */
export function getMapReview(storeId: string): Promise<MapReviewData> {
  return request(apiClient.get<MapReviewData>(`/api/stores/${encodeURIComponent(storeId)}/map-review`))
}
