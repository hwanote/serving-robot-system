import type { RawMapInfo, SemanticMap } from '../types/contracts'
import { ApiError, apiClient, request } from './client'
import { parseRawMap, parseSemanticMap } from './parsers'

const storePath = (storeId: string) => `/api/stores/${encodeURIComponent(storeId)}`

/** GET /api/stores/{store_id}/map — 저장된 지도가 없으면(404) null */
export async function getRawMap(storeId: string): Promise<RawMapInfo | null> {
  try {
    return parseRawMap(await request(apiClient.get<unknown>(`${storePath(storeId)}/map`)))
  } catch (error) {
    if (error instanceof ApiError && error.isNotFound) return null
    throw error
  }
}

/** GET /api/stores/{store_id}/semantics — 아직 없으면(404) null */
export async function getSemantics(storeId: string): Promise<SemanticMap | null> {
  try {
    return parseSemanticMap(await request(apiClient.get<unknown>(`${storePath(storeId)}/semantics`)))
  } catch (error) {
    if (error instanceof ApiError && error.isNotFound) return null
    throw error
  }
}

/** PUT /api/stores/{store_id}/semantics — 저장 후 서버가 돌려준 문서(없으면 보낸 문서)를 반환 */
export async function putSemantics(semantics: SemanticMap): Promise<SemanticMap> {
  const data = await request(apiClient.put<unknown>(`${storePath(semantics.store_id)}/semantics`, semantics))
  return data && typeof data === 'object' && 'entities' in data ? parseSemanticMap(data) : semantics
}
