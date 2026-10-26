import type { SemanticMap } from '../types/contracts'
import { toSemanticPayload } from '../utils/semantic'
import type { GeneratedGraph, LiveChangeResult, LiveStatus, MapBackup, SemanticStatus } from '../types/provisional'
import { apiClient, request } from './client'
import { parseBackups, parseGeneratedGraph, parseLiveChange, parseLiveStatus, parseSemanticStatus } from './parsers'

const storePath = (storeId: string) => `/api/stores/${encodeURIComponent(storeId)}/semantics`

/** GET .../semantics/status — 초안·적용 결과 상태 */
export async function getSemanticStatus(storeId: string): Promise<SemanticStatus> {
  return parseSemanticStatus(await request(apiClient.get<unknown>(`${storePath(storeId)}/status`)))
}

/** POST .../semantics/apply — 저장한 초안으로 노드·통로 자동 생성 (로봇 지도는 바뀌지 않음) */
export async function applySemantics(storeId: string): Promise<void> {
  await request(apiClient.post<unknown>(`${storePath(storeId)}/apply`, {}, { timeout: 60_000 }))
}

/** GET .../semantics/graph — 저장한 초안으로 자동 생성기가 만들 통로·정차 위치 (아무것도 저장하지 않음) */
export async function getGeneratedGraph(storeId: string): Promise<GeneratedGraph> {
  return parseGeneratedGraph(await request(apiClient.get<unknown>(`${storePath(storeId)}/graph`, { timeout: 60_000 })))
}

/** POST .../semantics/graph — 저장하지 않은 편집 내용으로 같은 계산 (손으로 옮긴 정차 위치를 놓자마자 검사). 아무것도 저장하지 않음 */
export async function postGeneratedGraph(storeId: string, doc: SemanticMap): Promise<GeneratedGraph> {
  return parseGeneratedGraph(await request(apiClient.post<unknown>(`${storePath(storeId)}/graph`, toSemanticPayload(doc), { timeout: 60_000 })))
}

/** GET .../semantics/live — 로봇이 읽는 지도 상태 */
export async function getLiveStatus(storeId: string): Promise<LiveStatus> {
  return parseLiveStatus(await request(apiClient.get<unknown>(`${storePath(storeId)}/live`)))
}

/** POST .../semantics/promote — 적용 결과를 로봇 지도(config/common)에 반영. 바꾸기 전 지도는 백업 */
export async function promoteMap(storeId: string): Promise<LiveChangeResult> {
  return parseLiveChange(await request(apiClient.post<unknown>(`${storePath(storeId)}/promote`, {})))
}

/** GET .../semantics/backups — 최신순 */
export async function listBackups(storeId: string): Promise<MapBackup[]> {
  return parseBackups(await request(apiClient.get<unknown>(`${storePath(storeId)}/backups`)))
}

/** POST .../semantics/backups/{id}/restore */
export async function restoreBackup(storeId: string, backupId: string): Promise<LiveChangeResult> {
  return parseLiveChange(await request(apiClient.post<unknown>(`${storePath(storeId)}/backups/${encodeURIComponent(backupId)}/restore`, {})))
}
