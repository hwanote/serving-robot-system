import { useMemo } from 'react'
import { useStoreData } from '../state/storeDataContext'
import { servableDestinations } from '../utils/semantic'

export interface RevisionMismatch {
  semantic: number
  map: number
}

/**
 * 저장된(서버 확정) Semantic Map 기준 서빙 목적지와 ID → label 조회.
 * Semantic 이 현재 Raw Map 과 revision 이 다르면(재매핑 후 미갱신) 목적지를 제공하지 않는다.
 */
export function useDestinations() {
  const { semantics, rawMap } = useStoreData()
  const doc = semantics.data
  const mapRevision = rawMap.data?.map_revision ?? null
  return useMemo(() => {
    const labels = new Map((doc?.entities ?? []).map((e) => [e.id, e.label || e.id]))
    const revisionMismatch: RevisionMismatch | null =
      doc && mapRevision !== null && doc.map_revision !== mapRevision ? { semantic: doc.map_revision, map: mapRevision } : null
    return {
      destinations: revisionMismatch ? [] : servableDestinations(doc),
      revisionMismatch,
      labelOf: (id: string | null | undefined) => (id ? (labels.get(id) ?? id) : '-'),
      loaded: semantics.loaded && rawMap.loaded,
      error: semantics.error,
    }
  }, [doc, mapRevision, semantics.loaded, rawMap.loaded, semantics.error])
}

/** robot_01 → Robot 01 */
export const robotDisplayName = (robotId: string) => robotId.replace(/^robot_(\d+)$/, 'Robot $1')
