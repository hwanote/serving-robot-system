import { useCallback, useEffect, useMemo, useState, type ReactNode } from 'react'
import { toApiError } from '../api/client'
import { getMappingStatus } from '../api/mappingApi'
import { getRawMap, getSemantics } from '../api/semanticApi'
import { STORE_ID } from '../config/env'
import type { RawMapInfo, SemanticMap } from '../types/contracts'
import type { MappingStatus } from '../types/provisional'
import { StoreDataContext, type Resource, type StoreDataContextValue } from './storeDataContext'

const initial = <T,>(data: T): Resource<T> => ({ data, loading: false, error: null, loaded: false })

function useResource<T>(fetcher: () => Promise<T>, empty: T) {
  const [resource, setResource] = useState<Resource<T>>(() => initial(empty))
  const reload = useCallback(async () => {
    setResource((r) => ({ ...r, loading: true, error: null }))
    try {
      const data = await fetcher()
      setResource({ data, loading: false, error: null, loaded: true })
    } catch (error) {
      // 실패 시 마지막으로 받은 데이터는 유지
      setResource((r) => ({ ...r, loading: false, error: toApiError(error).message, loaded: true }))
    }
  }, [fetcher])
  const set = useCallback((data: T) => setResource({ data, loading: false, error: null, loaded: true }), [])
  return [resource, reload, set] as const
}

export function StoreDataProvider({ children, storeId = STORE_ID }: { children: ReactNode; storeId?: string }) {
  const fetchMap = useCallback(() => getRawMap(storeId), [storeId])
  const fetchSemantics = useCallback(() => getSemantics(storeId), [storeId])
  const fetchMapping = useCallback(() => getMappingStatus(storeId), [storeId])

  const [rawMap, reloadRawMap] = useResource<RawMapInfo | null>(fetchMap, null)
  const [semantics, reloadSemantics, setSemantics] = useResource<SemanticMap | null>(fetchSemantics, null)
  const [mappingStatus, reloadMappingStatus, setMappingStatus] = useResource<MappingStatus | null>(fetchMapping, null)

  useEffect(() => {
    void reloadRawMap()
    void reloadSemantics()
    void reloadMappingStatus()
  }, [reloadRawMap, reloadSemantics, reloadMappingStatus])

  const value = useMemo<StoreDataContextValue>(
    () => ({
      storeId,
      rawMap,
      semantics,
      mappingStatus,
      reloadRawMap,
      reloadSemantics,
      reloadMappingStatus,
      setSemantics,
      setMappingStatus,
    }),
    [storeId, rawMap, semantics, mappingStatus, reloadRawMap, reloadSemantics, reloadMappingStatus, setSemantics, setMappingStatus],
  )

  return <StoreDataContext.Provider value={value}>{children}</StoreDataContext.Provider>
}
