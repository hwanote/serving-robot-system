import { createContext, useContext } from 'react'
import type { RawMapInfo, SemanticMap } from '../types/contracts'
import type { MappingStatus } from '../types/provisional'

export interface Resource<T> {
  data: T
  loading: boolean
  /** 마지막 조회 오류 메시지 */
  error: string | null
  /** 한 번이라도 조회를 완료했는지 */
  loaded: boolean
}

export interface StoreDataContextValue {
  storeId: string
  rawMap: Resource<RawMapInfo | null>
  semantics: Resource<SemanticMap | null>
  mappingStatus: Resource<MappingStatus | null>
  reloadRawMap: () => Promise<void>
  reloadSemantics: () => Promise<void>
  reloadMappingStatus: () => Promise<void>
  /** 서버 응답으로 확정된 값을 반영 */
  setSemantics: (doc: SemanticMap) => void
  setMappingStatus: (status: MappingStatus) => void
}

export const StoreDataContext = createContext<StoreDataContextValue | null>(null)

export function useStoreData(): StoreDataContextValue {
  const value = useContext(StoreDataContext)
  if (!value) throw new Error('useStoreData 는 StoreDataProvider 안에서 사용해야 합니다.')
  return value
}
