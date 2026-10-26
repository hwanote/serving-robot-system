import { useCallback, useRef, useState } from 'react'
import { toApiError } from '../api/client'
import { postGeneratedGraph } from '../api/liveMapApi'
import type { SemanticMap } from '../types/contracts'
import type { GeneratedGraph } from '../types/provisional'
import { serializeDoc } from '../utils/semanticEditor'

/**
 * 자동 생성 통로 미리보기.
 * - check(doc): 지금 편집 중인 문서(저장 전이어도 됨)로 서버에 계산을 시킨다. 아무것도 저장하지 않는다.
 * - 편집 중인 문서(docKey)가 계산했을 때와 달라지면 stale(옛 결과)
 */
export function useGeneratedGraph(storeId: string, docKey: string) {
  const [graph, setGraph] = useState<GeneratedGraph | null>(null)
  const [fetchedKey, setFetchedKey] = useState<string | null>(null)
  const [loading, setLoading] = useState(false)
  const [error, setError] = useState<string | null>(null)
  const seq = useRef(0)

  const check = useCallback(
    async (doc: SemanticMap) => {
      const mine = ++seq.current
      setLoading(true)
      setError(null)
      try {
        const g = await postGeneratedGraph(storeId, doc)
        if (mine !== seq.current) return
        setGraph(g)
        setFetchedKey(serializeDoc(doc))
      } catch (e) {
        if (mine === seq.current) setError(toApiError(e).message)
      } finally {
        if (mine === seq.current) setLoading(false)
      }
    },
    [storeId],
  )

  const clear = useCallback(() => {
    seq.current++
    setGraph(null)
    setError(null)
    setLoading(false)
  }, [])

  const stale = graph !== null && fetchedKey !== docKey
  return { graph, loading, error, stale, check, clear }
}
