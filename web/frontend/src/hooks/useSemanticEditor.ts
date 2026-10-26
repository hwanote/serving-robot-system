import { useCallback, useEffect, useMemo, useReducer, useRef, useState } from 'react'
import { toApiError } from '../api/client'
import { putSemantics } from '../api/semanticApi'
import type { SemanticMap } from '../types/contracts'
import { useStoreData } from '../state/storeDataContext'
import { useToast } from '../state/toastContext'
import { checkRevision, createEmptySemanticMap, hasErrors, toSemanticPayload, validateSemanticMap } from '../utils/semantic'
import { createEditorState, editorReducer, isDirty, serializeDoc } from '../utils/semanticEditor'

/**
 * Map Setup 편집 상태.
 * - 서버의 Semantic Map 을 초안(draft)으로 복사해 편집하고, Save Store Setup 시 PUT 으로 저장
 * - 저장 전 Contract 검증 + Map revision 일치 확인
 * - 편집 중(dirty)에는 서버 데이터가 다시 로드돼도 초안을 덮어쓰지 않음
 */
export function useSemanticEditor() {
  const { storeId, rawMap, semantics, setSemantics, reloadSemantics } = useStoreData()
  const { notify } = useToast()
  const map = rawMap.data
  const mapRevision = map?.map_revision
  const [state, dispatch] = useReducer(editorReducer, undefined, () => createEditorState(createEmptySemanticMap(storeId, 0)))
  const [saving, setSaving] = useState(false)
  const [saveError, setSaveError] = useState<string | null>(null)
  const savingRef = useRef(false)
  const dirty = isDirty(state)

  // 서버 데이터 → 초안 (편집 중이 아닐 때만)
  const serverDoc = semantics.data
  const ready = rawMap.loaded && semantics.loaded
  const sourceKey = ready ? `${serverDoc ? serializeDoc(serverDoc) : 'none'}|${mapRevision ?? 'nomap'}` : null
  const loadedKey = useRef<string | null>(null)
  useEffect(() => {
    if (!sourceKey || sourceKey === loadedKey.current || dirty) return
    loadedKey.current = sourceKey
    // Semantic 이 아직 없으면 현재 지도 revision 기준의 빈 문서로 시작
    dispatch({ type: 'load', doc: serverDoc ?? createEmptySemanticMap(storeId, mapRevision ?? 0) })
  }, [sourceKey, dirty, serverDoc, storeId, mapRevision])

  const issues = useMemo(() => validateSemanticMap(state.doc, map), [state.doc, map])
  const revision = useMemo(() => checkRevision(state.doc, map), [state.doc, map])
  const canSave = !saving && dirty && revision.ok && !hasErrors(issues)

  const save = useCallback(async () => {
    if (savingRef.current) return
    if (!revision.ok) {
      setSaveError(revision.message)
      return
    }
    if (hasErrors(issues)) {
      setSaveError('오류를 해결한 뒤 저장하세요.')
      return
    }
    savingRef.current = true
    setSaving(true)
    setSaveError(null)
    try {
      const saved = await putSemantics(toSemanticPayload(state.doc))
      loadedKey.current = `${serializeDoc(saved)}|${mapRevision ?? 'nomap'}`
      dispatch({ type: 'mark_saved', doc: saved })
      setSemantics(saved)
      notify('매장 설정을 저장했습니다.', 'success')
    } catch (e) {
      const message = toApiError(e).message
      setSaveError(message)
      notify(`저장 실패: ${message}`, 'error')
    } finally {
      savingRef.current = false
      setSaving(false)
    }
  }, [revision, issues, state.doc, mapRevision, setSemantics, notify])

  /** 편집 내용을 버리고 서버 상태로 되돌림 */
  const discard = useCallback(async () => {
    loadedKey.current = null
    dispatch({ type: 'load', doc: serverDoc ?? createEmptySemanticMap(storeId, mapRevision ?? 0) })
    setSaveError(null)
    await reloadSemantics()
  }, [serverDoc, storeId, mapRevision, reloadSemantics])

  /** 서버가 저장본을 바꿔 놓았을 때(지난 지도 복구): 편집 중인 내용을 버리고 그 문서로 바꾼다 */
  const replaceDraft = useCallback(
    (doc: SemanticMap) => {
      loadedKey.current = `${serializeDoc(doc)}|${mapRevision ?? 'nomap'}`
      dispatch({ type: 'load', doc })
      setSemantics(doc)
      setSaveError(null)
    },
    [mapRevision, setSemantics],
  )

  const rebase = useCallback(() => {
    if (map) dispatch({ type: 'rebase_revision', revision: map.map_revision })
  }, [map])

  return { state, dispatch, dirty, issues, revision, canSave, saving, saveError, save, discard, replaceDraft, rebase, map }
}
