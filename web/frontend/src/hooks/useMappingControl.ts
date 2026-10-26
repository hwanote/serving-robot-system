import { useCallback, useEffect, useRef, useState } from 'react'
import { toApiError } from '../api/client'
import { adoptSavedMap, getMappingStatus, saveMap, sendMappingCommand, type MappingCommand } from '../api/mappingApi'
import { useStoreData } from '../state/storeDataContext'
import { RESTART_NOTICE } from '../utils/robotControls'
import { useToast } from '../state/toastContext'
import type { MapSaveBody, MapSaveResult, MappingPhase } from '../types/provisional'

const SUCCESS_MESSAGE: Record<MappingCommand, string> = {
  start: '매핑을 시작했습니다.',
  stop: '매핑을 중지했습니다. 지금까지 만든 지도로 초안을 만듭니다.',
  discard: '임시 지도를 버렸습니다.',
}

const POLL_MS = 2000

export { RESTART_NOTICE }

export interface Adoption {
  /** 새 지도가 매장 지도로 반영되었다 */
  adopted: boolean
  needsRestart: boolean
  /** 저장은 됐지만 반영이 보류된 이유 */
  pendingReason: string | null
}

/**
 * Start / Stop / Discard 매핑과 Save Map(보정 저장 + 매장 지도 반영).
 * - 요청 중에는 모든 매핑 버튼 비활성 (중복 클릭 방지, ref 로 동기 차단)
 * - 상태는 서버 응답으로만 갱신 (ROS 를 직접 호출하지 않음)
 * - 매핑 중에는 2초마다 상태를 다시 읽어 진행 단계를 보여 주고, 스스로 끝난 것을 알아챈다
 * - onAdopted: 새 지도가 반영된 뒤 화면의 지도·편집 데이터를 다시 읽는 일 (페이지가 넘긴다)
 */
export function useMappingControl(options: { onAdopted?: () => Promise<void> | void } = {}) {
  const { storeId, mappingStatus, setMappingStatus } = useStoreData()
  const { notify } = useToast()
  const [pending, setPending] = useState<MappingCommand | 'save' | 'adopt' | null>(null)
  const [error, setError] = useState<string | null>(null)
  const [adoption, setAdoption] = useState<Adoption | null>(null)
  const inFlight = useRef(false)
  const onAdopted = useRef(options.onAdopted)
  useEffect(() => {
    onAdopted.current = options.onAdopted
  })

  const status = mappingStatus.data
  const phase: MappingPhase | null = status?.phase ?? null
  const draftReady = status?.draft_ready === true

  // 매핑 중 상태 폴링 + 스스로 끝났을 때 알림
  const lastPhase = useRef<MappingPhase | null>(null)
  useEffect(() => {
    if (lastPhase.current === 'mapping' && phase === 'stopped') {
      if (draftReady) notify('지도 초안이 준비되었습니다. 지도를 확인하고 저장하세요.', 'success')
      else if (status?.error) notify(status.error, 'error')
    }
    lastPhase.current = phase
  }, [phase, draftReady, status?.error, notify])

  useEffect(() => {
    if (phase !== 'mapping') return
    let alive = true
    const id = setInterval(() => {
      if (inFlight.current) return
      void getMappingStatus(storeId)
        .then((next) => alive && setMappingStatus(next))
        .catch(() => undefined) // 일시적인 조회 실패는 다음 주기에 다시 시도
    }, POLL_MS)
    return () => {
      alive = false
      clearInterval(id)
    }
  }, [phase, storeId, setMappingStatus])

  const run = useCallback(
    async (command: MappingCommand) => {
      if (inFlight.current) return
      inFlight.current = true
      setPending(command)
      setError(null)
      try {
        const next = await sendMappingCommand(command, storeId)
        setMappingStatus(next)
        if (command === 'start') setAdoption(null)
        notify(SUCCESS_MESSAGE[command], 'success')
      } catch (e) {
        const message = toApiError(e).message
        setError(message)
        notify(message, 'error')
      } finally {
        inFlight.current = false
        setPending(null)
      }
    },
    [storeId, setMappingStatus, notify],
  )

  const afterSave = useCallback(
    async (result: MapSaveResult) => {
      setMappingStatus(result)
      if (result.adopted) {
        setAdoption({ adopted: true, needsRestart: result.needs_robot_restart !== false, pendingReason: null })
        try {
          await onAdopted.current?.()
        } finally {
          notify('새 지도를 매장 지도로 반영했습니다.', 'success')
        }
      } else {
        const reason = result.adopt_error?.message ?? '지도 반영이 보류되었습니다.'
        setAdoption({ adopted: false, needsRestart: false, pendingReason: reason })
        notify(`보정은 저장했지만 지도 반영은 보류되었습니다: ${reason}`, 'info')
      }
      return result
    },
    [setMappingStatus, notify],
  )

  /** MapReviewPanel 의 "Save Map" */
  const save = useCallback(
    async (body: MapSaveBody) => {
      if (inFlight.current) throw new Error('다른 매핑 요청을 처리 중입니다.')
      inFlight.current = true
      setPending('save')
      setError(null)
      try {
        return await afterSave(await saveMap(body))
      } finally {
        inFlight.current = false
        setPending(null)
      }
    },
    [afterSave],
  )

  /** 저장은 됐는데 주문 때문에 반영이 보류된 경우 다시 시도 */
  const adopt = useCallback(async () => {
    if (inFlight.current) return
    inFlight.current = true
    setPending('adopt')
    setError(null)
    try {
      await afterSave(await adoptSavedMap())
    } catch (e) {
      const message = toApiError(e).message
      setError(message)
      notify(message, 'error')
    } finally {
      inFlight.current = false
      setPending(null)
    }
  }, [afterSave, notify])

  const enabled = status?.enabled !== false
  return {
    phase,
    status,
    statusError: mappingStatus.error,
    pending,
    error,
    adoption,
    draftReady,
    enabled,
    canStart: !pending && phase !== null && phase !== 'mapping' && enabled && !draftReady,
    canStop: !pending && phase === 'mapping' && status?.stop_requested !== true,
    canDiscard: !pending && phase !== 'mapping' && draftReady,
    start: () => run('start'),
    stop: () => run('stop'),
    discard: () => run('discard'),
    save,
    adopt,
  }
}

export type MappingControl = ReturnType<typeof useMappingControl>
