import { useCallback, useRef, useState } from 'react'
import { toApiError } from '../api/client'
import { sendMappingCommand, type MappingCommand } from '../api/mappingApi'
import { useStoreData } from '../state/storeDataContext'
import { useToast } from '../state/toastContext'
import type { MappingPhase } from '../types/provisional'

const SUCCESS_MESSAGE: Record<MappingCommand, string> = {
  start: '매핑을 시작했습니다.',
  stop: '매핑을 중지했습니다. 지도를 저장하세요.',
  save: '지도를 저장했습니다.',
}

/**
 * Start / Stop / Save Mapping.
 * - 요청 중에는 모든 매핑 버튼 비활성 (중복 클릭 방지, ref 로 동기 차단)
 * - 상태는 서버 응답으로만 갱신 (ROS 를 직접 호출하지 않음)
 */
export function useMappingControl() {
  const { storeId, mappingStatus, setMappingStatus, reloadRawMap } = useStoreData()
  const { notify } = useToast()
  const [pending, setPending] = useState<MappingCommand | null>(null)
  const [error, setError] = useState<string | null>(null)
  const inFlight = useRef(false)

  const phase: MappingPhase | null = mappingStatus.data?.phase ?? null

  const run = useCallback(
    async (command: MappingCommand) => {
      if (inFlight.current) return
      inFlight.current = true
      setPending(command)
      setError(null)
      try {
        const status = await sendMappingCommand(command, storeId)
        setMappingStatus(status)
        if (command === 'save') await reloadRawMap()
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
    [storeId, setMappingStatus, reloadRawMap, notify],
  )

  return {
    phase,
    status: mappingStatus.data,
    statusError: mappingStatus.error,
    pending,
    error,
    canStart: !pending && phase !== null && phase !== 'mapping',
    canStop: !pending && phase === 'mapping',
    canSave: !pending && (phase === 'mapping' || phase === 'stopped'),
    start: () => run('start'),
    stop: () => run('stop'),
    save: () => run('save'),
  }
}
