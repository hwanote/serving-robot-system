import { useNow } from '../../hooks/useNow'
import { useFleet } from '../../state/fleetContext'
import { SOCKET_STATUS_META } from '../../utils/labels'
import { Badge } from '../common/StatusBadge'
import { Icon } from '../common/Icon'

/** 상단바 실시간 연결 상태 표시 */
export function ConnectionIndicator() {
  const { state } = useFleet()
  const meta = SOCKET_STATUS_META[state.socket.status]
  return (
    <Badge tone={meta.tone} dot title={state.socket.lastError ?? undefined}>
      <span data-testid="socket-status">{meta.label}</span>
    </Badge>
  )
}

/** 연결이 끊겼을 때 전체 화면 상단 경고 */
export function ConnectionBanner() {
  const { state, reconnectSocket } = useFleet()
  const { status, attempt, nextRetryAt, retryDelayMs, lastError } = state.socket
  const now = useNow(1000, status === 'reconnecting')

  if (status === 'open' || status === 'closed') return null
  if (status === 'connecting' && !state.everConnected) {
    return null
  }

  // 실제 대기 시간을 넘지 않도록 상한 (now 가 갱신되기 전 첫 렌더 보호)
  const seconds = nextRetryAt
    ? Math.min(Math.ceil((retryDelayMs ?? Infinity) / 1000), Math.max(0, Math.ceil((nextRetryAt - now) / 1000)))
    : null
  return (
    <div className="banner banner-danger" role="alert">
      <Icon name="wifiOff" />
      <span>
        {Object.keys(state.robots).length > 0 ? (
          <>
            실시간 연결이 끊겼습니다. 로봇 상태는 <strong>마지막으로 수신한 값</strong>이며 실제와 다를 수 있습니다.
          </>
        ) : (
          <>실시간 연결(/ws/state)에 실패했습니다. 로봇 상태를 확인할 수 없습니다.</>
        )}
        {attempt > 0 && ` (재시도 ${attempt}회${seconds !== null ? `, ${seconds}초 후 재연결` : ''})`}
        {lastError && <span className="muted"> — {lastError}</span>}
      </span>
      <button type="button" className="btn btn-sm" style={{ marginLeft: 'auto' }} onClick={reconnectSocket}>
        지금 다시 연결
      </button>
    </div>
  )
}
