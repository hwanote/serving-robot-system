import { useRef, useState } from 'react'
import { Link } from 'react-router-dom'
import { toApiError } from '../../api/client'
import { useDestinations } from '../../hooks/useDestinations'
import { useFleet } from '../../state/fleetContext'
import { useToast } from '../../state/toastContext'
import type { Task } from '../../types/provisional'
import { Icon } from '../common/Icon'
import { InlineError } from '../common/StateViews'
import { TaskStatusBadge } from '../common/StatusBadge'

/**
 * 서빙 명령: { type: 'SERVE', destination_id } 만 전송.
 * 로봇 선택/좌표 지정 없음 — 배정은 Backend(Fleet) 책임.
 * Backend 응답 전에는 Task 가 생성/실행된 것으로 표시하지 않는다.
 */
export function ServePanel({ selected, onSelect }: { selected: string; onSelect: (id: string) => void }) {
  const { destinations, revisionMismatch, labelOf, loaded, error } = useDestinations()
  const { createServeTask, state } = useFleet()
  const { notify } = useToast()
  const [pending, setPending] = useState(false)
  const [submitError, setSubmitError] = useState<string | null>(null)
  const [lastTaskId, setLastTaskId] = useState<string | null>(null)
  const inFlight = useRef(false)

  const valid = destinations.some((d) => d.id === selected)
  const lastTask: Task | undefined = lastTaskId ? state.tasks[lastTaskId] : undefined

  const submit = async () => {
    if (inFlight.current || !valid) return
    inFlight.current = true
    setPending(true)
    setSubmitError(null)
    try {
      const task = await createServeTask(selected)
      setLastTaskId(task.task_id)
      notify(`${labelOf(selected)} 서빙 요청이 접수되었습니다. (${task.task_id})`, 'success')
    } catch (e) {
      const message = toApiError(e).message
      setSubmitError(message)
      notify(`서빙 요청 실패: ${message}`, 'error')
    } finally {
      inFlight.current = false
      setPending(false)
    }
  }

  return (
    <section className="card" aria-labelledby="serve-title">
      <div className="card-header">
        <h2 id="serve-title">서빙 명령</h2>
      </div>
      <div className="card-body" style={{ display: 'flex', flexDirection: 'column', gap: 12 }}>
        {!loaded ? (
          <div className="muted">목적지 정보를 불러오는 중…</div>
        ) : revisionMismatch ? (
          <div className="alert alert-warn" role="alert" data-testid="serve-revision-mismatch">
            <Icon name="alert" size={15} />
            <span>
              매장 설정(Semantic r{revisionMismatch.semantic})이 현재 지도(r{revisionMismatch.map})와 달라 서빙 명령을 보낼 수 없습니다.{' '}
              <Link to="/map">지도 설정</Link>에서 객체 위치를 확인하고 현재 지도 기준으로 갱신해 저장하세요.
            </span>
          </div>
        ) : destinations.length === 0 ? (
          <div className="alert alert-warn">
            <Icon name="alert" size={15} />
            <span>
              서빙 가능한 테이블이 없습니다. <Link to="/map">지도 설정</Link>에서 테이블과 Waypoint를 지정하고 저장하세요.
              {error && ` (${error})`}
            </span>
          </div>
        ) : (
          <>
            <label className="field">
              <span className="field-label">목적지 테이블</span>
              <select className="select" value={valid ? selected : ''} onChange={(e) => onSelect(e.target.value)} aria-label="목적지 테이블">
                <option value="" disabled>
                  테이블 선택
                </option>
                {destinations.map((d) => (
                  <option key={d.id} value={d.id}>
                    {d.label} ({d.id})
                  </option>
                ))}
              </select>
            </label>
            <button type="button" className="btn btn-primary" disabled={!valid || pending} onClick={() => void submit()}>
              {pending ? <span className="spinner spinner-sm" /> : <Icon name="play" size={14} />}
              {pending ? '요청 중…' : 'Serve'}
            </button>
            <p className="muted" style={{ margin: 0, fontSize: 12 }}>
              로봇 배정과 경로는 Fleet 시스템이 결정합니다. 전송 데이터: <span className="mono">{`{ type: "SERVE", destination_id: "${valid ? selected : '…'}" }`}</span>
            </p>
          </>
        )}
        {state.socket.status !== 'open' && destinations.length > 0 && (
          <div className="alert alert-warn">실시간 연결이 끊겨 있어 Task 진행 상황이 늦게 반영될 수 있습니다.</div>
        )}
        <InlineError message={submitError} />
        {lastTask && (
          <div className="alert alert-info" data-testid="last-task">
            <span>
              최근 요청 <span className="mono">{lastTask.task_id}</span> → {labelOf(lastTask.destination_id)}
            </span>
            <span style={{ marginLeft: 'auto' }}>
              <TaskStatusBadge status={lastTask.status} />
            </span>
          </div>
        )}
      </div>
    </section>
  )
}
