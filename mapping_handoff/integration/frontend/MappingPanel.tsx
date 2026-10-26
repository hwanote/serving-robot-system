import { useEffect, useState } from 'react'
import { useMappingControl } from '../../hooks/useMappingControl'
import { useStoreData } from '../../state/storeDataContext'
import { MAPPING_PHASE_META } from '../../utils/labels'
import { Icon } from '../common/Icon'
import { InlineError } from '../common/StateViews'
import { Badge } from '../common/StatusBadge'

function Elapsed({ since }: { since: string }) {
  const [now, setNow] = useState(() => Date.now())
  useEffect(() => {
    const id = setInterval(() => setNow(Date.now()), 1000)
    return () => clearInterval(id)
  }, [])
  const sec = Math.max(0, Math.floor((now - Date.parse(since)) / 1000))
  return (
    <span className="mono">
      {String(Math.floor(sec / 60)).padStart(2, '0')}:{String(sec % 60).padStart(2, '0')}
    </span>
  )
}

export function MappingPanel() {
  const m = useMappingControl()
  const { rawMap, reloadMappingStatus, mappingStatus } = useStoreData()
  const meta = m.phase ? MAPPING_PHASE_META[m.phase] : null

  return (
    <section className="card" aria-labelledby="mapping-title">
      <div className="card-header">
        <h2 id="mapping-title">매핑</h2>
        <div className="mapping-status">
          {meta ? (
            <Badge tone={meta.tone} dot>
              <span data-testid="mapping-phase">{meta.label}</span>
            </Badge>
          ) : mappingStatus.loading ? (
            <span className="spinner spinner-sm" />
          ) : (
            <Badge tone="gray">상태 알 수 없음</Badge>
          )}
          {m.phase === 'mapping' && m.status?.started_at && <Elapsed since={m.status.started_at} />}
        </div>
      </div>
      <div className="card-body" style={{ display: 'flex', flexDirection: 'column', gap: 12 }}>
        <div className="btn-row">
          <button type="button" className="btn btn-primary" disabled={!m.canStart} onClick={m.start}>
            {m.pending === 'start' ? <span className="spinner spinner-sm" /> : <Icon name="play" size={14} />}
            Start Mapping
          </button>
          <button type="button" className="btn" disabled={!m.canStop} onClick={m.stop}>
            {m.pending === 'stop' ? <span className="spinner spinner-sm" /> : <Icon name="stop" size={14} />}
            Stop Mapping
          </button>
          <button type="button" className="btn" disabled={!m.canSave} onClick={m.save}>
            {m.pending === 'save' ? <span className="spinner spinner-sm" /> : <Icon name="save" size={14} />}
            Save Map
          </button>
        </div>
        {m.phase === 'mapping' && (
          <div className="alert alert-info">
            <span className="spinner spinner-sm" /> 로봇이 매장을 주행하며 지도를 생성하고 있습니다. 완료 후 중지하고 저장하세요.
          </div>
        )}
        {m.phase === 'stopped' && <div className="alert alert-warn">매핑이 중지되었습니다. 결과를 사용하려면 Save Map을 눌러 저장하세요.</div>}
        {m.statusError && !m.status && (
          <div className="alert alert-error">
            매핑 상태를 확인할 수 없습니다: {m.statusError}
            <button type="button" className="btn btn-sm" onClick={() => void reloadMappingStatus()}>
              다시 시도
            </button>
          </div>
        )}
        <InlineError message={m.error} />
        <dl className="kv">
          <dt>저장된 지도</dt>
          <dd>{rawMap.data ? <span className="mono">revision {rawMap.data.map_revision}</span> : <span className="muted">없음</span>}</dd>
        </dl>
      </div>
    </section>
  )
}
