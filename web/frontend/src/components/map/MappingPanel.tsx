import { useEffect, useState } from 'react'
import { RESTART_NOTICE, type MappingControl } from '../../hooks/useMappingControl'
import { useStoreData } from '../../state/storeDataContext'
import { MAPPING_PHASE_META, MAPPING_STAGE_LABEL, MAPPING_STAGE_ORDER } from '../../utils/labels'
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

function Progress({ m }: { m: MappingControl }) {
  const status = m.status
  const stage = status?.stage
  const progress = status?.progress
  if (!stage || stage === 'idle' || !progress) return null
  const failed = stage === 'failed'
  const at = MAPPING_STAGE_ORDER.indexOf(stage)
  return (
    <div data-testid="mapping-progress" style={{ display: 'flex', flexDirection: 'column', gap: 8 }}>
      <ol className="mono" style={{ display: 'flex', gap: 8, flexWrap: 'wrap', listStyle: 'none', margin: 0, padding: 0, fontSize: 12 }}>
        {MAPPING_STAGE_ORDER.map((s, i) => (
          <li key={s} aria-current={s === stage ? 'step' : undefined}
            style={{ fontWeight: s === stage ? 700 : 400, opacity: failed || i > at ? 0.5 : 1 }}>
            {i + 1}. {MAPPING_STAGE_LABEL[s]}
          </li>
        ))}
      </ol>
      <div style={{ display: 'flex', alignItems: 'center', gap: 8 }}>
        <progress max={100} value={progress.percent} aria-label="추정 진행률" style={{ flex: 1 }} />
        <span className="mono" data-testid="mapping-percent">
          {progress.percent}%{progress.estimated ? ' (추정)' : ''}
        </span>
      </div>
      <dl className="kv">
        <dt>경과 시간</dt>
        <dd>{progress.elapsed_s == null ? '-' : <span className="mono">{Math.floor(progress.elapsed_s / 60)}분 {Math.floor(progress.elapsed_s % 60)}초</span>}</dd>
        <dt>탐색 지점</dt>
        <dd>
          <span className="mono" data-testid="mapping-goals">
            도착 {progress.goals_arrived} / 전송 {progress.goals_sent}
            {progress.goals_failed > 0 ? ` (못 간 곳 ${progress.goals_failed})` : ''}
          </span>
        </dd>
      </dl>
      {progress.estimated && <p className="muted" style={{ margin: 0, fontSize: 12 }}>방 크기를 미리 알 수 없어서 진행률은 대략적인 추정이에요.</p>}
    </div>
  )
}

export function MappingPanel({ mapping: m }: { mapping: MappingControl }) {
  const { rawMap, reloadMappingStatus, mappingStatus } = useStoreData()
  const meta = m.phase ? MAPPING_PHASE_META[m.phase] : null
  const status = m.status

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
          {m.phase === 'mapping' && status?.started_at && <Elapsed since={status.started_at} />}
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
          <button
            type="button"
            className="btn"
            disabled={!m.draftReady}
            title="아래 '지도 완성하기'에서 지도를 확인하고 저장해요"
            onClick={() => document.getElementById('map-review')?.scrollIntoView({ behavior: 'smooth', block: 'start' })}
          >
            {m.pending === 'save' ? <span className="spinner spinner-sm" /> : <Icon name="save" size={14} />}
            Save Map
          </button>
          {m.canDiscard && (
            <button type="button" className="btn" onClick={() => { if (window.confirm('임시 지도를 버릴까요? (복구용으로 보관됩니다)')) void m.discard() }}>
              임시 지도 버리기
            </button>
          )}
        </div>

        {!m.enabled && (
          <div className="alert alert-warn" role="alert">
            이 백엔드는 자동 매핑이 꺼져 있습니다. (설정 파일의 <span className="mono">mapping_enabled: true</span> 로 켜고 다시 실행하세요.)
          </div>
        )}
        {m.phase === 'mapping' && (
          <div className="alert alert-info">
            <span className="spinner spinner-sm" /> 로봇이 매장을 돌며 지도를 만들고 있어요. 탐색이 끝나면 지도 초안이 자동으로 만들어집니다.
            중간에 끊고 싶을 때만 Stop Mapping을 누르세요. 매핑 중에는 주문을 받지 않아요.
          </div>
        )}
        <Progress m={m} />
        {m.draftReady && m.phase !== 'mapping' && (
          <div className="alert alert-warn">
            {status?.message ?? '지도 초안이 준비되었습니다.'} 아래 '지도 완성하기'에서 확인하고 Save Map으로 저장하세요.
          </div>
        )}
        {status?.stage === 'failed' && status.error && <div className="alert alert-error" role="alert">{status.error}</div>}
        {m.adoption?.adopted && (
          <div className="alert alert-warn" role="status" data-testid="restart-notice" style={{ flexDirection: 'column', alignItems: 'flex-start' }}>
            <strong>{RESTART_NOTICE}</strong>
            <span>로봇은 지도를 시작할 때만 읽어요. 영업이 끝났거나 주문이 없을 때 로봇을 껐다 켜 주세요. 이전 지도는 백업에 보관됩니다.</span>
          </div>
        )}
        {m.statusError && !status && (
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
