import { useCallback, useEffect, useState } from 'react'
import { applySemantics, getLiveStatus, getSemanticStatus, listBackups, promoteMap, restoreBackup } from '../../api/liveMapApi'
import { toApiError } from '../../api/client'
import type { LiveStatus, MapBackup, SemanticStatus } from '../../types/provisional'
import { Icon } from '../common/Icon'
import { Badge } from '../common/StatusBadge'
import { useToast } from '../../state/toastContext'
import { formatBackupTime } from '../../utils/backup'

const KIND_LABEL: Record<MapBackup['kind'], string> = {
  promote: '반영 직전 지도',
  before_restore: '복구 직전 지도',
}

const summaryText = (b: MapBackup) => (b.summary ? `노드 ${b.summary.nodes} · 통로 ${b.summary.edges} · 테이블 ${b.summary.tables}` : '내용 정보 없음')

type Working = 'apply' | 'promote' | `restore:${string}` | null

interface Props {
  storeId: string
  /** 저장하지 않은 변경이 있으면 적용·반영을 막는다 */
  dirty: boolean
  /** 저장이 끝날 때마다 바뀌는 값 — 바뀌면 상태를 다시 읽는다 */
  savedKey: unknown
}

export function LivePanel({ storeId, dirty, savedKey }: Props) {
  const { notify } = useToast()
  const [status, setStatus] = useState<SemanticStatus | null>(null)
  const [live, setLive] = useState<LiveStatus | null>(null)
  const [backups, setBackups] = useState<MapBackup[]>([])
  const [loadError, setLoadError] = useState<string | null>(null)
  const [working, setWorking] = useState<Working>(null)
  const [actionError, setActionError] = useState<string | null>(null)
  const [restartNote, setRestartNote] = useState<string | null>(null)
  const [listOpen, setListOpen] = useState(false)

  const [tick, setTick] = useState(0)
  const refresh = useCallback(() => setTick((n) => n + 1), [])

  // 저장이 끝나거나(savedKey) 직접 새로고침(tick)할 때 상태를 다시 읽는다
  useEffect(() => {
    let alive = true
    void (async () => {
      try {
        const [s, l, b] = await Promise.all([getSemanticStatus(storeId), getLiveStatus(storeId), listBackups(storeId)])
        if (!alive) return
        setStatus(s)
        setLive(l)
        setBackups(b)
        setLoadError(null)
      } catch (e) {
        if (alive) setLoadError(toApiError(e).message)
      }
    })()
    return () => {
      alive = false
    }
  }, [storeId, savedKey, tick])

  const run = async (kind: NonNullable<Working>, job: () => Promise<string | void>) => {
    setWorking(kind)
    setActionError(null)
    try {
      const done = await job()
      if (done) notify(done, 'success')
    } catch (e) {
      setActionError(toApiError(e).message)
    } finally {
      setWorking(null)
      refresh()
    }
  }

  const onApply = () =>
    run('apply', async () => {
      await applySemantics(storeId)
      setRestartNote(null)
      return '노드·통로를 자동으로 만들었어요. 아직 로봇 지도는 바뀌지 않았어요.'
    })

  const onPromote = () => {
    if (!window.confirm('이 지도를 로봇이 읽는 지도로 반영할까요?\n\n· 지금 지도는 날짜·시간이 붙은 백업으로 남아서 나중에 되돌릴 수 있어요.\n· 주문이 진행 중이면 반영되지 않아요.\n· 로봇 노드는 다시 켜야 새 지도로 움직여요.')) return
    void run('promote', async () => {
      const r = await promoteMap(storeId)
      setRestartNote(`반영했어요. 로봇 노드를 다시 켜야 새 지도로 움직여요. (이전 지도 백업: ${formatBackupTime(r.backup)})`)
      return '로봇 지도에 반영했어요.'
    })
  }

  const onRestore = (b: MapBackup) => {
    if (!window.confirm(`${formatBackupTime(b)} 백업(${summaryText(b)})으로 되돌릴까요?\n\n· 지금 지도도 백업으로 남아서 다시 되돌릴 수 있어요.\n· 주문이 진행 중이면 되돌려지지 않아요.\n· 로봇 노드는 다시 켜야 해요.`)) return
    void run(`restore:${b.id}`, async () => {
      await restoreBackup(storeId, b.id)
      setRestartNote(`${formatBackupTime(b)} 백업으로 되돌렸어요. 로봇 노드를 다시 켜야 새 지도로 움직여요.`)
      return '백업으로 되돌렸어요.'
    })
  }

  const applied = status?.applied ?? null
  const inSync = !!status?.in_sync
  const live_ok = !!live?.matches_applied
  const busy = working !== null
  const canApply = !!status?.draft && !dirty && !busy
  const canPromote = !!applied && inSync && !live_ok && !dirty && !busy

  const applyBadge = !applied ? (
    <Badge tone="gray">아직 안 함</Badge>
  ) : inSync ? (
    <Badge tone="green">완료 · OK {applied.summary.OK} / 경고 {applied.summary.WARN}</Badge>
  ) : (
    <Badge tone="amber" dot>
      초안이 바뀜 · 다시 적용
    </Badge>
  )

  return (
    <section className="card" aria-labelledby="live-title">
      <div className="card-header">
        <h2 id="live-title">로봇 지도 반영</h2>
        <button type="button" className="btn btn-sm" onClick={refresh} disabled={busy} aria-label="상태 새로고침">
          <Icon name="refresh" size={14} />
        </button>
      </div>
      <div className="card-body" style={{ display: 'flex', flexDirection: 'column', gap: 10 }}>
        {loadError && (
          <div className="alert alert-error" role="alert">
            <Icon name="alert" size={15} />
            <span>상태를 불러오지 못했어요: {loadError}</span>
          </div>
        )}

        <ol className="live-steps" style={{ margin: 0, paddingLeft: 18, display: 'flex', flexDirection: 'column', gap: 6 }}>
          <li>
            <span>저장 </span>
            {dirty ? <Badge tone="amber" dot>변경 저장 필요</Badge> : status?.draft ? <Badge tone="green">초안 저장됨</Badge> : <Badge tone="gray">저장된 초안 없음</Badge>}
          </li>
          <li>
            <span>노드·통로 자동 생성 </span>
            {applyBadge}
          </li>
          <li>
            <span>로봇 지도 </span>
            {live_ok ? <Badge tone="green">반영됨</Badge> : <Badge tone="gray">아직 이전 지도</Badge>}
          </li>
        </ol>

        {live?.last && (
          <p className="muted" style={{ margin: 0, fontSize: 12.5 }}>
            마지막 변경: {live.last.note} · {new Date(live.last.at).toLocaleString('ko-KR')}
          </p>
        )}

        {actionError && (
          <div className="alert alert-error" role="alert" style={{ whiteSpace: 'pre-line' }}>
            <Icon name="alert" size={15} />
            <span>{actionError}</span>
          </div>
        )}
        {restartNote && (
          <div className="alert alert-warn" role="status">
            <Icon name="alert" size={15} />
            <span>{restartNote}</span>
          </div>
        )}

        <div className="btn-row">
          <button type="button" className="btn" disabled={!canApply} onClick={onApply} title={dirty ? '먼저 매장 설정을 저장하세요' : undefined}>
            {working === 'apply' ? <span className="spinner spinner-sm" /> : <Icon name="play" size={14} />}
            노드 자동 생성
          </button>
          <button type="button" className="btn btn-primary" disabled={!canPromote} onClick={onPromote} title={!applied ? '먼저 노드를 자동 생성하세요' : undefined}>
            {working === 'promote' ? <span className="spinner spinner-sm" /> : <Icon name="check" size={14} />}
            로봇에 반영
          </button>
        </div>

        <div>
          <button
            type="button"
            className="btn btn-sm"
            aria-expanded={listOpen}
            aria-controls="backup-list"
            onClick={() => setListOpen((v) => !v)}
            style={{ width: '100%', justifyContent: 'space-between' }}
          >
            <span>지난 지도 ({backups.length})</span>
            <span aria-hidden="true">{listOpen ? '▴' : '▾'}</span>
          </button>
          {listOpen &&
            (backups.length === 0 ? (
              <p className="muted" style={{ margin: '8px 0 0', fontSize: 12.5 }}>
                아직 백업이 없어요. 로봇에 반영하면 바꾸기 전 지도가 여기에 쌓여요.
              </p>
            ) : (
              // 한 번에 3~4개만 보이고 나머지는 스크롤로 (한 줄 약 54px)
              <ul id="backup-list" aria-label="지도 백업 목록" style={{ listStyle: 'none', margin: '8px 0 0', padding: 0, maxHeight: 4 * 54, overflowY: 'auto', display: 'flex', flexDirection: 'column', gap: 6 }}>
                {backups.map((b) => (
                  <li key={b.id} style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', gap: 8, padding: '6px 8px', border: '1px solid var(--border, #ddd)', borderRadius: 6, minHeight: 48, boxSizing: 'border-box' }}>
                    <div style={{ minWidth: 0 }}>
                      <div className="mono" style={{ fontSize: 12.5 }}>
                        {formatBackupTime(b)}
                      </div>
                      <div className="muted" style={{ fontSize: 12 }}>
                        {KIND_LABEL[b.kind]} · {summaryText(b)}
                      </div>
                    </div>
                    <button type="button" className="btn btn-sm" disabled={busy} onClick={() => onRestore(b)} aria-label={`${formatBackupTime(b)} 백업으로 복구`}>
                      {working === `restore:${b.id}` ? <span className="spinner spinner-sm" /> : null}
                      복구
                    </button>
                  </li>
                ))}
              </ul>
            ))}
        </div>
      </div>
    </section>
  )
}
