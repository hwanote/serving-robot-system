import { useCallback, useEffect, useState } from 'react'
import { applySemantics, getLiveStatus, getSemanticStatus, listBackups, promoteMap, restoreBackup } from '../../api/liveMapApi'
import { toApiError } from '../../api/client'
import type { SemanticMap } from '../../types/contracts'
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
  /** 지난 지도로 복구하면서 편집 화면 저장본도 되돌려졌을 때 — 편집 화면을 이 문서로 바꾼다 */
  onRestored?: (doc: SemanticMap) => void
}

export function LivePanel({ storeId, dirty, savedKey, onRestored }: Props) {
  const { notify } = useToast()
  const [status, setStatus] = useState<SemanticStatus | null>(null)
  const [live, setLive] = useState<LiveStatus | null>(null)
  const [backups, setBackups] = useState<MapBackup[]>([])
  const [loadError, setLoadError] = useState<string | null>(null)
  const [working, setWorking] = useState<Working>(null)
  const [actionError, setActionError] = useState<string | null>(null)
  const [restartNote, setRestartNote] = useState<string | null>(null)
  const [listOpen, setListOpen] = useState(false)
  const [regen, setRegen] = useState(false)

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
    setRegen(false)
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
    if (!window.confirm('이 지도를 로봇이 읽는 지도로 반영할까요?\n\n· 지금 지도는 날짜·시간이 붙은 백업으로 남아서 나중에 되돌릴 수 있어요.\n· 주문이 진행 중이면 반영되지 않아요.\n· 새 노드·통로·테이블 위치는 로봇 관제 화면에서 "지도 다시 읽기"를 눌러야 로봇이 써요. (지도 그림은 이 반영으로 바뀌지 않아요.)')) return
    void run('promote', async () => {
      const r = await promoteMap(storeId)
      setRestartNote(`반영했어요. 새 노드·통로·테이블 위치는 로봇 관제 화면에서 '지도 다시 읽기'를 눌러야 로봇이 써요. (이전 지도 백업: ${formatBackupTime(r.backup)})`)
      return '로봇 지도에 반영했어요.'
    })
  }

  const onRestore = (b: MapBackup) => {
    const lines = [
      `${formatBackupTime(b)} 백업(${summaryText(b)})으로 되돌릴까요?`,
      '',
      '· 지금 지도도 백업으로 남아서 다시 되돌릴 수 있어요.',
      '· 주문이 진행 중이면 되돌려지지 않아요.',
      "· 되돌린 노드·통로는 로봇 관제 화면에서 '지도 다시 읽기'를 눌러야 로봇이 써요.",
    ]
    if (b.has_semantics === false) lines.push('· 이 백업에는 편집 화면 저장본이 없어서, 편집 화면은 지도 파일에서 다시 만들어요. 직접 그린 구역은 빠져요.')
    else lines.push('· 편집 화면(Raw Map · Semantic 편집)의 저장본도 이 백업 때 모습으로 바뀌어요.')
    if (dirty) lines.push('', '⚠ 저장하지 않은 변경이 있어요. 되돌리면 그 변경은 사라져요.')
    if (!window.confirm(lines.join('\n'))) return
    void run(`restore:${b.id}`, async () => {
      const r = await restoreBackup(storeId, b.id)
      if (r.semantics_restored && r.semantics) onRestored?.(r.semantics)
      const base = `${formatBackupTime(b)} 백업으로 되돌렸어요. 로봇 관제 화면에서 '지도 다시 읽기'를 눌러야 로봇이 되돌린 노드·통로를 써요.`
      const extra = r.semantics_restored
        ? ` 편집 화면도 그때 모습으로 바뀌었어요.${r.semantics_note ? ` (${r.semantics_note})` : ''} 로봇에 다시 반영하려면 "노드 자동 생성"부터 해 주세요.`
        : r.semantics_note
          ? ` (${r.semantics_note})`
          : ''
      setRestartNote(base + extra)
      return '백업으로 되돌렸어요.'
    })
  }

  const applied = status?.applied ?? null
  const inSync = !!status?.in_sync
  const liveOk = !!live?.matches_applied
  const busy = working !== null
  const hasDraft = !!status?.draft

  // 지금 어느 단계인지. 한 번에 해야 할 것 하나만 켜지게 한다.
  //  none      저장된 초안 없음
  //  editing   편집 중(저장 전)
  //  toApply   저장함 → 노드·통로 자동 생성 필요
  //  toPromote 적용 끝남 → 로봇에 반영 필요
  //  same      로봇 지도와 같음 (바뀐 게 없음)
  const phase: 'none' | 'editing' | 'toApply' | 'toPromote' | 'same' = dirty
    ? 'editing'
    : !hasDraft
      ? 'none'
      : applied && inSync && liveOk
        ? 'same'
        : applied && inSync
          ? 'toPromote'
          : 'toApply'
  const canRegen = hasDraft && !dirty && (phase === 'same' || phase === 'toPromote')
  const canApply = hasDraft && !dirty && !busy && (phase === 'toApply' || (canRegen && regen))
  const canPromote = phase === 'toPromote' && !busy

  const step1 =
    phase === 'editing' ? (
      <Badge tone="amber" dot>변경 저장 필요</Badge>
    ) : phase === 'none' ? (
      <Badge tone="gray">저장된 초안 없음</Badge>
    ) : phase === 'same' ? (
      <Badge tone="gray">변경 없음</Badge>
    ) : (
      <Badge tone="green">초안 저장됨</Badge>
    )
  const step2 =
    phase === 'toApply' ? (
      <Badge tone="amber" dot>자동 생성 필요</Badge>
    ) : phase === 'toPromote' && applied ? (
      <Badge tone="green">완료 · OK {applied.summary.OK} / 경고 {applied.summary.WARN}</Badge>
    ) : (
      <Badge tone="gray">{phase === 'same' ? '해당 없음' : '아직 안 함'}</Badge>
    )
  const step3 =
    phase === 'toPromote' ? (
      <Badge tone="amber" dot>반영 필요</Badge>
    ) : phase === 'same' ? (
      <Badge tone="gray">로봇 지도와 같아요</Badge>
    ) : (
      <Badge tone="gray">아직 이전 지도</Badge>
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
            {step1}
          </li>
          <li>
            <span>노드·통로 자동 생성 </span>
            {step2}
          </li>
          <li>
            <span>로봇 지도 </span>
            {step3}
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

        {canRegen && (
          <label style={{ display: 'flex', alignItems: 'center', gap: 6, fontSize: 12.5 }}>
            <input type="checkbox" checked={regen} disabled={busy} onChange={(e) => setRegen(e.target.checked)} />
            동일 지도 노드 재생성
          </label>
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
                        {b.has_semantics === false ? ' · 지도 파일에서 복원' : ''}
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
