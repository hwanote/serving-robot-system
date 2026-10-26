import type { GeneratedGraph } from '../../types/provisional'
import { AXIS_DIRECTIONS } from '../../utils/manualStop'
import { compassOf, stopSideText, type GraphLayers } from '../../utils/graph'
import { Icon } from '../common/Icon'
import { Badge } from '../common/StatusBadge'

interface Props {
  view: { graph: GeneratedGraph | null; loading: boolean; error: string | null; stale: boolean }
  /** 지금 편집 중인 설정(저장 전이어도)으로 다시 계산 */
  onCheck: () => void
  onClear: () => void
  layers: GraphLayers
  onLayers: (next: GraphLayers) => void
  /** 정차 위치의 방향을 지정(= 그 자리를 수동으로 고정). yaw 는 rad */
  onSetYaw: (entityId: string, x: number, y: number, yaw: number) => void
  /** 수동 지정을 지우고 자동 위치로 되돌림 */
  onRevert: (entityId: string) => void
}

const LAYER_LABEL: [keyof GraphLayers, string][] = [
  ['edges', '통로'],
  ['lengths', '길이(m)'],
  ['stops', '자동 정차 위치'],
  ['myWaypoints', '내 Waypoint'],
]

function DirectionButtons({ name, activeName, disabled, onPick }: { name: string; activeName: string | null; disabled: boolean; onPick: (yaw: number) => void }) {
  return (
    <span role="group" aria-label={`${name} 방향`} style={{ display: 'inline-flex', gap: 3 }}>
      {AXIS_DIRECTIONS.map((d) => (
        <button key={d.name} type="button" className="btn btn-sm" aria-pressed={activeName === d.name} aria-label={`${name} 방향 ${d.name}쪽`} disabled={disabled} onClick={() => onPick(d.yaw)} style={activeName === d.name ? { fontWeight: 700 } : undefined}>
          {d.name}
        </button>
      ))}
    </span>
  )
}

export function GraphPreviewPanel({ view, onCheck, onClear, layers, onLayers, onSetYaw, onRevert }: Props) {
  const { graph, loading, error, stale } = view
  const failedPins = graph ? graph.excluded.filter((x) => x.pin && x.entity_id) : []

  return (
    <section className="card" aria-labelledby="graph-title">
      <div className="card-header">
        <h2 id="graph-title">자동 생성 통로 미리보기</h2>
        {graph && (
          <button type="button" className="btn btn-sm" onClick={onClear} aria-label="미리보기 지우기">
            <Icon name="x" size={14} />
          </button>
        )}
      </div>
      <div className="card-body" style={{ display: 'flex', flexDirection: 'column', gap: 10 }}>
        <p className="muted" style={{ margin: 0, fontSize: 12.5 }}>
          지금 편집 중인 설정(저장 전이어도)으로 자동 생성기를 돌려 통로와 로봇이 서는 자리·방향을 지도에 보여줘요. 아무것도 저장하거나 로봇에 반영하지 않아요.
        </p>

        <div className="btn-row">
          <button type="button" className="btn" disabled={loading} onClick={onCheck}>
            {loading ? <span className="spinner spinner-sm" /> : <Icon name={graph ? 'refresh' : 'play'} size={14} />}
            {graph ? '다시 계산' : '통로·정차 위치 보기'}
          </button>
        </div>

        {error && (
          <div className="alert alert-error" role="alert" style={{ whiteSpace: 'pre-line' }}>
            <Icon name="alert" size={15} />
            <span>{error}</span>
          </div>
        )}

        {graph && (
          <>
            <div className="btn-row" style={{ flexWrap: 'wrap' }}>
              {graph.ok ? <Badge tone="green">문제 없음</Badge> : <Badge tone="red" dot>적용이 거절될 문제가 있어요</Badge>}
              <Badge tone="gray">
                OK {graph.summary.OK} · 경고 {graph.summary.WARN} · 실패 {graph.summary.FAIL}
              </Badge>
              {stale && (
                <Badge tone="amber" dot title="설정을 바꾼 뒤 다시 계산하지 않았어요">
                  옛 결과 — 다시 계산
                </Badge>
              )}
              {loading && <Badge tone="gray">검사 중…</Badge>}
            </div>

            {!graph.ok && graph.problems.length > 0 && (
              <ul className="issue-list alert alert-error" aria-label="생성 문제" style={{ whiteSpace: 'pre-line' }}>
                {graph.problems.map((p, i) => (
                  <li key={i}>{p}</li>
                ))}
              </ul>
            )}

            <div style={{ display: 'flex', flexWrap: 'wrap', gap: '4px 12px' }} role="group" aria-label="미리보기 표시">
              {LAYER_LABEL.map(([key, label]) => (
                <label key={key} style={{ display: 'flex', alignItems: 'center', gap: 5, fontSize: 12.5 }}>
                  <input type="checkbox" checked={layers[key]} onChange={(e) => onLayers({ ...layers, [key]: e.target.checked })} />
                  {label}
                </label>
              ))}
            </div>
            <p className="muted" style={{ margin: 0, fontSize: 12 }}>
              회색 선 = 통로(숫자는 길이) · 청록 화살표 = 자동 정차 위치 · 보라 화살표 = 내가 옮긴(수동) 정차 위치 · 파란 화살표 = 내 Waypoint
              <br />
              정차 화살표를 <b>끌어서</b> 옮기거나 아래 방향 버튼을 누르면 그 자리를 직접 지정하고 바로 다시 검사해요. (선택 도구 상태에서)
            </p>

            {graph.stops.length > 0 && (
              <ul aria-label="자동 정차 위치 목록" style={{ listStyle: 'none', margin: 0, padding: 0, display: 'flex', flexDirection: 'column', gap: 6 }}>
                {graph.stops.map((s) => (
                  <li key={s.id} style={{ display: 'flex', flexWrap: 'wrap', alignItems: 'center', gap: '3px 8px', fontSize: 12.5 }}>
                    <span className="mono">{s.entity_id ?? s.id}</span>
                    {s.pinned && <Badge tone="indigo">수동</Badge>}
                    <span className="muted">{stopSideText(s)}</span>
                    {s.level !== 'OK' && <Badge tone={s.level === 'FAIL' ? 'red' : 'amber'}>{s.level}</Badge>}
                    {s.entity_id && (
                      <span style={{ display: 'inline-flex', alignItems: 'center', gap: 6 }}>
                        <DirectionButtons name={s.entity_id} activeName={compassOf(s.yaw_deg, 1)} disabled={loading} onPick={(yaw) => onSetYaw(s.entity_id!, s.x, s.y, yaw)} />
                        {s.pinned && (
                          <button type="button" className="btn btn-sm" disabled={loading} aria-label={`${s.entity_id} 자동으로 되돌리기`} onClick={() => onRevert(s.entity_id!)}>
                            자동으로 되돌리기
                          </button>
                        )}
                      </span>
                    )}
                  </li>
                ))}
              </ul>
            )}

            {failedPins.length > 0 && (
              <ul aria-label="지정이 거절된 정차 위치" style={{ listStyle: 'none', margin: 0, padding: 0, display: 'flex', flexDirection: 'column', gap: 6 }}>
                {failedPins.map((x) => (
                  <li key={x.id} style={{ display: 'flex', flexWrap: 'wrap', alignItems: 'center', gap: '3px 8px', fontSize: 12.5 }}>
                    <span className="mono">{x.entity_id}</span>
                    <Badge tone="red">거절</Badge>
                    <span style={{ color: '#d93a32' }}>{x.reason}</span>
                    <DirectionButtons name={x.entity_id!} activeName={x.pin!.yaw_deg === null ? null : compassOf(x.pin!.yaw_deg, 1)} disabled={loading} onPick={(yaw) => onSetYaw(x.entity_id!, x.pin!.x, x.pin!.y, yaw)} />
                    <button type="button" className="btn btn-sm" disabled={loading} aria-label={`${x.entity_id} 자동으로 되돌리기`} onClick={() => onRevert(x.entity_id!)}>
                      자동으로 되돌리기
                    </button>
                  </li>
                ))}
              </ul>
            )}

            {graph.warnings.length > 0 && (
              <details>
                <summary style={{ fontSize: 12.5, cursor: 'pointer' }}>경고 {graph.warnings.length}개</summary>
                <ul style={{ margin: '6px 0 0', paddingLeft: 18, fontSize: 12.5 }}>
                  {graph.warnings.map((w, i) => (
                    <li key={i}>{w}</li>
                  ))}
                </ul>
              </details>
            )}
          </>
        )}
      </div>
    </section>
  )
}
