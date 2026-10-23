import { useState, type Dispatch } from 'react'
import { ZONE_TYPES, type SemanticMap } from '../../types/contracts'
import { roundTo } from '../../utils/coordinates'
import { ENTITY_TYPE_LABEL, ZONE_NOT_APPLIED_NOTE, ZONE_TYPE_LABEL } from '../../utils/labels'
import { entityDisplayPose, isValidTopSize, mmToM, mToMm, TOP_SIZE_MAX_M, waypointForEntity } from '../../utils/semantic'
import type { EditorAction, Selection } from '../../utils/semanticEditor'
import { Icon } from '../common/Icon'

/** 숫자 입력: 입력 중에는 문자열 유지, blur/Enter 시 유효하면 반영 */
function NumberField({ label, value, unit, onCommit, step = 0.01 }: { label: string; value: number; unit: string; onCommit: (v: number) => void; step?: number }) {
  const [draft, setDraft] = useState<string | null>(null)
  const commit = () => {
    if (draft === null) return
    const n = Number(draft)
    if (draft.trim() !== '' && Number.isFinite(n)) onCommit(n)
    setDraft(null)
  }
  return (
    <label className="field">
      <span className="field-label">
        {label} <span className="muted">({unit})</span>
      </span>
      <input
        className="input input-sm mono"
        type="number"
        step={step}
        value={draft ?? String(value)}
        onChange={(e) => setDraft(e.target.value)}
        onBlur={commit}
        onKeyDown={(e) => {
          if (e.key === 'Enter') commit()
          if (e.key === 'Escape') setDraft(null)
          e.stopPropagation()
        }}
      />
    </label>
  )
}

/**
 * 테이블 상단 크기 입력 (mm). 값이 없으면 빈칸으로 두고 기본값을 채우지 않는다.
 * 내부 저장 단위는 m (1 mm 정밀도).
 */
function MmField({ label, valueM, onCommit }: { label: string; valueM: number | undefined; onCommit: (m: number) => void }) {
  const [draft, setDraft] = useState<string | null>(null)
  const shown = valueM && valueM > 0 ? String(mToMm(valueM)) : ''
  const commit = () => {
    if (draft === null) return
    const n = Number(draft)
    if (draft.trim() !== '' && Number.isFinite(n) && n > 0) onCommit(mmToM(n))
    setDraft(null)
  }
  const invalid = valueM !== undefined && !(valueM > 0 && valueM <= TOP_SIZE_MAX_M)
  return (
    <label className="field">
      <span className="field-label">
        {label} <span className="muted">(mm)</span>
      </span>
      <input
        className="input input-sm mono"
        type="number"
        min={1}
        max={TOP_SIZE_MAX_M * 1000}
        step={10}
        inputMode="numeric"
        placeholder="예: 800"
        aria-label={`${label} (mm)`}
        aria-invalid={invalid || undefined}
        value={draft ?? shown}
        onChange={(e) => setDraft(e.target.value)}
        onBlur={commit}
        onKeyDown={(e) => {
          if (e.key === 'Enter') commit()
          if (e.key === 'Escape') setDraft(null)
          e.stopPropagation()
        }}
      />
    </label>
  )
}

function LabelField({ id, value, dispatch }: { id: string; value: string; dispatch: Dispatch<EditorAction> }) {
  const [draft, setDraft] = useState<string | null>(null)
  const commit = () => {
    if (draft !== null && draft !== value) dispatch({ type: 'set_label', id, label: draft })
    setDraft(null)
  }
  return (
    <label className="field">
      <span className="field-label">표시 이름 (label)</span>
      <input
        className="input input-sm"
        value={draft ?? value}
        maxLength={40}
        onChange={(e) => setDraft(e.target.value)}
        onBlur={commit}
        onKeyDown={(e) => {
          if (e.key === 'Enter') commit()
          e.stopPropagation()
        }}
      />
    </label>
  )
}

const deg = (rad: number) => roundTo((rad * 180) / Math.PI, 1)
const rad = (d: number) => (d * Math.PI) / 180

export function Inspector({ doc, selection, dispatch }: { doc: SemanticMap; selection: Selection; dispatch: Dispatch<EditorAction> }) {
  if (!selection) {
    return <p className="muted" style={{ margin: 0 }}>지도에서 객체·Waypoint·구역을 선택하면 속성을 편집할 수 있습니다.</p>
  }

  if (selection.kind === 'entity') {
    const entity = doc.entities.find((e) => e.id === selection.id)
    if (!entity) return null
    const pose = entityDisplayPose(doc, entity)
    const wp = waypointForEntity(doc, entity.id)
    return (
      <div style={{ display: 'flex', flexDirection: 'column', gap: 10 }} key={entity.id}>
        <dl className="kv">
          <dt>ID</dt>
          <dd className="mono">{entity.id}</dd>
          <dt>타입</dt>
          <dd>
            {ENTITY_TYPE_LABEL[entity.type]} <span className="mono muted">({entity.type})</span>
          </dd>
        </dl>
        <LabelField id={entity.id} value={entity.label} dispatch={dispatch} />
        {pose && (
          <div className="pose-inputs" style={{ gridTemplateColumns: '1fr 1fr' }}>
            <NumberField label="x" unit="m" value={roundTo(pose.x)} onCommit={(x) => dispatch({ type: 'move_entity', id: entity.id, x, y: pose.y })} />
            <NumberField label="y" unit="m" value={roundTo(pose.y)} onCommit={(y) => dispatch({ type: 'move_entity', id: entity.id, x: pose.x, y })} />
          </div>
        )}
        {entity.type === 'table' && (
          <div style={{ display: 'flex', flexDirection: 'column', gap: 6 }} data-testid="top-size">
            <div className="pose-inputs" style={{ gridTemplateColumns: '1fr 1fr' }}>
              <MmField label="상단 가로" valueM={entity.top_size_m?.x} onCommit={(x) => dispatch({ type: 'set_top_size', id: entity.id, x })} />
              <MmField label="상단 세로" valueM={entity.top_size_m?.y} onCommit={(y) => dispatch({ type: 'set_top_size', id: entity.id, y })} />
            </div>
            {entity.top_size_m && isValidTopSize(entity.top_size_m) ? (
              <p className="muted" style={{ margin: 0, fontSize: 12 }}>
                가로는 지도 x축, 세로는 y축 방향이에요. 비스듬한 테이블은 전체를 감싸는 x·y축 정렬 사각형 크기를 입력하세요.
              </p>
            ) : (
              <p className="field-error" role="alert" style={{ margin: 0, fontSize: 12 }}>
                상단 가로·세로를 입력해야 저장할 수 있어요. 비스듬한 테이블은 전체를 감싸는 x·y축 정렬 사각형 크기를 입력하세요.
              </p>
            )}
          </div>
        )}
        <div className="alert alert-info" style={{ fontSize: 12.5 }}>
          {wp ? (
            <span>
              서빙 접근 위치 <span className="mono">{wp.id}</span> — 파란 화살표를 드래그하거나 <b>Waypoint 지정</b> 도구로 다시 지정하세요.
            </span>
          ) : (
            <span>Waypoint가 없습니다. <b>Waypoint 지정</b> 도구로 로봇 접근 위치를 지정하세요.</span>
          )}
        </div>
        <button type="button" className="btn btn-sm btn-danger" onClick={() => dispatch({ type: 'delete_selection' })}>
          <Icon name="trash" size={14} /> 객체 삭제 (Waypoint 포함)
        </button>
      </div>
    )
  }

  if (selection.kind === 'waypoint') {
    const wp = doc.waypoints.find((w) => w.id === selection.id)
    if (!wp) return null
    const set = (patch: Partial<typeof wp.pose>) => dispatch({ type: 'set_waypoint', entityId: wp.entity_id, pose: { ...wp.pose, ...patch } })
    return (
      <div style={{ display: 'flex', flexDirection: 'column', gap: 10 }} key={wp.id}>
        <dl className="kv">
          <dt>ID</dt>
          <dd className="mono">{wp.id}</dd>
          <dt>entity_id</dt>
          <dd>
            <button type="button" className="btn btn-sm btn-ghost mono" onClick={() => dispatch({ type: 'select', selection: { kind: 'entity', id: wp.entity_id } })}>
              {wp.entity_id}
            </button>
          </dd>
        </dl>
        <div className="pose-inputs">
          <NumberField label="x" unit="m" value={roundTo(wp.pose.x)} onCommit={(x) => set({ x })} />
          <NumberField label="y" unit="m" value={roundTo(wp.pose.y)} onCommit={(y) => set({ y })} />
          <NumberField label="yaw" unit="°" step={1} value={deg(wp.pose.yaw)} onCommit={(d) => set({ yaw: rad(d) })} />
        </div>
        <p className="muted" style={{ margin: 0, fontSize: 12 }}>
          yaw = {roundTo(wp.pose.yaw, 4)} rad · 지도에서 흰 원 핸들을 드래그해 방향을 조절할 수 있습니다.
        </p>
        <button type="button" className="btn btn-sm btn-danger" onClick={() => dispatch({ type: 'delete_selection' })}>
          <Icon name="trash" size={14} /> Waypoint 삭제
        </button>
      </div>
    )
  }

  const zone = doc.zones.find((z) => z.id === selection.id)
  if (!zone) return null
  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: 10 }} key={zone.id}>
      <dl className="kv">
        <dt>ID</dt>
        <dd className="mono">{zone.id}</dd>
        <dt>꼭짓점</dt>
        <dd>{zone.polygon.length}개</dd>
      </dl>
      <label className="field">
        <span className="field-label">구역 타입</span>
        <select className="select input-sm" value={zone.type} onChange={(e) => dispatch({ type: 'set_zone_type', id: zone.id, zoneType: e.target.value as typeof zone.type })}>
          {ZONE_TYPES.map((t) => (
            <option key={t} value={t}>
              {ZONE_TYPE_LABEL[t]} ({t})
            </option>
          ))}
        </select>
      </label>
      <p className="zone-soon-note" role="note">
        {ZONE_NOT_APPLIED_NOTE}
      </p>
      <div style={{ maxHeight: 150, overflow: 'auto' }}>
        <table className="table mono" style={{ fontSize: 12 }}>
          <thead>
            <tr>
              <th>#</th>
              <th>x (m)</th>
              <th>y (m)</th>
            </tr>
          </thead>
          <tbody>
            {zone.polygon.map(([x, y], i) => (
              <tr key={i} style={selection.vertex === i ? { background: 'var(--accent-soft)' } : undefined}>
                <td>{i}</td>
                <td>{x.toFixed(3)}</td>
                <td>{y.toFixed(3)}</td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>
      <p className="muted" style={{ margin: 0, fontSize: 12 }}>
        꼭짓점(사각 핸들) 드래그로 이동, 변 중간점 클릭으로 꼭짓점 추가, 꼭짓점 선택 후 Delete로 삭제합니다.
      </p>
      <div className="btn-row">
        {selection.vertex !== undefined && (
          <button type="button" className="btn btn-sm" onClick={() => dispatch({ type: 'delete_zone_vertex', id: zone.id, index: selection.vertex! })}>
            꼭짓점 {selection.vertex} 삭제
          </button>
        )}
        <button
          type="button"
          className="btn btn-sm btn-danger"
          onClick={() => {
            dispatch({ type: 'select', selection: { kind: 'zone', id: zone.id } })
            dispatch({ type: 'delete_selection' })
          }}
        >
          <Icon name="trash" size={14} /> 구역 삭제
        </button>
      </div>
    </div>
  )
}
