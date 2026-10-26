import type { Dispatch } from 'react'
import type { SemanticMap } from '../../types/contracts'
import { ENTITY_TYPE_LABEL, ZONE_TYPE_LABEL } from '../../utils/labels'
import type { EditorAction, Selection } from '../../utils/semanticEditor'
import { ENTITY_STYLE, ZONE_STYLE } from './mapStyles'

export function ObjectList({ doc, selection, dispatch }: { doc: SemanticMap; selection: Selection; dispatch: Dispatch<EditorAction> }) {
  if (!doc.entities.length && !doc.zones.length) {
    return <p className="muted" style={{ margin: '4px 12px 12px' }}>아직 지정된 객체나 구역이 없습니다. 상단 도구로 추가하세요.</p>
  }
  const entities = [...doc.entities].sort((a, b) => a.id.localeCompare(b.id))
  const hasWp = new Set(doc.waypoints.map((w) => w.entity_id))
  return (
    <ul className="object-list" aria-label="객체 목록">
      {entities.map((e) => (
        <li key={e.id}>
          <button
            type="button"
            aria-current={selection?.kind === 'entity' && selection.id === e.id}
            onClick={() => dispatch({ type: 'select', selection: { kind: 'entity', id: e.id } })}
          >
            <span className="swatch" style={{ background: ENTITY_STYLE[e.type].color }} />
            <span style={{ flex: 1, minWidth: 0 }}>
              <strong>{e.label || e.id}</strong> <span className="mono muted">{e.id}</span>
            </span>
            <span className="muted" style={{ fontSize: 12 }}>
              {ENTITY_TYPE_LABEL[e.type]}
              {!hasWp.has(e.id) && <span style={{ color: 'var(--warn)' }}> · WP 없음</span>}
            </span>
          </button>
        </li>
      ))}
      {doc.zones.map((z) => (
        <li key={z.id}>
          <button
            type="button"
            aria-current={selection?.kind === 'zone' && selection.id === z.id}
            onClick={() => dispatch({ type: 'select', selection: { kind: 'zone', id: z.id } })}
          >
            <span className="swatch" style={{ background: ZONE_STYLE[z.type].color, opacity: 0.7 }} />
            <span style={{ flex: 1 }} className="mono">
              {z.id}
            </span>
            <span className="muted" style={{ fontSize: 12 }}>
              {ZONE_TYPE_LABEL[z.type]} · {z.polygon.length}점
            </span>
          </button>
        </li>
      ))}
    </ul>
  )
}
