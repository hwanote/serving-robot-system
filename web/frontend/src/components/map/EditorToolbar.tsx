import type { Dispatch } from 'react'
import type { EntityType, ZoneType } from '../../types/contracts'
import { ENTITY_TYPE_LABEL, ZONE_NOT_APPLIED_NOTE } from '../../utils/labels'
import type { EditorAction, EditorState, EditorTool } from '../../utils/semanticEditor'
import { Icon } from '../common/Icon'
import { ENTITY_STYLE, WAYPOINT_COLOR, ZONE_STYLE } from './mapStyles'

const ADD_TOOLS: { type: EntityType; label: string }[] = [
  { type: 'table', label: 'Add Table' },
  { type: 'counter', label: 'Add Counter' },
  { type: 'kitchen', label: 'Add Kitchen' },
  { type: 'robot_wait', label: 'Add Robot Wait' },
]

const ZONE_TOOLS: { type: ZoneType; label: string }[] = [
  { type: 'exclusive', label: 'Exclusive Zone' },
  { type: 'no_go', label: 'No-Go Zone' },
  { type: 'slow', label: 'Slow Zone' },
]

interface Props {
  state: EditorState
  dispatch: Dispatch<EditorAction>
  disabled?: boolean
}

export function EditorToolbar({ state, dispatch, disabled }: Props) {
  const setTool = (tool: EditorTool) => dispatch({ type: 'set_tool', tool: state.tool === tool && tool !== 'select' ? 'select' : tool })
  const selectedEntity = state.selection?.kind === 'entity' ? state.selection.id : null
  const chip = (tool: EditorTool, content: React.ReactNode, title: string, extraDisabled = false) => (
    <button
      key={tool}
      type="button"
      className="tool-chip"
      aria-pressed={state.tool === tool}
      disabled={disabled || extraDisabled}
      title={title}
      onClick={() => setTool(tool)}
    >
      {content}
    </button>
  )

  return (
    <div className="toolbar" role="toolbar" aria-label="지도 편집 도구">
      <div className="toolbar-group">
        {chip('select', <><Icon name="cursor" size={14} /> 선택</>, '선택 / 이동 / 지도 이동 (V)')}
      </div>
      <div className="toolbar-group">
        {ADD_TOOLS.map((t) =>
          chip(
            `add_${t.type}`,
            <>
              <span className="swatch" style={{ background: ENTITY_STYLE[t.type].color }} /> {t.label}
            </>,
            `${ENTITY_TYPE_LABEL[t.type]} 추가 — 지도를 클릭`,
          ),
        )}
      </div>
      <div className="toolbar-group">
        {chip(
          'waypoint',
          <>
            <span className="swatch" style={{ background: WAYPOINT_COLOR, borderRadius: 999 }} /> Waypoint 지정
          </>,
          selectedEntity ? `${selectedEntity} 의 서빙 접근 위치 지정 — 지도를 클릭` : '먼저 객체를 선택하세요',
          !selectedEntity && state.tool !== 'waypoint',
        )}
      </div>
      <div className="toolbar-group">
        {ZONE_TOOLS.map((t) =>
          chip(
            `zone_${t.type}`,
            <>
              <span className="swatch" style={{ background: ZONE_STYLE[t.type].color, opacity: 0.7 }} /> {t.label} <span className="chip-soon">준비 중</span>
            </>,
            `지도를 클릭해 꼭짓점 지정, Enter 또는 첫 점 클릭으로 완료 — ${ZONE_NOT_APPLIED_NOTE}`,
          ),
        )}
      </div>
      <div className="toolbar-group">
        <button
          type="button"
          className="tool-chip"
          disabled={disabled || (!state.past.length && !state.zoneDraft.length)}
          onClick={() => dispatch({ type: 'undo' })}
          title="되돌리기 (Ctrl+Z)"
        >
          <Icon name="refresh" size={14} /> 되돌리기
        </button>
        <button
          type="button"
          className="tool-chip"
          disabled={disabled || !state.selection}
          onClick={() => dispatch({ type: 'delete_selection' })}
          title="선택 삭제 (Delete)"
        >
          <Icon name="trash" size={14} /> 삭제
        </button>
      </div>
    </div>
  )
}
