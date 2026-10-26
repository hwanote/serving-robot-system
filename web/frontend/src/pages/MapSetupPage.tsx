import { useCallback, useEffect, useMemo, useState } from 'react'
import { useBlocker } from 'react-router-dom'
import { Icon } from '../components/common/Icon'
import { EmptyState, ErrorState, InlineError, LoadingState } from '../components/common/StateViews'
import { Badge } from '../components/common/StatusBadge'
import { EditorToolbar } from '../components/map/EditorToolbar'
import { GeneratedGraphLayer, type DraggingStop } from '../components/map/GeneratedGraphLayer'
import { GraphPreviewPanel } from '../components/map/GraphPreviewPanel'
import { Inspector } from '../components/map/Inspector'
import { LivePanel } from '../components/map/LivePanel'
import { MappingPanel } from '../components/map/MappingPanel'
import { MapReviewPanel } from '../components/map/MapReviewPanel'
import { MapViewport, type MapOverlayContext, type MapPointerEvent } from '../components/map/MapViewport'
import { ZONE_STYLE } from '../components/map/mapStyles'
import { ObjectList } from '../components/map/ObjectList'
import { SemanticLayer, ZoneDraftLayer } from '../components/map/SemanticLayer'
import { resolveApiUrl } from '../config/env'
import { getSemantics } from '../api/semanticApi'
import { useGeneratedGraph } from '../hooks/useGeneratedGraph'
import { useMappingControl } from '../hooks/useMappingControl'
import { useSemanticEditor } from '../hooks/useSemanticEditor'
import { useStoreData } from '../state/storeDataContext'
import { useToast } from '../state/toastContext'
import { clampToImage, imageToMap, type Vec2 } from '../utils/coordinates'
import { ENTITY_TYPE_LABEL, ZONE_NOT_APPLIED_NOTE, ZONE_TYPE_LABEL } from '../utils/labels'
import { DEFAULT_GRAPH_LAYERS, type GraphLayers } from '../utils/graph'
import { entityDisplayPose } from '../utils/semantic'
import { editorReducer, serializeDoc, toolEntityType, toolZoneType, type EditorAction, type EditorTool } from '../utils/semanticEditor'

/** 첫 꼭짓점 근처(화면 px) 클릭 시 폴리곤 닫기 */
const CLOSE_DISTANCE_PX = 12

function toolHint(tool: EditorTool, draftCount: number, selectedEntity: string | null): string | null {
  const entityType = toolEntityType(tool)
  if (entityType) return `지도를 클릭해 ${ENTITY_TYPE_LABEL[entityType]}를 추가합니다. (Esc: 선택 도구)`
  const zoneType = toolZoneType(tool)
  if (zoneType) {
    if (draftCount === 0) return `${ZONE_TYPE_LABEL[zoneType]}: 지도를 클릭해 첫 꼭짓점을 지정하세요. (${ZONE_NOT_APPLIED_NOTE})`
    if (draftCount < 3) return `꼭짓점 ${draftCount}개 — 최소 3개 필요 (Backspace: 마지막 점 취소, Esc: 취소)`
    return `꼭짓점 ${draftCount}개 — 첫 점 클릭 또는 Enter로 완료`
  }
  if (tool === 'waypoint') {
    return selectedEntity
      ? `${selectedEntity}의 서빙 접근 위치를 클릭하세요. 방향은 객체를 바라보도록 설정됩니다.`
      : '먼저 Waypoint를 지정할 객체를 선택하세요.'
  }
  return null
}

const isTypingTarget = (t: EventTarget | null) =>
  t instanceof HTMLElement && (t.tagName === 'INPUT' || t.tagName === 'TEXTAREA' || t.tagName === 'SELECT' || t.isContentEditable)

export function MapSetupPage() {
  const { storeId, rawMap, reloadRawMap, semantics, reloadSemantics } = useStoreData()
  const editor = useSemanticEditor()
  const { replaceDraft } = editor
  // 새 지도가 매장 지도로 반영되면: 지도와 편집 화면을 새 지도 기준으로 다시 읽는다
  // (편집 중이던 내용은 버려진다. 이전 초안은 서버 백업에 남아 있다)
  const handleAdopted = useCallback(async () => {
    await reloadRawMap()
    try {
      const doc = await getSemantics(storeId)
      if (doc) replaceDraft(doc)
    } catch {
      await reloadSemantics()
    }
  }, [storeId, reloadRawMap, reloadSemantics, replaceDraft])
  const mapping = useMappingControl({ onAdopted: handleAdopted })
  const { state, dispatch, dirty } = editor
  const { notify } = useToast()
  const docKey = useMemo(() => serializeDoc(state.doc), [state.doc])
  const graphView = useGeneratedGraph(state.doc.store_id, docKey)
  const [graphLayers, setGraphLayers] = useState<GraphLayers>(DEFAULT_GRAPH_LAYERS)
  const [dragStop, setDragStop] = useState<DraggingStop | null>(null)
  const { check: checkGraph } = graphView
  const [hover, setHover] = useState<Vec2 | null>(null)
  const map = rawMap.data
  const selectedEntity = state.selection?.kind === 'entity' ? state.selection.id : null
  // waypoint 도구: 선택된 객체, 또는 방금 지정한 Waypoint 의 소유 객체를 대상으로 유지
  const activeWaypointTarget =
    state.tool !== 'waypoint'
      ? null
      : (selectedEntity ??
        (state.selection?.kind === 'waypoint'
          ? (state.doc.waypoints.find((w) => w.id === state.selection!.id)?.entity_id ?? null)
          : null))

  // ----- 편집 내용 보호 -----
  const blocker = useBlocker(dirty)
  useEffect(() => {
    if (blocker.state !== 'blocked') return
    if (window.confirm('저장하지 않은 매장 설정이 있습니다. 페이지를 떠나시겠습니까?')) blocker.proceed()
    else blocker.reset()
  }, [blocker])
  useEffect(() => {
    if (!dirty) return
    const onBeforeUnload = (e: BeforeUnloadEvent) => e.preventDefault()
    window.addEventListener('beforeunload', onBeforeUnload)
    return () => window.removeEventListener('beforeunload', onBeforeUnload)
  }, [dirty])

  // 편집기 안내/오류 메시지 → 토스트
  useEffect(() => {
    if (state.notice) notify(state.notice, 'error')
  }, [state.notice, notify])

  // ----- 지도 클릭 -----
  const onMapClick = useCallback(
    (e: MapPointerEvent) => {
      const tool = state.tool
      const entityType = toolEntityType(tool)
      const zoneType = toolZoneType(tool)
      if (tool === 'select') {
        dispatch({ type: 'select', selection: null })
        return
      }
      if (!e.inside) {
        notify('지도 범위 밖에는 지정할 수 없습니다.', 'error')
        return
      }
      if (entityType) {
        dispatch({ type: 'add_entity', entityType, x: e.map.x, y: e.map.y })
        return
      }
      if (zoneType) {
        // 첫 꼭짓점 근처 클릭(닫기)은 오버레이의 zone-close-target 이 처리
        dispatch({ type: 'zone_add_point', point: [e.map.x, e.map.y] })
        return
      }
      if (tool === 'waypoint') {
        const target = activeWaypointTarget
        const entity = target ? state.doc.entities.find((x) => x.id === target) : null
        if (!entity) {
          notify('먼저 Waypoint를 지정할 객체를 선택하세요.', 'error')
          return
        }
        const ep = entityDisplayPose(state.doc, entity)
        // 기본 방향: 접근 위치에서 객체를 바라봄
        const yaw = ep && (ep.x !== e.map.x || ep.y !== e.map.y) ? Math.atan2(ep.y - e.map.y, ep.x - e.map.x) : 0
        dispatch({ type: 'set_waypoint', entityId: entity.id, pose: { x: e.map.x, y: e.map.y, yaw } })
      }
    },
    [state.tool, state.doc, activeWaypointTarget, dispatch, notify],
  )

  // ----- 자동 정차 위치를 손으로 옮기기: 편집 내용에 반영하고 바로 생성기로 다시 검사 -----
  const applyStopAction = useCallback(
    (action: EditorAction) => {
      const next = editorReducer(state, action).doc
      dispatch(action)
      void checkGraph(next)
    },
    [state, dispatch, checkGraph],
  )

  const startStopDrag = useCallback(
    (ctx: MapOverlayContext, entityId: string, yaw: number, e: React.PointerEvent) => {
      const svg = (e.currentTarget as SVGElement).ownerSVGElement
      const rect = svg?.getBoundingClientRect()
      const start = { sx: e.clientX - (rect?.left ?? 0), sy: e.clientY - (rect?.top ?? 0) }
      let last: Vec2 | null = null
      ctx.beginDrag(e, {
        onMove: (ev) => {
          // 그냥 누르기만 한 경우(몇 px 이내)는 옮김으로 보지 않는다
          if (!last && Math.hypot(ev.screen.sx - start.sx, ev.screen.sy - start.sy) < 4) return
          last = imageToMap(ctx.geo, clampToImage(ctx.geo, ev.image))
          setDragStop({ entityId, x: last.x, y: last.y, yaw })
        },
        onEnd: () => {
          setDragStop(null)
          if (last) applyStopAction({ type: 'set_manual_stop', entityId, x: last.x, y: last.y, yaw })
        },
      })
    },
    [applyStopAction],
  )

  // ----- 단축키 -----
  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      if (isTypingTarget(e.target)) return
      if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === 'z') {
        e.preventDefault()
        dispatch({ type: 'undo' })
      } else if (e.key === 'Escape') {
        if (state.zoneDraft.length) dispatch({ type: 'zone_cancel' })
        else if (state.tool !== 'select') dispatch({ type: 'set_tool', tool: 'select' })
        else dispatch({ type: 'select', selection: null })
      } else if (e.key === 'Enter' && toolZoneType(state.tool)) {
        dispatch({ type: 'zone_finish' })
      } else if (e.key === 'Backspace' && state.zoneDraft.length) {
        e.preventDefault()
        dispatch({ type: 'zone_undo_point' })
      } else if ((e.key === 'Delete' || e.key === 'Backspace') && state.selection) {
        e.preventDefault()
        dispatch({ type: 'delete_selection' })
      } else if (e.key.toLowerCase() === 'v') {
        dispatch({ type: 'set_tool', tool: 'select' })
      }
    }
    window.addEventListener('keydown', onKey)
    return () => window.removeEventListener('keydown', onKey)
  }, [state.zoneDraft.length, state.tool, state.selection, dispatch])

  const zoneType = toolZoneType(state.tool)
  const hint = toolHint(state.tool, state.zoneDraft.length, activeWaypointTarget)
  const errors = editor.issues.filter((i) => i.level === 'error')
  const warnings = editor.issues.filter((i) => i.level === 'warning')
  const dataLoading = !rawMap.loaded || !semantics.loaded

  let mapArea: React.ReactNode
  if (dataLoading) mapArea = <LoadingState message="지도 정보를 불러오는 중…" />
  else if (rawMap.error && !map) mapArea = <ErrorState message={rawMap.error} onRetry={() => void reloadRawMap()} />
  else if (!map)
    mapArea = (
      <EmptyState title="저장된 지도가 없습니다.">
        <span>오른쪽 매핑 패널에서 Start Mapping → Stop Mapping → Save Map 순서로 지도를 생성하세요.</span>
      </EmptyState>
    )
  else
    mapArea = (
      <>
        <EditorToolbar state={state} dispatch={dispatch} />
        <div style={{ position: 'relative', flex: 1, display: 'flex', minHeight: 0 }}>
          <MapViewport
            geometry={map}
            imageUrl={resolveApiUrl(map.image_url)}
            revisionKey={map.map_revision}
            cursor={state.tool === 'select' ? 'grab' : 'crosshair'}
            onMapClick={onMapClick}
            onMapHover={(e) => setHover(e && e.inside ? e.map : null)}
            renderOverlay={(ctx) => {
              // 작성 중 Zone: 첫 점 근처 클릭으로 닫기 위한 표시
              const first = state.zoneDraft[0]
              const nearFirst =
                first && hover && state.zoneDraft.length >= 3
                  ? Math.hypot(ctx.toImage(first[0], first[1]).u - ctx.toImage(hover.x, hover.y).u, ctx.toImage(first[0], first[1]).v - ctx.toImage(hover.x, hover.y).v) *
                      ctx.view.scale <
                    CLOSE_DISTANCE_PX
                  : false
              return (
                <>
                  <SemanticLayer ctx={ctx} doc={state.doc} selection={state.selection} interactive={state.tool === 'select'} dispatch={dispatch} showWaypoints={graphLayers.myWaypoints} hideManualWaypoints={graphView.graph !== null && graphLayers.stops} />
                  {/* 정차 화살표는 맨 위에 그린다: 파란 Waypoint 보다 먼저 눌려야 끌 수 있다 */}
                  {graphView.graph && (
                    <GeneratedGraphLayer
                      ctx={ctx}
                      graph={graphView.graph}
                      layers={graphLayers}
                      stale={graphView.stale}
                      interactive={state.tool === 'select'}
                      dragging={dragStop}
                      onStopDragStart={(entityId, yaw, e) => startStopDrag(ctx, entityId, yaw, e)}
                    />
                  )}
                  {zoneType && (
                    <>
                      <ZoneDraftLayer ctx={ctx} points={state.zoneDraft} hover={nearFirst ? { x: first![0], y: first![1] } : hover} color={ZONE_STYLE[zoneType].color} />
                      {nearFirst && (
                        <circle
                          cx={ctx.toImage(first![0], first![1]).u}
                          cy={ctx.toImage(first![0], first![1]).v}
                          r={ctx.px(12)}
                          fill="transparent"
                          stroke={ZONE_STYLE[zoneType].color}
                          strokeWidth={ctx.px(2)}
                          style={{ cursor: 'pointer' }}
                          data-testid="zone-close-target"
                          onPointerDown={(e) => {
                            e.stopPropagation()
                            dispatch({ type: 'zone_finish' })
                          }}
                        />
                      )}
                    </>
                  )}
                </>
              )
            }}
          />
          {hint && <div className="map-hint">{hint}</div>}
          {zoneType && state.zoneDraft.length > 0 && (
            <div className="map-draft-actions">
              <button type="button" className="btn btn-sm btn-primary" disabled={state.zoneDraft.length < 3} onClick={() => dispatch({ type: 'zone_finish' })}>
                <Icon name="check" size={14} /> 구역 완료
              </button>
              <button type="button" className="btn btn-sm" onClick={() => dispatch({ type: 'zone_cancel' })}>
                취소
              </button>
            </div>
          )}
        </div>
      </>
    )

  return (
    <div className="page page-full">
      <div className="page-header">
        <div>
          <h1>지도 설정</h1>
          <p>매핑으로 Raw Map을 만들고, 테이블·대기 위치·로봇 접근 위치(Waypoint)·구역을 지정합니다. 모든 좌표는 ROS map frame(m, rad)으로 저장됩니다.</p>
        </div>
      </div>

      {mapping.draftReady && <MapReviewPanel mapping={mapping} />}

      <div className="setup-layout">
        <section className="card setup-map" aria-label="지도 편집">
          <div className="card-header">
            <h2>Raw Map · Semantic 편집</h2>
            <div className="btn-row">
              {map && <span className="mono muted">map r{map.map_revision}</span>}
              {map && !editor.revision.ok ? (
                <Badge tone="amber" dot>
                  갱신 필요
                </Badge>
              ) : dirty ? (
                <Badge tone="amber" dot>
                  저장 안 됨
                </Badge>
              ) : (
                map && <Badge tone="green">저장됨</Badge>
              )}
            </div>
          </div>
          {mapArea}
        </section>

        <div className="setup-side">
          <MappingPanel mapping={mapping} />

          <section className="card" aria-labelledby="setup-title">
            <div className="card-header">
              <h2 id="setup-title">매장 설정 저장</h2>
              <span className="mono muted">semantic r{state.doc.map_revision}</span>
            </div>
            <div className="card-body" style={{ display: 'flex', flexDirection: 'column', gap: 10 }}>
              {semantics.error && <InlineError message={`Semantic Map 조회 실패: ${semantics.error}`} />}
              {map && !editor.revision.ok && (
                <div className="alert alert-warn" role="alert" style={{ flexDirection: 'column' }}>
                  <span>{editor.revision.message}</span>
                  <button type="button" className="btn btn-sm" onClick={editor.rebase}>
                    현재 지도(r{map.map_revision}) 기준으로 갱신
                  </button>
                </div>
              )}
              {errors.length > 0 && (
                <ul className="issue-list alert alert-error" aria-label="저장 불가 오류">
                  {errors.map((i, n) => (
                    <li key={n}>• {i.message}</li>
                  ))}
                </ul>
              )}
              {warnings.length > 0 && (
                <ul className="issue-list alert alert-warn" aria-label="경고">
                  {warnings.map((i, n) => (
                    <li key={n}>• {i.message}</li>
                  ))}
                </ul>
              )}
              <InlineError message={editor.saveError} />
              <div className="btn-row">
                <button type="button" className="btn btn-primary" disabled={!editor.canSave} onClick={() => void editor.save()}>
                  {editor.saving ? <span className="spinner spinner-sm" /> : <Icon name="save" size={14} />}
                  Save Store Setup
                </button>
                <button
                  type="button"
                  className="btn"
                  disabled={!dirty || editor.saving}
                  onClick={() => {
                    if (window.confirm('저장하지 않은 변경 내용을 버리고 서버의 설정을 다시 불러올까요?')) void editor.discard()
                  }}
                >
                  변경 취소
                </button>
                {!dirty && semantics.error && (
                  <button type="button" className="btn btn-sm" onClick={() => void reloadSemantics()}>
                    다시 불러오기
                  </button>
                )}
              </div>
            </div>
          </section>

          {map && (
            <GraphPreviewPanel
              view={graphView}
              onCheck={() => void checkGraph(state.doc)}
              onClear={graphView.clear}
              layers={graphLayers}
              onLayers={setGraphLayers}
              onSetYaw={(entityId, x, y, yaw) => applyStopAction({ type: 'set_manual_stop', entityId, x, y, yaw })}
              onRevert={(entityId) => applyStopAction({ type: 'clear_manual_stop', entityId })}
            />
          )}

          {map && <LivePanel storeId={state.doc.store_id} dirty={dirty} savedKey={semantics.data} onRestored={editor.replaceDraft} />}

          <section className="card" aria-labelledby="inspector-title">
            <div className="card-header">
              <h2 id="inspector-title">선택 항목</h2>
            </div>
            <div className="card-body">
              <Inspector doc={state.doc} selection={state.selection} dispatch={dispatch} />
            </div>
          </section>

          <section className="card" aria-labelledby="objects-title">
            <div className="card-header">
              <h2 id="objects-title">객체 · 구역</h2>
              <span className="muted">
                {state.doc.entities.length}개 · {state.doc.zones.length}구역
              </span>
            </div>
            <div style={{ padding: '6px 0' }}>
              <ObjectList doc={state.doc} selection={state.selection} dispatch={dispatch} />
            </div>
          </section>
        </div>
      </div>
    </div>
  )
}
