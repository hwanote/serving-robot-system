import { useEffect, useRef, useState, type PointerEvent as ReactPointerEvent } from 'react'
import { toApiError } from '../../api/client'
import { getMapReview } from '../../api/mappingApi'
import { resolveApiUrl } from '../../config/env'
import type { MappingControl } from '../../hooks/useMappingControl'
import { useStoreData } from '../../state/storeDataContext'
import type { MapReviewData as ReviewData, ReviewSuggestion as Suggestion } from '../../types/provisional'
import { MapViewport, type MapPointerEvent, type MapOverlayContext } from './MapViewport'

type Point = { x: number; y: number }

type Stroke = {
  x1: number
  y1: number
  x2: number
  y2: number
  value: number
  suggestionId?: string
  points?: Point[]
}

type DrawMode = 'move' | 'wall' | 'free' | 'unknown'

const COLOR: Record<Exclude<DrawMode, 'move'>, string> = {
  wall: '#e94699',
  free: '#16a34a',
  unknown: '#94a3b8',
}

function getPixel(e: MapPointerEvent, data: ReviewData): Point {
  return {
    x: Math.min(data.width - 1, Math.max(0, Math.floor(e.image.u))),
    y: Math.min(data.height - 1, Math.max(0, Math.floor(e.image.v))),
  }
}

function pointerPixel(
  e: ReactPointerEvent<SVGRectElement>,
  ctx: MapOverlayContext,
  data: ReviewData,
): Point | null {
  const svg = e.currentTarget.ownerSVGElement
  if (!svg) return null
  const rect = svg.getBoundingClientRect()
  const u = (e.clientX - rect.left - ctx.view.offsetX) / ctx.view.scale
  const v = (e.clientY - rect.top - ctx.view.offsetY) / ctx.view.scale
  if (u < 0 || v < 0 || u >= data.width || v >= data.height) return null
  return { x: Math.floor(u), y: Math.floor(v) }
}

/**
 * 자동 매핑으로 만든 지도 초안을 확인·보정하고 "Save Map" 으로 저장한다 (맵핑팀 제작, 통합하며 수정).
 * 저장하면 백엔드가 이 지도를 매장 지도로 반영한다 (주문이 진행 중이면 반영만 보류).
 */
export function MapReviewPanel({ mapping }: { mapping: MappingControl }) {
  const { storeId } = useStoreData()
  const [data, setData] = useState<ReviewData | null>(null)
  const [error, setError] = useState<string | null>(null)
  const [mode, setMode] = useState<DrawMode>('move')
  const [freehand, setFreehand] = useState(false)
  const drawingRef = useRef<Point[]>([])
  const [drawing, setDrawing] = useState<Point[]>([])
  const [selectedStroke, setSelectedStroke] = useState<number | null>(null)
  const [start, setStart] = useState<Point | null>(null)
  const [hover, setHover] = useState<Point | null>(null)
  const [strokes, setStrokes] = useState<Stroke[]>([])
  const [history, setHistory] = useState<Stroke[][]>([])
  const [ignored, setIgnored] = useState<string[]>([])
  const [selected, setSelected] = useState<string | null>(null)
  const [confirmed, setConfirmed] = useState(false)
  const [saving, setSaving] = useState(false)
  const [saved, setSaved] = useState(false)
  const [saveError, setSaveError] = useState<string | null>(null)
  const [pendingReason, setPendingReason] = useState<string | null>(null)
  const mapRef = useRef<HTMLDivElement>(null)

  useEffect(() => {
    let alive = true
    void getMapReview(storeId)
      .then((value) => {
        if (alive) setData(value)
      })
      .catch((cause: unknown) => {
        if (alive) setError(toApiError(cause).message)
      })
    return () => { alive = false }
  }, [storeId])

  function update(next: Stroke[]) {
    setHistory((old) => [...old, strokes])
    setStrokes(next)
    setSelectedStroke(null)
    setSaved(false)
    setSaveError(null)
  }

  function onClickMap(e: MapPointerEvent) {
    if (!data || !e.inside || mode === 'move' || freehand || saved) return

    const end = getPixel(e, data)
    if (!start) {
      setStart(end)
      return
    }

    if (Math.hypot(end.x - start.x, end.y - start.y)
        * data.resolution > 1.000001) {
      setSaveError('한 번에 1m까지만 그릴 수 있습니다.')
      setStart(null)
      return
    }

    if (strokes.length >= 100) {
      setSaveError('수정 횟수가 많습니다. 먼저 지도를 저장해 주세요.')
      setStart(null)
      return
    }

    const value = mode === 'wall' ? 100
      : mode === 'free' ? 0 : 255

    update([...strokes, {
      x1: start.x, y1: start.y,
      x2: end.x, y2: end.y, value,
    }])
    setStart(null)
  }

  function onDrawStart(
    e: ReactPointerEvent<SVGRectElement>,
    ctx: MapOverlayContext,
  ) {
    if (!data || !freehand || mode === 'move' || saved || e.button !== 0)
      return
    const point = pointerPixel(e, ctx, data)
    if (!point) return
    e.stopPropagation()
    e.preventDefault()
    e.currentTarget.setPointerCapture(e.pointerId)
    drawingRef.current = [point]
    setDrawing([point])
    setSaveError(null)
  }

  function onDrawMove(
    e: ReactPointerEvent<SVGRectElement>,
    ctx: MapOverlayContext,
  ) {
    if (!data || !e.currentTarget.hasPointerCapture(e.pointerId)) return
    e.stopPropagation()
    const point = pointerPixel(e, ctx, data)
    if (!point) return

    const next = [...drawingRef.current]
    const last = next[next.length - 1]
    if (!last) return

    const dx = point.x - last.x
    const dy = point.y - last.y
    const steps = Math.max(Math.abs(dx), Math.abs(dy))
    if (!steps) return

    for (let i = 1; i <= steps; i++) {
      if (next.length >= 3000) {
        setSaveError('곡선 하나의 길이가 너무 깁니다. 나누어 그려 주세요.')
        break
      }
      const p = {
        x: Math.round(last.x + dx * i / steps),
        y: Math.round(last.y + dy * i / steps),
      }
      const prev = next[next.length - 1]
      if (p.x !== prev.x || p.y !== prev.y) next.push(p)
    }
    drawingRef.current = next
    setDrawing(next)
  }

  function onDrawEnd(
    e: ReactPointerEvent<SVGRectElement>,
    ctx: MapOverlayContext,
  ) {
    if (!e.currentTarget.hasPointerCapture(e.pointerId)) return
    onDrawMove(e, ctx)
    e.stopPropagation()
    e.currentTarget.releasePointerCapture(e.pointerId)

    const polyline = [...drawingRef.current]
    drawingRef.current = []
    setDrawing([])

    if (!polyline.length || mode === 'move') return
    if (strokes.length >= 100) {
      setSaveError('수정 항목이 100개를 초과했습니다.')
      return
    }

    const first = polyline[0]
    const last = polyline[polyline.length - 1]
    const value = mode === 'wall' ? 100
      : mode === 'free' ? 0 : 255

    update([...strokes, {
      x1: first.x, y1: first.y,
      x2: last.x, y2: last.y,
      value,
      points: polyline,
    }])
  }

  function toggleSuggestion(s: Suggestion) {
    if (saved) return
    const applied = strokes.some((v) => v.suggestionId === s.id)
    if (applied) {
      update(strokes.filter((v) => v.suggestionId !== s.id))
      return
    }
    if (strokes.length >= 100) return
    update([...strokes, {
      x1: s.x1, y1: s.y1,
      x2: s.x2, y2: s.y2,
      value: 100,
      suggestionId: s.id,
    }])
    setIgnored((old) => old.filter((id) => id !== s.id))
  }

  async function finalize() {
    if (!data || !confirmed || saving || saved) return
    if (!window.confirm('수정한 지도를 저장하고 매장 지도로 반영할까요?\n(이전 지도는 백업에 남습니다.)')) return

    setSaving(true)
    setSaveError(null)
    try {
      const result = await mapping.save({
        source_hash: data.source_hash,
        confirmed: true,
        strokes: strokes.map(({ x1, y1, x2, y2, value, points }) =>
          ({ x1, y1, x2, y2, value, ...(points ? { points } : {}) })),
      })
      if (!result.review_result?.saved) throw new Error('지도 저장을 확인하지 못했습니다.')
      setSaved(true)
      setPendingReason(result.adopted ? null : (result.adopt_error?.message ?? '지도 반영이 보류되었습니다.'))
    } catch (cause) {
      setSaveError(toApiError(cause).message)
    } finally {
      setSaving(false)
    }
  }

  return (
    <section id="map-review" className="card" aria-label="지도 완성하기"
      style={{ marginBottom: 16 }}>
      <div className="card-header">
        <h2>지도 완성하기</h2>
      </div>
      <div className="card-body"
        style={{ display: 'flex', flexDirection: 'column', gap: 14 }}>
        <p>자동으로 만든 지도를 확인하고, 추천 수정을 적용하거나
          직접 보완한 뒤 최종 저장하세요.</p>

        {error && (
          <div className="alert alert-warn" role="alert">
            지도 연결 상태: {error}
          </div>
        )}

        {!data && !error && <p>지도를 불러오는 중입니다…</p>}

        {data && (
          <>
            <div className="btn-row">
              <button type="button"
                className={freehand ? 'btn btn-primary' : 'btn'}
                aria-pressed={freehand}
                disabled={mode === 'move' || saved}
                onClick={() => {
                  setFreehand((v) => !v)
                  setStart(null)
                }}>
                자유곡선 {freehand ? '켜짐' : '꺼짐'}
              </button>
              {([
                ['move', '지도 이동'],
                ['wall', '벽 그리기'],
                ['free', '확인된 통로 그리기'],
                ['unknown', '미확인 공간 표시'],
              ] as const).map(([key, label]) => (
                <button key={key} type="button"
                  className={mode === key ? 'btn btn-primary' : 'btn'}
                  aria-pressed={mode === key}
                  disabled={saved}
                  onClick={() => {
                    setMode(key)
                    setStart(null)
                  }}>
                  {label}
                </button>
              ))}
              <button type="button" className="btn"
                disabled={!history.length || saved}
                onClick={() => {
                  const previous = history[history.length - 1]
                  setHistory(history.slice(0, -1))
                  setStrokes(previous)
                  setStart(null)
                }}>
                되돌리기
              </button>
              <button type="button" className="btn"
                disabled={!strokes.length || saved}
                onClick={() => {
                  if (window.confirm('지도 수정을 모두 취소할까요?')) {
                    update([])
                    setStart(null)
                  }
                }}>
                수정 모두 취소
              </button>
            </div>

            <p style={{ fontSize: 13 }}>
              {mode === 'move'
                ? '지도를 드래그해 이동하고 휠로 확대할 수 있습니다.'
                : freehand
                  ? '마우스를 누른 채 움직여 곡선을 그리고, 놓으면 한 항목으로 저장됩니다.'
                : start
                  ? '선의 끝점을 클릭하세요. (한 번에 최대 1m)'
                  : '선의 시작점과 끝점을 차례로 클릭하세요.'}
            </p>

            <div ref={mapRef}
              style={{ height: 450, minHeight: 450 }}>
              <MapViewport
                geometry={data}
                imageUrl={resolveApiUrl(data.image_url)}
                revisionKey={data.source_hash}
                cursor={mode === 'move' ? 'grab' : 'crosshair'}
                onMapClick={onClickMap}
                onMapHover={(e) =>
                  setHover(e && e.inside ? getPixel(e, data) : null)}
                renderOverlay={(ctx) => (
                  <g>
                    {freehand && mode !== 'move' && !saved && (
                      <rect
                        x={0} y={0}
                        width={data.width}
                        height={data.height}
                        fill="transparent"
                        onPointerDown={(e) => onDrawStart(e, ctx)}
                        onPointerMove={(e) => onDrawMove(e, ctx)}
                        onPointerUp={(e) => onDrawEnd(e, ctx)}
                        onPointerCancel={(e) => {
                          e.stopPropagation()
                          if (e.currentTarget.hasPointerCapture(e.pointerId))
                            e.currentTarget.releasePointerCapture(e.pointerId)
                          drawingRef.current = []
                          setDrawing([])
                        }}
                        style={{ cursor: 'crosshair', touchAction: 'none' }}
                      />
                    )}
                    {data.suggestions
                      .filter((s) => !ignored.includes(s.id))
                      .map((s, index) => {
                        const applied = strokes.some(
                          (v) => v.suggestionId === s.id)
                        return (
                          <g key={s.id}>
                            <line
                              x1={s.x1 + 0.5} y1={s.y1 + 0.5}
                              x2={s.x2 + 0.5} y2={s.y2 + 0.5}
                              stroke={applied ? '#16a34a' : '#f59e0b'}
                              strokeWidth={ctx.px(3)}
                              strokeDasharray={
                                applied ? undefined : `${ctx.px(6)} ${ctx.px(4)}`}
                              pointerEvents="none"
                            />
                            <circle
                              cx={(s.x1 + s.x2 + 1) / 2}
                              cy={(s.y1 + s.y2 + 1) / 2}
                              r={ctx.px(12)}
                              fill={selected === s.id ? '#b45309' : '#1d4ed8'}
                              stroke="#fff"
                              strokeWidth={ctx.px(2)}
                              onPointerDown={(e) => {
                                e.stopPropagation()
                                setSelected(s.id)
                              }}
                            />
                            <text
                              x={(s.x1 + s.x2 + 1) / 2}
                              y={(s.y1 + s.y2 + 1) / 2}
                              textAnchor="middle"
                              dominantBaseline="central"
                              fontSize={ctx.px(12)}
                              fontWeight="bold"
                              fill="white"
                              pointerEvents="none"
                            >{index + 1}</text>
                          </g>
                        )
                      })}
                    {strokes.map((s, i) => s.suggestionId ? null : (
                      <polyline key={i}
                        points={(s.points ?? [
                          { x: s.x1, y: s.y1 },
                          { x: s.x2, y: s.y2 },
                        ]).map((p) =>
                          `${p.x + 0.5},${p.y + 0.5}`).join(' ')}
                        fill="none"
                        stroke={s.value === 100 ? COLOR.wall
                          : s.value === 0 ? COLOR.free : COLOR.unknown}
                        strokeWidth={ctx.px(selectedStroke === i ? 5 : 3)}
                        strokeLinecap="round"
                        strokeLinejoin="round"
                        pointerEvents={freehand ? 'none' : 'stroke'}
                        onPointerDown={(e) => {
                          e.stopPropagation()
                          setSelectedStroke(i)
                        }}
                      />
                    ))}
                    {drawing.length > 0 && (
                      <polyline
                        points={drawing.map((p) =>
                          `${p.x + 0.5},${p.y + 0.5}`).join(' ')}
                        fill="none"
                        stroke={mode === 'wall' ? COLOR.wall
                          : mode === 'free' ? COLOR.free : COLOR.unknown}
                        strokeWidth={ctx.px(3)}
                        strokeLinecap="round"
                        strokeLinejoin="round"
                        pointerEvents="none"
                      />
                    )}
                    {start && hover && (
                      <line x1={start.x + 0.5}
                        y1={start.y + 0.5}
                        x2={hover.x + 0.5}
                        y2={hover.y + 0.5}
                        stroke="#06b6d4"
                        strokeWidth={ctx.px(2)}
                        pointerEvents="none"
                      />
                    )}
                  </g>
                )}
              />
            </div>

            <div style={{ padding: 12, border: '1px solid #5a6776',
              borderRadius: 8 }}>
              <h3>추천 수정 사항 ({data.suggestions.length}개)</h3>
              <p>끊어진 것으로 보이는 벽을 추천합니다.
                실제 구조를 확인한 뒤 적용하세요.</p>
              {data.suggestions.length === 0 && (
                <p>현재 자동 추천은 없습니다. 직접 수정할 수 있습니다.</p>
              )}
              {data.suggestions.map((s, index) => {
                const applied = strokes.some(
                  (v) => v.suggestionId === s.id)
                const skipped = ignored.includes(s.id)

                return (
                  <div key={s.id} style={{
                    display: 'flex', gap: 8,
                    alignItems: 'center', flexWrap: 'wrap',
                    padding: '8px 0',
                    borderTop: '1px solid #5a6776',
                    background: selected === s.id
                      ? 'rgba(234,179,8,.10)' : 'transparent',
                  }}>
                    <span>
                      {index + 1}번 벽 연결 · 간격 {s.gap_m}m
                      {applied ? ' · 적용됨'
                        : skipped ? ' · 건너뜀' : ''}
                    </span>
                    <button className="btn btn-sm" type="button"
                      onClick={() => {
                        setSelected(s.id)
                        mapRef.current?.scrollIntoView({
                          behavior: 'smooth', block: 'center',
                        })
                      }}>위치 보기</button>
                    <button className="btn btn-sm" type="button"
                      disabled={saved || skipped}
                      onClick={() => toggleSuggestion(s)}>
                      {applied ? '적용 취소' : '추천 적용'}
                    </button>
                    <button className="btn btn-sm" type="button"
                      disabled={saved}
                      onClick={() => {
                        if (applied) {
                          update(strokes.filter(
                            (v) => v.suggestionId !== s.id))
                        }
                        setIgnored((old) => skipped
                          ? old.filter((id) => id !== s.id)
                          : [...old, s.id])
                      }}>
                      {skipped ? '다시 검토' : '건너뛰기'}
                    </button>
                  </div>
                )
              })}
            </div>

            <div style={{
              padding: 12, border: '1px solid #5a6776',
              borderRadius: 8,
            }}>
              <h3>개별 수정 관리 ({strokes.length}개)</h3>
              <p>원하는 항목만 선택하고 삭제할 수 있습니다.
                다른 수정 내용은 그대로 유지됩니다.</p>
              {strokes.map((s, i) => (
                <div key={i} style={{
                  display: 'flex',
                  gap: 8,
                  alignItems: 'center',
                  flexWrap: 'wrap',
                  padding: '8px 0',
                  background: selectedStroke === i
                    ? 'rgba(234,179,8,.12)' : 'transparent',
                  borderBottom: '1px solid #5a6776',
                }}>
                  <span>
                    {i + 1}. {s.suggestionId
                      ? '추천 벽 연결'
                      : s.points ? '자유곡선 수정'
                      : '직선 수정'}
                  </span>
                  <button type="button" className="btn btn-sm"
                    onClick={() => setSelectedStroke(i)}>
                    선택
                  </button>
                  <button type="button" className="btn btn-sm"
                    disabled={saved}
                    onClick={() => update(
                      strokes.filter((_, j) => j !== i)
                    )}>
                    이 수정만 삭제
                  </button>
                </div>
              ))}
            </div>

            <label>
              <input type="checkbox" checked={confirmed}
                disabled={saved}
                onChange={(e) => setConfirmed(e.target.checked)} />
              {' '}실제 매장 구조를 확인했습니다.
              확인하지 못한 공간은 통로로 지정하지 않았습니다.
            </label>

            <button type="button" className="btn btn-primary"
              disabled={!confirmed || saving || saved}
              onClick={() => void finalize()}>
              {saving ? '지도 저장 중…' : 'Save Map · 지도 최종 저장'}
            </button>

            {saveError && (
              <div className="alert alert-error" role="alert">
                {saveError}
              </div>
            )}

            {saved && pendingReason && (
              <div className="alert alert-warn" role="status" style={{ flexDirection: 'column', alignItems: 'flex-start' }}>
                <span>보정한 지도는 저장했지만 아직 매장 지도로 반영하지 못했습니다: {pendingReason}</span>
                <button type="button" className="btn btn-sm" disabled={!!mapping.pending}
                  onClick={() => void mapping.adopt()}>
                  지도 반영 다시 시도
                </button>
              </div>
            )}
          </>
        )}
      </div>
    </section>
  )
}
