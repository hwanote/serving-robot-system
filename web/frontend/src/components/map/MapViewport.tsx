/**
 * Raw Map Viewer (SVG 기반)
 *
 * - 원본 지도 이미지를 이미지 픽셀 좌표계(1 unit = 1 원본 픽셀)로 그리고, 줌/팬은 <g transform> 하나로 처리한다.
 *   → Canvas CSS 크기 / 원본 픽셀 크기 / 줌 / 팬 차이가 모두 ViewTransform 으로 일원화됨.
 * - 화면 → map 좌표 변환은 utils/coordinates 의 screenToMap 만 사용한다.
 * - 오버레이는 이미지 픽셀 좌표 공간에서 렌더링 (ctx.toImage), 화면 크기 고정 요소는 ctx.px() 로 크기 보정.
 */
import { useCallback, useEffect, useLayoutEffect, useMemo, useRef, useState, type ReactNode } from 'react'
import {
  fitView,
  imageToMap,
  isInsideImage,
  mapToImage,
  screenToImage,
  zoomAt,
  panBy,
  type ImagePoint,
  type MapGeometry,
  type ScreenPoint,
  type Vec2,
  type ViewTransform,
} from '../../utils/coordinates'
import { Icon } from '../common/Icon'

export interface MapPointerEvent {
  map: Vec2
  image: ImagePoint
  screen: ScreenPoint
  /** 지도 이미지 범위 안인지 */
  inside: boolean
  shiftKey: boolean
  altKey: boolean
}

export interface DragHandlers {
  onMove: (e: MapPointerEvent) => void
  onEnd?: (e: MapPointerEvent) => void
}

export interface MapOverlayContext {
  geo: MapGeometry
  view: ViewTransform
  toImage: (x: number, y: number) => ImagePoint
  /** 화면 px 크기를 이미지 좌표 단위로 변환 (줌과 무관하게 일정한 크기로 그릴 때) */
  px: (screenPx: number) => number
  /** 오버레이 요소에서 드래그 시작 (포인터 캡처 + map 좌표로 이동 이벤트 전달) */
  beginDrag: (e: React.PointerEvent, handlers: DragHandlers) => void
}

interface Props {
  geometry: MapGeometry
  imageUrl: string
  /** revision 이 바뀌면 화면 맞춤 및 이미지 재로드 */
  revisionKey?: string | number
  renderOverlay?: (ctx: MapOverlayContext) => ReactNode
  /** 빈 공간 클릭 (드래그 없이) */
  onMapClick?: (e: MapPointerEvent) => void
  onMapHover?: (e: MapPointerEvent | null) => void
  cursor?: string
  /** 우측 상단 툴 영역 추가 요소 */
  overlayTop?: ReactNode
  ariaLabel?: string
}

const ZOOM_STEP = 1.25
const CLICK_TOLERANCE_PX = 4
const MAX_SCALE = 60

export function MapViewport({
  geometry,
  imageUrl,
  revisionKey,
  renderOverlay,
  onMapClick,
  onMapHover,
  cursor = 'crosshair',
  overlayTop,
  ariaLabel = '지도',
}: Props) {
  const wrapRef = useRef<HTMLDivElement>(null)
  const svgRef = useRef<SVGSVGElement>(null)
  const [size, setSize] = useState({ w: 0, h: 0 })
  const [view, setView] = useState<ViewTransform | null>(null)
  const [hover, setHover] = useState<MapPointerEvent | null>(null)
  const [loadedImage, setLoadedImage] = useState<{
    src: string
    status: 'ok' | 'error'
    natural?: { w: number; h: number }
  } | null>(null)
  const [reloadSeq, setReloadSeq] = useState(0)
  const [panning, setPanning] = useState(false)
  const viewRef = useRef<ViewTransform | null>(null)
  useLayoutEffect(() => {
    viewRef.current = view
  }, [view])

  // ----- 크기 추적 -----
  useLayoutEffect(() => {
    const el = wrapRef.current
    if (!el) return
    const update = () => setSize({ w: el.clientWidth, h: el.clientHeight })
    update()
    const ro = new ResizeObserver(update)
    ro.observe(el)
    return () => ro.disconnect()
  }, [])

  const fit = useCallback(() => {
    if (size.w > 0 && size.h > 0) setView(fitView(geometry, size.w, size.h))
  }, [geometry, size.w, size.h])

  // 최초 표시 / 지도 revision·해상도 변경 시 화면 맞춤
  const fitKey = `${revisionKey}|${geometry.width}x${geometry.height}|${geometry.resolution}`
  const lastFitKey = useRef<string | null>(null)
  useEffect(() => {
    if (size.w === 0 || size.h === 0) return
    if (lastFitKey.current !== fitKey) {
      lastFitKey.current = fitKey
      fit()
    }
  }, [fitKey, fit, size.w, size.h])

  // ----- 이미지 로드 (원본 크기 검증) -----
  const src = useMemo(() => {
    const sep = imageUrl.includes('?') ? '&' : '?'
    return `${imageUrl}${sep}rev=${encodeURIComponent(String(revisionKey ?? ''))}&n=${reloadSeq}`
  }, [imageUrl, revisionKey, reloadSeq])

  useEffect(() => {
    let alive = true
    const img = new Image()
    img.onload = () => alive && setLoadedImage({ src, status: 'ok', natural: { w: img.naturalWidth, h: img.naturalHeight } })
    img.onerror = () => alive && setLoadedImage({ src, status: 'error' })
    img.src = src
    return () => {
      alive = false
    }
  }, [src])
  // src 가 바뀌면 이전 결과는 무효 → loading
  const image = loadedImage?.src === src ? loadedImage : { status: 'loading' as const, natural: undefined }

  const sizeMismatch =
    image.natural && (image.natural.w !== geometry.width || image.natural.h !== geometry.height) ? image.natural : null

  const minScale = useMemo(() => (size.w ? fitView(geometry, size.w, size.h).scale * 0.4 : 0.05), [geometry, size.w, size.h])

  // ----- 좌표 변환 -----
  const toMapEvent = useCallback(
    (clientX: number, clientY: number, mods: { shiftKey: boolean; altKey: boolean }): MapPointerEvent | null => {
      const svg = svgRef.current
      const v = viewRef.current
      if (!svg || !v) return null
      const rect = svg.getBoundingClientRect()
      const screen = { sx: clientX - rect.left, sy: clientY - rect.top }
      const img = screenToImage(v, screen)
      return { map: imageToMap(geometry, img), image: img, screen, inside: isInsideImage(geometry, img), ...mods }
    },
    [geometry],
  )

  // ----- 휠 줌 (passive:false 필요) -----
  useEffect(() => {
    const svg = svgRef.current
    if (!svg) return
    const onWheel = (e: WheelEvent) => {
      e.preventDefault()
      const v = viewRef.current
      if (!v) return
      const rect = svg.getBoundingClientRect()
      const factor = Math.exp(-e.deltaY * 0.0015)
      setView(zoomAt(v, factor, { sx: e.clientX - rect.left, sy: e.clientY - rect.top }, { min: minScale, max: MAX_SCALE }))
    }
    svg.addEventListener('wheel', onWheel, { passive: false })
    return () => svg.removeEventListener('wheel', onWheel)
  }, [minScale])

  const zoomCenter = (factor: number) => {
    if (!view) return
    setView(zoomAt(view, factor, { sx: size.w / 2, sy: size.h / 2 }, { min: minScale, max: MAX_SCALE }))
  }

  // ----- 포인터: 배경 클릭/팬, 오버레이 드래그 -----
  const gesture = useRef<
    | { kind: 'background'; startX: number; startY: number; lastX: number; lastY: number; moved: boolean; pointerId: number }
    | { kind: 'drag'; handlers: DragHandlers; pointerId: number }
    | null
  >(null)

  const beginDrag = useCallback((e: React.PointerEvent, handlers: DragHandlers) => {
    if (e.button !== 0) return
    e.stopPropagation()
    e.preventDefault()
    svgRef.current?.setPointerCapture(e.pointerId)
    gesture.current = { kind: 'drag', handlers, pointerId: e.pointerId }
  }, [])

  const onPointerDown = (e: React.PointerEvent<SVGSVGElement>) => {
    if (gesture.current) return
    if (e.button !== 0 && e.button !== 1) return
    svgRef.current?.setPointerCapture(e.pointerId)
    gesture.current = {
      kind: 'background',
      startX: e.clientX,
      startY: e.clientY,
      lastX: e.clientX,
      lastY: e.clientY,
      moved: e.button === 1, // 가운데 버튼은 항상 팬
      pointerId: e.pointerId,
    }
    if (e.button === 1) e.preventDefault()
  }

  const onPointerMove = (e: React.PointerEvent<SVGSVGElement>) => {
    const ev = toMapEvent(e.clientX, e.clientY, e)
    setHover(ev)
    onMapHover?.(ev)
    const g = gesture.current
    if (!g || g.pointerId !== e.pointerId) return
    if (g.kind === 'drag') {
      if (ev) g.handlers.onMove(ev)
      return
    }
    const dist = Math.hypot(e.clientX - g.startX, e.clientY - g.startY)
    if (!g.moved && dist > CLICK_TOLERANCE_PX) {
      g.moved = true
      setPanning(true)
    }
    if (g.moved) {
      // 이동량은 지금 계산해야 함 (updater 는 lastX 갱신 이후 실행될 수 있음)
      const dx = e.clientX - g.lastX
      const dy = e.clientY - g.lastY
      setView((v) => (v ? panBy(v, dx, dy) : v))
    }
    g.lastX = e.clientX
    g.lastY = e.clientY
  }

  const endGesture = (e: React.PointerEvent<SVGSVGElement>, cancelled: boolean) => {
    const g = gesture.current
    if (!g || g.pointerId !== e.pointerId) return
    gesture.current = null
    setPanning(false)
    if (svgRef.current?.hasPointerCapture(e.pointerId)) svgRef.current.releasePointerCapture(e.pointerId)
    const ev = toMapEvent(e.clientX, e.clientY, e)
    if (g.kind === 'drag') {
      if (ev) g.handlers.onEnd?.(ev)
      return
    }
    if (!cancelled && !g.moved && ev) onMapClick?.(ev)
  }

  const onPointerLeave = () => {
    setHover(null)
    onMapHover?.(null)
  }

  // ----- 키보드 줌 -----
  const onKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === '+' || e.key === '=') zoomCenter(ZOOM_STEP)
    else if (e.key === '-') zoomCenter(1 / ZOOM_STEP)
    else if (e.key === '0') fit()
  }

  const scale = view?.scale ?? 1
  const px = (n: number) => n / scale
  const ctx = useMemo<MapOverlayContext | null>(
    () =>
      view
        ? { geo: geometry, view, toImage: (x, y) => mapToImage(geometry, x, y), px: (n) => n / view.scale, beginDrag }
        : null,
    [geometry, view, beginDrag],
  )

  const metersPerScreenPx = view ? geometry.resolution / view.scale : null
  const scaleBar = metersPerScreenPx ? pickScaleBar(metersPerScreenPx) : null
  const originImg = view ? mapToImage(geometry, 0, 0) : null

  return (
    <div className="map-viewport" ref={wrapRef} tabIndex={0} onKeyDown={onKeyDown} aria-label={ariaLabel} role="application">
      <svg
        ref={svgRef}
        className="map-svg"
        width="100%"
        height="100%"
        style={{ cursor: panning ? 'grabbing' : cursor }}
        onPointerDown={onPointerDown}
        onPointerMove={onPointerMove}
        onPointerUp={(e) => endGesture(e, false)}
        onPointerCancel={(e) => endGesture(e, true)}
        onPointerLeave={onPointerLeave}
        onContextMenu={(e) => e.preventDefault()}
        data-testid="map-svg"
      >
        {view && (
          <g transform={`translate(${view.offsetX} ${view.offsetY}) scale(${view.scale})`}>
            <rect x={0} y={0} width={geometry.width} height={geometry.height} fill="#cdcdcd" />
            {image.status === 'ok' && (
              <image
                href={src}
                x={0}
                y={0}
                width={geometry.width}
                height={geometry.height}
                preserveAspectRatio="none"
                style={{ imageRendering: 'pixelated' }}
              />
            )}
            <rect
              x={0}
              y={0}
              width={geometry.width}
              height={geometry.height}
              fill="none"
              stroke="#6b7a88"
              strokeWidth={px(1)}
            />
            {originImg && isInsideImage(geometry, originImg) && (
              <g transform={`translate(${originImg.u} ${originImg.v})`} pointerEvents="none" opacity={0.85}>
                <line x1={0} y1={0} x2={px(26)} y2={0} stroke="#d33" strokeWidth={px(2)} />
                <line x1={0} y1={0} x2={0} y2={-px(26)} stroke="#2a9d4b" strokeWidth={px(2)} />
                <text x={px(28)} y={px(4)} fontSize={px(10)} fill="#d33">
                  x
                </text>
                <text x={-px(3)} y={-px(30)} fontSize={px(10)} fill="#2a9d4b">
                  y
                </text>
              </g>
            )}
            {/* ctx.beginDrag 는 이벤트 핸들러에서만 ref 를 사용하므로 render 중 접근이 아님 (false positive) */}
            {/* oxlint-disable-next-line react/refs */}
            {ctx && renderOverlay?.(ctx)}
          </g>
        )}
      </svg>

      {image.status === 'loading' && (
        <div className="map-overlay-center">
          <div className="spinner" /> 지도 이미지를 불러오는 중…
        </div>
      )}
      {image.status === 'error' && (
        <div className="map-overlay-center map-overlay-error" role="alert">
          <Icon name="alert" /> 지도 이미지를 불러오지 못했습니다.
          <button type="button" className="btn btn-sm" onClick={() => setReloadSeq((n) => n + 1)}>
            다시 시도
          </button>
        </div>
      )}

      <div className="map-controls">
        {overlayTop}
        <div className="map-zoom">
          <button type="button" className="tool-btn" aria-label="확대" title="확대 (+)" onClick={() => zoomCenter(ZOOM_STEP)}>
            <Icon name="plus" />
          </button>
          <button type="button" className="tool-btn" aria-label="축소" title="축소 (-)" onClick={() => zoomCenter(1 / ZOOM_STEP)}>
            <Icon name="minus" />
          </button>
          <button type="button" className="tool-btn" aria-label="화면 맞춤" title="화면 맞춤 (0)" onClick={fit}>
            <Icon name="fit" />
          </button>
        </div>
      </div>

      <div className="map-readout" aria-live="off">
        {hover ? (
          hover.inside ? (
            <span className="mono" data-testid="map-cursor">
              x {hover.map.x.toFixed(2)} m · y {hover.map.y.toFixed(2)} m
              <span className="muted"> · px ({Math.floor(hover.image.u)}, {Math.floor(hover.image.v)})</span>
            </span>
          ) : (
            <span className="muted">지도 범위 밖</span>
          )
        ) : (
          <span className="muted">
            {geometry.width}×{geometry.height}px · {geometry.resolution} m/px · frame: map
          </span>
        )}
        {view && <span className="muted"> · {Math.round(view.scale * 100)}%</span>}
        {scaleBar && (
          <span className="map-scalebar">
            <span style={{ width: scaleBar.px }} />
            {scaleBar.label}
          </span>
        )}
      </div>

      {(sizeMismatch || geometry.origin.yaw !== 0) && (
        <div className="map-notice">
          {sizeMismatch && (
            <div className="alert alert-warn">
              <Icon name="alert" size={14} />
              이미지 크기({sizeMismatch.w}×{sizeMismatch.h})가 지도 정보({geometry.width}×{geometry.height})와 다릅니다. 지도 정보 기준으로
              표시합니다.
            </div>
          )}
          {geometry.origin.yaw !== 0 && (
            <div className="alert alert-info">origin yaw = {geometry.origin.yaw.toFixed(3)} rad — 회전 변환을 적용해 좌표를 계산합니다.</div>
          )}
        </div>
      )}
    </div>
  )
}

function pickScaleBar(metersPerPx: number): { px: number; label: string } {
  const candidates = [0.1, 0.2, 0.5, 1, 2, 5, 10]
  const target = 90 * metersPerPx
  const m = candidates.reduce((best, c) => (Math.abs(c - target) < Math.abs(best - target) ? c : best), candidates[0])
  return { px: m / metersPerPx, label: m < 1 ? `${m * 100} cm` : `${m} m` }
}
