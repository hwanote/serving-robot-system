/**
 * Semantic Map 오버레이 (Entity / Waypoint / Zone)
 * - 이미지 픽셀 좌표 공간에서 렌더링 (MapViewport 의 overlay ctx 사용)
 * - editable 이면 선택·드래그 편집 지원, 아니면 표시 전용 (Robot Control 등)
 */
import type { Dispatch } from 'react'
import type { MapPoint, SemanticMap, Waypoint, Zone } from '../../types/contracts'
import { clampToImage, imageToMap, imageVectorToYaw, yawToScreenDeg, type Vec2 } from '../../utils/coordinates'
import { isManualWaypoint } from '../../utils/manualStop'
import { entityDisplayPose, isValidTopSize } from '../../utils/semantic'
import type { EditorAction, Selection } from '../../utils/semanticEditor'
import type { MapOverlayContext, MapPointerEvent } from './MapViewport'
import { ENTITY_STYLE, WAYPOINT_COLOR, ZONE_STYLE } from './mapStyles'

interface Props {
  ctx: MapOverlayContext
  doc: SemanticMap
  selection?: Selection
  /** 편집 가능 (select 도구일 때만 객체가 포인터 이벤트를 받음) */
  interactive?: boolean
  dispatch?: Dispatch<EditorAction>
  /** 강조할 목적지 entity id 목록 (Robot Control) */
  highlightIds?: string[]
  showWaypoints?: boolean
  /** 손으로 지정한 정차 위치(manual waypoint)는 숨김: 통로 미리보기의 보라 화살표가 대신 보여준다 */
  hideManualWaypoints?: boolean
}

/** 드래그 중 지도 밖으로 나가지 않도록 이미지 범위로 제한한 map 좌표 */
const clampedMap = (ctx: MapOverlayContext, e: MapPointerEvent): Vec2 => imageToMap(ctx.geo, clampToImage(ctx.geo, e.image))

export function SemanticLayer({ ctx, doc, selection = null, interactive = false, dispatch, highlightIds = [], showWaypoints = true, hideManualWaypoints = false }: Props) {
  const { px } = ctx
  const pe = interactive ? 'auto' : 'none'

  const select = (sel: Selection) => dispatch?.({ type: 'select', selection: sel })

  // ---------- Zones ----------
  const zoneEls = doc.zones.map((zone) => {
    const style = ZONE_STYLE[zone.type]
    const selected = selection?.kind === 'zone' && selection.id === zone.id
    const pts = zone.polygon.map(([x, y]) => ctx.toImage(x, y))
    const points = pts.map((p) => `${p.u},${p.v}`).join(' ')

    const startMoveZone = (e: React.PointerEvent) => {
      if (!interactive || !dispatch) return
      select({ kind: 'zone', id: zone.id })
      dispatch({ type: 'checkpoint' })
      let last: Vec2 | null = null
      ctx.beginDrag(e, {
        onMove: (ev) => {
          const m = clampedMap(ctx, ev)
          if (last) dispatch({ type: 'translate_zone', id: zone.id, dx: m.x - last.x, dy: m.y - last.y, record: false })
          last = m
        },
      })
    }

    return (
      <g key={zone.id} pointerEvents={pe} data-zone-id={zone.id}>
        <polygon
          points={points}
          fill={style.color}
          fillOpacity={selected ? 0.32 : 0.2}
          stroke={style.color}
          strokeWidth={px(selected ? 2.5 : 1.5)}
          strokeDasharray={style.dash ? style.dash.split(' ').map((n) => px(Number(n))).join(' ') : undefined}
          style={{ cursor: interactive ? 'move' : undefined }}
          onPointerDown={startMoveZone}
        />
        {selected && interactive && dispatch && <ZoneHandles ctx={ctx} zone={zone} selection={selection} dispatch={dispatch} />}
        <ZoneLabel ctx={ctx} zone={zone} />
      </g>
    )
  })

  // ---------- Waypoint 연결선 + 화살표 ----------
  const waypointEls = showWaypoints
    ? doc.waypoints.filter((wp) => !(hideManualWaypoints && isManualWaypoint(wp))).map((wp) => {
        const entity = doc.entities.find((e) => e.id === wp.entity_id)
        const entityPose = entity ? entityDisplayPose(doc, entity) : null
        const selected = selection?.kind === 'waypoint' && selection.id === wp.id
        const ownerSelected = selection?.kind === 'entity' && selection.id === wp.entity_id
        const p = ctx.toImage(wp.pose.x, wp.pose.y)
        const ep = entityPose && entity?.pose ? ctx.toImage(entityPose.x, entityPose.y) : null
        return (
          <g key={wp.id} data-waypoint-id={wp.id}>
            {ep && (ep.u !== p.u || ep.v !== p.v) && (
              <line
                x1={ep.u}
                y1={ep.v}
                x2={p.u}
                y2={p.v}
                stroke={WAYPOINT_COLOR}
                strokeOpacity={0.6}
                strokeWidth={px(1.2)}
                strokeDasharray={`${px(4)} ${px(3)}`}
                pointerEvents="none"
              />
            )}
            <WaypointMarker
              ctx={ctx}
              wp={wp}
              selected={selected || ownerSelected}
              interactive={interactive}
              onPointerDown={(e) => {
                if (!interactive || !dispatch) return
                select({ kind: 'waypoint', id: wp.id })
                dispatch({ type: 'checkpoint' })
                ctx.beginDrag(e, {
                  onMove: (ev) => {
                    const m = clampedMap(ctx, ev)
                    dispatch({ type: 'move_waypoint', id: wp.id, x: m.x, y: m.y, record: false })
                  },
                })
              }}
            />
            {selected && interactive && dispatch && <YawHandle ctx={ctx} wp={wp} dispatch={dispatch} />}
          </g>
        )
      })
    : null

  // ---------- 테이블 상단 (입력한 가로×세로 크기, map frame 축 정렬) ----------
  const topEls = doc.entities.map((entity) => {
    if (entity.type !== 'table' || !entity.top_size_m || !isValidTopSize(entity.top_size_m)) return null
    const pose = entityDisplayPose(doc, entity)
    if (!pose) return null
    const c = ctx.toImage(pose.x, pose.y)
    const w = entity.top_size_m.x / ctx.geo.resolution
    const h = entity.top_size_m.y / ctx.geo.resolution
    const selected = selection?.kind === 'entity' && selection.id === entity.id
    return (
      <rect
        key={`top_${entity.id}`}
        data-top-of={entity.id}
        x={c.u - w / 2}
        y={c.v - h / 2}
        width={w}
        height={h}
        fill={ENTITY_STYLE.table.color}
        fillOpacity={selected ? 0.28 : 0.14}
        stroke={ENTITY_STYLE.table.color}
        strokeWidth={px(selected ? 2 : 1.2)}
        strokeDasharray={`${px(5)} ${px(3)}`}
        pointerEvents="none"
      />
    )
  })

  // ---------- Entities ----------
  const entityEls = doc.entities.map((entity) => {
    const pose = entityDisplayPose(doc, entity)
    if (!pose) return null
    const p = ctx.toImage(pose.x, pose.y)
    const style = ENTITY_STYLE[entity.type]
    const selected = selection?.kind === 'entity' && selection.id === entity.id
    const highlighted = highlightIds.includes(entity.id)
    const r = px(entity.type === 'robot_wait' ? 8 : 11)
    return (
      <g
        key={entity.id}
        transform={`translate(${p.u} ${p.v})`}
        pointerEvents={pe}
        style={{ cursor: interactive ? 'grab' : undefined }}
        data-entity-id={entity.id}
        onPointerDown={(e) => {
          if (!interactive || !dispatch) return
          select({ kind: 'entity', id: entity.id })
          dispatch({ type: 'checkpoint' })
          ctx.beginDrag(e, {
            onMove: (ev) => {
              const m = clampedMap(ctx, ev)
              dispatch({ type: 'move_entity', id: entity.id, x: m.x, y: m.y, record: false })
            },
          })
        }}
      >
        {(selected || highlighted) && (
          <circle r={r + px(5)} fill="none" stroke={highlighted ? '#0f8a7e' : '#17202a'} strokeWidth={px(2)} strokeDasharray={highlighted ? `${px(3)} ${px(2)}` : undefined} />
        )}
        {entity.type === 'robot_wait' ? (
          <circle r={r} fill="#fff" stroke={style.color} strokeWidth={px(2.5)} />
        ) : (
          <rect x={-r} y={-r} width={r * 2} height={r * 2} rx={px(4)} fill={style.color} stroke="#fff" strokeWidth={px(1.5)} />
        )}
        <text
          y={px(4)}
          textAnchor="middle"
          fontSize={px(11)}
          fontWeight={700}
          fill={entity.type === 'robot_wait' ? style.color : '#fff'}
          pointerEvents="none"
        >
          {style.letter}
        </text>
        <text
          y={r + px(13)}
          textAnchor="middle"
          fontSize={px(11)}
          fontWeight={600}
          fill="#17202a"
          stroke="#fff"
          strokeWidth={px(3)}
          paintOrder="stroke"
          pointerEvents="none"
        >
          {entity.label || entity.id}
        </text>
      </g>
    )
  })

  return (
    <g>
      {zoneEls}
      {topEls}
      {waypointEls}
      {entityEls}
    </g>
  )
}

function WaypointMarker({
  ctx,
  wp,
  selected,
  interactive,
  onPointerDown,
}: {
  ctx: MapOverlayContext
  wp: Waypoint
  selected: boolean
  interactive: boolean
  onPointerDown: (e: React.PointerEvent) => void
}) {
  const { px } = ctx
  const p = ctx.toImage(wp.pose.x, wp.pose.y)
  const deg = yawToScreenDeg(ctx.geo, wp.pose.yaw)
  const s = px(selected ? 9 : 7)
  return (
    <g
      transform={`translate(${p.u} ${p.v}) rotate(${deg})`}
      pointerEvents={interactive ? 'auto' : 'none'}
      style={{ cursor: interactive ? 'grab' : undefined }}
      onPointerDown={onPointerDown}
    >
      <circle r={s * 1.3} fill={WAYPOINT_COLOR} fillOpacity={selected ? 0.18 : 0.1} />
      <path d={`M ${s * 1.4} 0 L ${-s} ${s * 0.85} L ${-s * 0.45} 0 L ${-s} ${-s * 0.85} Z`} fill={WAYPOINT_COLOR} stroke="#fff" strokeWidth={px(1)} />
    </g>
  )
}

/** 선택된 Waypoint 의 방향(yaw) 조절 핸들 */
function YawHandle({ ctx, wp, dispatch }: { ctx: MapOverlayContext; wp: Waypoint; dispatch: Dispatch<EditorAction> }) {
  const { px } = ctx
  const p = ctx.toImage(wp.pose.x, wp.pose.y)
  const rad = (yawToScreenDeg(ctx.geo, wp.pose.yaw) * Math.PI) / 180
  const len = px(34)
  const hx = p.u + Math.cos(rad) * len
  const hy = p.v + Math.sin(rad) * len
  return (
    <g>
      <line x1={p.u} y1={p.v} x2={hx} y2={hy} stroke={WAYPOINT_COLOR} strokeWidth={px(1.5)} pointerEvents="none" />
      <circle
        cx={hx}
        cy={hy}
        r={px(6)}
        fill="#fff"
        stroke={WAYPOINT_COLOR}
        strokeWidth={px(2)}
        style={{ cursor: 'crosshair' }}
        aria-label="방향 조절"
        data-testid="yaw-handle"
        onPointerDown={(e) => {
          dispatch({ type: 'checkpoint' })
          ctx.beginDrag(e, {
            onMove: (ev) => {
              const yaw = imageVectorToYaw(ctx.geo, ev.image.u - p.u, ev.image.v - p.v)
              dispatch({ type: 'set_waypoint_yaw', id: wp.id, yaw, record: false })
            },
          })
        }}
      />
    </g>
  )
}

function ZoneHandles({
  ctx,
  zone,
  selection,
  dispatch,
}: {
  ctx: MapOverlayContext
  zone: Zone
  selection: Selection
  dispatch: Dispatch<EditorAction>
}) {
  const { px } = ctx
  const color = ZONE_STYLE[zone.type].color
  const pts = zone.polygon.map(([x, y]) => ctx.toImage(x, y))
  const activeVertex = selection?.kind === 'zone' ? selection.vertex : undefined
  return (
    <g>
      {/* 변 중간점: 클릭하면 꼭짓점 추가 */}
      {pts.map((p, i) => {
        const q = pts[(i + 1) % pts.length]
        const mid = { u: (p.u + q.u) / 2, v: (p.v + q.v) / 2 }
        const midMap = imageToMap(ctx.geo, mid)
        return (
          <circle
            key={`m${i}`}
            cx={mid.u}
            cy={mid.v}
            r={px(4)}
            fill="#fff"
            fillOpacity={0.9}
            stroke={color}
            strokeWidth={px(1)}
            style={{ cursor: 'copy' }}
            onPointerDown={(e) => {
              e.stopPropagation()
              dispatch({ type: 'insert_zone_vertex', id: zone.id, afterIndex: i, point: [midMap.x, midMap.y] as MapPoint })
            }}
          >
            <title>꼭짓점 추가</title>
          </circle>
        )
      })}
      {pts.map((p, i) => (
        <rect
          key={`v${i}`}
          x={p.u - px(5)}
          y={p.v - px(5)}
          width={px(10)}
          height={px(10)}
          fill={activeVertex === i ? color : '#fff'}
          stroke={color}
          strokeWidth={px(2)}
          style={{ cursor: 'move' }}
          data-testid={`zone-vertex-${i}`}
          onPointerDown={(e) => {
            dispatch({ type: 'select', selection: { kind: 'zone', id: zone.id, vertex: i } })
            dispatch({ type: 'checkpoint' })
            ctx.beginDrag(e, {
              onMove: (ev) => {
                const m = clampedMap(ctx, ev)
                dispatch({ type: 'move_zone_vertex', id: zone.id, index: i, point: [m.x, m.y], record: false })
              },
            })
          }}
        />
      ))}
    </g>
  )
}

function ZoneLabel({ ctx, zone }: { ctx: MapOverlayContext; zone: Zone }) {
  if (!zone.polygon.length) return null
  const cx = zone.polygon.reduce((s, p) => s + p[0], 0) / zone.polygon.length
  const cy = zone.polygon.reduce((s, p) => s + p[1], 0) / zone.polygon.length
  const p = ctx.toImage(cx, cy)
  return (
    <text
      x={p.u}
      y={p.v}
      textAnchor="middle"
      fontSize={ctx.px(10)}
      fontWeight={600}
      fill={ZONE_STYLE[zone.type].color}
      stroke="#fff"
      strokeWidth={ctx.px(2.5)}
      paintOrder="stroke"
      pointerEvents="none"
    >
      {zone.id}
    </text>
  )
}

/** 작성 중인 Zone (꼭짓점 + 커서까지 고무줄 선) */
export function ZoneDraftLayer({ ctx, points, hover, color }: { ctx: MapOverlayContext; points: MapPoint[]; hover: Vec2 | null; color: string }) {
  const { px } = ctx
  const pts = points.map(([x, y]) => ctx.toImage(x, y))
  const h = hover ? ctx.toImage(hover.x, hover.y) : null
  const line = [...pts, ...(h ? [h] : [])].map((p) => `${p.u},${p.v}`).join(' ')
  return (
    <g pointerEvents="none">
      {pts.length >= 2 && <polygon points={pts.map((p) => `${p.u},${p.v}`).join(' ')} fill={color} fillOpacity={0.12} />}
      <polyline points={line} fill="none" stroke={color} strokeWidth={px(2)} strokeDasharray={`${px(5)} ${px(3)}`} />
      {pts.map((p, i) => (
        <circle key={i} cx={p.u} cy={p.v} r={px(i === 0 ? 6 : 4)} fill={i === 0 ? color : '#fff'} stroke={color} strokeWidth={px(2)} />
      ))}
    </g>
  )
}
