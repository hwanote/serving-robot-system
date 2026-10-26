/**
 * 자동 노드 생성기가 만든 통로·노드·정차 위치 오버레이.
 * - 통로(선) + 길이(m) + 노드(점) + 정차 위치(화살표: 로봇이 서는 자리와 바라보는 방향)
 * - 손으로 지정한 정차 위치(수동)는 보라색, 통로 위에 놓지 못한 점은 빨간 ✕ (손으로 지정한 자리가 실패했으면 빨간 화살표)
 * - interactive 이면 정차 화살표를 끌어 옮길 수 있다 (옮긴 뒤의 처리는 onStopDragStart 를 받는 쪽)
 * - 이미지 픽셀 좌표 공간에서 렌더링 (MapViewport 의 overlay ctx 사용)
 */
import type { GeneratedGraph } from '../../types/provisional'
import { yawToScreenDeg } from '../../utils/coordinates'
import { GRAPH_EDGE_COLOR, GRAPH_PINNED_COLOR, GRAPH_STOP_COLOR, type GraphLayers } from '../../utils/graph'
import type { MapOverlayContext } from './MapViewport'

/** 끌고 있는 정차 위치 (놓기 전 미리보기) */
export interface DraggingStop {
  entityId: string
  x: number
  y: number
  /** rad */
  yaw: number
}

interface Props {
  ctx: MapOverlayContext
  graph: GeneratedGraph
  layers: GraphLayers
  stale?: boolean
  /** 정차 화살표를 끌 수 있음 (선택 도구일 때) */
  interactive?: boolean
  dragging?: DraggingStop | null
  /** 화살표에서 포인터를 누름: (객체 id, 현재 방향 rad, 이벤트) */
  onStopDragStart?: (entityId: string, yaw: number, e: React.PointerEvent) => void
}

function Label({ ctx, u, v, text, size = 11, color = '#17202a', testId }: { ctx: MapOverlayContext; u: number; v: number; text: string; size?: number; color?: string; testId?: string }) {
  const { px } = ctx
  return (
    <text
      x={u}
      y={v}
      fontSize={px(size)}
      textAnchor="middle"
      dominantBaseline="central"
      fill={color}
      stroke="#fff"
      strokeWidth={px(3)}
      strokeLinejoin="round"
      paintOrder="stroke"
      pointerEvents="none"
      data-testid={testId}
    >
      {text}
    </text>
  )
}

/** 길이 숫자는 통로 가운데에서 선의 수직 방향(화면 아래쪽)으로 조금 비껴 놓는다 — 정차 이름표와 겹치지 않게 */
function LengthLabel({ ctx, a, b, text, testId }: { ctx: MapOverlayContext; a: { u: number; v: number }; b: { u: number; v: number }; text: string; testId: string }) {
  const dx = b.u - a.u
  const dy = b.v - a.v
  const len = Math.hypot(dx, dy) || 1
  let nx = -dy / len
  let ny = dx / len
  if (ny < 0 || (ny === 0 && nx < 0)) {
    nx = -nx
    ny = -ny
  }
  const off = ctx.px(13)
  return <Label ctx={ctx} u={(a.u + b.u) / 2 + nx * off} v={(a.v + b.v) / 2 + ny * off} text={text} testId={testId} />
}

function StopArrow({ ctx, x, y, yaw, color, ghost = false }: { ctx: MapOverlayContext; x: number; y: number; yaw: number; color: string; ghost?: boolean }) {
  const { px } = ctx
  const p = ctx.toImage(x, y)
  const deg = yawToScreenDeg(ctx.geo, yaw)
  const r = px(8)
  return (
    <g transform={`translate(${p.u} ${p.v}) rotate(${deg})`}>
      <circle r={r * 1.5} fill={color} fillOpacity={ghost ? 0.2 : 0.12} stroke={color} strokeWidth={px(ghost ? 1.5 : 1)} strokeDasharray={`${px(3)} ${px(2)}`} />
      <path d={`M ${r * 1.5} 0 L ${-r} ${r * 0.9} L ${-r * 0.4} 0 L ${-r} ${-r * 0.9} Z`} fill={color} fillOpacity={ghost ? 0.7 : 1} stroke="#fff" strokeWidth={px(1)} />
    </g>
  )
}

export function GeneratedGraphLayer({ ctx, graph, layers, stale = false, interactive = false, dragging = null, onStopDragStart }: Props) {
  const { px } = ctx
  const nodeAt = new Map(graph.nodes.map((n) => [n.id, ctx.toImage(n.x, n.y)]))
  const opacity = stale ? 0.4 : 1
  const canDrag = interactive && !!onStopDragStart
  const grab = (entityId: string | null, yaw: number) =>
    canDrag && entityId
      ? {
          pointerEvents: 'auto' as const,
          style: { cursor: 'grab' },
          onPointerDown: (e: React.PointerEvent) => onStopDragStart!(entityId, yaw, e),
        }
      : { pointerEvents: 'none' as const }

  return (
    <g pointerEvents="none" opacity={opacity} data-testid="graph-layer" data-stale={stale ? 'true' : 'false'}>
      {layers.edges &&
        graph.edges.map((e) => {
          const a = nodeAt.get(e.from)
          const b = nodeAt.get(e.to)
          if (!a || !b) return null
          return (
            <g key={e.id} data-testid={`graph-edge-${e.id}`} data-level={e.level}>
              <line x1={a.u} y1={a.v} x2={b.u} y2={b.v} stroke={GRAPH_EDGE_COLOR[e.level]} strokeWidth={px(3.5)} strokeLinecap="round" strokeDasharray={stale ? `${px(6)} ${px(4)}` : undefined} />
              {layers.lengths && <LengthLabel ctx={ctx} a={a} b={b} text={`${e.length_m.toFixed(1)} m`} testId={`graph-length-${e.id}`} />}
            </g>
          )
        })}

      {layers.edges &&
        graph.nodes.map((n) => {
          const p = nodeAt.get(n.id)!
          return <circle key={n.id} cx={p.u} cy={p.v} r={px(4.5)} fill="#fff" stroke={GRAPH_EDGE_COLOR[n.level]} strokeWidth={px(2)} data-testid={`graph-node-${n.id}`} />
        })}

      {layers.stops &&
        graph.stops.map((s) => {
          const p = ctx.toImage(s.x, s.y)
          const deg = yawToScreenDeg(ctx.geo, s.yaw)
          const rad = (deg * Math.PI) / 180
          const color = s.level === 'OK' && s.pinned ? GRAPH_PINNED_COLOR : GRAPH_STOP_COLOR[s.level]
          const moving = dragging !== null && dragging.entityId === s.entity_id
          return (
            <g key={s.id} data-testid={`graph-stop-${s.id}`} data-level={s.level} data-pinned={s.pinned ? 'true' : 'false'} opacity={moving ? 0.25 : 1} {...grab(s.entity_id, s.yaw)}>
              <StopArrow ctx={ctx} x={s.x} y={s.y} yaw={s.yaw} color={color} />
              <Label ctx={ctx} u={p.u + Math.cos(rad) * px(30)} v={p.v + Math.sin(rad) * px(30)} text={`${s.entity_id ?? s.id}${s.pinned ? ' (수동)' : ''}`} size={10.5} color={color} />
            </g>
          )
        })}

      {graph.excluded.map((x) => {
        const moving = dragging !== null && dragging.entityId === x.entity_id
        if (x.pin) {
          // 손으로 지정한 자리가 검사에 실패: 그 자리에 빨간 화살표 (다시 끌어 옮길 수 있음)
          const yaw = ((x.pin.yaw_deg ?? 0) * Math.PI) / 180
          const p = ctx.toImage(x.pin.x, x.pin.y)
          return (
            <g key={x.id} data-testid={`graph-excluded-${x.id}`} data-pinned="true" opacity={moving ? 0.25 : 1} {...grab(x.entity_id, yaw)}>
              <StopArrow ctx={ctx} x={x.pin.x} y={x.pin.y} yaw={yaw} color="#d93a32" />
              <Label ctx={ctx} u={p.u} v={p.v + px(26)} text={`${x.entity_id ?? x.id} 배치 불가 (수동)`} size={10.5} color="#d93a32" />
            </g>
          )
        }
        if (x.x === null || x.y === null) return null
        const p = ctx.toImage(x.x, x.y)
        const k = px(7)
        return (
          <g key={x.id} data-testid={`graph-excluded-${x.id}`}>
            <circle cx={p.u} cy={p.v} r={k * 1.7} fill="#fff" fillOpacity={0.85} stroke="#d93a32" strokeWidth={px(1.5)} />
            <path d={`M ${p.u - k} ${p.v - k} L ${p.u + k} ${p.v + k} M ${p.u + k} ${p.v - k} L ${p.u - k} ${p.v + k}`} stroke="#d93a32" strokeWidth={px(3)} strokeLinecap="round" />
            <Label ctx={ctx} u={p.u} v={p.v + k * 3} text={`${x.entity_id ?? x.id} 배치 불가`} size={10.5} color="#d93a32" />
          </g>
        )
      })}

      {dragging && (
        <g data-testid="graph-drag-ghost" pointerEvents="none">
          <StopArrow ctx={ctx} x={dragging.x} y={dragging.y} yaw={dragging.yaw} color={GRAPH_PINNED_COLOR} ghost />
        </g>
      )}
    </g>
  )
}
