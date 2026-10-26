import { robotDisplayName } from '../../hooks/useDestinations'
import type { RobotView } from '../../state/fleetReducer'
import { yawToScreenDeg } from '../../utils/coordinates'
import type { MapOverlayContext } from '../map/MapViewport'

/** 지도 위 로봇 위치 표시 (map frame pose) */
export function RobotLayer({ ctx, robots, stale, colorOf }: { ctx: MapOverlayContext; robots: RobotView[]; stale: boolean; colorOf: (id: string) => string }) {
  const { px } = ctx
  return (
    <g pointerEvents="none">
      {robots.map((r) => {
        const p = ctx.toImage(r.pose.x, r.pose.y)
        const deg = yawToScreenDeg(ctx.geo, r.pose.yaw)
        const color = r.state === 'ERROR' ? '#d93a32' : colorOf(r.robot_id)
        const rad = px(10)
        return (
          <g key={r.robot_id} transform={`translate(${p.u} ${p.v})`} opacity={stale ? 0.45 : 1} data-robot-id={r.robot_id}>
            <circle r={rad + px(4)} fill={color} fillOpacity={0.18} />
            <g transform={`rotate(${deg})`}>
              <circle r={rad} fill={color} stroke="#fff" strokeWidth={px(2)} />
              <path d={`M ${rad * 0.9} 0 L ${-rad * 0.3} ${rad * 0.55} L ${-rad * 0.3} ${-rad * 0.55} Z`} fill="#fff" />
            </g>
            <text
              y={-rad - px(7)}
              textAnchor="middle"
              fontSize={px(11)}
              fontWeight={700}
              fill={color}
              stroke="#fff"
              strokeWidth={px(3)}
              paintOrder="stroke"
            >
              {robotDisplayName(r.robot_id)}
            </text>
          </g>
        )
      })}
    </g>
  )
}
