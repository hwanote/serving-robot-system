import { robotDisplayName } from '../../hooks/useDestinations'
import type { RobotView } from '../../state/fleetReducer'
import { formatAgo } from '../../utils/labels'
import { formatYawDeg } from '../../utils/coordinates'
import { Icon } from '../common/Icon'
import { RobotStateBadge } from '../common/StatusBadge'

function BatteryBar({ percent }: { percent: number }) {
  const color = percent <= 20 ? 'var(--danger)' : percent <= 40 ? 'var(--warn)' : 'var(--ok)'
  return (
    <div style={{ display: 'flex', alignItems: 'center', gap: 8 }} aria-label={`배터리 ${percent}%`}>
      <div style={{ flex: 1, height: 8, background: '#e8ecf0', borderRadius: 999, overflow: 'hidden' }}>
        <div style={{ width: `${percent}%`, height: '100%', background: color, transition: 'width .3s' }} />
      </div>
      <span className="mono" style={{ minWidth: 40, textAlign: 'right' }}>
        {percent}%
      </span>
    </div>
  )
}

interface Props {
  robot: RobotView
  /** 실시간 연결이 끊긴 상태 → 마지막 수신값임을 표시 */
  stale: boolean
  labelOf: (id: string | null) => string
  color: string
  now: number
}

export function RobotCard({ robot, stale, labelOf, color, now }: Props) {
  return (
    <article className="card" style={{ opacity: stale ? 0.75 : 1 }} data-testid={`robot-card-${robot.robot_id}`} aria-label={robotDisplayName(robot.robot_id)}>
      <div className="card-header">
        <div style={{ display: 'flex', alignItems: 'center', gap: 10 }}>
          <span style={{ width: 10, height: 10, borderRadius: 999, background: color }} />
          <h3 style={{ fontSize: 15 }}>{robotDisplayName(robot.robot_id)}</h3>
          <span className="mono muted">{robot.robot_id}</span>
        </div>
        <RobotStateBadge state={robot.state} />
      </div>
      <div className="card-body" style={{ display: 'flex', flexDirection: 'column', gap: 10 }}>
        <BatteryBar percent={robot.battery_percent} />
        <dl className="kv">
          <dt>현재 목적지</dt>
          <dd>{robot.destination_id ? <strong>{labelOf(robot.destination_id)}</strong> : <span className="muted">없음</span>}</dd>
          <dt>현재 Task</dt>
          <dd className="mono">{robot.task_id ?? <span className="muted">없음</span>}</dd>
          <dt>위치 (map)</dt>
          <dd className="mono">
            {robot.pose.x.toFixed(2)}, {robot.pose.y.toFixed(2)} m · {formatYawDeg(robot.pose.yaw)}
          </dd>
        </dl>
        <div className="muted" style={{ fontSize: 12, display: 'flex', alignItems: 'center', gap: 6 }}>
          {stale ? (
            <>
              <Icon name="wifiOff" size={13} /> 연결 끊김 — 마지막 수신 {formatAgo(robot.receivedAt, now)} (현재 상태 아님)
            </>
          ) : (
            <>마지막 수신 {formatAgo(robot.receivedAt, now)}</>
          )}
        </div>
      </div>
    </article>
  )
}
