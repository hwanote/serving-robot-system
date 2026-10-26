import { useState } from 'react'
import { robotDisplayName } from '../../hooks/useDestinations'
import type { RobotView } from '../../state/fleetReducer'
import { formatAgo } from '../../utils/labels'
import { formatYawDeg } from '../../utils/coordinates'
import { homeHint, mapBadge, needsRestart, nodeLabel, RESTART_CAUTION, RESTART_NOTICE, reloadHint } from '../../utils/robotControls'
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
  /** 카드 버튼. 없으면 버튼 줄을 그리지 않는다 (확인창·오류 안내는 호출하는 쪽) */
  actions?: {
    onReset: (node: string) => void
    onHome: () => void
    onReloadMap: () => void
    /** 지금 요청 중인 버튼 */
    pending: 'reset' | 'home' | 'reload' | null
  }
}

function LocalizationNote({ value }: { value: string | null | undefined }) {
  if (!value || value === 'OK') return null
  const text = value === 'DEGRADED' ? '위치가 조금 불안정해요' : '위치를 모릅니다 — 리셋이 필요해요'
  return (
    <div role="status" style={{ fontSize: 12, color: 'var(--danger)', display: 'flex', alignItems: 'center', gap: 6 }}>
      <Icon name="wifiOff" size={13} /> {text} ({value})
    </div>
  )
}

function ControlRow({ robot, stale, actions }: { robot: RobotView; stale: boolean; actions: NonNullable<Props['actions']> }) {
  const controls = robot.controls
  const nodes = controls?.reset_nodes ?? []
  const [node, setNode] = useState('')
  if (!controls) return null
  const chosen = nodes.includes(node) ? node : (nodes[0] ?? '')
  const off = stale || actions.pending !== null
  const badge = mapBadge(robot)
  const homeOff = off || !controls.home
  const restart = needsRestart(robot)
  const reloadOff = off || !controls.reload_map || restart
  return (
    <div className="robot-controls" style={{ display: 'flex', flexDirection: 'column', gap: 8, borderTop: '1px solid var(--border)', paddingTop: 10 }}>
      <LocalizationNote value={robot.localization} />
      {restart && (
        <div role="status" className="alert alert-warn" data-testid={`restart-notice-${robot.robot_id}`} style={{ flexDirection: 'column', alignItems: 'flex-start', fontSize: 12 }}>
          <strong>{RESTART_NOTICE}</strong>
          <span>{RESTART_CAUTION}</span>
        </div>
      )}
      <div style={{ display: 'flex', alignItems: 'center', gap: 6, flexWrap: 'wrap' }}>
        <label className="muted" style={{ fontSize: 12 }} htmlFor={`reset-node-${robot.robot_id}`}>
          위치 리셋
        </label>
        <select
          id={`reset-node-${robot.robot_id}`}
          className="select"
          value={chosen}
          disabled={off || !controls.reset || nodes.length < 2}
          onChange={(e) => setNode(e.target.value)}
          aria-label={`${robotDisplayName(robot.robot_id)} 리셋할 위치`}
        >
          {nodes.map((n) => (
            <option key={n} value={n}>
              {nodeLabel(n)}
            </option>
          ))}
        </select>
        <button type="button" className="btn btn-sm btn-danger" disabled={off || !controls.reset || !chosen} onClick={() => actions.onReset(chosen)} title="로봇을 그 위치에 직접 올려놓은 뒤 누르세요. 진행 중인 주문은 취소돼요.">
          {actions.pending === 'reset' ? <span className="spinner spinner-sm" /> : null} 리셋
        </button>
        <button type="button" className="btn btn-sm" disabled={homeOff} onClick={actions.onHome} title={homeOff && !stale ? homeHint(robot) : '대기 위치로 보냅니다'}>
          {actions.pending === 'home' ? <span className="spinner spinner-sm" /> : <Icon name="home" size={13} />} 대기 위치로
        </button>
      </div>
      <div style={{ display: 'flex', alignItems: 'center', gap: 8, flexWrap: 'wrap' }}>
        <span className="muted" style={{ fontSize: 12, color: badge.tone === 'ok' ? 'var(--ok)' : badge.tone === 'warn' ? 'var(--warn)' : undefined }} data-testid={`map-badge-${robot.robot_id}`}>
          {badge.text}
        </span>
        <button type="button" className="btn btn-sm" disabled={reloadOff} onClick={actions.onReloadMap} title={reloadOff && !stale ? reloadHint(robot) : '로봇이 노드·통로·테이블 위치(config/common)를 다시 읽습니다. 지도 그림이 바뀐 경우에는 재시작이 필요해요'}>
          {actions.pending === 'reload' ? <span className="spinner spinner-sm" /> : null} 지도 다시 읽기
        </button>
      </div>
    </div>
  )
}

export function RobotCard({ robot, stale, labelOf, color, now, actions }: Props) {
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
        {actions ? <ControlRow robot={robot} stale={stale} actions={actions} /> : null}
      </div>
    </article>
  )
}
