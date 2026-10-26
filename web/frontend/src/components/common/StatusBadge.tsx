import type { RobotState, TaskStatus } from '../../types/contracts'
import { ROBOT_STATE_META, TASK_STATUS_META, type Tone } from '../../utils/labels'

export function Badge({ tone, children, dot = false, title }: { tone: Tone; children: React.ReactNode; dot?: boolean; title?: string }) {
  return (
    <span className={`badge tone-${tone}${dot ? ' badge-dot' : ''}`} title={title}>
      {children}
    </span>
  )
}

export function RobotStateBadge({ state }: { state: RobotState }) {
  const meta = ROBOT_STATE_META[state]
  return (
    <Badge tone={meta.tone} dot title={state}>
      {meta.label}
      <span className="mono" style={{ fontSize: 10.5, opacity: 0.7 }}>
        {state}
      </span>
    </Badge>
  )
}

export function TaskStatusBadge({ status }: { status: TaskStatus }) {
  const meta = TASK_STATUS_META[status]
  return (
    <Badge tone={meta.tone} title={status}>
      {meta.label}
      <span className="mono" style={{ fontSize: 10.5, opacity: 0.7 }}>
        {status}
      </span>
    </Badge>
  )
}
