import { robotDisplayName } from '../../hooks/useDestinations'
import type { Task } from '../../types/provisional'
import { formatTime } from '../../utils/labels'
import { isActiveTask } from '../../state/fleetReducer'
import { TaskStatusBadge } from '../common/StatusBadge'

interface Props {
  tasks: Task[]
  labelOf: (id: string | null) => string
  onCancel?: (taskId: string) => void
  cancelingIds?: string[]
  emptyText: string
  timeField?: 'created_at' | 'updated_at'
}

export function TaskTable({ tasks, labelOf, onCancel, cancelingIds = [], emptyText, timeField = 'created_at' }: Props) {
  if (!tasks.length) return <p className="muted" style={{ margin: 0, padding: '14px 18px' }}>{emptyText}</p>
  return (
    <div style={{ overflowX: 'auto' }}>
      <table className="table">
        <thead>
          <tr>
            <th>Task</th>
            <th>목적지</th>
            <th>상태</th>
            <th>로봇</th>
            <th>{timeField === 'created_at' ? '생성' : '갱신'}</th>
            {onCancel && <th aria-label="작업" />}
          </tr>
        </thead>
        <tbody>
          {tasks.map((t) => {
            const canceling = cancelingIds.includes(t.task_id)
            return (
              <tr key={t.task_id} data-testid={`task-row-${t.task_id}`}>
                <td className="mono">{t.task_id}</td>
                <td>
                  <strong>{labelOf(t.destination_id)}</strong> <span className="mono muted">{t.destination_id}</span>
                </td>
                <td>
                  <TaskStatusBadge status={t.status} />
                </td>
                <td>{t.robot_id ? robotDisplayName(t.robot_id) : <span className="muted">배정 대기</span>}</td>
                <td className="mono muted">{formatTime(t[timeField])}</td>
                {onCancel && (
                  <td style={{ textAlign: 'right' }}>
                    {isActiveTask(t) && (
                      <button type="button" className="btn btn-sm btn-danger" disabled={canceling} onClick={() => onCancel(t.task_id)}>
                        {canceling ? <span className="spinner spinner-sm" /> : null} 취소
                      </button>
                    )}
                  </td>
                )}
              </tr>
            )
          })}
        </tbody>
      </table>
    </div>
  )
}
