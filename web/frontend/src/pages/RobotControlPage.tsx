import { useState } from 'react'
import { toApiError } from '../api/client'
import { ErrorState, LoadingState } from '../components/common/StateViews'
import { robotColor } from '../components/map/mapStyles'
import { FleetMap } from '../components/robots/FleetMap'
import { RobotCard } from '../components/robots/RobotCard'
import { ServePanel } from '../components/robots/ServePanel'
import { TaskTable } from '../components/robots/TaskTable'
import { useDestinations } from '../hooks/useDestinations'
import { useNow } from '../hooks/useNow'
import { useFleet } from '../state/fleetContext'
import { activeTasks, recentFinishedTasks, sortedRobots } from '../state/fleetReducer'
import { useToast } from '../state/toastContext'

export function RobotControlPage() {
  const { state, cancelTask, refreshTasks } = useFleet()
  const { labelOf } = useDestinations()
  const { notify } = useToast()
  const [destination, setDestination] = useState('')
  const [canceling, setCanceling] = useState<string[]>([])
  const now = useNow(5000)

  const robots = sortedRobots(state)
  const stale = state.socket.status !== 'open'
  const active = activeTasks(state)
  const finished = recentFinishedTasks(state, 6)

  const onCancel = async (taskId: string) => {
    if (canceling.includes(taskId)) return
    if (!window.confirm(`${taskId} 작업을 취소할까요?`)) return
    setCanceling((ids) => [...ids, taskId])
    try {
      await cancelTask(taskId)
      notify(`${taskId} 취소를 요청했습니다.`, 'success')
    } catch (e) {
      notify(`취소 실패: ${toApiError(e).message}`, 'error')
    } finally {
      setCanceling((ids) => ids.filter((id) => id !== taskId))
    }
  }

  return (
    <div className="page">
      <div className="page-header">
        <div>
          <h1>로봇 관제</h1>
          <p>목적지 테이블만 선택하면 Fleet 시스템이 로봇을 배정합니다. 로봇 상태는 실시간(/ws/state)으로 갱신됩니다.</p>
        </div>
      </div>

      <div className="robot-layout">
        <div className="robot-side">
          <ServePanel selected={destination} onSelect={setDestination} />

          <section aria-label="로봇 상태" style={{ display: 'flex', flexDirection: 'column', gap: 12 }}>
            {robots.length === 0 ? (
              <div className="card">
                {state.socket.status === 'reconnecting' ? (
                  <ErrorState title="로봇 상태를 수신하지 못했습니다." message="실시간 연결을 재시도하고 있습니다." />
                ) : (
                  <LoadingState message="로봇 상태 수신 대기 중…" />
                )}
              </div>
            ) : (
              robots.map((r) => <RobotCard key={r.robot_id} robot={r} stale={stale} labelOf={labelOf} color={robotColor(r.robot_id)} now={now} />)
            )}
          </section>
        </div>

        <section className="card robot-map" aria-label="로봇 위치">
          <div className="card-header">
            <h2>로봇 위치</h2>
            <span className="muted" style={{ fontSize: 12 }}>
              점선 테두리: 선택한 목적지
            </span>
          </div>
          <div style={{ flex: 1, minHeight: 0, display: 'flex' }}>
            <FleetMap highlightIds={destination ? [destination] : []} />
          </div>
        </section>
      </div>

      <section className="card" aria-labelledby="active-tasks-title">
        <div className="card-header">
          <h2 id="active-tasks-title">진행 중 Task</h2>
          <button type="button" className="btn btn-sm" onClick={() => void refreshTasks()} disabled={state.tasksLoading}>
            {state.tasksLoading ? <span className="spinner spinner-sm" /> : null} 새로고침
          </button>
        </div>
        {state.tasksError && !active.length ? (
          <ErrorState title="Task 목록을 불러오지 못했습니다." message={state.tasksError} onRetry={() => void refreshTasks()} />
        ) : (
          <TaskTable tasks={active} labelOf={labelOf} onCancel={(id) => void onCancel(id)} cancelingIds={canceling} emptyText="진행 중인 Task가 없습니다." />
        )}
      </section>

      <section className="card" aria-labelledby="done-tasks-title">
        <div className="card-header">
          <h2 id="done-tasks-title">최근 종료 Task</h2>
        </div>
        <TaskTable tasks={finished} labelOf={labelOf} emptyText="최근 종료된 Task가 없습니다." timeField="updated_at" />
      </section>
    </div>
  )
}
