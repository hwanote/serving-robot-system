import { useState } from 'react'
import { toApiError } from '../api/client'
import { reloadAllRobotMaps, reloadRobotMap, resetRobot, sendRobotHome } from '../api/robotApi'
import { Icon } from '../components/common/Icon'
import { ErrorState, LoadingState } from '../components/common/StateViews'
import { robotColor } from '../components/map/mapStyles'
import { FleetMap } from '../components/robots/FleetMap'
import { RobotCard } from '../components/robots/RobotCard'
import { ServePanel } from '../components/robots/ServePanel'
import { TaskTable } from '../components/robots/TaskTable'
import { robotDisplayName, useDestinations } from '../hooks/useDestinations'
import { useNow } from '../hooks/useNow'
import { useFleet } from '../state/fleetContext'
import { activeTasks, recentFinishedTasks, sortedRobots } from '../state/fleetReducer'
import { useToast } from '../state/toastContext'
import { nodeLabel } from '../utils/robotControls'

export function RobotControlPage() {
  const { state, cancelTask, refreshTasks } = useFleet()
  const { labelOf } = useDestinations()
  const { notify } = useToast()
  const [destination, setDestination] = useState('')
  const [canceling, setCanceling] = useState<string[]>([])
  const now = useNow(5000)
  /** 요청 중인 카드 버튼: { robot_01: 'reset' } */
  const [pending, setPending] = useState<Record<string, 'reset' | 'home' | 'reload'>>({})

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

  const runAction = async (robotId: string, kind: 'reset' | 'home' | 'reload', call: () => Promise<void>, done: string) => {
    if (pending[robotId]) return
    setPending((p) => ({ ...p, [robotId]: kind }))
    try {
      await call()
      notify(done, 'success')
    } catch (e) {
      notify(toApiError(e).message, 'error')
    } finally {
      setPending((p) => {
        const next = { ...p }
        delete next[robotId]
        return next
      })
    }
  }

  const onReset = (robotId: string, node: string) => {
    const name = robotDisplayName(robotId)
    if (!window.confirm(`${name}을(를) '${nodeLabel(node)}'에 직접 올려놓으셨나요?\n진행 중인 주문이 있으면 취소되고, 로봇은 그 위치에 있다고 믿게 됩니다.`)) return
    void runAction(robotId, 'reset', () => resetRobot(robotId, node), `${name} 위치를 '${nodeLabel(node)}'로 리셋했습니다. 위치가 잡히면 '대기 위치로' 버튼을 쓸 수 있어요.`)
  }
  const onHome = (robotId: string) => void runAction(robotId, 'home', () => sendRobotHome(robotId), `${robotDisplayName(robotId)}을(를) 대기 위치로 보냅니다.`)
  const onReloadMap = (robotId: string) => void runAction(robotId, 'reload', () => reloadRobotMap(robotId), `${robotDisplayName(robotId)}에 지도를 다시 읽도록 했습니다.`)

  const [reloadingAll, setReloadingAll] = useState(false)
  const onReloadAll = async () => {
    if (reloadingAll) return
    setReloadingAll(true)
    try {
      const out = await reloadAllRobotMaps()
      const word: Record<string, string> = { sent: '다시 읽는 중', current: '이미 최신', busy: '임무 중이라 건너뜀', offline: '연결 끊김' }
      const text = Object.entries(out)
        .map(([id, r]) => `${robotDisplayName(id)} ${word[r] ?? r}`)
        .join(' · ')
      notify(text || '대상 로봇이 없습니다.', Object.values(out).some((r) => r === 'busy' || r === 'offline') ? 'error' : 'success')
    } catch (e) {
      notify(toApiError(e).message, 'error')
    } finally {
      setReloadingAll(false)
    }
  }

  return (
    <div className="page">
      <div className="page-header">
        <div>
          <h1>로봇 관제</h1>
          <p>목적지 테이블만 선택하면 Fleet 시스템이 로봇을 배정합니다. 로봇 상태는 실시간(/ws/state)으로 갱신됩니다.</p>
        </div>
        {robots.some((r) => r.controls) ? (
          <button type="button" className="btn btn-sm" onClick={() => void onReloadAll()} disabled={reloadingAll || stale} title="쉬고 있는 모든 로봇이 지도(config/common)를 다시 읽습니다">
            {reloadingAll ? <span className="spinner spinner-sm" /> : <Icon name="refresh" size={14} />} 모든 로봇 지도 다시 읽기
          </button>
        ) : null}
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
              robots.map((r) => (
                <RobotCard
                  key={r.robot_id}
                  robot={r}
                  stale={stale}
                  labelOf={labelOf}
                  color={robotColor(r.robot_id)}
                  now={now}
                  actions={{
                    onReset: (node) => onReset(r.robot_id, node),
                    onHome: () => onHome(r.robot_id),
                    onReloadMap: () => onReloadMap(r.robot_id),
                    pending: pending[r.robot_id] ?? null,
                  }}
                />
              ))
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
