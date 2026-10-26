import type { ReactNode } from 'react'
import { Link } from 'react-router-dom'
import { Icon } from '../components/common/Icon'
import { ErrorState } from '../components/common/StateViews'
import { Badge, RobotStateBadge } from '../components/common/StatusBadge'
import { TaskTable } from '../components/robots/TaskTable'
import { API_MODE, STORE_ID, STORE_NAME } from '../config/env'
import { robotDisplayName, useDestinations } from '../hooks/useDestinations'
import { useNow } from '../hooks/useNow'
import { useFleet } from '../state/fleetContext'
import { activeTasks, recentFinishedTasks, sortedRobots } from '../state/fleetReducer'
import { useStoreData } from '../state/storeDataContext'
import { formatAgo, isWorkingState, MAPPING_PHASE_META, SOCKET_STATUS_META } from '../utils/labels'

function Stat({ label, value, sub, testId }: { label: string; value: ReactNode; sub?: ReactNode; testId?: string }) {
  return (
    <div className="card stat" data-testid={testId}>
      <span className="stat-label">{label}</span>
      <span className="stat-value">{value}</span>
      {sub && <span className="stat-sub">{sub}</span>}
    </div>
  )
}

export function DashboardPage() {
  const { state, refreshTasks } = useFleet()
  const { rawMap, semantics, mappingStatus } = useStoreData()
  const { labelOf, destinations, revisionMismatch } = useDestinations()
  const now = useNow(5000)

  const robots = sortedRobots(state)
  const working = robots.filter((r) => isWorkingState(r.state)).length
  const idle = robots.filter((r) => r.state === 'IDLE').length
  const other = robots.length - working - idle
  const socketMeta = SOCKET_STATUS_META[state.socket.status]
  const phase = mappingStatus.data?.phase
  const realtimeKnown = state.everConnected || robots.length > 0
  const stale = state.socket.status !== 'open'

  return (
    <div className="page">
      <div className="page-header">
        <div>
          <h1>대시보드</h1>
          <p>매장 운영 현황과 로봇·작업 상태를 한눈에 확인합니다.</p>
        </div>
        <div className="btn-row">
          <Link className="btn" to="/map">
            <Icon name="map" size={14} /> 지도 설정
          </Link>
          <Link className="btn btn-primary" to="/robots">
            <Icon name="robot" size={14} /> 로봇 관제
          </Link>
        </div>
      </div>

      <div className="grid grid-stats">
        <Stat label="매장" value={<span style={{ fontSize: 18 }}>{STORE_NAME}</span>} sub={<span className="mono">{STORE_ID}</span>} />
        <Stat
          label="매핑 상태"
          testId="stat-mapping"
          value={
            phase ? (
              <Badge tone={MAPPING_PHASE_META[phase].tone} dot>
                {MAPPING_PHASE_META[phase].label}
              </Badge>
            ) : mappingStatus.error ? (
              <Badge tone="gray">알 수 없음</Badge>
            ) : (
              <span className="spinner spinner-sm" />
            )
          }
          sub={mappingStatus.error ?? undefined}
        />
        <Stat
          label="Map Revision"
          testId="stat-revision"
          value={rawMap.data ? `r${rawMap.data.map_revision}` : rawMap.loaded ? '지도 없음' : '…'}
          sub={
            revisionMismatch
              ? <span style={{ color: 'var(--warn)' }} data-testid="dash-revision-mismatch">⚠ Semantic r{revisionMismatch.semantic} ≠ 지도 r{revisionMismatch.map} · 갱신 필요</span>
              : semantics.data
              ? `Semantic r${semantics.data.map_revision} · 서빙 테이블 ${destinations.length}개`
              : rawMap.error
                ? rawMap.error
                : 'Semantic 미설정'
          }
        />
        <Stat label="로봇" testId="stat-robots" value={realtimeKnown ? `${robots.length}대` : '…'} sub={other > 0 ? `충전/정지/오류 ${other}대` : undefined} />
        <Stat label="작업 중 로봇" testId="stat-working" value={realtimeKnown ? `${working}대` : '…'} sub="이동·서빙·복귀" />
        <Stat label="대기 중 로봇" testId="stat-idle" value={realtimeKnown ? `${idle}대` : '…'} sub="IDLE" />
        <Stat
          label="시스템 연결"
          testId="stat-connection"
          value={
            <Badge tone={socketMeta.tone} dot>
              {socketMeta.label}
            </Badge>
          }
          sub={`${API_MODE === 'mock' ? 'Mock' : 'Backend'} · /ws/state${state.socket.attempt ? ` · 재시도 ${state.socket.attempt}회` : ''}`}
        />
      </div>

      <div className="grid grid-2">
        <section className="card" aria-labelledby="dash-active">
          <div className="card-header">
            <h2 id="dash-active">현재 Task</h2>
            <span className="muted">{activeTasks(state).length}건</span>
          </div>
          {state.tasksError && !Object.keys(state.tasks).length ? (
            <ErrorState title="Task 목록을 불러오지 못했습니다." message={state.tasksError} onRetry={() => void refreshTasks()} />
          ) : (
            <TaskTable tasks={activeTasks(state)} labelOf={labelOf} emptyText="진행 중인 Task가 없습니다." />
          )}
        </section>

        <section className="card" aria-labelledby="dash-done">
          <div className="card-header">
            <h2 id="dash-done">최근 완료 Task</h2>
          </div>
          <TaskTable tasks={recentFinishedTasks(state, 5)} labelOf={labelOf} emptyText="최근 종료된 Task가 없습니다." timeField="updated_at" />
        </section>
      </div>

      <section className="card" aria-labelledby="dash-robots">
        <div className="card-header">
          <h2 id="dash-robots">로봇 현황</h2>
          {stale && realtimeKnown && <Badge tone="red">마지막 수신값</Badge>}
        </div>
        {robots.length === 0 ? (
          <p className="muted" style={{ margin: 0, padding: '14px 18px' }}>
            {state.socket.status === 'reconnecting' ? '실시간 연결이 끊겨 로봇 상태를 알 수 없습니다.' : '로봇 상태 수신 대기 중…'}
          </p>
        ) : (
          <table className="table">
            <thead>
              <tr>
                <th>로봇</th>
                <th>상태</th>
                <th>배터리</th>
                <th>목적지</th>
                <th>Task</th>
                <th>마지막 수신</th>
              </tr>
            </thead>
            <tbody>
              {robots.map((r) => (
                <tr key={r.robot_id}>
                  <td>
                    <strong>{robotDisplayName(r.robot_id)}</strong> <span className="mono muted">{r.robot_id}</span>
                  </td>
                  <td>
                    <RobotStateBadge state={r.state} />
                  </td>
                  <td className="mono">{r.battery_percent}%</td>
                  <td>{r.destination_id ? labelOf(r.destination_id) : <span className="muted">-</span>}</td>
                  <td className="mono">{r.task_id ?? <span className="muted">-</span>}</td>
                  <td className="muted">{formatAgo(r.receivedAt, now)}</td>
                </tr>
              ))}
            </tbody>
          </table>
        )}
      </section>
    </div>
  )
}
