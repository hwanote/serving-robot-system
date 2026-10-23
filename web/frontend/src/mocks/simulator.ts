/**
 * Mock Fleet 시뮬레이터 (시연용).
 * 실제 구현에서는 Backend/Fleet 이벤트가 이 역할을 대신한다.
 *
 * Robot: IDLE → NAVIGATING → SERVING → RETURNING → IDLE
 * Task : QUEUED → ASSIGNED → RUNNING → SUCCEEDED
 */
import type { Pose, RobotState, RobotStateEvent } from '../types/contracts'
import type { Task, TaskStateEvent } from '../types/provisional'
import { mockDb, commitMockDb } from './db'
import { DEFAULT_HOME_POSES, MOCK_ROBOT_IDS } from './fixtures'

const TICK_MS = 200
const SPEED_MPS = 0.6
const ASSIGN_DELAY_MS = 900
const START_DELAY_MS = 900
const SERVING_MS = 3500
const HEARTBEAT_MS = 2000

interface SimRobot {
  id: string
  state: RobotState
  pose: Pose
  battery: number
  taskId: string | null
  destinationId: string | null
  target: Pose | null
  /** 현재 단계 종료 시각 (ASSIGNED 대기, SERVING) */
  phaseUntil: number | null
  /** Dev 패널에서 강제로 ERROR 상태로 만든 경우 */
  forcedError: boolean
}

type Broadcast = (message: string) => void

const robots = new Map<string, SimRobot>()
let broadcast: Broadcast = () => {}
let timer: ReturnType<typeof setInterval> | null = null
let lastHeartbeat = 0

function homePose(robotId: string): Pose {
  const waits = (mockDb.semantics?.waypoints ?? [])
    .filter((w) => mockDb.semantics?.entities.some((e) => e.id === w.entity_id && e.type === 'robot_wait'))
    .sort((a, b) => a.entity_id.localeCompare(b.entity_id))
  const index = MOCK_ROBOT_IDS.indexOf(robotId as (typeof MOCK_ROBOT_IDS)[number])
  return { ...(waits[index]?.pose ?? DEFAULT_HOME_POSES[robotId] ?? { x: 0, y: 0, yaw: 0 }) }
}

function ensureRobots() {
  if (robots.size) return
  MOCK_ROBOT_IDS.forEach((id, i) => {
    robots.set(id, {
      id,
      state: 'IDLE',
      pose: homePose(id),
      battery: 92 - i * 9,
      taskId: null,
      destinationId: null,
      target: null,
      phaseUntil: null,
      forcedError: false,
    })
  })
}

export function robotEvent(r: SimRobot): RobotStateEvent {
  return {
    type: 'robot_state',
    robot_id: r.id,
    state: r.state,
    task_id: r.taskId,
    destination_id: r.destinationId,
    battery_percent: Math.round(r.battery),
    pose: { x: round(r.pose.x), y: round(r.pose.y), yaw: round(r.pose.yaw) },
  }
}

const round = (n: number) => Math.round(n * 1000) / 1000

function emitRobot(r: SimRobot) {
  broadcast(JSON.stringify(robotEvent(r)))
}

function emitTask(t: Task) {
  const event: TaskStateEvent = {
    type: 'task_state',
    task_id: t.task_id,
    status: t.status,
    robot_id: t.robot_id,
    destination_id: t.destination_id,
    updated_at: t.updated_at,
  }
  broadcast(JSON.stringify(event))
}

function setTask(task: Task, patch: Partial<Task>) {
  Object.assign(task, patch, { updated_at: new Date().toISOString() })
  emitTask(task)
  commitMockDb()
}

function setRobot(r: SimRobot, state: RobotState, patch: Partial<SimRobot> = {}) {
  Object.assign(r, patch)
  r.state = state
  emitRobot(r)
}

function destinationPose(destinationId: string): Pose | null {
  return mockDb.semantics?.waypoints.find((w) => w.entity_id === destinationId)?.pose ?? null
}

/** 목표 지점으로 이동. 도착하면 true */
function moveToward(r: SimRobot, target: Pose, dt: number): boolean {
  const dx = target.x - r.pose.x
  const dy = target.y - r.pose.y
  const dist = Math.hypot(dx, dy)
  const step = SPEED_MPS * dt
  if (dist <= step) {
    r.pose = { ...target }
    return true
  }
  r.pose = { x: r.pose.x + (dx / dist) * step, y: r.pose.y + (dy / dist) * step, yaw: Math.atan2(dy, dx) }
  r.battery = Math.max(5, r.battery - 0.01)
  return false
}

function tick() {
  const now = Date.now()
  const dt = TICK_MS / 1000

  // 1) 대기 중인 Task 배정
  for (const task of mockDb.tasks) {
    if (task.status !== 'QUEUED') continue
    if (now - Date.parse(task.created_at) < ASSIGN_DELAY_MS) continue
    const robot = [...robots.values()].find((r) => r.state === 'IDLE' && !r.taskId && !r.forcedError)
    if (!robot) break
    setTask(task, { status: 'ASSIGNED', robot_id: robot.id })
    robot.taskId = task.task_id
    robot.destinationId = task.destination_id
    robot.phaseUntil = now + START_DELAY_MS
    emitRobot(robot)
  }

  // 2) 로봇 상태 진행
  for (const r of robots.values()) {
    if (r.forcedError) continue
    const task = r.taskId ? mockDb.tasks.find((t) => t.task_id === r.taskId) : undefined

    if (r.state === 'IDLE' && task?.status === 'ASSIGNED' && r.phaseUntil !== null && now >= r.phaseUntil) {
      const target = destinationPose(task.destination_id)
      if (!target) {
        setTask(task, { status: 'FAILED' })
        setRobot(r, 'IDLE', { taskId: null, destinationId: null, phaseUntil: null })
        continue
      }
      setTask(task, { status: 'RUNNING' })
      setRobot(r, 'NAVIGATING', { target, phaseUntil: null })
    } else if (r.state === 'NAVIGATING' && r.target) {
      if (moveToward(r, r.target, dt)) {
        setRobot(r, 'SERVING', { phaseUntil: now + SERVING_MS })
      }
    } else if (r.state === 'SERVING' && r.phaseUntil !== null && now >= r.phaseUntil) {
      if (task && task.status === 'RUNNING') setTask(task, { status: 'SUCCEEDED' })
      setRobot(r, 'RETURNING', { taskId: null, destinationId: null, target: homePose(r.id), phaseUntil: null })
    } else if (r.state === 'RETURNING') {
      const home = r.target ?? homePose(r.id)
      if (moveToward(r, home, dt)) {
        setRobot(r, 'IDLE', { target: null, pose: { ...home } })
      }
    }
  }

  // 3) 이동 중 위치 / 주기적 heartbeat
  const heartbeat = now - lastHeartbeat >= HEARTBEAT_MS
  if (heartbeat) lastHeartbeat = now
  for (const r of robots.values()) {
    if (heartbeat || r.state === 'NAVIGATING' || r.state === 'RETURNING') emitRobot(r)
  }
}

// ---------- 외부 API (handlers / dev 패널) ----------

export function startSimulator(send: Broadcast) {
  broadcast = send
  ensureRobots()
  if (!timer) timer = setInterval(tick, TICK_MS)
}

export function stopSimulator() {
  if (timer) clearInterval(timer)
  timer = null
}

/** 새 연결에 현재 상태 스냅샷 전송 */
export function snapshotMessages(): string[] {
  ensureRobots()
  const robotMsgs = [...robots.values()].map((r) => JSON.stringify(robotEvent(r)))
  return robotMsgs
}

export function enqueueTask(task: Task) {
  emitTask(task)
}

/** 취소 처리. 이미 종료된 Task 면 false */
export function cancelSimTask(task: Task): boolean {
  if (task.status === 'SUCCEEDED' || task.status === 'FAILED' || task.status === 'CANCELED') return false
  setTask(task, { status: 'CANCELED' })
  const r = task.robot_id ? robots.get(task.robot_id) : undefined
  if (r && r.taskId === task.task_id) {
    if (r.state === 'NAVIGATING' || r.state === 'SERVING') {
      setRobot(r, 'RETURNING', { taskId: null, destinationId: null, target: homePose(r.id), phaseUntil: null })
    } else {
      setRobot(r, r.state, { taskId: null, destinationId: null, phaseUntil: null })
    }
  }
  return true
}

/** Dev 패널: 로봇 오류 상태 토글 */
export function toggleRobotError(robotId: string) {
  const r = robots.get(robotId)
  if (!r) return
  if (r.forcedError) {
    r.forcedError = false
    // 이동 목표가 있으면 주행 재개, 배정만 된 상태면 곧바로 출발 처리
    setRobot(r, r.taskId && r.target ? 'NAVIGATING' : 'IDLE', { phaseUntil: r.taskId ? Date.now() : null })
  } else {
    r.forcedError = true
    setRobot(r, 'ERROR')
  }
}

export function isRobotForcedError(robotId: string): boolean {
  return robots.get(robotId)?.forcedError ?? false
}

/** Dev 패널: 로봇 위치/상태 초기화 */
export function resetSimulator() {
  robots.clear()
  ensureRobots()
  robots.forEach(emitRobot)
}
