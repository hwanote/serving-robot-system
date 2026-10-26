/**
 * 통로 미리보기 Mock. 실제 자동 생성기를 흉내 내지 않고, 지정된 Waypoint 들을 지나는
 * 가로 통로 하나를 만들어 화면(통로·길이·정차 위치 표시, 손으로 옮긴 정차 위치의 검사)을 서버 없이 시험할 수 있게 한다.
 * 손으로 지정한 waypoint(manual)는 통로 위로 붙이고, 통로에서 1.5 m 넘게 떨어져 있으면 거절한다.
 */
import type { SemanticMap } from '../types/contracts'
import type { GeneratedGraph, GraphExcluded, GraphStop } from '../types/provisional'
import { isManualWaypoint, snapAxisYaw } from '../utils/manualStop'

const r3 = (n: number) => Math.round(n * 1000) / 1000
const pad = (n: number) => String(n).padStart(2, '0')

const KIND: Record<string, GraphStop['kind']> = { table: 'table_service', kitchen: 'pickup', counter: 'pickup', robot_wait: 'standby' }
const SNAP_MAX_M = 1.5

/** Waypoint 가 하나도 없으면 null (실제 서버는 이때 통로를 만들 수 없다고 422 로 답한다) */
export function buildMockGraph(doc: SemanticMap, sha: string, matchesApplied: boolean): GeneratedGraph | null {
  const items = doc.waypoints.flatMap((w) => {
    const e = doc.entities.find((x) => x.id === w.entity_id)
    return e ? [{ w, e }] : []
  })
  if (items.length === 0) return null

  const ys = items.map(({ w }) => w.pose.y).sort((a, b) => a - b)
  const spineY = r3(ys[Math.floor(ys.length / 2)])

  const excluded: GraphExcluded[] = []
  const placed = items.filter(({ w, e }) => {
    if (!isManualWaypoint(w) || Math.abs(w.pose.y - spineY) <= SNAP_MAX_M) return true
    const at = e.pose ?? w.pose
    excluded.push({
      id: e.id,
      entity_id: e.id,
      reason: '지정한 위치 가까이(1.5 m 안)에 통로가 없어요. 통로 쪽으로 옮겨 주세요.',
      x: at.x,
      y: at.y,
      pin: { x: w.pose.x, y: w.pose.y, yaw_deg: r3((snapAxisYaw(w.pose.yaw) * 180) / Math.PI) },
    })
    return false
  })

  const xs = [...new Set(placed.map(({ w }) => r3(w.pose.x)))].sort((a, b) => a - b)
  const nodes = xs.map((x, i) => ({ id: `n${pad(i + 1)}`, x, y: spineY, level: 'OK' as const }))
  const nodeOfX = new Map(xs.map((x, i) => [x, nodes[i].id]))
  const edges = nodes.slice(1).map((n, i) => ({ id: `e${pad(i + 1)}`, from: nodes[i].id, to: n.id, length_m: r3(Math.abs(n.x - nodes[i].x)), level: 'OK' as const }))
  const stops: GraphStop[] = placed.map(({ w, e }) => {
    const id = e.type === 'robot_wait' && e.id.startsWith('robot_wait_') ? `standby_${e.id.slice('robot_wait_'.length)}` : e.id
    const yaw = isManualWaypoint(w) ? snapAxisYaw(w.pose.yaw) : w.pose.yaw
    return {
      id,
      entity_id: e.id,
      node: nodeOfX.get(r3(w.pose.x))!,
      x: r3(w.pose.x),
      y: spineY,
      yaw,
      yaw_deg: r3((yaw * 180) / Math.PI),
      kind: KIND[e.type] ?? 'table_service',
      level: 'OK',
      pinned: isManualWaypoint(w),
    }
  })

  return {
    source: 'draft',
    draft_sha: sha,
    matches_applied: matchesApplied,
    ok: excluded.length === 0,
    nodes,
    edges,
    stops,
    excluded,
    problems: excluded.map((x) => `· ${x.id}: ${x.reason}`),
    warnings: [],
    notes: [],
    summary: { OK: nodes.length + edges.length + stops.length, WARN: 0, FAIL: 0 },
  }
}
