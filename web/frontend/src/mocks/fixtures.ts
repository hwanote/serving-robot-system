/**
 * Mock 전용 고정 데이터. 실제 카페 지도의 최종 구조라고 가정하지 않는다.
 * 지도 이미지는 scripts/generate-mock-map.mjs 로 생성 (같은 geometry 사용)
 */
import type { Pose, RawMapInfo, SemanticMap } from '../types/contracts'

export const MOCK_STORE_ID = 'demo_store'

export const MOCK_MAP_GEOMETRY: Pick<RawMapInfo, 'resolution' | 'width' | 'height' | 'origin'> = {
  resolution: 0.05,
  width: 158,
  height: 120,
  origin: { x: -0.75, y: -0.82, yaw: 0.0 },
}

export function buildRawMap(storeId: string, revision: number): RawMapInfo {
  return {
    store_id: storeId,
    map_revision: revision,
    image_url: `/api/stores/${storeId}/map.png`,
    ...MOCK_MAP_GEOMETRY,
    origin: { ...MOCK_MAP_GEOMETRY.origin },
    frame_id: 'map',
  }
}

export const MOCK_ROBOT_IDS = ['robot_01', 'robot_02'] as const

/** robot_wait Waypoint 가 없을 때 사용하는 기본 대기 위치 */
export const DEFAULT_HOME_POSES: Record<string, Pose> = {
  robot_01: { x: 0.1, y: 0.2, yaw: 0 },
  robot_02: { x: 0.1, y: 1.0, yaw: 0 },
}

const HALF_PI = 1.5708

/** Mock 개발 패널 '데모 데이터 채우기' 용 Semantic Map */
export function buildSeedSemantics(storeId: string, revision: number): SemanticMap {
  return {
    store_id: storeId,
    map_revision: revision,
    entities: [
      { id: 'table_01', type: 'table', label: 'Table 1', pose: { x: 2.0, y: 1.0, yaw: 0 }, top_size_m: { x: 0.8, y: 0.8 } },
      { id: 'table_02', type: 'table', label: 'Table 2', pose: { x: 4.0, y: 1.0, yaw: 0 }, top_size_m: { x: 0.8, y: 0.8 } },
      { id: 'table_03', type: 'table', label: 'Table 3', pose: { x: 5.8, y: 1.0, yaw: 0 }, top_size_m: { x: 0.8, y: 0.8 } },
      { id: 'counter_01', type: 'counter', label: 'Counter 1', pose: { x: 3.0, y: 4.3, yaw: 0 } },
      { id: 'kitchen_01', type: 'kitchen', label: 'Kitchen 1', pose: { x: 0.5, y: 4.2, yaw: 0 } },
      { id: 'robot_wait_01', type: 'robot_wait', label: 'Robot Wait 1', pose: { x: 0.1, y: 0.2, yaw: 0 } },
      { id: 'robot_wait_02', type: 'robot_wait', label: 'Robot Wait 2', pose: { x: 0.1, y: 1.0, yaw: 0 } },
    ],
    waypoints: [
      { id: 'wp_table_01', entity_id: 'table_01', pose: { x: 2.0, y: 1.75, yaw: -HALF_PI } },
      { id: 'wp_table_02', entity_id: 'table_02', pose: { x: 4.0, y: 1.75, yaw: -HALF_PI } },
      { id: 'wp_table_03', entity_id: 'table_03', pose: { x: 5.8, y: 1.75, yaw: -HALF_PI } },
      { id: 'wp_counter_01', entity_id: 'counter_01', pose: { x: 3.0, y: 3.7, yaw: HALF_PI } },
      { id: 'wp_kitchen_01', entity_id: 'kitchen_01', pose: { x: 1.3, y: 3.0, yaw: HALF_PI } },
      { id: 'wp_robot_wait_01', entity_id: 'robot_wait_01', pose: { x: 0.1, y: 0.2, yaw: 0 } },
      { id: 'wp_robot_wait_02', entity_id: 'robot_wait_02', pose: { x: 0.1, y: 1.0, yaw: 0 } },
    ],
    zones: [
      {
        id: 'zone_exclusive_01',
        type: 'exclusive',
        polygon: [
          [1.0, 3.0],
          [1.6, 3.0],
          [1.6, 3.6],
          [1.0, 3.6],
        ],
      },
      {
        id: 'zone_no_go_01',
        type: 'no_go',
        polygon: [
          [5.5, 3.5],
          [6.3, 3.5],
          [6.3, 4.3],
          [5.5, 4.3],
        ],
      },
      {
        id: 'zone_slow_01',
        type: 'slow',
        polygon: [
          [2.1, 3.6],
          [3.9, 3.6],
          [3.9, 4.0],
          [2.1, 4.0],
        ],
      },
    ],
  }
}
