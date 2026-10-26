/**
 * Mock 모드 전용 개발 패널. 실패/연결 끊김 시나리오를 재현한다.
 * VITE_API_MODE=mock 일 때만 lazy import 되며 real 빌드에는 포함되지 않는다.
 */
import { useState, useSyncExternalStore } from 'react'
import { Icon } from '../components/common/Icon'
import { dropAllSocketClients } from './browser'
import { FAILURE_KEYS, FAILURE_LABELS, mockDb, commitMockDb, resetMockDb, seedMockDb, subscribeMockDb } from './db'
import { MOCK_ROBOT_IDS } from './fixtures'
import { isRobotForcedError, isRobotLost, resetSimulator, toggleRobotError, toggleRobotLost } from './simulator'

let version = 0
subscribeMockDb(() => {
  version++
})

function useMockDbVersion() {
  return useSyncExternalStore(subscribeMockDb, () => version)
}

const panelStyle: React.CSSProperties = {
  position: 'fixed',
  left: 16,
  bottom: 16,
  zIndex: 40,
  width: 300,
  background: '#fff',
  border: '1px solid var(--border-strong)',
  borderRadius: 10,
  boxShadow: '0 10px 30px rgba(0,0,0,.15)',
  fontSize: 13,
}

export default function MockDevPanel() {
  const [open, setOpen] = useState(false)
  const [, force] = useState(0)
  useMockDbVersion()

  const update = (fn: () => void) => {
    fn()
    commitMockDb()
    force((n) => n + 1)
  }

  const reloadApp = () => window.location.reload()

  if (!open) {
    return (
      <button
        type="button"
        className="btn btn-sm"
        style={{ position: 'fixed', left: 16, bottom: 16, zIndex: 40, width: 188 }}
        onClick={() => setOpen(true)}
      >
        <Icon name="flask" size={14} /> Mock 제어
      </button>
    )
  }

  return (
    <div style={panelStyle} role="dialog" aria-label="Mock 제어 패널">
      <div className="card-header" style={{ padding: '10px 14px' }}>
        <strong>Mock 제어 (개발용)</strong>
        <button type="button" className="btn btn-sm btn-ghost" aria-label="닫기" onClick={() => setOpen(false)}>
          <Icon name="x" size={14} />
        </button>
      </div>
      <div style={{ padding: 14, display: 'flex', flexDirection: 'column', gap: 12 }}>
        <section>
          <div className="field-label">API 실패 시뮬레이션 (HTTP 503)</div>
          {FAILURE_KEYS.map((key) => (
            <label key={key} style={{ display: 'flex', gap: 8, alignItems: 'center', marginTop: 4 }}>
              <input
                type="checkbox"
                checked={mockDb.failures[key]}
                onChange={(e) => update(() => (mockDb.failures[key] = e.target.checked))}
              />
              {FAILURE_LABELS[key]}
            </label>
          ))}
          <label style={{ display: 'flex', gap: 8, alignItems: 'center', marginTop: 8 }}>
            응답 지연
            <select
              className="select input-sm"
              value={mockDb.latencyMs}
              onChange={(e) => update(() => (mockDb.latencyMs = Number(e.target.value)))}
            >
              <option value={0}>없음</option>
              <option value={300}>300ms</option>
              <option value={1500}>1.5s</option>
              <option value={4000}>4s</option>
            </select>
          </label>
        </section>

        <section>
          <div className="field-label">WebSocket</div>
          <label style={{ display: 'flex', gap: 8, alignItems: 'center', marginTop: 4 }}>
            <input
              type="checkbox"
              checked={mockDb.wsDown}
              onChange={(e) =>
                update(() => {
                  mockDb.wsDown = e.target.checked
                  if (e.target.checked) dropAllSocketClients()
                })
              }
            />
            서버 다운 (연결 거부)
          </label>
          <button type="button" className="btn btn-sm" style={{ marginTop: 6 }} onClick={dropAllSocketClients}>
            현재 연결 끊기
          </button>
        </section>

        <section>
          <div className="field-label">로봇</div>
          <div className="btn-row" style={{ marginTop: 4 }}>
            {MOCK_ROBOT_IDS.map((id) => (
              <button key={id} type="button" className="btn btn-sm" onClick={() => update(() => toggleRobotError(id))}>
                {id} {isRobotForcedError(id) ? '오류 해제' : '오류 발생'}
              </button>
            ))}
            {MOCK_ROBOT_IDS.map((id) => (
              <button key={`lost-${id}`} type="button" className="btn btn-sm" onClick={() => update(() => toggleRobotLost(id))}>
                {id} {isRobotLost(id) ? '위치 복구' : '위치 잃음'}
              </button>
            ))}
            <button type="button" className="btn btn-sm" onClick={() => update(resetSimulator)}>
              위치 초기화
            </button>
          </div>
        </section>

        <section>
          <div className="field-label">데이터</div>
          <div className="btn-row" style={{ marginTop: 4 }}>
            <button type="button" className="btn btn-sm" onClick={() => { seedMockDb(); reloadApp() }}>
              데모 데이터 채우기
            </button>
            <button type="button" className="btn btn-sm btn-danger" onClick={() => { resetMockDb(); reloadApp() }}>
              초기화 (지도 없음)
            </button>
          </div>
          <div className="muted" style={{ marginTop: 6, fontSize: 12 }}>
            지도 r{mockDb.rawMap?.map_revision ?? '-'} · Semantic r{mockDb.semantics?.map_revision ?? '-'} · 매핑 {mockDb.mappingPhase}
          </div>
        </section>
      </div>
    </div>
  )
}
