import { lazy, Suspense } from 'react'
import { NavLink, Outlet } from 'react-router-dom'
import { API_MODE, STORE_ID, STORE_NAME } from '../../config/env'
import { Icon, type IconName } from '../common/Icon'
import { ConnectionBanner, ConnectionIndicator } from './ConnectionStatus'

// Mock 모드에서만 로드 (import.meta.env 직접 비교 → real 빌드에서 제거됨)
const MockDevPanel = import.meta.env.VITE_API_MODE !== 'real' ? lazy(() => import('../../mocks/MockDevPanel')) : null

const NAV: { to: string; label: string; icon: IconName }[] = [
  { to: '/', label: '대시보드', icon: 'dashboard' },
  { to: '/map', label: '지도 설정', icon: 'map' },
  { to: '/robots', label: '로봇 관제', icon: 'robot' },
]

export function AppLayout() {
  return (
    <div className="app-shell">
      <aside className="sidebar">
        <div className="sidebar-brand">
          <span className="sidebar-brand-mark">
            <Icon name="robot" size={18} />
          </span>
          Serving Fleet
        </div>
        <nav aria-label="주 메뉴">
          {NAV.map((item) => (
            <NavLink
              key={item.to}
              to={item.to}
              end={item.to === '/'}
              className={({ isActive }) => `sidebar-link${isActive ? ' active' : ''}`}
            >
              <Icon name={item.icon} />
              {item.label}
            </NavLink>
          ))}
        </nav>
        <div className="sidebar-footer">Frontend MVP</div>
      </aside>

      <div className="main">
        <header className="topbar">
          <div className="topbar-store">
            <strong>{STORE_NAME}</strong>
            <span className="mono muted">{STORE_ID}</span>
          </div>
          <div className="topbar-spacer" />
          {API_MODE === 'mock' && (
            <span className="badge tone-amber" title="VITE_API_MODE=mock — Mock 서버 사용 중">
              <Icon name="flask" size={13} /> MOCK
            </span>
          )}
          <ConnectionIndicator />
        </header>
        <ConnectionBanner />
        <main className="content">
          <Outlet />
        </main>
      </div>
      {MockDevPanel && (
        <Suspense fallback={null}>
          <MockDevPanel />
        </Suspense>
      )}
    </div>
  )
}
