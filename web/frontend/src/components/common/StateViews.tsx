import type { ReactNode } from 'react'
import { Icon } from './Icon'

export function LoadingState({ message = '불러오는 중…' }: { message?: string }) {
  return (
    <div className="state-box" role="status">
      <div className="spinner" />
      <span>{message}</span>
    </div>
  )
}

export function ErrorState({ title = '데이터를 불러오지 못했습니다.', message, onRetry }: { title?: string; message?: string | null; onRetry?: () => void }) {
  return (
    <div className="state-box state-box-error" role="alert">
      <Icon name="alert" size={22} />
      <strong>{title}</strong>
      {message && <span>{message}</span>}
      {onRetry && (
        <button type="button" className="btn btn-sm" onClick={onRetry}>
          <Icon name="refresh" size={14} /> 다시 시도
        </button>
      )}
    </div>
  )
}

export function EmptyState({ title, children }: { title: string; children?: ReactNode }) {
  return (
    <div className="state-box">
      <strong>{title}</strong>
      {children}
    </div>
  )
}

export function InlineError({ message }: { message: string | null | undefined }) {
  if (!message) return null
  return (
    <div className="alert alert-error" role="alert">
      <Icon name="alert" size={15} />
      <span>{message}</span>
    </div>
  )
}
