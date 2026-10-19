import { useCallback, useMemo, useRef, useState, type ReactNode } from 'react'
import { Icon } from '../components/common/Icon'
import { ToastContext, type ToastKind } from './toastContext'

interface ToastItem {
  id: number
  message: string
  kind: ToastKind
}

export function ToastProvider({ children }: { children: ReactNode }) {
  const [items, setItems] = useState<ToastItem[]>([])
  const seq = useRef(0)

  const dismiss = useCallback((id: number) => setItems((list) => list.filter((t) => t.id !== id)), [])

  const notify = useCallback(
    (message: string, kind: ToastKind = 'info') => {
      const id = ++seq.current
      setItems((list) => [...list.slice(-3), { id, message, kind }])
      setTimeout(() => dismiss(id), kind === 'error' ? 7000 : 3500)
    },
    [dismiss],
  )

  const value = useMemo(() => ({ notify }), [notify])

  return (
    <ToastContext.Provider value={value}>
      {children}
      <div className="toast-stack" role="status" aria-live="polite">
        {items.map((t) => (
          <div key={t.id} className={`toast toast-${t.kind}`}>
            <Icon name={t.kind === 'error' ? 'alert' : 'check'} />
            <span>{t.message}</span>
            <button type="button" aria-label="닫기" onClick={() => dismiss(t.id)}>
              <Icon name="x" size={14} />
            </button>
          </div>
        ))}
      </div>
    </ToastContext.Provider>
  )
}
