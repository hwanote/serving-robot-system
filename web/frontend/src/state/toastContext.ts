import { createContext, useContext } from 'react'

export type ToastKind = 'info' | 'success' | 'error'

export interface ToastContextValue {
  notify: (message: string, kind?: ToastKind) => void
}

export const ToastContext = createContext<ToastContextValue>({ notify: () => {} })

export const useToast = () => useContext(ToastContext)
