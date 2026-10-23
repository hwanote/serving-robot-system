import { useEffect, useState } from 'react'

/** 주기적으로 갱신되는 현재 시각 (상대 시간 표시용) */
export function useNow(intervalMs = 1000, enabled = true) {
  const [now, setNow] = useState(() => Date.now())
  useEffect(() => {
    if (!enabled) return
    const tick = () => setNow(Date.now())
    // 활성화 시점에 즉시 갱신 (비활성 동안 멈춰 있던 오래된 시각으로 계산하지 않도록)
    const first = setTimeout(tick, 0)
    const id = setInterval(tick, intervalMs)
    return () => {
      clearTimeout(first)
      clearInterval(id)
    }
  }, [intervalMs, enabled])
  return now
}
