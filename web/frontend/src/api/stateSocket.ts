import { parseStateEvent, type StateSocketEvent } from './parsers'

/**
 * connecting   : 최초 연결 시도 중
 * open         : 연결됨 (실시간 상태 수신)
 * reconnecting : 연결이 끊겨 재시도 대기/시도 중
 * closed       : 앱이 연결을 종료함
 */
export type SocketStatus = 'connecting' | 'open' | 'reconnecting' | 'closed'

export interface SocketStatusInfo {
  status: SocketStatus
  /** 연속 재연결 시도 횟수 (연결 성공 시 0) */
  attempt: number
  /** 다음 재연결 예정 시각 (ms epoch) */
  nextRetryAt: number | null
  /** 이번 재연결 대기 시간 (ms) */
  retryDelayMs: number | null
  lastError: string | null
}

export interface MinimalWebSocket {
  onopen: ((ev: Event) => void) | null
  onclose: ((ev: CloseEvent) => void) | null
  onerror: ((ev: Event) => void) | null
  onmessage: ((ev: MessageEvent) => void) | null
  close(code?: number, reason?: string): void
}

export interface StateSocketOptions {
  url: string
  onEvent: (event: StateSocketEvent) => void
  onStatus: (info: SocketStatusInfo) => void
  /** 형식이 잘못된 메시지 (Contract 위반) */
  onInvalidMessage?: (reason: string) => void
  createSocket?: (url: string) => MinimalWebSocket
  /** 재연결 대기 (ms): min(maxDelay, baseDelay * 2^attempt) */
  baseDelayMs?: number
  maxDelayMs?: number
}

/**
 * /ws/state 연결 관리자.
 * 연결이 끊겨도 로봇 상태를 임의로 바꾸지 않는다 (마지막 수신 상태 유지는 상위 Store 책임).
 */
export class StateSocket {
  private socket: MinimalWebSocket | null = null
  private retryTimer: ReturnType<typeof setTimeout> | null = null
  private attempt = 0
  private stopped = true
  private readonly opts: Required<Omit<StateSocketOptions, 'onInvalidMessage'>> &
    Pick<StateSocketOptions, 'onInvalidMessage'>

  constructor(options: StateSocketOptions) {
    // undefined 로 전달된 옵션이 기본값을 덮어쓰지 않도록 개별 지정
    this.opts = {
      ...options,
      createSocket: options.createSocket ?? ((url) => new WebSocket(url)),
      baseDelayMs: options.baseDelayMs ?? 1000,
      maxDelayMs: options.maxDelayMs ?? 30_000,
    }
  }

  start() {
    if (!this.stopped) return
    this.stopped = false
    this.attempt = 0
    this.emit('connecting', null, null)
    this.open()
  }

  stop() {
    this.stopped = true
    if (this.retryTimer) clearTimeout(this.retryTimer)
    this.retryTimer = null
    const socket = this.socket
    this.socket = null
    if (socket) {
      socket.onopen = socket.onclose = socket.onerror = socket.onmessage = null
      socket.close(1000, 'client stop')
    }
    this.emit('closed', null, null)
  }

  /** 즉시 재연결 시도 (사용자 '다시 연결' 버튼) */
  reconnectNow() {
    if (this.stopped) return
    if (this.retryTimer) clearTimeout(this.retryTimer)
    this.retryTimer = null
    if (!this.socket) this.open()
  }

  private open() {
    let socket: MinimalWebSocket
    try {
      socket = this.opts.createSocket(this.opts.url)
    } catch (error) {
      this.scheduleRetry(error instanceof Error ? error.message : 'WebSocket 생성 실패')
      return
    }
    this.socket = socket
    socket.onopen = () => {
      this.attempt = 0
      this.emit('open', null, null)
    }
    socket.onmessage = (ev) => this.handleMessage(ev.data)
    socket.onerror = () => {
      // 브라우저는 error 직후 close 를 발생시키므로 재연결은 onclose 에서 처리
    }
    socket.onclose = (ev) => {
      if (this.socket !== socket) return
      this.socket = null
      if (this.stopped) return
      this.scheduleRetry(ev.reason || `연결 종료 (code ${ev.code})`)
    }
  }

  private scheduleRetry(reason: string) {
    const delay = Math.min(this.opts.maxDelayMs, this.opts.baseDelayMs * 2 ** this.attempt)
    this.attempt += 1
    this.emit('reconnecting', Date.now() + delay, reason, delay)
    this.retryTimer = setTimeout(() => {
      this.retryTimer = null
      if (!this.stopped) this.open()
    }, delay)
  }

  private handleMessage(data: unknown) {
    try {
      const event = parseStateEvent(data)
      if (event) this.opts.onEvent(event)
    } catch (error) {
      this.opts.onInvalidMessage?.(error instanceof Error ? error.message : String(error))
    }
  }

  private emit(status: SocketStatus, nextRetryAt: number | null, lastError: string | null, retryDelayMs: number | null = null) {
    this.opts.onStatus({ status, attempt: this.attempt, nextRetryAt, retryDelayMs, lastError })
  }
}
