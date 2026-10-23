import type { MinimalWebSocket } from '../api/stateSocket'

/** 테스트용 가짜 WebSocket */
export class FakeSocket implements MinimalWebSocket {
  static instances: FakeSocket[] = []
  onopen: ((ev: Event) => void) | null = null
  onclose: ((ev: CloseEvent) => void) | null = null
  onerror: ((ev: Event) => void) | null = null
  onmessage: ((ev: MessageEvent) => void) | null = null
  closed = false
  readonly url: string

  constructor(url: string) {
    this.url = url
    FakeSocket.instances.push(this)
  }
  open() {
    this.onopen?.(new Event('open'))
  }
  message(data: unknown) {
    this.onmessage?.(new MessageEvent('message', { data: typeof data === 'string' ? data : JSON.stringify(data) }))
  }
  serverClose(code = 1006, reason = '') {
    this.onclose?.({ code, reason } as CloseEvent)
  }
  close() {
    this.closed = true
  }
}
