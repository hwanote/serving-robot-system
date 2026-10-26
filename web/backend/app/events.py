"""Fan-out of events to the WebSocket clients (thread-safe entry point)."""
import asyncio


class EventHub:
    def __init__(self):
        self.loop = None
        self.clients = set()

    def emit(self, event):
        """Called from any thread."""
        if self.loop is not None:
            self.loop.call_soon_threadsafe(self.push, event)

    def push(self, event):
        for q in list(self.clients):
            if q.full():
                try:
                    q.get_nowait()
                except asyncio.QueueEmpty:
                    pass
            q.put_nowait(event)

    def subscribe(self):
        q = asyncio.Queue(maxsize=400)
        self.clients.add(q)
        return q

    def unsubscribe(self, q):
        self.clients.discard(q)
