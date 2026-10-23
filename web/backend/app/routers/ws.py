import asyncio
import json

from fastapi import APIRouter, WebSocket

router = APIRouter()


@router.websocket('/ws/state')
async def ws_state(ws: WebSocket):
    await ws.accept()
    fleet, hub = ws.app.state.fleet, ws.app.state.hub
    q = hub.subscribe()

    async def sender():
        for ev in fleet.snapshot():
            await ws.send_text(json.dumps(ev))
        while True:
            await ws.send_text(json.dumps(await q.get()))

    async def receiver():
        while True:
            await ws.receive_text()          # only to notice that the client left

    tasks = [asyncio.create_task(sender()), asyncio.create_task(receiver())]
    try:
        await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
    finally:
        for t in tasks:
            t.cancel()
        hub.unsubscribe(q)
