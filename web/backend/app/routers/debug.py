from fastapi import APIRouter, Request

router = APIRouter()


@router.get('/api/debug')
def debug(request: Request):
    return request.app.state.fleet.debug()


@router.get('/api/snapshot')
def snapshot(request: Request):
    """REST snapshot for resync after a WebSocket reconnect."""
    return request.app.state.fleet.state_snapshot()


@router.get('/api/metrics')
def metrics(request: Request):
    return request.app.state.fleet.metrics()
