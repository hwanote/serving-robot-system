from fastapi import APIRouter, Request
from fastapi.concurrency import run_in_threadpool
from fastapi.responses import Response

from ..semantics import SemanticError
from .errors import err

router = APIRouter()


@router.get('/api/mapping/status')
def mapping_status(request: Request):
    site = request.app.state.site
    return {'store_id': site.store_id, 'phase': 'idle', 'started_at': None,
            'last_saved_revision': site.revision}


@router.post('/api/mapping/{command}')
def mapping_command(command: str):
    return err(501, 'NOT_SUPPORTED', '이 백엔드는 아직 매핑을 지원하지 않습니다.')


@router.get('/api/stores/{store_id}/map')
def get_map(store_id: str, request: Request):
    site = request.app.state.site
    return site.raw_map() if store_id == site.store_id else err(404, 'NO_MAP', '저장된 지도가 없습니다.')


@router.get('/api/stores/{store_id}/map.png')
def get_png(store_id: str, request: Request):
    site = request.app.state.site
    if store_id != site.store_id:
        return err(404, 'NO_MAP', '저장된 지도가 없습니다.')
    return Response(site.png, media_type='image/png')


def _no_semantics():
    return err(404, 'NO_SEMANTICS', 'Semantic Map이 없습니다.')


def _semantic_err(e):
    return err(e.status, e.code, e.message, e.details)


@router.get('/api/stores/{store_id}/semantics')
def get_semantics(store_id: str, request: Request):
    """The saved draft if there is one, else what config/common describes."""
    site = request.app.state.site
    if store_id != site.store_id:
        return _no_semantics()
    return request.app.state.semantics.load_draft() or site.semantics


@router.put('/api/stores/{store_id}/semantics')
async def put_semantics(store_id: str, request: Request, apply: bool = False):
    """Save the draft. With ?apply=true also generate the nodes from it (see semantics.py)."""
    state = request.app.state
    if store_id != state.site.store_id:
        return _no_semantics()
    try:
        body = await request.json()
    except ValueError:
        return err(400, 'INVALID_JSON', 'JSON 형식이 아닙니다.')
    try:
        saved = await run_in_threadpool(state.semantics.save_draft, body)
        if not apply:
            return saved
        result = await run_in_threadpool(state.semantics.apply, saved)
    except SemanticError as e:
        return _semantic_err(e)
    return {**saved, 'apply_result': result}


@router.post('/api/stores/{store_id}/semantics/apply')
async def apply_semantics(store_id: str, request: Request):
    """Generate the nodes from the saved draft."""
    state = request.app.state
    if store_id != state.site.store_id:
        return _no_semantics()
    try:
        return await run_in_threadpool(state.semantics.apply)
    except SemanticError as e:
        return _semantic_err(e)


@router.get('/api/stores/{store_id}/semantics/status')
def semantics_status(store_id: str, request: Request):
    state = request.app.state
    if store_id != state.site.store_id:
        return _no_semantics()
    return state.semantics.status()
