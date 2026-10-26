from fastapi import APIRouter, Request
from fastapi.concurrency import run_in_threadpool
from fastapi.responses import Response

from ..mapping.session import MappingError
from ..semantics import SemanticError, validate
from .errors import err

router = APIRouter()


def _mapping_err(e):
    return err(e.status, e.code, e.message, e.details)


def _mapping_status(state):
    st = state.mapping.status()
    st['saved'] = state.mapping.saved
    st['last_saved_revision'] = state.site.revision      # revision of the map the store has now
    return st


@router.get('/api/mapping/status')
def mapping_status(request: Request):
    return _mapping_status(request.app.state)


@router.post('/api/mapping/start')
async def mapping_start(request: Request):
    state = request.app.state
    try:
        await run_in_threadpool(state.mapping.start, state.fleet.is_busy)
    except MappingError as e:
        return _mapping_err(e)
    return _mapping_status(state)


@router.post('/api/mapping/stop')
async def mapping_stop(request: Request):
    state = request.app.state
    try:
        await run_in_threadpool(state.mapping.stop)
    except MappingError as e:
        return _mapping_err(e)
    return _mapping_status(state)


@router.post('/api/mapping/discard')
async def mapping_discard(request: Request):
    state = request.app.state
    try:
        await run_in_threadpool(state.mapping.discard)
    except MappingError as e:
        return _mapping_err(e)
    return _mapping_status(state)


async def _save_and_adopt(state, body):
    """Save Map: finalize the corrections, then make the result the store's map.
    If only the adoption is refused (orders running) the saved result is kept and
    POST /api/mapping/adopt can be pressed again later."""
    try:
        review = await run_in_threadpool(state.mapping.save_review, body)
    except MappingError as e:
        return _mapping_err(e)
    out = {'review_result': review, 'adopted': False, 'adopt_error': None}
    try:
        out.update(await run_in_threadpool(state.adopter.adopt))
    except SemanticError as e:
        out['adopt_error'] = {'code': e.code, 'message': e.message}
    return {**_mapping_status(state), **out}


@router.post('/api/mapping/adopt')
async def mapping_adopt(request: Request):
    state = request.app.state
    try:
        result = await run_in_threadpool(state.adopter.adopt)
    except SemanticError as e:
        return _semantic_err(e)
    return {**_mapping_status(state), **result}


@router.post('/api/mapping/save')
async def mapping_save(request: Request):
    try:
        body = await request.json()
    except ValueError:
        return err(422, 'INVALID_JSON', 'Save Map에는 보정 내용과 최종 확인이 필요합니다.')
    return await _save_and_adopt(request.app.state, body)


def _review(store_id, state):
    if store_id != state.site.store_id:
        raise MappingError(404, 'NO_MAP', '매장을 찾지 못했어요.')
    return state.mapping.review_service()


@router.get('/api/stores/{store_id}/map-review')
async def review_info(store_id: str, request: Request):
    try:
        service = _review(store_id, request.app.state)
        return await run_in_threadpool(service.describe)
    except MappingError as e:
        return _mapping_err(e)


@router.get('/api/stores/{store_id}/map-review.png')
async def review_image(store_id: str, request: Request):
    try:
        service = _review(store_id, request.app.state)
        data = await run_in_threadpool(service.preview_png)
    except MappingError as e:
        return _mapping_err(e)
    return Response(data, media_type='image/png')


@router.post('/api/stores/{store_id}/map-review/finalize')
async def review_finalize(store_id: str, request: Request):
    """Same as POST /api/mapping/save (kept for the mapping team's panel)."""
    if store_id != request.app.state.site.store_id:
        return err(404, 'NO_MAP', '매장을 찾지 못했어요.')
    try:
        body = await request.json()
    except ValueError:
        return err(400, 'INVALID_JSON', 'JSON 형식이 아닙니다.')
    return await _save_and_adopt(request.app.state, body)


@router.get('/api/stores/{store_id}/map/backups')
def map_backups(store_id: str, request: Request):
    if store_id != request.app.state.site.store_id:
        return err(404, 'NO_MAP', '저장된 지도가 없습니다.')
    return request.app.state.adopter.list_backups()


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


@router.get('/api/stores/{store_id}/semantics/graph')
async def semantics_graph(store_id: str, request: Request):
    """The corridors, nodes and stop poses the node generator makes from the saved draft.
    Writes nothing, and answers (ok: false + the reasons) even when apply would reject the draft."""
    state = request.app.state
    if store_id != state.site.store_id:
        return _no_semantics()
    try:
        return await run_in_threadpool(state.semantics.graph)
    except SemanticError as e:
        return _semantic_err(e)


@router.post('/api/stores/{store_id}/semantics/graph')
async def semantics_graph_of(store_id: str, request: Request):
    """Same as GET .../graph, but for the Semantic Map in the body (an edit that is not saved yet),
    so a moved stop can be checked the moment it is dropped. Writes nothing."""
    state = request.app.state
    if store_id != state.site.store_id:
        return _no_semantics()
    try:
        body = await request.json()
    except ValueError:
        return err(400, 'INVALID_JSON', 'JSON 형식이 아닙니다.')
    try:
        validate(body, state.site.store_id, state.site.revision)
        return await run_in_threadpool(state.semantics.graph, body)
    except SemanticError as e:
        return _semantic_err(e)


# ---- put the applied map into config/common (what the robots read), with dated backups
@router.get('/api/stores/{store_id}/semantics/live')
def live_status(store_id: str, request: Request):
    if store_id != request.app.state.site.store_id:
        return _no_semantics()
    return request.app.state.live.status()


@router.post('/api/stores/{store_id}/semantics/promote')
async def promote(store_id: str, request: Request):
    if store_id != request.app.state.site.store_id:
        return _no_semantics()
    try:
        return await run_in_threadpool(request.app.state.live.promote)
    except SemanticError as e:
        return _semantic_err(e)


@router.get('/api/stores/{store_id}/semantics/backups')
def list_backups(store_id: str, request: Request):
    if store_id != request.app.state.site.store_id:
        return _no_semantics()
    return request.app.state.live.list_backups()


@router.post('/api/stores/{store_id}/semantics/backups/{backup_id}/restore')
async def restore_backup(store_id: str, backup_id: str, request: Request):
    if store_id != request.app.state.site.store_id:
        return _no_semantics()
    try:
        return await run_in_threadpool(request.app.state.live.restore, backup_id)
    except SemanticError as e:
        return _semantic_err(e)
