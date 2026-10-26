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


# R16-223 MAP REVIEW ENDPOINTS
@router.get('/api/stores/{store_id}/map-review')
def review_info(store_id: str, request: Request):
    from ..map_review import MapReviewService, ReviewError
    site = request.app.state.site
    if store_id != site.store_id:
        return err(404, 'NO_MAP', '지도를 찾지 못했습니다.')
    try:
        return MapReviewService(site.repo, store_id).describe()
    except ReviewError as exc:
        return err(exc.status, 'MAP_REVIEW_ERROR', str(exc))


@router.get('/api/stores/{store_id}/map-review.png')
def review_preview(store_id: str, request: Request):
    from ..map_review import MapReviewService, ReviewError
    site = request.app.state.site
    if store_id != site.store_id:
        return err(404, 'NO_MAP', '지도를 찾지 못했습니다.')
    try:
        data = MapReviewService(site.repo, store_id).preview_png()
        return Response(data, media_type='image/png')
    except ReviewError as exc:
        return err(exc.status, 'MAP_REVIEW_ERROR', str(exc))


@router.post('/api/stores/{store_id}/map-review/finalize')
async def finalize_review(store_id: str, request: Request):
    from ..map_review import MapReviewService, ReviewError
    site = request.app.state.site
    if store_id != site.store_id:
        return err(404, 'NO_MAP', '지도를 찾지 못했습니다.')
    try:
        body = await request.json()
        service = MapReviewService(site.repo, store_id)
        return await run_in_threadpool(service.finalize, body)
    except ValueError:
        return err(400, 'INVALID_REQUEST', '요청을 읽을 수 없습니다.')
    except ReviewError as exc:
        return err(exc.status, 'MAP_REVIEW_ERROR', str(exc))
