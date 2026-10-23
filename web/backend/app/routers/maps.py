from fastapi import APIRouter, Request
from fastapi.responses import Response

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


@router.get('/api/stores/{store_id}/semantics')
def get_semantics(store_id: str, request: Request):
    site = request.app.state.site
    return site.semantics if store_id == site.store_id else err(404, 'NO_SEMANTICS', 'Semantic Map이 없습니다.')


@router.put('/api/stores/{store_id}/semantics')
def put_semantics(store_id: str):
    return err(501, 'NOT_SUPPORTED', '이 백엔드는 아직 Semantic Map 저장을 지원하지 않습니다(읽기 전용).')
