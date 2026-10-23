from fastapi import APIRouter, Request

from .errors import err

router = APIRouter()


@router.get('/api/robots')
def list_robots(request: Request):
    return request.app.state.fleet.list_robots()


@router.post('/api/robots/{robot_id}/pause')
def pause_robot(robot_id: str, request: Request):
    r = request.app.state.fleet.set_robot_paused(robot_id, True)
    return r or err(404, 'ROBOT_NOT_FOUND', '로봇을 찾을 수 없습니다: %s' % robot_id)


@router.post('/api/robots/{robot_id}/resume')
def resume_robot(robot_id: str, request: Request):
    r = request.app.state.fleet.set_robot_paused(robot_id, False)
    return r or err(404, 'ROBOT_NOT_FOUND', '로봇을 찾을 수 없습니다: %s' % robot_id)


@router.post('/api/robots/{robot_id}/recover')
async def recover_robot(robot_id: str, request: Request):
    """After a localization loss: body {"action": "resume" | "home"}."""
    try:
        body = await request.json()
    except ValueError:
        body = None
    action = body.get('action') if isinstance(body, dict) else None
    if action not in ('resume', 'home'):
        return err(400, 'BAD_ACTION', "action 은 'resume' 또는 'home' 이어야 합니다.")
    summary, problem = request.app.state.fleet.robot_command(robot_id, action)
    if problem == 'not found':
        return err(404, 'ROBOT_NOT_FOUND', '로봇을 찾을 수 없습니다: %s' % robot_id)
    if problem == 'offline':
        return err(409, 'ROBOT_OFFLINE', '로봇과 연결이 끊겨 있습니다.')
    if problem:
        return err(409, 'NOT_RECOVERABLE', '위치가 복구된 직후에만 선택할 수 있습니다.')
    return summary


@router.post('/api/dispatch/pause')
def pause_dispatch(request: Request):
    request.app.state.fleet.set_dispatch_paused(True)
    return {'dispatch_paused': True}


@router.post('/api/dispatch/resume')
def resume_dispatch(request: Request):
    request.app.state.fleet.set_dispatch_paused(False)
    return {'dispatch_paused': False}
