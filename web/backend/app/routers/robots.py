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


def _refusal(robot_id, problem, texts):
    if problem == 'not found':
        return err(404, 'ROBOT_NOT_FOUND', '로봇을 찾을 수 없습니다: %s' % robot_id)
    code, status, message = texts[problem]
    return err(status, code, message)


_OFFLINE = ('ROBOT_OFFLINE', 409, '로봇과 연결이 끊겨 있습니다.')


@router.post('/api/robots/{robot_id}/reset')
async def reset_robot(robot_id: str, request: Request):
    """Emergency reset: the robot was put on a node by hand. Body {"node": "standby_01"}
    (optional, default the robot's home). A running order is canceled."""
    try:
        body = await request.json()
    except ValueError:
        body = None
    node = body.get('node') if isinstance(body, dict) else None
    if node is not None and not isinstance(node, str):
        return err(400, 'BAD_NODE', 'node 는 문자열이어야 합니다.')
    summary, problem = request.app.state.fleet.reset_robot(robot_id, node)
    if problem:
        return _refusal(robot_id, problem, {
            'offline': _OFFLINE,
            'bad node': ('BAD_NODE', 400, '지도에 없는 위치예요: %s' % node)})
    return summary


@router.post('/api/robots/{robot_id}/home')
def home_robot(robot_id: str, request: Request):
    """Send the robot home. Only when it knows where it is (after a reset the position has to
    be OK again) and has nothing to do; after a localization loss it is the robot's "home" choice."""
    summary, problem = request.app.state.fleet.go_home(robot_id)
    if problem:
        return _refusal(robot_id, problem, {
            'offline': _OFFLINE,
            'not localized': ('NOT_LOCALIZED', 409,
                              '로봇이 자기 위치를 아직 모릅니다. 먼저 리셋해서 위치를 잡아 주세요.'),
            'busy': ('ROBOT_BUSY', 409, '로봇이 임무 중입니다. 끝난 뒤에 눌러 주세요.'),
            'at home': ('ALREADY_HOME', 409, '이미 대기 위치에 있습니다.')})
    return summary


_BUSY_MAP = ('ROBOT_BUSY', 409, '로봇이 쉬고 있을 때만 지도를 다시 읽을 수 있어요.')


@router.post('/api/robots/reload-map')
def reload_map_all(request: Request):
    """Every robot that is idle reads config/common again. -> {robot_id: sent|current|busy|offline}"""
    return {'robots': request.app.state.fleet.reload_map_all()}


@router.post('/api/robots/{robot_id}/reload-map')
def reload_map(robot_id: str, request: Request):
    """The robot reads config/common again (the robot nodes read it only at start otherwise).
    Only while it is idle. The result shows up as `map_current` in its state."""
    summary, problem = request.app.state.fleet.reload_map(robot_id)
    if problem:
        return _refusal(robot_id, problem, {'offline': _OFFLINE, 'busy': _BUSY_MAP})
    return summary


@router.post('/api/dispatch/pause')
def pause_dispatch(request: Request):
    request.app.state.fleet.set_dispatch_paused(True)
    return {'dispatch_paused': True}


@router.post('/api/dispatch/resume')
def resume_dispatch(request: Request):
    request.app.state.fleet.set_dispatch_paused(False)
    return {'dispatch_paused': False}
