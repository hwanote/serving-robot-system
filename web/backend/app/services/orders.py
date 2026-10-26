FORBIDDEN_KEYS = ('x', 'y', 'pose', 'robot_id', 'position', 'goal')


class OrderError(Exception):
    def __init__(self, status, code, message):
        super().__init__(message)
        self.status, self.code, self.message = status, code, message


class OrderService:
    def __init__(self, fleet, site):
        self.fleet, self.site = fleet, site

    def create(self, body):
        """Validate an order body and queue the task. -> (task dict, created)"""
        if not isinstance(body, dict):
            raise OrderError(400, 'BAD_BODY', '요청 본문이 올바르지 않습니다.')
        if self.fleet.mapping_active():
            raise OrderError(409, 'MAPPING_BUSY', '지도를 만드는 중이라 주문을 받을 수 없어요. 매핑이 끝난 뒤에 시도해 주세요.')
        if body.get('type') != 'SERVE':
            raise OrderError(400, 'UNSUPPORTED_TYPE', '지원하지 않는 Task 타입입니다: %s' % body.get('type'))
        bad = [k for k in FORBIDDEN_KEYS if k in body]
        if bad:
            raise OrderError(400, 'FORBIDDEN_FIELD', '좌표나 로봇을 직접 지정할 수 없습니다: %s' % ', '.join(bad))
        dest = body.get('destination_id')
        if not isinstance(dest, str):
            raise OrderError(400, 'MISSING_DESTINATION', 'destination_id 가 필요합니다.')
        if dest not in self.site.tables:
            raise OrderError(422, 'INVALID_DESTINATION', '유효하지 않은 목적지입니다: %s' % dest)
        crid = body.get('client_request_id')
        if crid is not None and (not isinstance(crid, str) or not crid or len(crid) > 128):
            raise OrderError(400, 'BAD_REQUEST_ID', 'client_request_id 는 128자 이하의 문자열이어야 합니다.')
        task, created = self.fleet.create_task(dest, crid)
        if not created and task['destination_id'] != dest:
            raise OrderError(409, 'REQUEST_ID_CONFLICT', '같은 client_request_id 로 다른 주문이 이미 있습니다: %s'
                             % task['task_id'])
        return task, created

    def cancel(self, task_id):
        t, problem = self.fleet.cancel_task(task_id)
        if problem == 'not found':
            raise OrderError(404, 'TASK_NOT_FOUND', 'Task를 찾을 수 없습니다: %s' % task_id)
        if problem == 'finished':
            raise OrderError(409, 'TASK_FINISHED', '이미 종료된 Task입니다. (%s)' % t['status'])
        return t

    def list(self):
        return self.fleet.list_tasks()
