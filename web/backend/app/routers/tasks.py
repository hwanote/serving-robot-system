from fastapi import APIRouter, Request
from fastapi.responses import JSONResponse

from ..services.orders import OrderError
from .errors import err

router = APIRouter()


@router.get('/api/tasks')
def list_tasks(request: Request):
    return request.app.state.orders.list()


@router.post('/api/tasks')
async def create_task(request: Request):
    try:
        body = await request.json()
    except ValueError:
        body = None
    try:
        task, created = request.app.state.orders.create(body)
    except OrderError as e:
        return err(e.status, e.code, e.message)
    return JSONResponse(task, status_code=201 if created else 200)


@router.post('/api/tasks/{task_id}/cancel')
def cancel_task(task_id: str, request: Request):
    try:
        return request.app.state.orders.cancel(task_id)
    except OrderError as e:
        return err(e.status, e.code, e.message)
