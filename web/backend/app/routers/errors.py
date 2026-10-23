from fastapi.responses import JSONResponse


def err(status, code, message, details=None):
    """Error body: `message` is for people (the frontend shows it), `code` is for programs.
    `details` (optional) carries structured facts, e.g. which points were rejected."""
    body = {'code': code, 'message': message}
    if details is not None:
        body['details'] = details
    return JSONResponse(body, status_code=status)
