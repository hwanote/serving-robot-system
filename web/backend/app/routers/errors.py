from fastapi.responses import JSONResponse


def err(status, code, message):
    """Error body: `message` is for people (the frontend shows it), `code` is for programs."""
    return JSONResponse({'code': code, 'message': message}, status_code=status)
