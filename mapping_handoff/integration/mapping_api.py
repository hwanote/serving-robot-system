"""Team integration API for Robot 01 mapping and map review.

Import `router` into the team's FastAPI app, replacing its 501 mapping
routes; or run `mapping_api:app` as a local standalone service.

One API process, one Robot 01 mapping session at a time.
ROS execution is disabled by default.
"""
import json
import os
import signal
import subprocess
import sys
import threading
import uuid
from datetime import datetime, timezone
from pathlib import Path

from fastapi import APIRouter, FastAPI, HTTPException, Request
from fastapi.responses import Response

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "integration"))
sys.path.insert(0, str(ROOT / "review/backend"))

from ingest_map import ingest
from map_review import MapReviewService, ReviewError

STORE = os.environ.get("MAPPING_STORE_ID", "demo_store")
RUNNER = Path(os.environ.get(
    "MAPPING_RUNNER",
    str(ROOT / "ros_ws/src/serving_robot_mapping/"
        "tools/run_lidar_mapping_session.py"),
))
ROS_PYTHON = os.environ.get("MAPPING_ROS_PYTHON", "/usr/bin/python3")
DATA = Path(os.environ.get(
    "MAPPING_DATA_DIR",
    str(Path.home() / ".local/share/serving-robot/team-mapping"),
))


class MappingSession:
    def __init__(self):
        self.lock = threading.RLock()
        self.process = None
        self.session_dir = None
        self.phase = "idle"
        self.started_at = None
        self.stop_requested = False
        self.draft_ready = False
        self.completion = None
        self.message = None
        self.error = None
        self.saved = False

    def _result(self):
        if self.session_dir is None:
            return {}
        path = self.session_dir / "R16-147_session_result.json"
        if not path.is_file():
            return {}
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
            return data if isinstance(data, dict) else {}
        except (OSError, ValueError):
            return {}

    def _refresh(self):
        if self.process is None or self.phase != "mapping":
            return

        if self.process.poll() is None:
            return

        report = self._result()
        self.completion = report.get("explorer_terminal_status")

        if self.completion == "COVERAGE_LIMITED":
            self.message = (
                "더 이상 안전하게 탐색할 수 있는 영역을 찾지 "
                "못했습니다. 지도를 검토해 주세요."
            )
        elif self.completion == "EXPLORER_REPORTED_MAP_COMPLETE":
            self.message = "자동 매핑을 완료했습니다."
        else:
            self.message = (
                "자동 탐색이 종료되었습니다. "
                "지도 저장 상태를 확인해 주세요."
            )

        # Let the user press Stop after automatic exploration finishes.
        if not self.stop_requested:
            return

        self.phase = "stopped"
        if report.get("map_draft_ready") is not True:
            self.error = (
                "검토 가능한 지도 초안이 생성되지 않았습니다. "
                f"종료 상태: {self.completion or 'UNKNOWN'}"
            )
            return

        source_yaml = (
            self.session_dir / "R16-147_official_raw_map.yaml"
        )
        target = ROOT / "mapping_drafts" / STORE / "current"

        # Replace only an earlier prepared draft; preserve its source
        # in the unique, immutable session folder.
        if target.exists():
            self.error = (
                "이전 임시 지도가 남아 있습니다. "
                "기존 보정 지도를 확인한 후 다시 진행해 주세요."
            )
            return

        try:
            ingest(source_yaml, target)
        except (OSError, ValueError) as exc:
            self.error = f"임시 지도 변환 실패: {exc}"
            return

        os.environ["SERVING_MAPPING_DRAFT_DIR"] = str(target)
        self.draft_ready = True
        self.error = None

    def status(self):
        with self.lock:
            self._refresh()
            return {
                "store_id": STORE,
                "phase": self.phase,
                "started_at": self.started_at,
                "last_saved_revision": 1 if self.saved else 0,
                "draft_ready": self.draft_ready,
                "completion_reason": self.completion,
                "message": self.message,
                "error": self.error,
                "stop_requested": self.stop_requested,
                "navigation_authorized": False,
            }

    def start(self):
        with self.lock:
            self._refresh()
            if self.phase == "mapping":
                raise HTTPException(409, "이미 매핑 중입니다.")
            if self.process is not None and self.process.poll() is None:
                raise HTTPException(409, "이전 매핑 프로세스가 실행 중입니다.")

            if os.environ.get("MAPPING_ENABLE_ROS") != "1":
                raise HTTPException(
                    503, "ROS 매핑 실행이 비활성화되어 있습니다."
                )
            if os.environ.get("ROS_DISTRO") != "jazzy":
                raise HTTPException(
                    503, "ROS 2 Jazzy 환경을 먼저 설정해야 합니다."
                )
            if not RUNNER.is_file() or not Path(ROS_PYTHON).is_file():
                raise HTTPException(
                    503, "자동 매핑 실행기 또는 ROS Python을 찾지 못했습니다."
                )

            if (ROOT / "mapping_drafts" / STORE / "current").exists():
                raise HTTPException(
                    409, "기존 임시 지도 보정/저장을 먼저 완료해야 합니다."
                )

            session_dir = DATA / ("session_" + uuid.uuid4().hex)
            session_dir.mkdir(parents=True, exist_ok=False)
            log = session_dir / "mapping_api_runner.log"

            command = [
                ROS_PYTHON, str(RUNNER),
                "--run", "--log-dir", str(session_dir),
            ]
            try:
                with log.open("wb") as output:
                    proc = subprocess.Popen(
                        command,
                        stdout=output,
                        stderr=subprocess.STDOUT,
                        start_new_session=True,
                        close_fds=True,
                    )
            except OSError as exc:
                raise HTTPException(
                    503, f"매핑 실행기를 시작하지 못했습니다: {exc}"
                ) from exc

            self.process = proc
            self.session_dir = session_dir
            self.phase = "mapping"
            self.started_at = datetime.now(
                timezone.utc
            ).isoformat()
            self.stop_requested = False
            self.draft_ready = False
            self.completion = None
            self.message = "Robot 01 자동 탐색을 시작했습니다."
            self.error = None
            self.saved = False
            return self.status()

    def stop(self):
        with self.lock:
            self._refresh()
            if self.phase != "mapping" or self.process is None:
                raise HTTPException(409, "중지할 매핑 세션이 없습니다.")

            self.stop_requested = True
            if self.process.poll() is None:
                # Signal only the runner process group created by us.
                if os.getpgid(self.process.pid) != self.process.pid:
                    raise HTTPException(
                        409, "매핑 프로세스 그룹이 일치하지 않습니다."
                    )
                os.killpg(self.process.pid, signal.SIGINT)
                self.message = "매핑 종료 및 임시 지도 저장을 확인 중입니다."

            self._refresh()
            return self.status()


session = MappingSession()
router = APIRouter()


@router.get("/api/mapping/status")
def mapping_status():
    return session.status()


@router.post("/api/mapping/{command}")
async def mapping_command(command: str, request: Request):
    if command == "start":
        return session.start()
    if command == "stop":
        return session.stop()
    if command != "save":
        raise HTTPException(404, "지원하지 않는 매핑 명령입니다.")

    if not session.status()["draft_ready"]:
        raise HTTPException(409, "보정 가능한 임시 지도가 없습니다.")

    try:
        body = await request.json()
    except ValueError:
        raise HTTPException(
            422, "Save Map에는 보정 내용과 최종 확인이 필요합니다."
        )

    if not isinstance(body, dict) or body.get("confirmed") is not True:
        raise HTTPException(
            422, "사용자 지도 보정 최종 확인이 필요합니다."
        )

    try:
        result = MapReviewService(ROOT, STORE).finalize(body)
    except ReviewError as exc:
        raise HTTPException(exc.status, str(exc))

    with session.lock:
        session.saved = result.get("saved") is True
    return {**session.status(), "review_result": result}


def review_service(store_id):
    if store_id != STORE:
        raise HTTPException(404, "매장을 찾지 못했습니다.")
    if not session.status()["draft_ready"]:
        raise HTTPException(409, "아직 보정 가능한 지도가 없습니다.")
    return MapReviewService(ROOT, STORE)


@router.get("/api/stores/{store_id}/map-review")
def review_info(store_id: str):
    try:
        return review_service(store_id).describe()
    except ReviewError as exc:
        raise HTTPException(exc.status, str(exc))


@router.get("/api/stores/{store_id}/map-review.png")
def review_image(store_id: str):
    try:
        data = review_service(store_id).preview_png()
    except ReviewError as exc:
        raise HTTPException(exc.status, str(exc))
    return Response(data, media_type="image/png")


@router.post("/api/stores/{store_id}/map-review/finalize")
async def review_finalize(store_id: str, request: Request):
    service = review_service(store_id)
    try:
        body = await request.json()
        result = service.finalize(body)
    except ReviewError as exc:
        raise HTTPException(exc.status, str(exc))
    except ValueError:
        raise HTTPException(400, "잘못된 수정 요청입니다.")
    return result


app = FastAPI(title="Robot 01 Mapping Delivery")
app.include_router(router)
