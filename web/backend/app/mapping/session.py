"""One automatic-mapping session: start the runner script, follow it, turn its map into a
reviewable draft, and keep that state across a backend restart.

Based on the mapping team's `integration/mapping_api.py` (MappingSession), changed so that
  * every path comes from MappingConfig (no module-level environment lookups),
  * a map that was explored to its end is ingested by itself (Stop is only for cutting early),
  * the state is written to <work_dir>/state.json and read again at start-up,
  * progress is reported (stage, goal counters, elapsed time, an estimated percentage),
  * a draft that is left over can be discarded, and a finished draft is archived (never deleted).

The ROS stack itself (Gazebo / robot / SLAM / Nav2) is started by a person in a terminal beforehand;
this class only controls the *session script* (design "A": no stack controller in the backend).
"""
import json
import math
import os
import re
import shutil
import signal
import subprocess
import threading
import time
import uuid
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path

from .ingest import ingest
from .review import MapReviewService, ReviewError

RESULT = 'R16-147_session_result.json'
LOG = 'R16-147_exploration.log'
RAW_YAML = 'R16-147_official_raw_map.yaml'

_SENT = re.compile(r'MAPPING NAV2 GOAL SENT')
_DONE = re.compile(r'Mapping navigation result:\s*(\w+)')

# Stage names the frontend shows (kept short; the text is in the frontend labels)
STAGES = ('idle', 'starting', 'exploring', 'finishing', 'converting', 'ready', 'failed')


class MappingError(Exception):
    def __init__(self, status, code, message, details=None):
        super().__init__(message)
        self.status, self.code, self.message, self.details = status, code, message, details


@dataclass
class MappingConfig:
    store_id: str
    work_dir: str                       # sessions/, drafts/, outputs/, state.json live here
    runner: str                         # tools/run_lidar_mapping_session.py
    ros_python: str = '/usr/bin/python3'
    enabled: bool = False               # off by default: starting needs a running ROS stack
    require_ros_env: bool = True        # ROS_DISTRO must be "jazzy" (turned off by tests)
    env: dict = field(default_factory=dict)          # extra environment (e.g. ROS_DOMAIN_ID)
    ready_nodes: tuple = ('slam_toolbox', 'bt_navigator', 'controller_server')
    ready_check: object = None          # callable() -> None | message; replaces the ros2 probe
    expected_goals: float = 12.0        # goals that bring the estimate to ~63 % (see _estimate)
    early_exit_s: float = 20.0          # runner gone this fast without a result = it never ran

    @property
    def sessions_dir(self):
        return Path(self.work_dir) / 'sessions'

    @property
    def draft_dir(self):
        return Path(self.work_dir) / 'drafts' / 'current'

    @property
    def archive_dir(self):
        return Path(self.work_dir) / 'drafts' / 'archive'

    @property
    def output_root(self):
        return Path(self.work_dir) / 'outputs'

    @property
    def state_path(self):
        return Path(self.work_dir) / 'state.json'


def _now():
    return datetime.now(timezone.utc).isoformat()


def ros2_probe(cfg):
    """None when the stack looks up, else a sentence for the operator."""
    try:
        out = subprocess.run(['ros2', 'node', 'list'], capture_output=True, text=True,
                             timeout=10, env={**os.environ, **cfg.env})
    except FileNotFoundError:
        return 'ros2 명령을 찾지 못했어요. ROS 2 환경(source)이 된 터미널에서 백엔드를 실행해 주세요.'
    except subprocess.TimeoutExpired:
        return 'ROS 노드 목록을 읽는 데 시간이 너무 오래 걸려요.'
    if out.returncode != 0:
        return 'ROS 노드 목록을 읽지 못했어요: %s' % (out.stderr.strip()[:200] or out.returncode)
    nodes = out.stdout
    missing = [n for n in cfg.ready_nodes if n not in nodes]
    if missing:
        return '매핑용 로봇 스택이 아직 안 떠 있어요 (없는 노드: %s). 매핑 스택을 먼저 실행해 주세요.' \
            % ', '.join(missing)
    return None


class MappingSession:
    def __init__(self, cfg):
        self.cfg = cfg
        self.lock = threading.RLock()
        self.process = None               # Popen, or None after a restart of the backend
        self.pid = None
        self._reset()
        self._load()

    # ---- state ----------------------------------------------------------------------
    def _reset(self):
        self.phase = 'idle'               # idle | mapping | stopped
        self.stage = 'idle'
        self.session_dir = None
        self.started_at = None
        self.ended_at = None
        self.stop_requested = False
        self.draft_ready = False
        self.completion = None
        self.message = None
        self.error = None
        self.saved = False
        self.output_dir = None
        self.progress_max = 0

    _FIELDS = ('phase', 'stage', 'started_at', 'ended_at', 'stop_requested', 'draft_ready',
               'completion', 'message', 'error', 'saved', 'output_dir', 'progress_max', 'pid')

    def _save(self):
        data = {k: getattr(self, k) for k in self._FIELDS}
        data['session_dir'] = str(self.session_dir) if self.session_dir else None
        path = self.cfg.state_path
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix('.tmp')
        tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')
        os.replace(tmp, path)

    def _load(self):
        try:
            data = json.loads(self.cfg.state_path.read_text(encoding='utf-8'))
        except (OSError, ValueError):
            return
        if not isinstance(data, dict):
            return
        for k in self._FIELDS:
            if k in data:
                setattr(self, k, data[k])
        sd = data.get('session_dir')
        self.session_dir = Path(sd) if sd else None
        # a draft that was prepared must still be on disk, otherwise forget it
        if self.draft_ready and not self.cfg.draft_dir.is_dir():
            self.draft_ready, self.saved = False, False
            if self.phase == 'stopped':
                self.phase, self.stage = 'idle', 'idle'

    # ---- the runner process -----------------------------------------------------------
    def _running(self):
        if self.process is not None:
            return self.process.poll() is None
        if self.pid:
            try:
                os.kill(self.pid, 0)
            except OSError:
                return False
            return self._is_our_runner(self.pid, os.path.basename(self.cfg.runner))
        return False

    @staticmethod
    def _is_our_runner(pid, name):
        """After a backend restart only a pid is known: make sure it is still a python runner,
        not an unrelated process that got the number."""
        try:
            with open('/proc/%d/cmdline' % pid, 'rb') as f:
                return name.encode() in f.read()
        except OSError:
            return True        # no /proc (not Linux): trust the pid

    def _result(self):
        if self.session_dir is None:
            return {}
        try:
            data = json.loads((self.session_dir / RESULT).read_text(encoding='utf-8'))
            return data if isinstance(data, dict) else {}
        except (OSError, ValueError):
            return {}

    def _log_text(self):
        try:
            return (self.session_dir / LOG).read_text(encoding='utf-8', errors='replace')
        except (OSError, TypeError):
            return ''

    def _counters(self):
        sent = arrived = failed = 0
        for line in self._log_text().splitlines():
            if _SENT.search(line):
                sent += 1
                continue
            m = _DONE.search(line)
            if m:
                if m.group(1).upper() == 'SUCCEEDED':
                    arrived += 1
                else:
                    failed += 1
        return sent, arrived, failed

    def _elapsed(self):
        if not self.started_at:
            return None
        end = self.ended_at or _now()
        try:
            a = datetime.fromisoformat(self.started_at)
            b = datetime.fromisoformat(end)
            return max(0.0, (b - a).total_seconds())
        except ValueError:
            return None

    def _estimate(self, arrived):
        """A made-up-but-monotonic percentage. The explorer does not know how big the room is,
        so this is only a feel for 'how far along': it rises with the goals that were reached,
        never goes back, and stays below 100 until the draft is ready (shown as an estimate)."""
        if self.draft_ready:
            return 100
        est = int(95 * (1 - math.exp(-arrived / max(1.0, self.cfg.expected_goals))))
        self.progress_max = max(self.progress_max, min(est, 99))
        return self.progress_max

    def _tail(self, n=3):
        for name in ('mapping_api_runner.log', LOG):
            try:
                lines = (self.session_dir / name).read_text(encoding='utf-8',
                                                            errors='replace').strip().splitlines()
            except (OSError, TypeError):
                continue
            if lines:
                return ' / '.join(lines[-n:])[:300]
        return ''

    def _refresh(self):
        """Look at the runner; when it has ended, turn its map into the draft."""
        if self.phase != 'mapping':
            return
        if self._running():
            if self.stage == 'starting' and self._counters()[0] > 0:
                self.stage = 'exploring'
                self._save()
            return
        self.ended_at = self.ended_at or _now()
        report = self._result()
        self.completion = report.get('explorer_terminal_status')
        self.phase = 'stopped'
        if report.get('map_draft_ready') is not True:
            self.stage = 'failed'
            reason = self.completion or 'UNKNOWN'
            took = self._elapsed()
            if not report and took is not None and took < self.cfg.early_exit_s:
                self.error = ('매핑 실행기가 바로 끝났어요. 로봇 스택이 떠 있는지 확인해 주세요. (%s)'
                              % (self._tail() or '로그 없음'))
            else:
                self.error = '검토할 수 있는 지도 초안이 만들어지지 않았어요. (종료 상태: %s)' % reason
            self.message = None
            self._save()
            return
        self.stage = 'converting'
        target = self.cfg.draft_dir
        if target.exists():
            self.stage = 'failed'
            self.error = '이전 임시 지도가 남아 있어요. 먼저 보정/저장하거나 버려 주세요.'
            self._save()
            return
        try:
            ingest(self.session_dir / RAW_YAML, target)
        except (OSError, ValueError) as exc:
            self.stage = 'failed'
            self.error = '임시 지도 변환에 실패했어요: %s' % exc
            self._save()
            return
        self.draft_ready, self.error, self.stage = True, None, 'ready'
        if self.completion == 'COVERAGE_LIMITED':
            self.message = '더 이상 안전하게 탐색할 곳이 없어 마쳤어요. 지도를 검토해 주세요.'
        elif self.completion == 'EXPLORER_REPORTED_MAP_COMPLETE':
            self.message = '자동 매핑을 마쳤어요. 지도를 검토해 주세요.'
        elif self.stop_requested:
            self.message = '중지해서 지금까지 만든 지도로 초안을 만들었어요. 지도를 검토해 주세요.'
        else:
            self.message = '자동 탐색이 끝났어요. 지도를 검토해 주세요.'
        self._save()

    # ---- public --------------------------------------------------------------------
    @property
    def active(self):
        """True while the robot is being driven by the mapping run (orders must wait)."""
        with self.lock:
            self._refresh()
            return self.phase == 'mapping'

    def status(self):
        with self.lock:
            self._refresh()
            sent, arrived, failed = (self._counters() if self.phase != 'idle' else (0, 0, 0))
            return {
                'store_id': self.cfg.store_id,
                'phase': self.phase,
                'started_at': self.started_at,
                'last_saved_revision': 1 if self.saved else 0,
                'draft_ready': self.draft_ready,
                'completion_reason': self.completion,
                'message': self.message,
                'error': self.error,
                'stop_requested': self.stop_requested,
                'navigation_authorized': False,
                'enabled': self.cfg.enabled,
                'stage': self.stage,
                'progress': {
                    'percent': self._estimate(arrived) if self.phase != 'idle' else 0,
                    'estimated': not self.draft_ready,
                    'elapsed_s': self._elapsed(),
                    'goals_sent': sent, 'goals_arrived': arrived, 'goals_failed': failed,
                },
            }

    def _check_ready(self):
        cfg = self.cfg
        if not cfg.enabled:
            raise MappingError(503, 'MAPPING_DISABLED',
                               '이 백엔드는 자동 매핑이 꺼져 있어요. (mapping_enabled: true 로 켜고 다시 실행)')
        if cfg.require_ros_env and os.environ.get('ROS_DISTRO') != 'jazzy':
            raise MappingError(503, 'STACK_NOT_READY',
                               'ROS 2 Jazzy 환경(source)이 된 터미널에서 백엔드를 실행해 주세요.')
        if not Path(cfg.runner).is_file() or not Path(cfg.ros_python).is_file():
            raise MappingError(503, 'RUNNER_NOT_FOUND', '자동 매핑 실행기 또는 ROS Python을 찾지 못했어요.')
        why = cfg.ready_check() if cfg.ready_check else ros2_probe(cfg)
        if why:
            raise MappingError(503, 'STACK_NOT_READY', why)

    def start(self, busy=None):
        """busy: callable() -> bool, True while customer orders are active."""
        with self.lock:
            self._refresh()
            if self.phase == 'mapping' or self._running():
                raise MappingError(409, 'ALREADY_MAPPING', '이미 매핑 중이에요.')
            if self.cfg.draft_dir.exists() or self.draft_ready:
                raise MappingError(409, 'DRAFT_PENDING',
                                   '아직 반영하지 않은 임시 지도가 있어요. 보정해서 저장하거나 버린 뒤에 다시 시작해 주세요.')
            if busy and busy():
                raise MappingError(409, 'ORDERS_ACTIVE',
                                   '주문이 진행 중이거나 대기 중이라 매핑을 시작할 수 없어요. 주문이 끝난 뒤에 시도해 주세요.')
            self._check_ready()
            session_dir = self.cfg.sessions_dir / ('session_' + uuid.uuid4().hex)
            session_dir.mkdir(parents=True, exist_ok=False)
            cmd = [self.cfg.ros_python, str(self.cfg.runner), '--run', '--log-dir', str(session_dir)]
            try:
                with (session_dir / 'mapping_api_runner.log').open('wb') as out:
                    proc = subprocess.Popen(cmd, stdout=out, stderr=subprocess.STDOUT,
                                            start_new_session=True, close_fds=True,
                                            env={**os.environ, **self.cfg.env})
            except OSError as exc:
                raise MappingError(503, 'RUNNER_START_FAILED', '매핑 실행기를 시작하지 못했어요: %s' % exc)
            self._reset()
            self.process, self.pid = proc, proc.pid
            self.session_dir, self.phase, self.stage = session_dir, 'mapping', 'starting'
            self.started_at = _now()
            self.message = '자동 탐색을 시작했어요.'
            self._save()
            return self.status()

    def stop(self):
        with self.lock:
            self._refresh()
            if self.phase != 'mapping':
                raise MappingError(409, 'NOT_MAPPING', '중지할 매핑이 없어요.')
            self.stop_requested = True
            if self._running():
                pid = self.process.pid if self.process is not None else self.pid
                try:
                    if os.getpgid(pid) != pid:
                        raise MappingError(409, 'BAD_PROCESS_GROUP', '매핑 프로세스 그룹이 일치하지 않아요.')
                    os.killpg(pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
                self.stage = 'finishing'
                self.message = '매핑을 마치고 임시 지도를 저장하는 중이에요.'
            self._save()
            self._refresh()
            return self.status()

    # ---- the draft ----------------------------------------------------------------
    def review_service(self):
        with self.lock:
            self._refresh()
            if not self.draft_ready:
                raise MappingError(409, 'NO_DRAFT', '보정할 수 있는 임시 지도가 아직 없어요.')
            try:
                return MapReviewService(self.cfg.draft_dir, self.cfg.output_root, self.cfg.store_id)
            except ReviewError as e:
                raise MappingError(e.status, 'DRAFT_INVALID', str(e))

    def save_review(self, body):
        """Finalize the person's corrections -> {..., output_dir}. Marks the draft saved."""
        service = self.review_service()
        try:
            result = service.finalize(body)
        except ReviewError as e:
            raise MappingError(e.status, 'REVIEW_REJECTED', str(e))
        with self.lock:
            self.saved = result.get('saved') is True
            self.output_dir = result.get('output_dir')
            self._save()
        return result

    def _archive_draft(self):
        draft = self.cfg.draft_dir
        if not draft.exists():
            return None
        self.cfg.archive_dir.mkdir(parents=True, exist_ok=True)
        dest = self.cfg.archive_dir / time.strftime('%Y%m%d-%H%M%S')
        n = 1
        while dest.exists():
            n += 1
            dest = self.cfg.archive_dir / ('%s-%d' % (time.strftime('%Y%m%d-%H%M%S'), n))
        shutil.move(str(draft), str(dest))
        return str(dest)

    def discard(self):
        """Drop an unfinished draft (kept in drafts/archive) so a new mapping can start."""
        with self.lock:
            self._refresh()
            if self.phase == 'mapping':
                raise MappingError(409, 'ALREADY_MAPPING', '매핑 중에는 버릴 수 없어요. 먼저 중지해 주세요.')
            archived = self._archive_draft()
            self._reset()
            self.process = self.pid = None
            self._save()
            return {'archived_to': archived, **self.status()}

    def finish_adoption(self):
        """The saved map is now the store's map: archive the draft and go back to idle."""
        with self.lock:
            archived = self._archive_draft()
            self._reset()
            self.process = self.pid = None
            self._save()
            return archived
