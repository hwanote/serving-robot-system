"""Read Gazebo *named world poses* using the existing ``gz topic`` executable.

Gazebo Harmonic Pose_V -> TFMessage bridge can yield empty child_frame_id;
indexless TFMessage transforms must NOT be treated as named world model poses.
No gz Python bindings or modifications to the robot stack are needed.
"""
import math
import re
import subprocess
import threading
import time

_NAME = re.compile(r'^\s*name:\s*"([^"\n]+)"\s*$', re.M)
_POSITION = re.compile(r'\bposition\s*\{([^{}]*)\}', re.S)
_X = re.compile(r'\bx\s*:\s*([^\s}]+)')
_Y = re.compile(r'\by\s*:\s*([^\s}]+)')


def parse_pose(text):
    """Return (name, x, y) for a protobuf-text Pose block, or None."""
    name = _NAME.search(text)
    position = _POSITION.search(text)
    if name is None or position is None:
        return None
    x = _X.search(position.group(1))
    y = _Y.search(position.group(1))
    # Proto3 omits explicit zero-valued coordinates from text format.
    try:
        px = float(x.group(1)) if x else 0.0
        py = float(y.group(1)) if y else 0.0
    except ValueError:
        return None
    if not math.isfinite(px) or not math.isfinite(py):
        return None
    return name.group(1), px, py


class PoseStream:
    """Independent, stoppable text subscriber for Gazebo pose/info."""

    def __init__(self, world, allowed_prefix='test_person_', robot='robot_01'):
        if not re.fullmatch(r'[A-Za-z0-9_-]+', world):
            raise ValueError('invalid Gazebo world name')
        self.robots = {robot} if isinstance(robot,str) else set(robot)
        if not self.robots:raise ValueError('No robot names provided')
        self.prefix = allowed_prefix
        self._lock = threading.Lock()
        self._poses = {}  # name -> ((x,y), monotonic receive time)
        self._exit_code = None
        try:
            self._proc = subprocess.Popen(
                ['gz', 'topic', '-e', '-t', f'/world/{world}/pose/info'],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                bufsize=1,
            )
        except OSError as exc:
            raise RuntimeError(f'Unable to execute Gazebo CLI: {exc}') from exc
        self._thread = threading.Thread(target=self._listen, daemon=True, name='gazebo-pose-cli')
        self._thread.start()

    def _listen(self):
        in_pose = False
        depth = 0
        lines = []
        try:
            for line in self._proc.stdout:
                if not in_pose:
                    if line.strip() == 'pose {':
                        in_pose = True
                        lines = [line]
                        depth = line.count('{') - line.count('}')
                    continue
                lines.append(line)
                depth += line.count('{') - line.count('}')
                if depth == 0:
                    pose = parse_pose(''.join(lines))
                    if pose is not None:
                        name, x, y = pose
                        if name in self.robots or (name.startswith(self.prefix) and
                            name[len(self.prefix):].isdigit()):
                            with self._lock:
                                self._poses[name] = ((x, y), time.monotonic())
                    in_pose = False
                    lines = []
                elif len(lines) > 100:  # malformed/unbounded message: abandon block
                    in_pose = False
                    lines = []
        finally:
            self._exit_code = self._proc.poll()

    def snapshot(self):
        with self._lock:
            return dict(self._poses)

    def alive(self):
        return self._proc.poll() is None

    def close(self):
        if self._proc.poll() is None:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self._proc.kill()
                self._proc.wait(timeout=2)
        if self._proc.stdout:
            self._proc.stdout.close()
        self._thread.join(timeout=2)
