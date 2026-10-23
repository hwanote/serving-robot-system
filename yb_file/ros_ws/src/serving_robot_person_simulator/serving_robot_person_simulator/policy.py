"""Pure decision logic with no ROS/Gazebo imports. Deterministic per seed."""
import math
import random
from dataclasses import dataclass, field


def validate(cfg):
    if cfg.get('people_max') != 8:
        raise ValueError('people_max must be 8 (user-defined maximum)')
    b = cfg['behavior']
    speed = b['speed']
    actions = b['action_percent']
    if set(speed) != {'slow', 'normal', 'fast'}:
        raise ValueError('speed choices must be slow/normal/fast')
    if set(actions) != {'continue', 'gradual_stop', 'sudden_stop', 'long_block'}:
        raise ValueError('unknown action choices')
    for probs in ([speed[k]['percent'] for k in speed], list(actions.values())):
        if any(type(p) not in (int, float) or p < 0 for p in probs) or abs(sum(probs)-100)>1e-6:
            raise ValueError('percentage set must sum to 100')
    for s in speed.values():
        if not (0 < s['mps'] <= 1.5):
            raise ValueError('person speed must be >0 and <=1.5m/s')
    if not (0 <= b['give_way_percent'] <= 100):
        raise ValueError('give_way_percent outside 0..100')
    for k in ('decision_interval_s', 'gradual_stop_duration_s', 'gradual_pause_s',
              'sudden_pause_s', 'long_block_pause_s'):
        vals = b[k]
        if len(vals) != 2 or not (0 < vals[0] <= vals[1]):
            raise ValueError(f'invalid interval {k}')
    for k in ('robot_stop_radius_m', 'robot_release_radius_m',
              'person_stop_radius_m', 'person_release_radius_m'):
        if cfg[k] <= 0:
            raise ValueError(f'invalid {k}')
    if cfg['robot_stop_radius_m'] >= cfg['robot_release_radius_m']:
        raise ValueError('robot distance hysteresis invalid')
    if cfg['person_stop_radius_m'] >= cfg['person_release_radius_m']:
        raise ValueError('person distance hysteresis invalid')
    if cfg['control_rate_hz'] < 1 or cfg['control_rate_hz'] > 20:
        raise ValueError('control_rate_hz outside 1..20')
    if ('person_deadlock_timeout_s' in cfg) != ('person_hard_min_radius_m' in cfg):
        raise ValueError('person_deadlock_timeout_s and person_hard_min_radius_m must be set together')
    if 'person_deadlock_timeout_s' in cfg:
        if not (0 < cfg['person_deadlock_timeout_s'] <= 120):
            raise ValueError('person_deadlock_timeout_s outside (0,120]')
        if not (2 * cfg['person_radius_m'] < cfg['person_hard_min_radius_m'] < cfg['person_stop_radius_m']):
            raise ValueError('person_hard_min_radius_m must be > 2*person_radius_m and < person_stop_radius_m')
    for lane in cfg['lanes']:
        a, z = lane['start'], lane['end']
        if len(a)!=2 or len(z)!=2 or any(not math.isfinite(v) for v in a+z):
            raise ValueError('lane coordinates invalid')
        if math.dist(a,z) < 0.7:
            raise ValueError('lane too short (<0.7m)')
    return cfg


OVERLAY_KEYS = ('person_deadlock_timeout_s', 'person_hard_min_radius_m')


def apply_deadlock_overlay(cfg, overlay):
    """Return a validated copy of cfg with ONLY the two deadlock keys taken from overlay.

    Any other key (speeds, radii, probabilities, lanes, ...) is rejected so an overlay
    can never change existing policy values.
    """
    if not isinstance(overlay, dict):
        raise ValueError('overlay must be a mapping')
    extra = sorted(set(overlay) - set(OVERLAY_KEYS))
    if extra:
        raise ValueError(f'overlay may only set {OVERLAY_KEYS}; rejected keys: {extra}')
    merged = dict(cfg)
    merged.update(overlay)
    return validate(merged)


def choose(rng, weighted):
    """Weighted draw; weights are percentages, event-based, never frame-based."""
    p = rng.random()*100
    acc = 0.0
    for key, weight in weighted.items():
        acc += weight
        if p < acc:
            return key
    return list(weighted)[-1]


def point_near_polyline(x, y, points, width):
    if len(points)<2:
        return False
    for a,b in zip(points,points[1:]):
        vx,vy=b[0]-a[0],b[1]-a[1]
        t=max(0,min(1,((x-a[0])*vx+(y-a[1])*vy)/max(1e-12,vx*vx+vy*vy)))
        if math.hypot(x-a[0]-t*vx,y-a[1]-t*vy)<=width:
            return True
    return False


@dataclass
class Walker:
    ident: int
    name: str
    start: tuple
    end: tuple
    target_end: bool = True
    speed_mps: float = 0.0
    phase: str = 'moving'
    paused: bool = False
    next_decision: float = 0.0
    until: float = 0.0
    stop_start: float = 0.0
    decel_start_speed: float = 0.0
    yield_selected: bool | None = None
    proximity_hold: bool = False
    person_hold: bool = False
    current_action: str = 'continue'
    last_position: tuple | None = None
    published_v: tuple = field(default_factory=lambda: (0.0,0.0))
    # Person-person deadlock resolution state (see PersonRules). Defaults keep old behaviour.
    person_pass: bool = False
    person_block_since: float | None = None
    person_block_tick: float | None = None
    person_pass_since: float | None = None
    person_retreats: int = 0


def validate_spawn_positions(starts, robot, existing, robot_margin, person_margin):
    """Reject the WHOLE spawn batch before a single Gazebo entity is created."""
    if robot is None or not all(math.isfinite(v) for v in robot):
        raise ValueError('fresh robot ground truth required for spawn')
    if any(p is None for p in existing):
        raise ValueError('existing person pose unknown')
    for i, (start, end) in enumerate(starts):
        if math.dist(start,robot) < robot_margin or math.dist(end,robot) < robot_margin:
            raise ValueError(f'lane {i+1} endpoint too close to robot')
        if any(math.dist(start,p) < person_margin for p in existing):
            raise ValueError(f'lane {i+1} starts too close to existing person')
        if any(math.dist(start,other[0]) < person_margin for other in starts[:i]):
            raise ValueError('multiple people spawn too close together')


def draw_speed(rng, cfg):
    s=cfg['behavior']['speed']
    key=choose(rng,{k:v['percent'] for k,v in s.items()})
    return key, float(s[key]['mps'])


def decide_action(rng,cfg,now,x,y):
    b=cfg['behavior']
    weights=dict(b['action_percent'])
    if not point_near_polyline(x,y,cfg['blockage_corridor'],cfg['blockage_corridor_half_width_m']):
        # Long blockage only where manually configured robot corridor is located.
        weights['continue'] += weights.pop('long_block')
    kind=choose(rng,weights)
    duration=0.0
    if kind=='gradual_stop':
        duration=rng.uniform(*b['gradual_stop_duration_s'])
    elif kind=='sudden_stop':
        duration=rng.uniform(*b['sudden_pause_s'])
    elif kind=='long_block':
        duration=rng.uniform(*b['long_block_pause_s'])
    return kind,duration


def next_time(rng,cfg,now):
    return now+rng.uniform(*cfg['behavior']['decision_interval_s'])


def can_escape_person_hold(position, velocity, neighbors,
                           stop_radius, release_radius):
    """Only permit motion that increases separation from nearby people."""
    import math

    vx, vy = velocity
    if math.hypot(vx, vy) <= 1e-9:
        return False

    for other_pos, other_velocity in neighbors:
        if other_pos is None:
            return False

        dx = position[0] - other_pos[0]
        dy = position[1] - other_pos[1]
        distance = math.hypot(dx, dy)

        # The hard stop radius remains mandatory.
        if distance <= stop_radius:
            return False

        if distance < release_radius:
            relative_vx = vx - other_velocity[0]
            relative_vy = vy - other_velocity[1]

            # Positive radial relative velocity means separating.
            separating = dx * relative_vx + dy * relative_vy

            if separating <= 1e-8:
                return False

    return True


# ---------------------------------------------------------------------------
# Person-person rules (extracted from manager.tick, behaviour-identical unless a
# deadlock "pass" is active). Pure: no ROS/Gazebo imports.
# ---------------------------------------------------------------------------
PREDICT_HORIZON_S = 0.8
ARBITRATION_RADIUS_M = 2.0
PASS_MAX_FACTOR = 3.0  # a pass lasts at most 3 x person_deadlock_timeout_s


def future_conflict(person, velocity, others, radius, horizon=PREDICT_HORIZON_S):
    """True if the closest approach within `horizon` to any other person is < radius.

    others: iterable of (position, velocity). Identical math to the original manager.
    """
    vx, vy = velocity
    for other_pos, (ovx, ovy) in others:
        rx = person[0] - other_pos[0]
        ry = person[1] - other_pos[1]
        rvx = vx - ovx
        rvy = vy - ovy
        rel_sq = rvx * rvx + rvy * rvy
        if rel_sq > 1e-9:
            t = max(0.0, min(horizon, -(rx * rvx + ry * rvy) / rel_sq))
        else:
            t = 0.0
        if math.hypot(rx + rvx * t, ry + rvy * t) < radius:
            return True
    return False


class PersonRules:
    """Person-vs-person stop/hold rules plus a bounded-wait deadlock breaker.

    Without a pass the rules equal the original manager logic (soft radius
    person_stop_radius_m). If a person has been blocked ONLY by these rules for
    person_deadlock_timeout_s, the lowest-id such person within 2 m gets a "pass":
    it may move as long as it keeps person_hard_min_radius_m (> body diameter)
    from everyone. The pass ends when it is >= person_release_radius_m from all
    others, or after another timeout spent blocked even under the relaxed rule
    (it then turns around on its own lane and the next candidate gets its turn).
    A pass also expires after PASS_MAX_FACTOR x timeout so one person cannot starve
    the others. Robot-distance rules are untouched.
    peers: list of (id, position, published_velocity, walker).
    """

    def __init__(self, cfg):
        self.stop = cfg['person_stop_radius_m']
        self.release = cfg['person_release_radius_m']
        self.timeout = cfg.get('person_deadlock_timeout_s')
        self.hard = cfg.get('person_hard_min_radius_m')
        self.enabled = self.timeout is not None
        self.gap = max(0.5, 2.5 / cfg['control_rate_hz'])

    def _drop_pass(self, w):
        w.person_pass = False
        w.person_pass_since = None
        w.person_block_since = None
        w.person_block_tick = None

    def begin(self, w, now, person, peers):
        """Hold hysteresis + pass clearing. Returns distance to nearest other person."""
        nearest = min((math.dist(person, pos) for _, pos, _, _ in peers), default=math.inf)
        if nearest <= self.stop:
            w.person_hold = True
        elif nearest >= self.release:
            w.person_hold = False
        if w.person_pass and (nearest >= self.release or (
                self.enabled and w.person_pass_since is not None
                and now - w.person_pass_since >= PASS_MAX_FACTOR * self.timeout)):
            self._drop_pass(w)
        return nearest

    def relaxed(self, w):
        return self.enabled and w.person_pass

    def hold_blocked(self, w, person, velocity, peers):
        neighbors = [(pos, v) for _, pos, v, _ in peers]
        if self.relaxed(w):
            return any(math.dist(person, pos) <= self.hard for pos, _ in neighbors)
        if w.person_hold:
            return not can_escape_person_hold(person, velocity, neighbors, self.stop, self.release)
        return False

    def future_blocked(self, w, person, velocity, peers):
        radius = self.hard if self.relaxed(w) else self.stop
        return future_conflict(person, velocity, [(pos, v) for _, pos, v, _ in peers], radius)

    def pass_active(self, w, now):
        """A pass counts only until its maximum duration (even if its holder is paused)."""
        if not (self.enabled and w.person_pass):
            return False
        return w.person_pass_since is None or now - w.person_pass_since < PASS_MAX_FACTOR * self.timeout

    def _age(self, w, now):
        if (w.person_block_since is None or w.person_block_tick is None
                or now - w.person_block_tick > self.gap):
            return 0.0
        return now - w.person_block_since

    def mark(self, ident, w, now, person, peers, blocked_by_person):
        """Call each tick after the person-rule verdict; maintains timer and pass.

        Returns 'grant', 'retreat' or None (for logging only).
        """
        if not self.enabled:
            return None
        if not blocked_by_person:
            w.person_block_since = None
            w.person_block_tick = None
            return None
        if (w.person_block_since is None or w.person_block_tick is None
                or now - w.person_block_tick > self.gap):
            w.person_block_since = now
        w.person_block_tick = now
        age = now - w.person_block_since
        if age < self.timeout:
            return None
        if w.person_pass:  # blocked even when relaxed: back off along own lane, give a turn
            self._drop_pass(w)
            w.target_end = not w.target_end
            w.person_retreats += 1
            return 'retreat'
        for oid, pos, _, ow in peers:
            if math.dist(person, pos) > ARBITRATION_RADIUS_M:
                continue
            if self.pass_active(ow, now):
                return None
            if oid < ident and self._age(ow, now) >= self.timeout:
                return None
        w.person_pass = True
        w.person_pass_since = now
        w.person_block_since = now
        w.person_block_tick = now
        return 'grant'
