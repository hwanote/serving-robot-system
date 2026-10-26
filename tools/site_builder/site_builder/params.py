"""Robot and rule numbers. Read from the project files so no value is copied:
the body from the Nav2 footprint, the HardStop margin from runtime_limits.yaml."""
import ast
import math
import os
import re

import yaml

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))


class Params:
    def __init__(self, **kw):
        self.front_m = 0.26          # body front / rear edge from the base_footprint
        self.rear_m = 0.26
        self.half_width_m = 0.20
        self.hard_stop_m = 0.10      # HardStop = body + this on every side
        self.min_corridor_m = 0.80
        self.spur_m = 0.40
        self.dp_tol_m = 0.10
        self.gap_min_m = 0.15        # robot front -> table / counter (validator rule)
        self.gap_max_m = 0.90
        self.gap_lo_m = 0.20         # accepted front gap for a generated stop (> validator 0.15)
        self.gap_hi_m = 0.90
        self.yaw_snap_deg = 8.0      # a stop yaw this close to an axis is set on the axis
        self.node_reuse_m = 0.15     # a stop this close to a node uses the node
        # margins beyond the HardStop shape [proposal, see DECISIONS]
        self.turn_fail_margin_m = 0.02
        self.turn_warn_margin_m = 0.10
        self.edge_fail_margin_m = 0.02
        self.edge_warn_margin_m = 0.10
        self.default_top_m = 0.80    # table top when the label has no size
        # approach branch: a straight spur from the corridor node towards the table, ending at a
        # dock node where the robot serves. The guest can reach the last bit, so no closer than needed.
        self.approach = False         # OFF: designed and unit-tested, never run on a robot (see DECISIONS #98)
        self.serve_gap_m = 0.30      # wanted robot front -> table top edge at the dock
        self.min_branch_m = 0.30     # a shorter branch is not built (Nav2 arrives within 0.15 m)
        self.snap_max_m = 1.5        # a standby point further than this from a corridor
        self.pickup_search_m = 0.30  # how far from the click to look for the counter
        self.zone_pad_m = 0.70
        self.speed_mps = 0.25        # max_linear_mps of runtime_limits.yaml (read below)
        self.omega_rps = 0.80        # max_angular_rps
        self.tight_extra_m = 0.10    # a drive that passed within HardStop half width + this is TIGHT
        for k, v in kw.items():
            if not hasattr(self, k):
                raise TypeError('unknown parameter %s' % k)
            setattr(self, k, v)

    @property
    def length_m(self):
        return self.front_m + self.rear_m

    @property
    def width_m(self):
        return 2 * self.half_width_m

    @property
    def hs_front_m(self):
        return self.front_m + self.hard_stop_m

    @property
    def hs_rear_m(self):
        return self.rear_m + self.hard_stop_m

    @property
    def hs_half_m(self):
        return self.half_width_m + self.hard_stop_m

    @property
    def corner_radius_m(self):
        """Radius of the circle the HardStop corners draw when the robot turns in place."""
        return math.hypot(max(self.hs_front_m, self.hs_rear_m), self.hs_half_m)

    def as_dict(self):
        d = {k: v for k, v in self.__dict__.items()}
        d['corner_radius_m'] = round(self.corner_radius_m, 4)
        return d


def load_params(repo=REPO, **overrides):
    p = Params()
    try:
        text = open(os.path.join(repo, 'ros_ws/src/serving_robot_navigation/config/nav2.yaml'),
                    encoding='utf-8').read()
        m = re.search(r'footprint:\s*"(\[\[.*?\]\])"', text)
        pts = ast.literal_eval(m.group(1))
        p.front_m = max(x for x, _ in pts)
        p.rear_m = -min(x for x, _ in pts)
        p.half_width_m = max(abs(y) for _, y in pts)
    except Exception:   # noqa: BLE001 - keep the defaults, they are the same numbers
        pass
    try:
        with open(os.path.join(repo, 'config/common/runtime_limits.yaml'), encoding='utf-8') as f:
            lim = yaml.safe_load(f)
        p.hard_stop_m = float(lim['obstacle_reaction']['hard_stop_distance_m'])
        p.speed_mps = float(lim.get('max_linear_mps', p.speed_mps))
        p.omega_rps = float(lim.get('max_angular_rps', p.omega_rps))
    except Exception:   # noqa: BLE001
        pass
    for k, v in overrides.items():
        setattr(p, k, v)
    return p
