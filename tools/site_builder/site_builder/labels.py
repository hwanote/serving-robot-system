"""labels.json: the points a person marked on the map.  [proposal, not a team contract]

{
  "schema_version": 1,
  "map_revision": 1,                       (optional; must match the map when both exist)
  "labels": [
    {"id": "table_01", "kind": "table",   "x_m": -1.60, "y_m": 0.65,
     "top_size_m": {"x": 0.8, "y": 0.8},   (optional)
     "chairs": [{"dx": -0.65, "dy": 0.0, "size_m": 0.45}],   (optional)
     "aliases": [],                      (optional)
     "stop": {"x_m": -1.6, "y_m": 0.2, "yaw_deg": 90}},   (optional: a person placed the stop by hand)
    {"id": "kitchen",  "kind": "pickup",  "x_m": -2.30, "y_m": 2.40},
    {"id": "standby_01", "kind": "standby", "x_m": -3.20, "y_m": -2.40}
  ]
}
x_m / y_m are map-frame metres (what a front end gets by converting the clicked pixel
with the origin and resolution of map.yaml).
table   : centre (any point on the table) of a table to serve
pickup  : the counter / desk where the drink is loaded; the robot faces it
standby : where the robot waits (home); the robot is placed on the nearest corridor
stop    : (optional, any kind) the spot a person chose for the robot to stand: the generator puts the stop
          on the corridor point nearest to it, facing yaw_deg (east/north/west/south), and runs the
          same checks as for a generated stop. If they fail the label is left out with the reason;
          it is never replaced by a generated stop. yaw_deg may be omitted (table / pickup: face the
          target; standby: along the corridor).
"""
import json
import math
import re

KINDS = ('table', 'pickup', 'standby')
ID_RE = re.compile(r'^[a-z][a-z0-9_]*$')


class LabelError(ValueError):
    pass


class NoStandbyError(LabelError):
    """The set is fine but has no standby point yet (a half-finished edit)."""


def load_labels(source, map_revision=None):
    if isinstance(source, dict):
        data = source
    else:
        with open(source, 'r', encoding='utf-8') as f:
            data = json.load(f)
    if data.get('schema_version') != 1:
        raise LabelError('schema_version must be 1')
    rev = data.get('map_revision')
    if rev is not None and map_revision is not None and rev != map_revision:
        raise LabelError('labels were made for map_revision %s but the map is %s: '
                         'mark the points again on the new map' % (rev, map_revision))
    out, seen = [], set()
    for i, raw in enumerate(data.get('labels') or []):
        where = 'label #%d' % (i + 1)
        lid = str(raw.get('id', ''))
        if not ID_RE.match(lid):
            raise LabelError("%s: id '%s' must be lower case letters, digits, '_'" % (where, lid))
        if lid in seen:
            raise LabelError("duplicate label id '%s'" % lid)
        seen.add(lid)
        kind = raw.get('kind')
        if kind not in KINDS:
            raise LabelError("%s (%s): kind must be one of %s" % (where, lid, ', '.join(KINDS)))
        try:
            x, y = float(raw['x_m']), float(raw['y_m'])
        except (KeyError, TypeError, ValueError):
            raise LabelError('%s (%s): x_m and y_m are required numbers' % (where, lid))
        if not (math.isfinite(x) and math.isfinite(y)):
            raise LabelError('%s (%s): x_m / y_m must be finite' % (where, lid))
        lab = {'id': lid, 'kind': kind, 'x': x, 'y': y,
               'aliases': [str(a) for a in raw.get('aliases') or []]}
        if raw.get('top_size_m'):
            lab['top_size_m'] = {'x': float(raw['top_size_m']['x']),
                                 'y': float(raw['top_size_m']['y'])}
        if raw.get('chairs'):
            lab['chairs'] = [{'dx': float(c['dx']), 'dy': float(c['dy']),
                              'size_m': float(c['size_m'])} for c in raw['chairs']]
        st = raw.get('stop')
        if st is not None:
            try:
                sx, sy = float(st['x_m']), float(st['y_m'])
                syaw = None if st.get('yaw_deg') is None else float(st['yaw_deg'])
            except (KeyError, TypeError, ValueError, AttributeError):
                raise LabelError('%s (%s): stop needs the numbers x_m and y_m (yaw_deg is optional)'
                                 % (where, lid))
            if not (math.isfinite(sx) and math.isfinite(sy) and (syaw is None or math.isfinite(syaw))):
                raise LabelError('%s (%s): stop values must be finite' % (where, lid))
            lab['stop'] = {'x': sx, 'y': sy, 'yaw_deg': syaw}
        out.append(lab)
    if not any(l['kind'] == 'standby' for l in out):
        raise NoStandbyError('대기(홈) 점이 하나는 필요해요. 점이 없다면 「예시 불러오기」를 누르거나 대기 점을 찍어 주세요')
    return {'map_revision': rev, 'labels': out}
