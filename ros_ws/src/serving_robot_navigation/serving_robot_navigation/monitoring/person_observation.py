"""What one camera saw: the people it found and how far away they are (no ROS).

The camera node (scripts/monitoring/person_detector.py) only reports facts: it
runs YOLO on the RGB image, measures each person on the depth image and sends
one observation per tick on /{robot}/person_observations (JSON):

    {"camera": "camera_front", "time": 12.3, "new_frame": true, "count": 2,
     "people": [{"distance_m": 1.42, "bearing_rad": 0.31, "confidence": 0.81,
                 "bbox": [x1, y1, x2, y2]},
                {"distance_m": null, ...}]}

new_frame  false = no new camera image since the last tick (the observation is
           still sent: it tells the judge that the camera node is alive)
distance_m  real distance from the camera (m). The depth image gives the
            distance ALONG the optical axis; a person off the axis by angle a is
            really depth / cos(a) away (half again at the edge of a 120 deg
            camera), see true_range. null = seen, but the depth was not usable
bearing_rad direction of the person seen from the robot (0 = straight ahead,
            + = left), camera yaw + pixel angle. null = unknown (an old
            observation): the judge treats it as straight ahead

The state (NORMAL / SLOW / STOP) is decided elsewhere (decision/person_judge.py).
"""
import json
import math

import numpy as np


def fx_from_fov(width, hfov_rad):
    """Focal length in pixels of a pinhole camera of this width and view."""
    return (width / 2.0) / math.tan(hfov_rad / 2.0)


def pixel_angle(u, cx, fx):
    """Horizontal angle of image column u off the optical axis (+ = left)."""
    return math.atan2(cx - u, fx)


def true_range(depth_m, off_axis_rad, min_cos=0.2):
    """Distance along the line of sight from the depth along the axis."""
    return depth_m / max(math.cos(off_axis_rad), min_cos)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def box_distance(depth, box, min_m, max_m, min_pixels=10):
    """Median depth of the middle half of the person box; None if too few valid pixels."""
    h, w = depth.shape
    x1, y1, x2, y2 = box
    cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
    hw, hh = (x2 - x1) / 4.0, (y2 - y1) / 4.0
    xa, xb = int(max(0, cx - hw)), int(min(w, cx + hw) + 1)
    ya, yb = int(max(0, cy - hh)), int(min(h, cy + hh) + 1)
    roi = depth[ya:yb, xa:xb].ravel()
    roi = roi[np.isfinite(roi) & (roi > min_m) & (roi < max_m)]
    if roi.size < min_pixels:
        return None
    return float(np.median(roi))


def make_observation(camera, now, new_frame, found=()):
    """`found`: (distance_m or None, confidence, [x1, y1, x2, y2]
    [, bearing_rad or None]) per person."""
    people = []
    for f in found:
        d, c, box = f[0], f[1], f[2]
        b = f[3] if len(f) > 3 else None
        people.append({'distance_m': None if d is None else float(d),
                       'bearing_rad': None if b is None else round(float(b), 4),
                       'confidence': float(c),
                       'bbox': [float(v) for v in box]})
    return {'camera': str(camera), 'time': round(float(now), 3),
            'new_frame': bool(new_frame), 'count': len(people), 'people': people}


def parse_observation(text):
    """JSON text -> observation dict, or None when it cannot be used."""
    try:
        obs = json.loads(text)
    except (TypeError, ValueError):
        return None
    if not isinstance(obs, dict) or not isinstance(obs.get('people', []), list):
        return None
    obs.setdefault('people', [])
    obs.setdefault('new_frame', False)
    obs.setdefault('camera', '')
    obs.setdefault('count', len(obs['people']))
    return obs


def measured(obs):
    """People of the observation that have a usable distance."""
    return [p for p in obs.get('people', [])
            if isinstance(p, dict) and isinstance(p.get('distance_m'), (int, float))]
