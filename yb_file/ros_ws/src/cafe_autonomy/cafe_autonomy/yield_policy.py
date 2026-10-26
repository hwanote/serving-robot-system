"""Geometry-only candidate screening for two-robot traffic yields.

Nav2 must still approve each candidate path before a robot receives a YIELD.
No candidate is a HOME waypoint or a command to reverse the robot.
"""
import math


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def in_rectangle(point, bounds, margin=0.0):
    x0, x1 = sorted(bounds[:2])
    y0, y1 = sorted(bounds[2:])
    return x0 - margin <= point[0] <= x1 + margin and y0 - margin <= point[1] <= y1 + margin


def _angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def candidate_poses(here, peer, history, zones, no_go, homes, peer_goal,
                    separation, zone_margin):
    """Offer recent clear positions and nearby exits; never declare them safe."""
    x, y, heading = here
    options = [p[:2] for p in reversed(history)]
    for bounds in zones.values():
        if in_rectangle(here, bounds):
            x0, x1 = sorted(bounds[:2]); y0, y1 = sorted(bounds[2:])
            offset = zone_margin + separation / 2
            options.extend(((x0 - offset, y), (x1 + offset, y),
                            (x, y0 - offset), (x, y1 + offset)))
    away = math.atan2(y - peer[1], x - peer[0])
    for radius in (separation * 1.5, separation * 2):
        for turn in (0, math.pi / 2, -math.pi / 2):
            angle = away + turn
            options.append((x + radius * math.cos(angle),
                            y + radius * math.sin(angle)))
    result = []
    for point in options:
        if any(distance(point, other[:2]) < separation / 2 for other in result):
            continue
        if distance(point, here) < separation:
            continue
        if distance(point, here) > 4 * separation:
            continue
        if distance(point, peer) < separation + zone_margin:
            continue
        if peer_goal and distance(point, peer_goal) < separation + zone_margin:
            continue
        if any(distance(point, home) < separation for home in homes):
            continue
        if any(in_rectangle(point, b, zone_margin) for b in zones.values()):
            continue
        if any(in_rectangle(point, b) for b in no_go.values()):
            continue
        direction = math.atan2(point[1] - y, point[0] - x)
        result.append((point[0], point[1], direction,
                       abs(_angle(direction - heading)) > math.pi / 2))
    return result


def segment_distance(point, a, b):
    dx = b[0] - a[0]; dy = b[1] - a[1]
    length_sq = dx * dx + dy * dy
    fraction = max(0.0, min(1.0, ((point[0] - a[0]) * dx +
                                    (point[1] - a[1]) * dy) / length_sq)) if length_sq else 0.0
    return distance(point, (a[0] + fraction * dx, a[1] + fraction * dy))


def assess_path(points, here, candidate, peer, separation, start_tolerance,
                goal_tolerance):
    """Reject a stale, incomplete, or peer-intersecting Nav2 path."""
    if len(points) < 2 or distance(points[0], here) > start_tolerance:
        return None
    if distance(points[-1], candidate) > goal_tolerance:
        return None
    clearance = min(segment_distance(peer, a, b) for a, b in zip(points, points[1:]))
    if clearance < separation:
        return None
    length = sum(distance(a, b) for a, b in zip(points, points[1:]))
    return {"length_m": length, "peer_clearance_m": clearance}


def choose_yield(options, last_yielder=None):
    """Prefer feasible front/side motion, then the shorter safe route."""
    if not options:
        return None
    return min(options, key=lambda o: (bool(o["rear_turn"]), o["length_m"],
                                       o["turn_rad"], o["robot"] == last_yielder))
