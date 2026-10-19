"""Furniture that is NOT in the static map (table tops, chairs) as rectangles."""
import yaml


def table_rects(entities):
    """[(x0, x1, y0, y1)] for every table top and chair in an entities dict."""
    rects = []
    for t in (entities.get('tables') or {}).values():
        cx, cy = t['center']['x'], t['center']['y']
        sx, sy = t['top_size_m']['x'] / 2, t['top_size_m']['y'] / 2
        rects.append((cx - sx, cx + sx, cy - sy, cy + sy))
        for c in t.get('chairs') or []:
            h = c['size_m'] / 2
            rects.append((cx + c['dx'] - h, cx + c['dx'] + h,
                          cy + c['dy'] - h, cy + c['dy'] + h))
    return rects


def load_entities(path):
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f) or {}
