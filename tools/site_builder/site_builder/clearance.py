"""Distance to the nearest blocked cell, capped (numpy only, no scipy).

For every cell the result is the distance in metres from the cell centre to the
nearest blocked cell, minus half a cell so that the number means "to the edge
of the obstacle square", the same meaning as the validator's clearance.
Cells at or beyond `cap_m` read `cap_m`. Outside of the map counts as blocked.

Cost: about pi * (cap_m / res)^2 array passes, fine for a store map at 5 cm
(cap 1.0 m = ~1250 passes). For very large maps lower `cap_m`.
"""
import math

import numpy as np


def clearance_map(blocked, res, cap_m=1.0):
    h, w = blocked.shape
    r = int(math.ceil(cap_m / res))
    padded = np.ones((h + 2 * r, w + 2 * r), dtype=bool)
    padded[r:r + h, r:r + w] = blocked
    best = np.full((h, w), cap_m + res, dtype=np.float32)   # centre distance
    offsets = []
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            d = math.hypot(dy, dx) * res
            if 0.0 < d <= cap_m + res:
                offsets.append((d, dy, dx))
    offsets.sort()
    for d, dy, dx in offsets:
        shifted = padded[r + dy:r + dy + h, r + dx:r + dx + w]
        np.minimum(best, np.where(shifted, np.float32(d), best), out=best)
    out = np.minimum(best - res / 2.0, cap_m).clip(min=0.0)
    out[blocked] = 0.0
    return out.astype(np.float32)


def rasterize_rects(shape, mp, rects):
    """Boolean mask of the cells whose centre lies inside any rect (x0,x1,y0,y1)."""
    mask = np.zeros(shape, dtype=bool)
    rows = np.arange(mp.h)
    cols = np.arange(mp.w)
    xs = mp.ox + (cols + 0.5) * mp.res
    ys = mp.oy + (mp.h - 1 - rows + 0.5) * mp.res
    for x0, x1, y0, y1 in rects:
        cx = (xs >= x0) & (xs <= x1)
        cy = (ys >= y0) & (ys <= y1)
        mask |= cy[:, None] & cx[None, :]
    return mask


def exact_sampler(mp, obst, cap_m=1.0):
    """at(x, y) -> exact distance from the point to the nearest blocked cell rectangle
    (outside the map counts as blocked), capped. Not quantised to the cell grid, so it
    matches scripts/validate_site_config.py on a ridge such as the middle of a 1.0 m aisle."""
    half = int(cap_m / mp.res) + 2
    pad = np.pad(obst, half, constant_values=True)

    def at(x, y):
        r, c = mp.cell_of(x, y)
        r0, c0 = r, c      # window in padded coordinates: [r0, r0 + 2 half]
        win = pad[r0:r0 + 2 * half + 1, c0:c0 + 2 * half + 1]
        rr, cc = np.nonzero(win)
        if len(rr) == 0:
            return cap_m
        ro, co = rr + r0 - half, cc + c0 - half          # original cell indices
        x0 = mp.ox + co * mp.res
        y0 = mp.oy + (mp.h - 1 - ro) * mp.res
        dx = np.maximum(np.maximum(x0 - x, x - (x0 + mp.res)), 0.0)
        dy = np.maximum(np.maximum(y0 - y, y - (y0 + mp.res)), 0.0)
        return float(min(cap_m, np.sqrt(dx * dx + dy * dy).min()))
    return at
