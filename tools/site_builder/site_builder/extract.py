"""Corridor extraction helpers shared by the command line and the pipeline."""
import numpy as np

from .graph import build_graph
from .skeleton import thin


def clearance_sampler(mp, clr):
    def at(x, y):
        r, c = mp.cell_of(x, y)
        return float(clr[r, c]) if mp.inside(r, c) else 0.0
    return at


def keep_big_components(mask, min_cells):
    """Drop 8-connected blobs smaller than min_cells."""
    h, w = mask.shape
    seen = np.zeros_like(mask, dtype=bool)
    out = np.zeros_like(mask, dtype=bool)
    for r0, c0 in zip(*np.nonzero(mask)):
        if seen[r0, c0]:
            continue
        stack, comp = [(r0, c0)], []
        seen[r0, c0] = True
        while stack:
            r, c = stack.pop()
            comp.append((r, c))
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    rr, cc = r + dr, c + dc
                    if 0 <= rr < h and 0 <= cc < w and mask[rr, cc] and not seen[rr, cc]:
                        seen[rr, cc] = True
                        stack.append((rr, cc))
        if len(comp) >= min_cells:
            for r, c in comp:
                out[r, c] = True
    return out


def generate_graph(mp, clr, need, spur_m, dp_tol_m, min_area_m2=0.25):
    drivable = mp.free & (clr >= need)
    drivable = keep_big_components(drivable, int(min_area_m2 / (mp.res * mp.res)))
    skel = thin(drivable)
    return build_graph(skel, mp.xy_of, clearance_sampler(mp, clr),
                       spur_m=spur_m, dp_tol_m=dp_tol_m, need=need), drivable, skel
