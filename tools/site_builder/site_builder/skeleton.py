"""Thin a boolean mask to a one pixel wide centre line (Zhang-Suen, numpy only)."""
import numpy as np


def thin(mask):
    img = np.pad(mask.astype(np.uint8), 1)
    while True:
        changed = False
        for step in (0, 1):
            p2, p3, p4 = img[:-2, 1:-1], img[:-2, 2:], img[1:-1, 2:]
            p5, p6, p7 = img[2:, 2:], img[2:, 1:-1], img[2:, :-2]
            p8, p9 = img[1:-1, :-2], img[:-2, :-2]
            ring = [p2, p3, p4, p5, p6, p7, p8, p9]
            b = sum(x.astype(np.int16) for x in ring)
            a = sum(((ring[i] == 0) & (ring[(i + 1) % 8] == 1)).astype(np.int16)
                    for i in range(8))
            if step == 0:
                c1 = (p2 * p4 * p6) == 0
                c2 = (p4 * p6 * p8) == 0
            else:
                c1 = (p2 * p4 * p8) == 0
                c2 = (p2 * p6 * p8) == 0
            kill = (img[1:-1, 1:-1] == 1) & (b >= 2) & (b <= 6) & (a == 1) & c1 & c2
            if kill.any():
                img[1:-1, 1:-1][kill] = 0
                changed = True
        if not changed:
            break
    return img[1:-1, 1:-1].astype(bool)
