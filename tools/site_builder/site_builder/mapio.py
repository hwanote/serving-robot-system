"""Read a Nav2 style map (map.yaml + PGM) and write small PNG previews.

Conventions (same as scripts/validate_site_config.py):
  * image row 0 is the TOP of the map (largest y)
  * cell centre x = origin_x + (col + 0.5) * res
  * cell centre y = origin_y + (h - 1 - row + 0.5) * res
Only trinary maps with origin yaw 0 are supported (what the demo map and a
SLAM Toolbox save give); anything else raises MapError instead of guessing.
"""
import math
import os
import struct
import zlib

import numpy as np
import yaml


class MapError(ValueError):
    pass


def read_pgm(path):
    """Binary PGM (P5, 8 bit) -> uint8 array (h, w)."""
    with open(path, 'rb') as f:
        data = f.read()
    pos = 0
    tokens = []
    while len(tokens) < 4:
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            while data[pos:pos + 1] not in (b'\n', b''):
                pos += 1
            continue
        start = pos
        while pos < len(data) and not data[pos:pos + 1].isspace():
            pos += 1
        tokens.append(data[start:pos])
    magic, w, h, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    if magic != b'P5' or maxval != 255:
        raise MapError('%s: only binary 8 bit PGM (P5, maxval 255) is supported' % path)
    pos += 1  # single whitespace after maxval
    img = np.frombuffer(data, dtype=np.uint8, count=w * h, offset=pos)
    return img.reshape(h, w).copy()


class MapData:
    def __init__(self, yaml_path):
        with open(yaml_path, 'r', encoding='utf-8') as f:
            m = yaml.safe_load(f)
        self.yaml_path = os.path.abspath(yaml_path)
        self.res = float(m['resolution'])
        self.ox, self.oy = float(m['origin'][0]), float(m['origin'][1])
        if abs(float(m['origin'][2])) > 1e-9:
            raise MapError('origin yaw != 0 is not supported')
        if m.get('mode', 'trinary') != 'trinary':
            raise MapError("map mode '%s' is not supported (trinary only)" % m.get('mode'))
        img = read_pgm(os.path.join(os.path.dirname(self.yaml_path), m['image']))
        if int(m.get('negate', 0)):
            img = 255 - img
        self.img = img
        self.h, self.w = img.shape
        occ_p = (255.0 - img.astype(float)) / 255.0
        occ_t = float(m.get('occupied_thresh', 0.65))
        free_t = float(m.get('free_thresh', 0.25))
        self.occupied = occ_p >= occ_t
        self.free = occ_p < free_t
        self.unknown = ~(self.occupied | self.free)
        self.revision = None
        meta = os.path.join(os.path.dirname(self.yaml_path), 'map_meta.yaml')
        if os.path.exists(meta):
            with open(meta, 'r', encoding='utf-8') as f:
                self.revision = (yaml.safe_load(f) or {}).get('map_revision')

    @property
    def blocked(self):
        """Occupied or unknown: the robot may not plan through either."""
        return ~self.free

    def cell_of(self, x, y):
        col = int(math.floor((x - self.ox) / self.res))
        row = self.h - 1 - int(math.floor((y - self.oy) / self.res))
        return row, col

    def xy_of(self, row, col):
        return (self.ox + (col + 0.5) * self.res,
                self.oy + (self.h - 1 - row + 0.5) * self.res)

    def inside(self, row, col):
        return 0 <= row < self.h and 0 <= col < self.w


def png_bytes(rgb):
    """uint8 array (h, w, 3) -> PNG bytes (stdlib only)."""
    rgb = np.ascontiguousarray(rgb, dtype=np.uint8)
    h, w, _ = rgb.shape
    raw = b''.join(b'\x00' + rgb[r].tobytes() for r in range(h))

    def chunk(tag, body):
        c = struct.pack('>I', len(body)) + tag + body
        return c + struct.pack('>I', zlib.crc32(tag + body) & 0xffffffff)

    png = b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0))
    return png + chunk(b'IDAT', zlib.compress(raw, 9)) + chunk(b'IEND', b'')


def save_png(path, rgb):
    """uint8 array (h, w, 3) -> PNG file."""
    with open(path, 'wb') as f:
        f.write(png_bytes(rgb))
