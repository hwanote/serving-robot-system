"""Stand-in for tools/run_lidar_mapping_session.py used by the mapping tests.

  --run --log-dir DIR     FAKE_MAP_MODE: finish (default) | hang | crash | nomap
Writes the same files the real runner does: R16-147_exploration.log, the raw map and
R16-147_session_result.json. 'hang' waits for SIGINT (like Stop) and then writes the map.
"""
import json
import signal
import sys
import os
import time

args = sys.argv[1:]
out = args[args.index('--log-dir') + 1]
mode = os.environ.get('FAKE_MAP_MODE', 'finish')
stop = []
signal.signal(signal.SIGINT, lambda *a: stop.append(1))
from PIL import Image      # imported after the handler: a Stop that comes early is not lost


def write_map():
    w = h = 12
    px = bytearray([255] * (w * h))
    for y in range(1, h - 1):
        for x in range(1, w - 1):
            px[y * w + x] = 0                       # free room
    for i in range(w):
        for y in (0, h - 1):
            px[y * w + i] = 100                     # walls
        px[i * w] = px[i * w + w - 1] = 100
    Image.frombytes('L', (w, h), bytes(px)).save(os.path.join(out, 'R16-147_official_raw_map.pgm'), format='PPM')
    with open(os.path.join(out, 'R16-147_official_raw_map.yaml'), 'w') as f:
        f.write('image: R16-147_official_raw_map.pgm\nmode: raw\nresolution: 0.1\n'
                'origin: [0.0, 0.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    return w, h, px


with open(os.path.join(out, 'R16-147_exploration.log'), 'w', buffering=1) as log:
    for i in range(3):
        log.write('MAPPING NAV2 GOAL SENT: (%.2f, 0.90), path=1.0 m\n' % (0.8 + i))
        log.write('Mapping navigation result: SUCCEEDED: Nav2 reported arrival\n')
    log.write('MAPPING NAV2 GOAL SENT: (3.20, 0.90), path=1.0 m\n')
    log.write('Mapping navigation result: CANCELED: Safety check invalid\n')
if mode == 'crash':
    sys.exit(3)
if mode == 'hang':
    while not stop:
        time.sleep(0.05)
if mode != 'nomap':
    w, h, px = write_map()
    result = {'map_draft_ready': True, 'navigation_map_authorized': False,
              'map_dimensions': [w, h], 'free_cells': px.count(0), 'occupied_cells': px.count(100),
              'unknown_cells': px.count(255),
              'explorer_terminal_status': 'COVERAGE_LIMITED' if not stop else 'INTERRUPTED'}
else:
    result = {'map_draft_ready': False, 'explorer_terminal_status': 'NO_MAP'}
with open(os.path.join(out, 'R16-147_session_result.json'), 'w') as f:
    json.dump(result, f)
