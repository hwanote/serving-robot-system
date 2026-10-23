"""Sample folders that some tests read: out/gen_demo and out/drive_demo.

out/ is not stored in git, so these are created when they are missing, with the same
two commands as step 0 of DRIVE_TEST_GUIDE.md (generate, then drive plan).
"""
import os
import subprocess
import sys

TOOL = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
REPO = os.path.abspath(os.path.join(TOOL, '..', '..'))


def ensure_demo_fixtures():
    gen = os.path.join(TOOL, 'out', 'gen_demo')
    drive = os.path.join(TOOL, 'out', 'drive_demo')
    if os.path.isfile(os.path.join(gen, 'waypoints.yaml')) and os.path.isfile(os.path.join(drive, 'tour.json')):
        return
    steps = (
        ['-m', 'site_builder.cli', 'generate',
         '--map', os.path.join(REPO, 'maps', 'stores', 'demo_store', 'map.yaml'),
         '--labels', os.path.join(TOOL, 'examples', 'labels_demo.json'), '--out-dir', gen],
        ['-m', 'site_builder.drive', 'plan', '--candidate', gen, '--out-dir', drive],
    )
    for args in steps:
        r = subprocess.run([sys.executable] + args, cwd=TOOL, capture_output=True, text=True)
        if r.returncode != 0:
            raise RuntimeError('could not create the sample folders (%s):\n%s%s'
                               % (' '.join(args[:3]), r.stdout, r.stderr))
