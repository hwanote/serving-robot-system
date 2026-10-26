#!/usr/bin/env python3
"""C-4 in a real (headless) browser: drive-test panel, proposals, approve, adjusted view, export.
Needs playwright + Chromium; without them it says so and exits 0.
Run: python3 tools/site_builder/tests/ui_c4.py [screenshot_dir]"""
import os, re, sys, tempfile
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, '..'))
try:
    from playwright.sync_api import sync_playwright
except ImportError:
    print('playwright is not installed: UI test skipped'); sys.exit(0)
from test_c4_server import Srv   # noqa: E402


def check(cond, text):
    print(('ok   ' if cond else 'FAIL ') + text)
    if not cond:
        check.failed = True


check.failed = False


def main():
    shots = sys.argv[1] if len(sys.argv) > 1 else None
    with tempfile.TemporaryDirectory() as tmp:
        s = Srv(tmp)
        errs = []
        try:
            with sync_playwright() as p:
                b = p.chromium.launch()
                pg = b.new_page(viewport={'width': 1280, 'height': 1000})
                pg.on('console', lambda m: errs.append(m.text) if m.type == 'error' else None)
                pg.on('pageerror', lambda e: errs.append(str(e)))
                pg.goto('http://127.0.0.1:%d/' % s.port)
                pg.wait_for_selector('#drive .sum')
                t = pg.inner_text('#drive')
                check('시험 32/32' in t and '실패 2' in t, 'panel shows 32/32 steps and 2 failures')
                check(pg.locator('#drive table.dt tr').count() == 17, 'one row per corridor (16 + header)')
                check(pg.locator('#map line[stroke-width="0.2"]').count() == 16, 'measured result drawn on the map')
                check(pg.locator('#adjprev').count() == 0, 'no apply button before there are proposals')
                pg.click('#propose')
                pg.wait_for_selector('.prop')
                check(pg.locator('.prop').count() == 2, 'two proposals listed')
                check(pg.locator('#adjprev').is_disabled(), 'proposals alone do not enable apply')
                for i in range(2):
                    pg.locator('.prop .btns button', has_text=re.compile(r'^승인$')).first.click()
                    pg.wait_for_function('document.querySelectorAll(".prop.approved").length >= %d' % (i + 1))
                check(pg.locator('.prop.approved').count() == 2, 'both approved')
                pg.click('#adjprev')
                pg.wait_for_selector('#drive .sum >> nth=1')
                check('적용 2개' in pg.inner_text('#drive'), 'adjusted view reports 2 applied')
                check(pg.locator('#map line[stroke-width="0.04"]').count() == 16, 'original corridors kept as dotted grey')
                check(not pg.locator('#adjexport').is_disabled(), 'export enabled when there is no FAIL')
                if shots:
                    os.makedirs(shots, exist_ok=True)
                    pg.screenshot(path=os.path.join(shots, 'c4_adjusted.png'), full_page=True)
                pg.click('#adjexport')
                pg.wait_for_selector('#banner.info')
                check('내보냈어요' in pg.inner_text('#banner'), 'export message shown')
                check(os.path.isfile(os.path.join(s.out, 'adjustments_applied.json')), 'adjustments_applied.json written')
                pg.click('#adjprev')
                check(pg.locator('#map line[stroke-width="0.2"]').count() == 16, 'back to the measured view')
                # a rejected proposal can no longer be applied
                pg.locator('.prop .btns button', has_text='되돌리기').first.click()
                pg.wait_for_function('document.querySelectorAll(".prop.approved").length == 1')
                check(True, 'undo returns a proposal to undecided')
                b.close()
        finally:
            s.close()
    check(not errs, 'no console errors' + ('' if not errs else ': ' + '; '.join(errs)))
    print('FAILED' if check.failed else 'all ok')
    return 1 if check.failed else 0


if __name__ == '__main__':
    sys.exit(main())
