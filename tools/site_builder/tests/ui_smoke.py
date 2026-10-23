#!/usr/bin/env python3
"""Opens the labelling screen in a real (headless) browser and clicks through it.
Needs playwright + a Chromium; without them it says so and exits 0.
Run: python3 tools/site_builder/tests/ui_smoke.py [screenshot_dir]"""
import os
import sys
import tempfile
import threading

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
try:
    from playwright.sync_api import sync_playwright
except ImportError:
    print('playwright is not installed: UI smoke test skipped')
    sys.exit(0)
from site_builder.server import App, serve   # noqa: E402

REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
DEMO_MAP = os.path.join(REPO, 'maps', 'stores', 'demo_store', 'map.yaml')
EXAMPLE = os.path.join(HERE, '..', 'examples', 'labels_demo.json')


def check(cond, text):
    print(('ok   ' if cond else 'FAIL ') + text)
    if not cond:
        check.failed = True


check.failed = False


def main():
    shots = sys.argv[1] if len(sys.argv) > 1 else None
    with tempfile.TemporaryDirectory() as tmp:
        app = App(DEMO_MAP, os.path.join(tmp, 'labels.json'), os.path.join(tmp, 'generated'), EXAMPLE)
        httpd = serve(app, 0)
        threading.Thread(target=httpd.serve_forever, daemon=True).start()
        url = 'http://127.0.0.1:%d/' % httpd.server_address[1]
        errs = []
        with sync_playwright() as p:
            b = p.chromium.launch()
            pg = b.new_page(viewport={'width': 1280, 'height': 860})
            # the browser logs the one 400 we ask for on purpose (generate with no points)
            pg.on('console', lambda m: errs.append(m.text) if m.type == 'error'
                  and 'status of 400' not in m.text else None)
            pg.on('pageerror', lambda e: errs.append(str(e)))
            pg.on('dialog', lambda d: d.accept())
            pg.goto(url)
            pg.wait_for_selector('#map image')
            check('demo_store' in pg.inner_text('#mapname'), 'map name shown')
            pg.click('#gen')                                  # nothing marked yet
            pg.wait_for_selector('#banner.err')
            check('대기(홈) 점' in pg.inner_text('#banner'), 'no points -> readable error')
            pg.click('#load-demo')
            pg.wait_for_selector('.item')
            check(pg.locator('.item').count() == 6, 'example loads 6 points')
            pg.click('#gen')
            pg.wait_for_selector('.pill.ok')
            check('FAIL 0' in pg.inner_text('.sum'), 'generated without FAIL')
            check(pg.is_enabled('#export'), 'export enabled after a clean run')
            if shots:
                pg.screenshot(path=os.path.join(shots, 'ui_generated.png'))
            pg.click('text=kitchen →')
            check(pg.locator('#map polygon').count() == 1, 'HardStop rectangle drawn for a pose')
            pg.click('button[data-tool=standby]')
            box = pg.locator('#map').bounding_box()
            pg.mouse.click(box['x'] + box['width'] * 0.8, box['y'] + box['height'] * 0.85)
            check(pg.locator('.item').count() == 7, 'click adds a point')
            check(not pg.is_enabled('#export'), 'export disabled while the result is outdated')
            pg.locator('.item input').first.fill('Bad Name')
            pg.locator('.item input').first.dispatch_event('change')
            check('이름' in pg.inner_text('#banner'), 'bad name refused with a message')
            vb0 = pg.get_attribute('#map', 'viewBox')
            pg.click('button[data-tool=move]')
            pg.mouse.move(box['x'] + 300, box['y'] + 300)
            pg.mouse.wheel(0, -400)
            check(pg.get_attribute('#map', 'viewBox') != vb0, 'wheel zooms')
            pg.locator('.item button').last.click()
            pg.click('#gen')
            pg.wait_for_selector('.sum')
            pg.click('#export')
            pg.wait_for_selector('text=내보냈어요')
            check(os.path.isfile(os.path.join(tmp, 'generated', 'waypoints.yaml')), 'export wrote waypoints.yaml')
            pg.reload()
            pg.wait_for_selector('.item')
            check(pg.locator('.item').count() == 6, 'points survive a reload')
            b.close()
        httpd.shutdown()
        check(not errs, 'no console errors %s' % errs)
    print('FAILED' if check.failed else 'all passed')
    return 1 if check.failed else 0


if __name__ == '__main__':
    sys.exit(main())
