"""Browser map-bound route editing with disposable synthetic map. No ROS motion."""
from pathlib import Path
import json,shutil,sys,time
from playwright.sync_api import sync_playwright
REPO=Path(__file__).resolve().parents[1];sys.path.insert(0,str(REPO))
from workbench.maps import Maps
from workbench.runtime import ROOT
m=Maps(ROOT)._save(40,40,bytes([254])*1600,dict(resolution=.2,origin=[-4.,-4.,0.],occupied_thresh=.65,free_thresh=.196,negate=0),dict(test_fixture=True,parent=None))
before={p.parent.name for p in (ROOT/'routes').glob('*/route.json')};report=REPO/'docs/evidence/field_loop_20261023'
try:
 with sync_playwright() as p:
    browser=p.chromium.launch(executable_path='/usr/bin/chromium',headless=True,args=['--no-sandbox']);page=browser.new_page(viewport={'width':1440,'height':1100});errors=[]
    page.on('pageerror',lambda e:errors.append(str(e)))
    page.goto('http://127.0.0.1:8765');page.wait_for_timeout(1200)
    for tab in ['robot','calibration','mapping','maps','navigation','diagnostics']:
        page.locator(f'[data-page="{tab}"]').click();page.wait_for_timeout(150)
    page.screenshot(path=str(report/'workbench_diagnostics.png'),full_page=True)
    page.locator('[data-page="navigation"]').click();page.select_option('#route-map',m['id']);page.wait_for_timeout(300)
    canvas=page.locator('#route-canvas');box=canvas.bounding_box()
    canvas.click(position={'x':box['width']*.35,'y':box['height']*.4});canvas.click(position={'x':box['width']*.65,'y':box['height']*.6})
    page.fill('#route-name','SYNTHETIC_BROWSER_TEST');page.click('#route-save');page.wait_for_timeout(500)
    routes=[json.loads(p.read_text()) for p in (ROOT/'routes').glob('*/route.json') if p.parent.name not in before]
    assert len(routes)==1 and len(routes[0]['points'])==2,routes
    assert routes[0]['map_sha256']==m['sha256'];assert routes[0]['status']=='DRAFT'
    page.screenshot(path=str(report/'workbench_routes.png'),full_page=True)
    assert not errors,errors
    (report/'browser.json').write_text(json.dumps(dict(state='PASS',scope='SOFTWARE_BROWSER_ONLY',tabs=6,route=routes[0],javascript_errors=errors,physical_motion='NOT_RUN'),indent=2))
    browser.close()
finally:
 shutil.rmtree(Path(m['yaml']).parent)
 for p in (ROOT/'routes').glob('*/route.json'):
    if p.parent.name not in before:shutil.rmtree(p.parent)
