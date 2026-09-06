"""Capture the actual browser UI; optionally record an entire real experiment."""
import argparse,time,json
from pathlib import Path
from playwright.sync_api import sync_playwright
R=Path(__file__).resolve().parents[1];p=argparse.ArgumentParser();p.add_argument('--run',action='store_true');a=p.parse_args()
with sync_playwright() as pw:
 browser=pw.chromium.launch(headless=True,args=['--no-sandbox','--proxy-server=direct://','--proxy-bypass-list=*'])
 context=browser.new_context(viewport={'width':1500,'height':1080},record_video_dir=str(R/'outputs/video') if a.run else None,record_video_size={'width':1500,'height':1080} if a.run else None)
 page=context.new_page();errors=[];page.on('pageerror',lambda e:errors.append(str(e)));page.on('console',lambda m:errors.append(m.text) if m.type=='error' else None);page.goto('http://127.0.0.1:9433');page.wait_for_timeout(2000)
 if a.run:
  page.get_by_role('button',name='运行实验',exact=True).click()
  deadline=time.monotonic()+240
  while time.monotonic()<deadline:
   s=page.request.get('http://127.0.0.1:9433/api/state').json()
   if not s['running'] and s['run_id']:break
   page.wait_for_timeout(1000)
  page.wait_for_timeout(2000)
 page.screenshot(path=str(R/'outputs/dashboard.png'),full_page=True)
 (R/'outputs/ui-check.json').write_text(json.dumps({'browser_errors':errors,'title':page.title(),'captured_at':time.time(),'run':a.run},indent=2))
 video=page.video;context.close()
 if video:print(video.path())
 browser.close()
print('Browser errors:',errors)
