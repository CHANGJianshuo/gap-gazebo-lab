#!/usr/bin/env python3
"""Update the self-contained HTML notebook and its live JSON, without fabricated metrics."""
import argparse
import datetime as dt
import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DIRECTORY = ROOT / 'docs' / 'progress'

def update(title, message, stage=None, status=None, metrics=None, artifacts=None, assumptions=None):
    target = DIRECTORY / 'progress.json'
    data = json.loads(target.read_text()) if target.exists() else {
        'status': '正在制作', 'stage': 0, 'events': [], 'metrics': {}, 'artifacts': {}}
    now = dt.datetime.now(dt.timezone(dt.timedelta(hours=8))).isoformat(timespec='seconds')
    data.update(updated=now, current=message)
    data['events'].append(dict(time=now, title=title, message=message))
    if stage is not None: data['stage'] = stage
    if status is not None: data['status'] = status
    if metrics: data['metrics'].update(metrics)
    if artifacts: data['artifacts'].update(artifacts)
    if assumptions: data['assumptions'] = assumptions
    payload = json.dumps(data, ensure_ascii=False, indent=2)
    tmp = target.with_suffix('.tmp')
    tmp.write_text(payload + '\n')
    tmp.replace(target)
    page = DIRECTORY / 'index.html'
    html = page.read_text()
    html = re.sub(r'(<script id="snapshot" type="application/json">).*?(</script>)',
                  lambda m: m[1] + payload.replace('<', '\\u003c') + m[2], html, flags=re.S)
    page.write_text(html)
    print(f'{title}: {message}')

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('title')
    parser.add_argument('message')
    parser.add_argument('--stage', type=int)
    parser.add_argument('--status')
    args = parser.parse_args()
    update(args.title, args.message, args.stage, args.status)
