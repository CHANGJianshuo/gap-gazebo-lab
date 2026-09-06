#!/usr/bin/env python3
"""Create a compact evidence file from a completed run (never infer success)."""
import argparse,json,math
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('run_id');a=p.parse_args();R=Path(__file__).resolve().parents[1]
r=R/'outputs/runs'/a.run_id;s=json.loads((r/'summary.json').read_text());events=s['events']
verified=[e for e in events if e['kind']=='verified_success'];motions=[e['outputs'] for e in events if e['kind']=='skill_output' and 'validation' in e['outputs']]
d={'run_id':a.run_id,'success':s['phase']=='success' and bool(verified),'provider':s['provider'],'versions':[{'index':v['index'],'status':v['status'],'hash':v['hash'],'failed_node':(v.get('feedback') or {}).get('node')} for v in s['versions']],'elapsed_wall_s':events[-1]['time']-events[0]['time'],'motion_count':len(motions),'collision_samples':sum(m['validation']['collision_samples'] for m in motions),'max_endpoint_error_m':max((m['position_error_m'] for m in motions),default=None),'final':verified[-1]['result'] if verified else None}
if motions:d['commanded_polynomial_peaks']={k:max(m['validation']['polynomial_peak_limits'][k] for m in motions) for k in ['velocity','acceleration','jerk']}
(R/'outputs/verification.json').write_text(json.dumps(d,ensure_ascii=False,indent=2));print(json.dumps(d,ensure_ascii=False,indent=2))
