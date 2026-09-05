#!/usr/bin/env python3
"""Reproducible, isolated end-to-end Gazebo run and video recording."""
import argparse
from datetime import datetime
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from snapshot_run import snapshot
from video_utils import probe_video

ROOT=Path(__file__).resolve().parents[1]
def main():
    p=argparse.ArgumentParser();p.add_argument('--task',choices=['basic','sequence'],default='sequence');p.add_argument('--order',default='CAB');p.add_argument('--run-id');p.add_argument('--no-video',action='store_true');p.add_argument('--verify',action='store_true');args=p.parse_args()
    run_id=args.run_id or f'{args.task}_{datetime.now():%Y%m%d_%H%M%S}'
    if Path(run_id).name!=run_id:p.error('run-id must be a simple directory name')
    run=ROOT/'data/runs'/run_id
    if run.exists() and any(run.iterdir()):p.error(f'Run directory already contains results: {run}')
    run.mkdir(parents=True,exist_ok=True);video=ROOT/'data/videos'/f'{run_id}.mp4';video.parent.mkdir(parents=True,exist_ok=True)
    snapshot(run,args.task)
    lock=Path(f'/tmp/mtc_demo_{os.environ.get("ROS_DOMAIN_ID","73")}.lock').open('w')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:raise SystemExit('Another project demo is already running in this ROS domain')
    processes=[];files=[]
    def start(name,command):
        log=(run/f'{name}_launcher.log').open('w');files.append(log)
        proc=subprocess.Popen(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);processes.append((name,proc));print(name,proc.pid,flush=True);return proc
    def stop(proc):
        if proc.poll() is not None:return
        try:os.killpg(proc.pid,signal.SIGINT)
        except ProcessLookupError:return
        try:proc.wait(timeout=12)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid,signal.SIGTERM)
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
    def wait_file(path,proc,timeout):
        deadline=time.monotonic()+timeout
        while not path.exists():
            if proc.poll() is not None:raise RuntimeError(f'{path.name} unavailable: process exited with {proc.returncode}')
            if time.monotonic()>deadline:raise TimeoutError(f'Waiting for {path}')
            time.sleep(.2)
    interrupted=False
    def interrupt(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,interrupt)
    code=1
    try:
        sim=start('simulation',[sys.executable,'scripts/launch_sim.py','--task',args.task,'--run-dir',str(run)])
        wait_file(run/'ready.json',sim,100)
        start('planning',[sys.executable,'scripts/launch_planner.py','--task',args.task,'--run-dir',str(run)])
        recorder=None
        if not args.no_video:
            recorder=start('recording',[sys.executable,'scripts/capture_frames.py','--output',str(video),'--ready-file',str(run/'recording.ready'),'--status-file',str(run/'demo_status.json')])
            wait_file(run/'recording.ready',recorder,45)
        demo=start('demo',[sys.executable,'scripts/demo.py','--task',args.task,'--order',args.order,'--output',str(run)])
        last_phase=''
        while demo.poll() is None:
            for name,proc in processes:
                if proc!=demo and proc.poll() is not None:raise RuntimeError(f'{name} exited unexpectedly')
            try:
                phase=json.loads((run/'demo_status.json').read_text())['phase']
                if phase!=last_phase:print(phase,flush=True);last_phase=phase
            except (OSError,json.JSONDecodeError):pass
            time.sleep(.5)
        code=demo.returncode
        if recorder:stop(recorder)
        if recorder and recorder.returncode!=0:raise RuntimeError('Recorder did not finish successfully')
        result=json.loads((run/'summary.json').read_text())
        if not result['success']:code=1
        if video.exists():
            metadata={'path':str(video.relative_to(ROOT)),'sha256':hashlib.sha256(video.read_bytes()).hexdigest(),'bytes':video.stat().st_size,'probe':probe_video(video)}
            (run/'video.json').write_text(json.dumps(metadata,indent=2))
        if args.verify and code==0:
            verification=start('verification',[sys.executable,'scripts/check_rejections.py','--task',args.task,'--output',str(run/'rejections.json')])
            if verification.wait(timeout=150)!=0:code=1
        print(f'Completed: success={code==0}\nReport: {run}/summary.json\nVideo: {video}',flush=True)
    except KeyboardInterrupt:interrupted=True;print('Interrupted; stopping only this run’s processes.',flush=True)
    finally:
        for _,proc in reversed(processes):stop(proc)
        for f in files:f.close()
        lock.close()
    return 130 if interrupted else code

if __name__=='__main__':raise SystemExit(main())
