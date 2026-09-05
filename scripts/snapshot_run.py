#!/usr/bin/env python3
"""Record the exact relevant inputs and built binaries for a simulation run."""
import argparse
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
ROOT=Path(__file__).resolve().parents[1]
def snapshot(output,task,when='before_run'):
    run=Path(output);run.mkdir(parents=True,exist_ok=True)
    paths=[ROOT/'电池道具.stp',ROOT/'《2026年挑战赛任务底图》.pdf']
    for part in ['src/mtc_motion_planning','src/mtc_simulation','src/mtc_description','src/mtc_interfaces','scripts']:
        paths.extend(p for p in (ROOT/part).rglob('*') if p.is_file() and p.suffix in ['.cpp','.hpp','.py','.sh','.patch','.yaml','.json','.urdf','.srdf','.action','.srv','.sdf','.xml'])
    paths.extend(ROOT/p for p in ['vendor_ws/install/lib/libgz_ros2_control-system.so','vendor_ws/install/lib/libjoint_trajectory_controller.so','install/mtc_motion_planning/lib/mtc_motion_planning/planner','install/mtc_simulation/lib/libmtc_simulation_system.so'])
    files={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths if p.exists()}
    for p in [ROOT/f'src/mtc_simulation/config/{task}.json',ROOT/f'src/mtc_simulation/worlds/{task}.sdf',ROOT/'src/mtc_description/urdf/s3_latch.urdf',ROOT/'src/mtc_motion_planning/config/controllers.yaml',ROOT/'src/mtc_motion_planning/config/joint_limits.yaml']:
        dest=run/'input_snapshot'/p.relative_to(ROOT);dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,dest)
    commits={name:subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT/'vendor_ws/src'/name,text=True).strip() for name in ['gz_ros2_control','ros2_controllers']}
    report={'capture_time_utc':datetime.now(timezone.utc).isoformat(),'capture_phase':when,'task':task,'sha256':files,'control_source_commits':commits}
    (run/'source_manifest.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('output');p.add_argument('--task',default='sequence');p.add_argument('--when',default='before_run');a=p.parse_args();snapshot(a.output,a.task,a.when)
