#!/usr/bin/env python3
"""Start only this project's Gazebo/ROS processes and keep their logs together."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import yaml
from runtime_assets import runtime_asset

ROOT=Path(__file__).resolve().parents[1]

def main():
    p=argparse.ArgumentParser();p.add_argument('--task',choices=['basic','sequence'],default='sequence');p.add_argument('--gui',action='store_true');p.add_argument('--run-dir',default='data/runs/live');args=p.parse_args()
    run=(ROOT/args.run_dir).resolve();run.mkdir(parents=True,exist_ok=True);processes=[];files=[];(run/'launcher.pid').write_text(str(os.getpid()));(run/'ready.json').unlink(missing_ok=True)
    def start(name,command):
        f=(run/f'{name}.log').open('w');files.append(f);proc=subprocess.Popen(command,cwd=ROOT,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);processes.append((name,proc));(run/'processes.json').write_text(json.dumps({n:q.pid for n,q in processes},indent=2));print(f'{name}: PID {proc.pid}',flush=True);return proc
    def run_checked(command,timeout=45):
        r=subprocess.run(command,cwd=ROOT,capture_output=True,text=True,timeout=timeout);print(r.stdout[-3000:],flush=True)
        if r.returncode:raise RuntimeError(r.stderr[-3000:] or r.stdout[-3000:])
    def stop(*_,exit_code=0):
        for _,proc in reversed(processes):
            if proc.poll() is None:
                try:os.killpg(proc.pid,signal.SIGINT)
                except ProcessLookupError:pass
        deadline=time.monotonic()+5
        for _,proc in reversed(processes):
            try:proc.wait(timeout=max(.1,deadline-time.monotonic()))
            except subprocess.TimeoutExpired:
                try:os.killpg(proc.pid,signal.SIGTERM)
                except ProcessLookupError:pass
        for f in files:f.close()
        raise SystemExit(exit_code)
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    try:
        robot_path=runtime_asset('src/mtc_description/urdf/s3_latch.urdf',resolve_packages=True)
        robot=robot_path.read_text();params=run/'robot_parameters.yaml';params.write_text(yaml.safe_dump({'robot_state_publisher':{'ros__parameters':{'robot_description':robot,'use_sim_time':True,'publish_frequency':100.}}}))
        start('robot_state_publisher',['ros2','run','robot_state_publisher','robot_state_publisher','--ros-args','--params-file',str(params)])
        cmd=['gz','sim','-s','-v','3',str(runtime_asset(f'src/mtc_simulation/worlds/{args.task}.sdf'))]
        if not args.gui:cmd.append('--headless-rendering')
        start('gazebo',cmd)
        if args.gui:start('gazebo_gui',['gz','sim','-g'])
        start('bridge',['ros2','run','ros_gz_bridge','parameter_bridge',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/overview/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/wrist_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/wrist_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/wrist_camera/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/wrist_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'])
        run_checked(['ros2','run','ros_gz_sim','create','-world','mtc','-name','mtc_s3','-file',str(robot_path)],60)
        spawn=start('controllers',['ros2','run','controller_manager','spawner','joint_state_broadcaster','arm_controller','latch_controller','--controller-manager-timeout','50','--switch-timeout','30'])
        time.sleep(1)
        run_checked(['gz','service','-s','/world/mtc/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','5000','--req','pause: false'])
        if spawn.wait(timeout=65)!=0:raise RuntimeError('Controller activation failed; see controllers.log')
        print('SIMULATION_READY',flush=True);(run/'ready.json').write_text(json.dumps({'task':args.task,'ready':True,'ros_domain_id':os.environ.get('ROS_DOMAIN_ID'),'gz_partition':os.environ.get('GZ_PARTITION')}))
        while True:
            for name,proc in processes:
                if name!='controllers' and proc.poll() is not None:raise RuntimeError(f'{name} stopped with code {proc.returncode}')
            time.sleep(1)
    except Exception as e:
        print(f'ERROR: {e}',file=sys.stderr,flush=True);stop(exit_code=1)

if __name__=='__main__':main()
