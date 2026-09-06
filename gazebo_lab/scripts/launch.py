#!/usr/bin/env python3
"""Launch only this isolated lab. SIGINT stops owned process groups."""
import os,sys,json,time,signal,subprocess,fcntl
from pathlib import Path
R=Path(__file__).resolve().parents[1];O=R/'outputs/live';O.mkdir(parents=True,exist_ok=True)
lock=(O/'launcher.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
procs=[];files=[]
def start(name,cmd):
 f=(O/(name+'.log')).open('w');files.append(f);p=subprocess.Popen(cmd,cwd=R,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);procs.append((name,p));(O/'processes.json').write_text(json.dumps({n:p.pid for n,p in procs},indent=2));print(name,p.pid,flush=True);return p
def run(cmd,timeout=60):
 p=subprocess.run(cmd,capture_output=True,text=True,timeout=timeout)
 if p.returncode:raise RuntimeError(p.stdout[-1500:]+p.stderr[-1500:])
 return p.stdout
def stop(*args):
 signal.signal(signal.SIGINT,signal.SIG_IGN);signal.signal(signal.SIGTERM,signal.SIG_IGN)
 # Signal owned groups even if a ros2 wrapper already exited but left children.
 for sig,grace in [(signal.SIGINT,4),(signal.SIGTERM,2),(signal.SIGKILL,0)]:
  for _,p in reversed(procs):
   try:os.killpg(p.pid,sig)
   except ProcessLookupError:pass
  end=time.monotonic()+grace
  for _,p in reversed(procs):
   try:p.wait(timeout=max(.01,end-time.monotonic()))
   except subprocess.TimeoutExpired:pass
  if grace:time.sleep(max(0,end-time.monotonic()))
 (O/'ready.json').unlink(missing_ok=True)
 raise SystemExit(0 if args else 1)
signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
(O/'launcher.pid').write_text(str(os.getpid()));(O/'ready.json').unlink(missing_ok=True)
try:
 start('rsp',['ros2','run','robot_state_publisher','robot_state_publisher','--ros-args','--params-file',str(R/'config/rsp.yaml')])
 start('gazebo',['gz','sim','-s','-v','3','--headless-rendering',str(R/'config/world.sdf')])
 if '--gui' in sys.argv:start('gazebo_gui',['gz','sim','-g'])
 start('bridge',['ros2','run','ros_gz_bridge','parameter_bridge','/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock','/gap/camera@sensor_msgs/msg/Image[gz.msgs.Image'])
 print(run(['ros2','run','ros_gz_sim','create','-world','gap_lab','-name','panda','-file',str(R/'config/panda.urdf')]),flush=True)
 spawn=start('controllers',['ros2','run','controller_manager','spawner','joint_state_broadcaster','arm_controller','hand_controller','--controller-manager-timeout','90','--switch-timeout','90','--service-call-timeout','60'])
 time.sleep(1);print(run(['gz','service','-s','/world/gap_lab/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','5000','--req','pause: false']),flush=True)
 if spawn.wait(timeout=150):raise RuntimeError('Controllers failed')
 start('move_group',['ros2','run','moveit_ros_move_group','move_group','--ros-args','--params-file',str(R/'config/move_group.yaml')])
 start('adapter',['/usr/bin/python3',str(R/'scripts/ros_adapter.py')])
 # API/graph process is independent of ROS's Python 3.10 ABI.
 env=os.environ.copy();env['PYTHONPATH']=str(R.parent/'graph-as-policy')
 f=(O/'web.log').open('w');files.append(f);p=subprocess.Popen([str(R.parent/'.venv-audit/bin/python'),str(R/'scripts/server.py')],cwd=R,env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);procs.append(('web',p))
 (O/'processes.json').write_text(json.dumps({n:p.pid for n,p in procs},indent=2))
 (O/'ready.json').write_text(json.dumps({'ready':True,'url':'http://127.0.0.1:9433','domain':84,'partition':'gap_lab'}));print('LAB_READY http://127.0.0.1:9433',flush=True)
 while True:
  for n,p in procs:
   if n!='controllers' and p.poll() is not None:raise RuntimeError(f'{n} exited {p.returncode}; inspect {O}/{n}.log')
  time.sleep(1)
except Exception as e:print(str(e),flush=True);stop()
