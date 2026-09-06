#!/usr/bin/env python3
"""ROS Humble bridge. MoveIt plans; FollowJointTrajectory executes in Gazebo."""
import json,time,threading,math,io,traceback,subprocess
import xml.etree.ElementTree as ET
from trajectory_math import segment,required_scale
import numpy as np
from http.server import ThreadingHTTPServer,BaseHTTPRequestHandler
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState,Image
from std_msgs.msg import String
from std_srvs.srv import SetBool
from geometry_msgs.msg import Pose,PoseStamped
from moveit_msgs.msg import RobotState,CollisionObject,AttachedCollisionObject,PlanningScene,Constraints,JointConstraint,AllowedCollisionEntry
from moveit_msgs.srv import GetPositionIK,GetMotionPlan,GetCartesianPath,ApplyPlanningScene,GetStateValidity
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from PIL import Image as PILImage
R=Path(__file__).resolve().parents[1]
ARM=[f'panda_joint{i}' for i in range(1,8)];FINGERS=['panda_finger_joint1','panda_finger_joint2']

def pose(xyz):
 p=Pose();p.position.x,p.position.y,p.position.z=map(float,xyz);p.orientation.x=1.;p.orientation.w=0.;return p

def wait(f,timeout=30):
 e=threading.Event();f.add_done_callback(lambda _:e.set())
 if not e.wait(timeout):raise RuntimeError('ROS request timed out')
 return f.result()

class Adapter(Node):
 def __init__(self):
  super().__init__('gap_gazebo_adapter');self.joints={};self.world={};self.world_at=0.;self.jpeg=b'';self.image_at=0.;self.cancel=threading.Event();self.goal=None;self.lock=threading.Lock();self.metrics=[];self.unexpected_contacts=[]
  self.create_subscription(JointState,'/joint_states',self.joint_cb,10)
  self.create_subscription(String,'/gap/world',self.world_cb,10)
  self.create_subscription(Image,'/gap/camera',self.image_cb,10)
  self.ik=self.create_client(GetPositionIK,'/compute_ik');self.plan=self.create_client(GetMotionPlan,'/plan_kinematic_path');self.cart=self.create_client(GetCartesianPath,'/compute_cartesian_path');self.scene=self.create_client(ApplyPlanningScene,'/apply_planning_scene');self.valid=self.create_client(GetStateValidity,'/check_state_validity');self.attach=self.create_client(SetBool,'/gap/attach')
  self.arm=ActionClient(self,FollowJointTrajectory,'/arm_controller/follow_joint_trajectory');self.hand=ActionClient(self,FollowJointTrajectory,'/hand_controller/follow_joint_trajectory')
 def joint_cb(self,m):self.joints.update(zip(m.name,m.position))
 def world_cb(self,m):
  self.world=json.loads(m.data);self.world_at=time.monotonic()
  for c in self.world.get('contacts',[]):
   pair=c['a']+' '+c['b']
   unexpected=('panda' in pair and any(n in pair for n in ['barrier','table','floor'])) or ('cube' in pair and 'barrier' in pair)
   if unexpected:
    self.unexpected_contacts.append(c)
    if self.goal:self.goal.cancel_goal_async()

 def image_cb(self,m):
  if m.encoding not in ('rgb8','bgr8'):return
  im=PILImage.frombytes('RGB',(m.width,m.height),bytes(m.data),'raw','RGB' if m.encoding=='rgb8' else 'BGR',m.step)
  f=io.BytesIO();im.save(f,format='JPEG',quality=82);self.jpeg=f.getvalue();self.image_at=time.monotonic()
 def state(self):
  s=RobotState();s.joint_state.name=list(self.joints);s.joint_state.position=[float(x) for x in self.joints.values()];s.is_diff=True;return s
 def fresh(self):
  if time.monotonic()-self.world_at>2 or not all(n in self.joints for n in ARM):raise RuntimeError('Gazebo state unavailable or stale')
  if self.cancel.is_set():raise RuntimeError('CANCELLED')
 def rpc(self,client,req,timeout=30):
  if not client.wait_for_service(timeout_sec=5.):raise RuntimeError('ROS service unavailable: '+client.srv_name)
  return wait(client.call_async(req),timeout)
 def box(self,name,xyz,size):
  c=CollisionObject();c.header.frame_id='world';c.id=name;c.operation=CollisionObject.ADD;s=SolidPrimitive();s.type=SolidPrimitive.BOX;s.dimensions=list(map(float,size));p=Pose();p.position.x,p.position.y,p.position.z=map(float,xyz);p.orientation.w=1.;c.primitives=[s];c.primitive_poses=[p];return c
 def sync_scene(self):
  self.fresh();s=PlanningScene();s.is_diff=True;s.robot_state.is_diff=True
  s.world.collision_objects=[self.box('table',[.55,0,.1],[.75,1,.1]),self.box('floor',[0,0,-.035],[4,4,.05]),self.box('barrier',[.52,0,.265],[.32,.065,.23])]
  if not self.world.get('attached'):s.world.collision_objects.append(self.box('cube',self.world['cube'],[.04]*3))
  sr=ET.parse(R/'config/panda.srdf');pairs={frozenset((e.get('link1'),e.get('link2'))) for e in sr.findall('disable_collisions')}
  pairs.update(frozenset(('cube',n)) for n in ['panda_hand','panda_leftfinger','panda_rightfinger'])
  names=sorted(set(x for pair in pairs for x in pair));s.allowed_collision_matrix.entry_names=names
  for a in names:
   e=AllowedCollisionEntry();e.enabled=[a==b or frozenset((a,b)) in pairs for b in names];s.allowed_collision_matrix.entry_values.append(e)
  req=ApplyPlanningScene.Request();req.scene=s
  if not self.rpc(self.scene,req).success:raise RuntimeError('Planning scene update rejected')
 def execute(self,t,client=None):
  self.fresh();client=client or self.arm
  if not client.wait_for_server(timeout_sec=5.):raise RuntimeError('Trajectory action unavailable')
  g=FollowJointTrajectory.Goal();g.trajectory=t;g.goal_time_tolerance=Duration(sec=3)
  handle=wait(client.send_goal_async(g),10);self.goal=handle
  if not handle.accepted:raise RuntimeError('Controller rejected trajectory')
  future=handle.get_result_async()
  duration=t.points[-1].time_from_start.sec+t.points[-1].time_from_start.nanosec*1e-9
  try:out=wait(future,min(75,max(30,duration*4+10)))
  except Exception:
   handle.cancel_goal_async()
   try:wait(future,5)
   except Exception:pass
   raise
  finally:self.goal=None
  if out.result.error_code!=0 or out.status!=4:raise RuntimeError(f'Controller failure {out.status}: {out.result.error_string}')
  self.fresh();time.sleep(.15)
 def preflight(self,t):
  if len(t.points)<2:raise RuntimeError('Trajectory requires at least two samples')
  times=[p.time_from_start.sec+p.time_from_start.nanosec*1e-9 for p in t.points]
  if abs(times[0])>1e-8 or any(b<=a for a,b in zip(times,times[1:])):raise RuntimeError('Invalid trajectory timestamps')
  for p in t.points:
   if not p.velocities:p.velocities=[0.]*len(p.positions)
   if not p.accelerations:p.accelerations=[0.]*len(p.positions)
  # End at rest, including acceleration, before another policy node takes control.
  for p in (t.points[0],t.points[-1]):p.velocities=[0.]*len(p.positions);p.accelerations=[0.]*len(p.positions)
  segments=[(segment(a.positions,a.velocities,a.accelerations,b.positions,b.velocities,b.accelerations,T),T) for a,b,T in zip(t.points,t.points[1:],[b-a for a,b in zip(times,times[1:])])]
  factor,peaks=required_scale(segments)
  for p,stamp in zip(t.points,times):
   ns=round(stamp*factor*1e9);p.time_from_start=Duration(sec=ns//10**9,nanosec=ns%10**9);p.velocities=[v/factor for v in p.velocities];p.accelerations=[a/factor**2 for a in p.accelerations]
  # Validate the controller's polynomial path, not only planner waypoints.
  # This is discrete collision checking at <=20 ms of native path time;
  # no continuous-collision or hardware torque guarantee is claimed.
  checks=0
  for coeff,T in segments:
   for u in np.linspace(0.,1.,max(3,int(math.ceil(T/.02))+1)):
    self.fresh();q=np.polynomial.polynomial.polyval(u,coeff);req=GetStateValidity.Request();req.group_name='panda_arm';req.robot_state=self.state();vals=dict(zip(req.robot_state.joint_state.name,req.robot_state.joint_state.position));vals.update(zip(t.joint_names,q));req.robot_state.joint_state.position=[float(vals[n]) for n in req.robot_state.joint_state.name]
    res=self.rpc(self.valid,req)
    # Supporting contact at the lift boundary has sub-mm FCL/Gazebo error.
    # A fixed 0.1 mm tolerance applies ONLY to cube/table; every other pair stays strict.
    support_only=bool(res.contacts) and all({c.contact_body_1,c.contact_body_2}=={'table','cube'} and c.depth<=.0001 for c in res.contacts)
    if not res.valid and not support_only:raise RuntimeError('INTERPOLATED_PATH_COLLISION '+str([(c.contact_body_1,c.contact_body_2,c.depth) for c in res.contacts]))
    checks+=1
  return {'time_scale':factor,'polynomial_peak_limits':peaks,'collision_samples':checks,'duration_s':times[-1]*factor}
 def move(self,xyz,mode='cartesian'):
  self.sync_scene();t0=time.monotonic();p=pose(xyz)
  if mode=='cartesian':
   req=GetCartesianPath.Request();req.header.frame_id='world';req.start_state=self.state();req.group_name='panda_arm';req.link_name='panda_tcp';req.waypoints=[p];req.max_step=.005;req.jump_threshold=2.;req.avoid_collisions=True
   res=self.rpc(self.cart,req);fraction=res.fraction
   if fraction<.999:raise RuntimeError(json.dumps({'code':'CARTESIAN_BLOCKED','fraction':fraction,'requested_target':xyz,'world':self.world,'suggestion':'insert lift and transfer at safe clearance; keep collision checks enabled'}))
   t=res.solution.joint_trajectory
   # Keep the MoveIt service's native TOTG timing and derivatives.
  elif mode=='free':
   req=GetPositionIK.Request();req.ik_request.group_name='panda_arm';req.ik_request.ik_link_name='panda_tcp';req.ik_request.pose_stamped.header.frame_id='world';req.ik_request.pose_stamped.pose=p;req.ik_request.robot_state=self.state();req.ik_request.avoid_collisions=True;req.ik_request.timeout=Duration(sec=2)
   ik=self.rpc(self.ik,req)
   if ik.error_code.val!=1:raise RuntimeError(f'IK_FAILED {ik.error_code.val} target={xyz}')
   req=GetMotionPlan.Request();m=req.motion_plan_request;m.group_name='panda_arm';m.planner_id='RRTConnect';m.allowed_planning_time=5.;m.num_planning_attempts=5;m.start_state=self.state();m.max_velocity_scaling_factor=.25;m.max_acceleration_scaling_factor=.2
   c=Constraints();vals=dict(zip(ik.solution.joint_state.name,ik.solution.joint_state.position))
   for n in ARM:
    j=JointConstraint();j.joint_name=n;j.position=vals[n];j.tolerance_above=.001;j.tolerance_below=.001;j.weight=1.;c.joint_constraints.append(j)
   m.goal_constraints=[c];res=self.rpc(self.plan,req)
   if res.motion_plan_response.error_code.val!=1:raise RuntimeError(f'PLANNING_FAILED {res.motion_plan_response.error_code.val}')
   t=res.motion_plan_response.trajectory.joint_trajectory
  else:raise ValueError('Unknown motion mode')
  if not t.points:raise RuntimeError('Empty trajectory')
  validation=self.preflight(t);self.execute(t);err=math.dist(self.world['tcp'],xyz)
  data={'target':xyz,'tcp':self.world['tcp'],'position_error_m':err,'waypoints':len(t.points),'wall_s':time.monotonic()-t0,'mode':mode,'validation':validation}
  self.metrics.append(data)
  if err>.02:raise RuntimeError('TRACKING_FAILED '+json.dumps(data))
  return data
 def gripper(self,close):
  self.fresh();was_attached=self.world.get('attached',False);t=JointTrajectory();t.joint_names=FINGERS;p=JointTrajectoryPoint();p.positions=[.021 if close else .04]*2;p.time_from_start=Duration(sec=1);t.points=[p];self.execute(t,self.hand)
  req=SetBool.Request();req.data=close;res=self.rpc(self.attach,req)
  if not res.success:raise RuntimeError('GRASP_FAILED '+res.message)
  time.sleep(.15)
  if not close and not was_attached:self.sync_scene();return {'attached':False,'detail':res.message}
  s=PlanningScene();s.is_diff=True;s.robot_state.is_diff=True;a=AttachedCollisionObject();a.link_name='panda_hand';a.touch_links=['panda_hand','panda_leftfinger','panda_rightfinger']
  if close:
   a.object=self.box('cube',[0,0,.1034],[.04]*3);a.object.header.frame_id='panda_hand'
  else:a.object.id='cube';a.object.operation=CollisionObject.REMOVE
  s.robot_state.attached_collision_objects=[a];req=ApplyPlanningScene.Request();req.scene=s
  if not self.rpc(self.scene,req).success:raise RuntimeError('Attachment scene update failed')
  self.sync_scene();return {'attached':self.world['attached'],'detail':res.message}
 def act(self,op,args):
  if op=='observe':self.fresh();return dict(self.world)
  if op=='move':return self.move(**args)
  if op=='gripper':return self.gripper(**args)
  if op=='verify':
   self.fresh();first=list(self.world['cube']);time.sleep(.5);self.fresh();w=dict(self.world);ok=math.dist(w['cube'],w['target'])<.035 and not w['attached'] and math.dist(first,w['cube'])<.002 and not self.unexpected_contacts
   if not ok:raise RuntimeError('GOAL_NOT_MET '+json.dumps(w))
   return {'success':True,'world':w,'settle_displacement_m':math.dist(first,w['cube']),'unexpected_contacts':len(self.unexpected_contacts)}
  if op=='reset':
   self.cancel.clear();self.fresh()
   # Episode reset is explicit and is NOT registered as a GaP/LLM skill.
   self.gripper(False)
   tcp=list(self.world['tcp'])
   if tcp[2]<.49:self.move([tcp[0],tcp[1],.49],'cartesian')
   self.move([.307,0,.487],'free')
   cmd=['gz','service','-s','/world/gap_lab/set_pose','--reqtype','gz.msgs.Pose','--reptype','gz.msgs.Boolean','--timeout','5000','--req','name: "cube", position: {x: 0.5, y: -0.24, z: 0.171}, orientation: {w: 1}']
   result=subprocess.run(cmd,capture_output=True,text=True,timeout=8)
   if result.returncode or 'true' not in result.stdout:raise RuntimeError('Scene reset failed')
   time.sleep(.7);self.unexpected_contacts=[];self.metrics=[];self.sync_scene();return {'reset':True,'world':self.world}
  raise ValueError('Unknown operation')

class Handler(BaseHTTPRequestHandler):
 def log_message(self,*a):pass
 def send(self,code,data,kind='application/json'):
  if kind=='application/json':data=json.dumps(data).encode()
  self.send_response(code);self.send_header('Content-Type',kind);self.send_header('Cache-Control','no-store');self.end_headers();self.wfile.write(data)
 def do_GET(self):
  if self.path=='/camera.jpg':return self.send(200,A.jpeg,'image/jpeg')
  if self.path=='/state':return self.send(200,{'world':A.world,'joints':A.joints,'fresh':time.monotonic()-A.world_at<2,'camera_fresh':time.monotonic()-A.image_at<2,'metrics':A.metrics[-30:],'unexpected_contacts':A.unexpected_contacts[-20:]})
  self.send(404,{'error':'Not found'})
 def do_POST(self):
  try:
   d=json.loads(self.rfile.read(int(self.headers.get('Content-Length',0))))
   if self.path=='/stop':
    A.cancel.set()
    if A.goal:A.goal.cancel_goal_async()
    return self.send(200,{'stopping':True})
   if self.path=='/resume':A.cancel.clear();return self.send(200,{'resumed':True})
   if self.path!='/act':return self.send(404,{'error':'Not found'})
   if not A.lock.acquire(blocking=False):return self.send(409,{'error':'Robot busy'})
   try:out=A.act(d['op'],d.get('args',{}))
   finally:A.lock.release()
   self.send(200,out)
  except Exception as e:traceback.print_exc();self.send(422,{'error':str(e)})

if __name__=='__main__':
 rclpy.init();A=Adapter();ex=MultiThreadedExecutor(num_threads=4);ex.add_node(A);threading.Thread(target=ex.spin,daemon=True).start();print('ROS adapter http://127.0.0.1:9431',flush=True)
 try:ThreadingHTTPServer(('127.0.0.1',9431),Handler).serve_forever()
 finally:ex.shutdown();A.destroy_node();rclpy.shutdown()
