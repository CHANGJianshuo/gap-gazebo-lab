#!/usr/bin/env python3
"""Ground-truth integration harness. Production task scheduling belongs to SJM.

The planning server returns a checked trajectory. This small harness supplies
known targets, sends it to ros2_control, and records what actually happened.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint
from mtc_interfaces.action import PlanMotion
from mtc_interfaces.srv import Latch
from rosidl_runtime_py.convert import message_to_ordereddict

ROOT=Path(__file__).resolve().parents[1]
JOINTS=['shoulder_joint','upperArm_joint','foreArm_joint','wrist1_joint','wrist2_joint','wrist3_joint']

def multiply(a,b):
    x,y,z,w=a;X,Y,Z,W=b
    return np.array([w*X+x*W+y*Z-z*Y,w*Y-x*Z+y*W+z*X,w*Z+x*Y-y*X+z*W,w*W-x*X-y*Y-z*Z])

def rotate(q,v):
    return multiply(multiply(q,[*v,0]),[-q[0],-q[1],-q[2],q[3]])[:3]

def pose(position,quaternion):
    p=PoseStamped();p.header.frame_id='world'
    p.pose.position.x,p.pose.position.y,p.pose.position.z=map(float,position)
    p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w=map(float,quaternion)
    return p

class Demo(Node):
    def __init__(self,args):
        super().__init__('mtc_planning_demo',parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.args=args;self.run=ROOT/args.output;self.run.mkdir(parents=True,exist_ok=True)
        self.config=json.loads((ROOT/f'src/mtc_simulation/config/{args.task}.json').read_text())
        self.joints=None;self.truth={};self.contacts={};self.phase='启动';self.active=False;self.rows=[];self.plans=[];self.placements=[];self.recorded=[];self.events=[];self.started=time.monotonic();self.started_sim=None;self.plan_index=0
        self.create_subscription(JointState,'/joint_states',self.on_joints,qos_profile_sensor_data)
        self.create_subscription(TFMessage,'/simulation/ground_truth',self.on_truth,10)
        self.create_subscription(String,'/simulation/contact_report',self.on_contacts,10)
        self.create_subscription(JointTrajectoryControllerState,'/arm_controller/controller_state',self.on_control,10)
        self.planner=ActionClient(self,PlanMotion,'/motion_planning/plan')
        self.arm=ActionClient(self,FollowJointTrajectory,'/arm_controller/follow_joint_trajectory')
        self.fingers=ActionClient(self,FollowJointTrajectory,'/latch_controller/follow_joint_trajectory')
        self.lock=self.create_client(Latch,'/simulation/latch')

    def on_joints(self,msg):self.joints=msg
    def on_contacts(self,msg):self.contacts=json.loads(msg.data)
    def on_truth(self,msg):
        for t in msg.transforms:
            a=t.transform.translation;b=t.transform.rotation
            self.truth[t.child_frame_id]=(np.array([a.x,a.y,a.z]),np.array([b.x,b.y,b.z,b.w]))
        if self.started_sim is not None and 'tcp' in self.truth:
            stamp=msg.transforms[0].header.stamp
            row={'sim_time':stamp.sec+stamp.nanosec*1e-9,'phase':self.phase,'tcp':self.truth['tcp'][0].tolist()}
            row['objects']={k:v[0].tolist() for k,v in self.truth.items() if k.startswith('battery_')};self.recorded.append(row)

    def on_control(self,msg):
        if not self.active:return
        t=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if len(msg.error.positions)!=6:return
        actual=msg.feedback if msg.feedback.positions else msg.actual
        ref=msg.reference if msg.reference.positions else msg.desired
        row=[t,self.phase]+list(msg.error.positions)+list(actual.positions)+list(actual.velocities)+list(ref.positions)
        row+=list(msg.output.effort) if len(msg.output.effort)==6 else [float('nan')]*6
        self.rows.append(row)

    def spin_until(self,predicate,timeout=60):
        deadline=time.monotonic()+timeout
        while not predicate():
            if time.monotonic()>deadline:raise TimeoutError(f'{self.phase}: timed out after {timeout}s')
            rclpy.spin_once(self,timeout_sec=.05)

    def future(self,f,timeout=60):
        self.spin_until(f.done,timeout)
        return f.result()

    def sim_time(self):return self.get_clock().now().nanoseconds*1e-9
    def dwell(self,seconds):
        start=self.sim_time();self.spin_until(lambda:self.sim_time()-start>=seconds,max(60,seconds*50))

    def status(self,text):
        self.phase=text
        row={'phase':text,'sim_time':self.sim_time(),'wall_elapsed':time.monotonic()-self.started}
        self.events.append(row)
        for p in [self.run/'demo_status.json',ROOT/'data/runs/live/demo_status.json']:
            p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(row,ensure_ascii=False,indent=2))
        print(f'[{self.sim_time():7.3f}s] {text}',flush=True)

    def ready(self):
        self.status('等待仿真和规划服务')
        self.spin_until(lambda:self.joints is not None and len(self.truth)>=6,60)
        for client in [self.planner,self.arm,self.fingers]:
            if not client.wait_for_server(timeout_sec=30):raise RuntimeError('Action server unavailable')
        if not self.lock.wait_for_service(timeout_sec=10):raise RuntimeError('Latch service unavailable')
        self.status('等待关节稳定，随后开始计时');self.dwell(.2);stable=[None]
        def settled():
            velocity=dict(zip(self.joints.name,self.joints.velocity))
            if any(abs(velocity.get(n,1.))>.005 for n in JOINTS):stable[0]=None;return False
            if stable[0] is None:stable[0]=self.sim_time()
            return self.sim_time()-stable[0]>=.15
        self.spin_until(settled,90);self.started_sim=self.sim_time()

    def plan(self,target,mode='free',touching='',label='运动',joint_goal=None):
        self.status(label+' · 规划')
        g=PlanMotion.Goal();g.mode=mode;g.touching_object=touching;g.velocity_scaling=self.args.speed;g.acceleration_scaling=1.;g.planning_timeout=12.;g.seed=20260905
        if target is not None:g.target_pose=target
        if joint_goal is not None:g.goal_state.name=JOINTS;g.goal_state.position=list(joint_goal)
        handle=self.future(self.planner.send_goal_async(g),10)
        if not handle.accepted:raise RuntimeError('Planner rejected the goal')
        try:result=self.future(handle.get_result_async(),90).result
        except TimeoutError: self.future(handle.cancel_goal_async(),5);raise
        report=json.loads(result.diagnostics_json);report.update(label=label,mode=mode,touching_object=touching,target=message_to_ordereddict(g.target_pose));self.plans.append(report);self.plan_index+=1
        (self.run/f'plan_{self.plan_index:02d}.json').write_text(json.dumps({'request':message_to_ordereddict(g),'result':message_to_ordereddict(result)},ensure_ascii=False,indent=2))
        print(f"  {result.reason}; plan={result.planning_time:.3f}s, trajectory={result.duration:.3f}s, torque={result.maximum_torque_ratio:.3f}, selected={report.get('selected')}",flush=True)
        if not result.success:raise RuntimeError(f'{label}: {result.reason}')
        return result.trajectory.joint_trajectory

    def execute(self,trajectory,label):
        self.status(label+' · 执行');self.active=True
        g=FollowJointTrajectory.Goal();g.trajectory=trajectory
        handle=self.future(self.arm.send_goal_async(g),10)
        if not handle.accepted:raise RuntimeError('Controller rejected trajectory')
        duration=trajectory.points[-1].time_from_start.sec+trajectory.points[-1].time_from_start.nanosec*1e-9
        try:res=self.future(handle.get_result_async(),max(120,duration*50)).result
        except TimeoutError:self.future(handle.cancel_goal_async(),5);raise
        self.active=False
        if res.error_code!=0:raise RuntimeError(f'Controller error {res.error_code}: {res.error_string}')
        self.dwell(.10)

    def move(self,target,mode='free',touching='',label='运动'):
        trajectory=self.plan(target,mode,touching,label)
        if not self.args.plan_only:self.execute(trajectory,label)

    def finger(self,closed):
        self.status('闭合半圆锁扣' if closed else '打开半圆锁扣')
        g=FollowJointTrajectory.Goal();g.trajectory.joint_names=['latch_left_joint','latch_right_joint'];p=JointTrajectoryPoint();p.positions=[0.,0.] if closed else [.022,.022];p.velocities=[0.,0.];p.time_from_start.sec=0;p.time_from_start.nanosec=600000000;g.trajectory.points=[p]
        handle=self.future(self.fingers.send_goal_async(g),10)
        if not handle.accepted:raise RuntimeError('Latch controller rejected goal')
        result=self.future(handle.get_result_async(),60).result
        if result.error_code!=0:raise RuntimeError(result.error_string)
        self.dwell(.15)

    def latch(self,obj,close):
        request=Latch.Request();request.object_id=obj;request.close=close
        result=self.future(self.lock.call_async(request),10)
        print(' ',result.message,flush=True)
        if not result.success:raise RuntimeError(result.message)
        self.dwell(.05)

    def transfer(self,letter,target_name):
        obj='battery_'+letter;position,rotation=self.truth[obj]
        grasp=position+rotate(rotation,[0,0,.074]);orientation=multiply(rotation,[1,0,0,0])
        above=grasp+[0,0,.11]
        self.move(pose(above,orientation),label=f'{letter}：到达把手上方')
        if self.args.plan_only:return
        # Read the actual resting pose again, then approach vertically.
        position,rotation=self.truth[obj];grasp=position+rotate(rotation,[0,0,.074]);orientation=multiply(rotation,[1,0,0,0])
        self.move(pose(grasp,orientation),'cartesian',obj,f'{letter}：垂直接近把手')
        self.finger(True);self.latch(obj,True)
        tcp,rot=self.truth['tcp'];self.move(pose(tcp+[0,0,.12],rot),'cartesian',obj,f'{letter}：锁住后垂直抬升')
        if self.args.task=='basic':self.status('保持举起 2 秒');self.dwell(2.)
        # Keep the captured relative transform: placement position derives from actual attachment.
        tcp,rot=self.truth['tcp'];body,bodyrot=self.truth[obj]
        destination=np.array(self.config['targets'][target_name]);landing_tcp=tcp+(destination-body)+[0,0,.0005]
        self.move(pose(landing_tcp+[0,0,.12],rot),'free','',f'{letter}：搬运至 {target_name.replace("_inner", "")} 上方')
        self.move(pose(landing_tcp,rot),'cartesian',obj,f'{letter}：垂直放下')
        self.latch(obj,False);self.finger(False)
        tcp,rot=self.truth['tcp'];self.move(pose(tcp+[0,0,.12],rot),'cartesian',obj,f'{letter}：打开后垂直脱离')
        self.status(f'{letter}：释放后稳定观察 3 秒');before=self.truth[obj][0].copy();self.dwell(3.);after=self.truth[obj][0].copy()
        row={'object':obj,'target':target_name,'requested_xyz':destination.tolist(),'actual_xyz':after.tolist(),'xy_error_m':float(np.linalg.norm(after[:2]-destination[:2])),'drift_over_3s_m':float(np.linalg.norm(after-before)),'upright_cosine':float(rotate(self.truth[obj][1],[0,0,1])[2])}
        row['success']=row['xy_error_m']<.005 and row['drift_over_3s_m']<.002 and row['upright_cosine']>.995
        self.placements.append(row);print('  placement',json.dumps(row),flush=True)
        if not row['success']:raise RuntimeError('Placement acceptance failed')

    def save(self,success,error=''):
        headers=['sim_time','phase']+[f'{field}_{j}' for field in ['error','q','v','reference','command_effort'] for j in JOINTS]
        with (self.run/'tracking.csv').open('w',newline='') as f:
            writer=csv.writer(f);writer.writerow(headers);writer.writerows(self.rows)
        (self.run/'ground_truth_trace.json').write_text(json.dumps(self.recorded))
        (self.run/'events.json').write_text(json.dumps(self.events,ensure_ascii=False,indent=2))
        (self.run/'contacts.json').write_text(json.dumps(self.contacts,indent=2))
        arr=np.asarray([r[2:8] for r in self.rows]) if self.rows else np.zeros((0,6))
        report={'success':success,'error':error,'task':self.args.task,'demo_order':self.args.order,'plan_only':self.args.plan_only,'wall_duration_s':time.monotonic()-self.started,'simulation_duration_s':None if self.started_sim is None else self.sim_time()-self.started_sim,'total_planned_motion_s':sum(p.get('duration',0) for p in self.plans),'total_planning_wall_s':sum(p.get('planning_time',0) for p in self.plans),'max_tracking_error_rad':None if len(arr)==0 else float(np.abs(arr).max()),'rms_tracking_error_rad':None if len(arr)==0 else float(np.sqrt(np.mean(arr**2))),'controller_samples':len(self.rows),'placements':self.placements,'plans':self.plans,'assumptions':self.config['assumptions']}
        unexpected={k:v for k,v in self.contacts.get('pairs',{}).items() if v['kind']=='unexpected'}
        report['contacts']=self.contacts;report['unexpected_contact_pairs']=len(unexpected)
        report['contact_monitor_present']=bool(self.contacts.get('physics_steps_observed',0))
        if unexpected:report['success']=False;report['error']='Unexpected physical contacts recorded'
        if not self.args.plan_only and not report['contact_monitor_present']:report['success']=False;report['error']='Physics contact monitor unavailable'
        efforts=np.asarray([r[26:32] for r in self.rows]) if self.rows else np.zeros((0,6))
        report['max_command_effort_ratio']=None if len(efforts)==0 else float(np.nanmax(np.abs(efforts)/np.array([49,49,39,9.8,9.8,9.8])))
        (self.run/'summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2));print(f'RESULT: {self.run}/summary.json',flush=True)

def main():
    p=argparse.ArgumentParser();p.add_argument('--task',choices=['basic','sequence'],default='sequence');p.add_argument('--order',default='CAB');p.add_argument('--output',default='data/runs/commissioning');p.add_argument('--speed',type=float,default=1.);p.add_argument('--plan-only',action='store_true');args=p.parse_args()
    if not args.order or any(x not in 'ABCD' for x in args.order) or len(set(args.order))!=len(args.order):p.error('order must contain unique A/B/C/D letters')
    if args.task=='sequence' and len(args.order)>3:p.error('sequence demo has three targets')
    rclpy.init();node=Demo(args);ok=False;error=''
    try:
        node.ready()
        for i,letter in enumerate(args.order[:1] if args.task=='basic' else args.order):
            node.transfer(letter,'T0' if args.task=='basic' else f'P{i+1}_inner')
            if args.plan_only:break
        node.status('规划测试通过' if args.plan_only else '演示完成 · 所有放置通过检查');ok=True
    except (Exception,KeyboardInterrupt) as e:error=str(e) or 'Interrupted';node.status('停止：'+error);print(error,flush=True)
    finally:node.save(ok,error);node.destroy_node();rclpy.try_shutdown()
    raise SystemExit(0 if ok else 1)

if __name__=='__main__':main()
