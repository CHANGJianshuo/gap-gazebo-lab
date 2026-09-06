import rclpy,json,urllib.request
from rclpy.node import Node
from moveit_msgs.srv import GetCartesianPath,GetStateValidity
from geometry_msgs.msg import Pose
rclpy.init();n=Node('gap_timing_probe');c=n.create_client(GetCartesianPath,'/compute_cartesian_path');c.wait_for_service(timeout_sec=5)
o=urllib.request.build_opener(urllib.request.ProxyHandler({}));s=json.load(o.open('http://127.0.0.1:9431/state'))
r=GetCartesianPath.Request();r.header.frame_id='world';r.group_name='panda_arm';r.link_name='panda_tcp';r.max_step=.005;r.avoid_collisions=True;r.jump_threshold=2.;r.start_state.joint_state.name=list(s['joints']);r.start_state.joint_state.position=list(s['joints'].values());p=Pose();p.position.x=.5;p.position.y=-.24;p.position.z=.25;p.orientation.x=1.;p.orientation.w=0.;r.waypoints=[p]
f=c.call_async(r);rclpy.spin_until_future_complete(n,f,timeout_sec=10);res=f.result();print('fraction',res.fraction,'points',len(res.solution.joint_trajectory.points));v=n.create_client(GetStateValidity,'/check_state_validity');v.wait_for_service(timeout_sec=3)
from trajectory_math import segment
import numpy as np
ps=res.solution.joint_trajectory.points
ps[-1].accelerations=[0.]*7
for p0,p1 in zip(ps,ps[1:]):
 T=(p1.time_from_start.sec-p0.time_from_start.sec)+(p1.time_from_start.nanosec-p0.time_from_start.nanosec)*1e-9
 coeff=segment(p0.positions,p0.velocities,p0.accelerations,p1.positions,p1.velocities,p1.accelerations,T)
 for u in np.linspace(0,1,7):
  q=GetStateValidity.Request();q.group_name='panda_arm';q.robot_state.is_diff=True;q.robot_state.joint_state.name=res.solution.joint_trajectory.joint_names;q.robot_state.joint_state.position=list(np.polynomial.polynomial.polyval(u,coeff));f=v.call_async(q);rclpy.spin_until_future_complete(n,f,timeout_sec=5);z=f.result()
  if not z.valid:print('contacts',u,[(c.contact_body_1,c.contact_body_2,c.depth) for c in z.contacts],flush=True)
n.destroy_node();rclpy.shutdown()
