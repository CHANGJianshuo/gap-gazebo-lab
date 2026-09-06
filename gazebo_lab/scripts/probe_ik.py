import rclpy,json
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK,GetStateValidity,GetPlanningScene,GetPositionFK
from geometry_msgs.msg import Pose
from builtin_interfaces.msg import Duration
import urllib.request
rclpy.init();n=Node('gap_ik_probe')
c=n.create_client(GetPositionIK,'/compute_ik');c.wait_for_service(timeout_sec=5.)
o=urllib.request.build_opener(urllib.request.ProxyHandler({}));s=json.load(o.open('http://127.0.0.1:9431/state'))
for xyz in [[.5,-.24,.36],[.4,-.2,.5],[.5,-.24,.17]]:
 for collision in [False,True]:
  q=GetPositionIK.Request();q.ik_request.group_name='panda_arm';q.ik_request.ik_link_name='panda_tcp';q.ik_request.robot_state.joint_state.name=list(s['joints']);q.ik_request.robot_state.joint_state.position=list(s['joints'].values());q.ik_request.pose_stamped.header.frame_id='world';p=q.ik_request.pose_stamped.pose;p.position.x,p.position.y,p.position.z=xyz;p.orientation.x=1.;p.orientation.w=0.;q.ik_request.avoid_collisions=collision;q.ik_request.timeout=Duration(sec=2)
  f=c.call_async(q);rclpy.spin_until_future_complete(n,f,timeout_sec=5);r=f.result();print(xyz,collision,r.error_code.val,flush=True)
  if r.error_code.val==1:
   v=n.create_client(GetStateValidity,'/check_state_validity');v.wait_for_service(timeout_sec=5);req=GetStateValidity.Request();req.robot_state=r.solution;req.group_name='panda_arm';f=v.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=5);out=f.result();print('valid',out.valid,[(x.contact_body_1,x.contact_body_2,x.depth) for x in out.contacts],flush=True)
   fk=n.create_client(GetPositionFK,'/compute_fk');fk.wait_for_service(timeout_sec=5);req=GetPositionFK.Request();req.robot_state=r.solution;req.header.frame_id='world';req.fk_link_names=['panda_tcp','panda_hand'];f=fk.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=5);print('FK',f.result().pose_stamped,flush=True)
n.destroy_node();rclpy.shutdown()
