#!/usr/bin/env python3
"""Plan-only negative cases; no arm trajectory is sent for execution."""
import argparse
import copy
import json
from pathlib import Path
import subprocess
import rclpy
from demo import Demo,JOINTS,ROOT,pose
from mtc_interfaces.action import PlanMotion
from mtc_interfaces.srv import Latch

def main():
    p=argparse.ArgumentParser();p.add_argument('--task',default='basic');p.add_argument('--output',default='data/validation/rejections.json');args=p.parse_args()
    settings=argparse.Namespace(task=args.task,output='data/runs/rejection_probe',speed=1.,plan_only=True,order='C')
    rclpy.init();node=Demo(settings);results=[]
    def request(label,goal,expected):
        handle=node.future(node.planner.send_goal_async(goal),10)
        if not handle.accepted:raise RuntimeError(f'{label}: unexpectedly rejected at transport level')
        response=node.future(handle.get_result_async(),45).result
        passed=(response.success if expected=='VALIDATED' else not response.success) and response.reason.startswith(expected)
        results.append({'case':label,'pass':passed,'reason':response.reason,'diagnostics':json.loads(response.diagnostics_json)});print(label,response.reason,flush=True)
    try:
        node.ready();tcp,rotation=node.truth['tcp'];base=PlanMotion.Goal();base.mode='free';base.target_pose=pose(tcp+[0,0,.02],rotation);base.planning_timeout=3.;base.seed=20260905
        request('valid nearby target',base,'VALIDATED')
        g=copy.deepcopy(base);g.target_pose=pose([3.,3.,3.],rotation);request('unreachable target',g,'NO_VALID_IK_SOLUTION')
        g=copy.deepcopy(base);g.target_pose.pose.orientation.x=0.;g.target_pose.pose.orientation.y=0.;g.target_pose.pose.orientation.z=0.;g.target_pose.pose.orientation.w=0.;request('invalid quaternion',g,'Invalid target quaternion')
        g=copy.deepcopy(base);g.target_pose.header.frame_id='unknown_frame';request('unsupported coordinate frame',g,'Target frame must be')
        g=copy.deepcopy(base);g.start_state.name=JOINTS;g.start_state.position=[10.,0.,0.,0.,0.,0.];request('out-of-limit start',g,'START_JOINT_LIMIT')
        # Independent FK picks a known below-table configuration. It is supplied as
        # a fictitious planning start only; the real robot remains at its safe pose.
        inputs=ROOT/'data/validation/dynamics_input.json'
        if inputs.exists():
            rows=json.loads(inputs.read_text());raw=subprocess.check_output([str(ROOT/'install/mtc_motion_planning/lib/mtc_motion_planning/dynamics_probe'),str(ROOT/'src/mtc_description/urdf/s3_latch.urdf'),str(ROOT/'src/mtc_motion_planning/config/s3.srdf'),str(inputs)],text=True)
            states=json.loads(raw.strip().splitlines()[-1])
            chosen=next((r for r,s in zip(rows,states) if -.1<s['tcp'][0]<.45 and -.35<s['tcp'][1]<.4 and .3<s['tcp'][2]<.6),None)
            if chosen:
                g=copy.deepcopy(base);g.start_state.name=JOINTS;g.start_state.position=chosen['q'];request('colliding start',g,'START_COLLISION')
        req=Latch.Request();req.object_id='battery_D';req.close=True;res=node.future(node.lock.call_async(req),10)
        results.append({'case':'latch cannot engage with open halves','pass':not res.success and 'Close both' in res.message,'reason':res.message})
    finally:node.destroy_node();rclpy.try_shutdown()
    report={'pass':all(r['pass'] for r in results) and len(results)>=7,'cases':results,'scope':'planning and latch input rejection; no rejected trajectory executed'}
    target=Path(args.output);target.parent.mkdir(parents=True,exist_ok=True);target.write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2));return 0 if report['pass'] else 1
if __name__=='__main__':raise SystemExit(main())
