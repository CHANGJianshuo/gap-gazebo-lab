#!/usr/bin/env python3
"""Load the generated S3 model and the selected ground-truth scene into the planning action server."""
import argparse
import os
from pathlib import Path
import yaml
from runtime_assets import asset_text, runtime_asset

ROOT=Path(__file__).resolve().parents[1]
def main():
    p=argparse.ArgumentParser();p.add_argument('--task',choices=['basic','sequence'],default='sequence');p.add_argument('--run-dir',default='data/runs/live');args=p.parse_args();run=ROOT/args.run_dir;run.mkdir(parents=True,exist_ok=True)
    config=ROOT/'src/mtc_motion_planning/config'
    params={'use_sim_time':True,'scene_config':str(runtime_asset(f'src/mtc_simulation/config/{args.task}.json')),
            'robot_description':asset_text('src/mtc_description/urdf/s3_latch.urdf'),
            'robot_description_semantic':(config/'s3.srdf').read_text(),
            'robot_description_kinematics':yaml.safe_load((config/'kinematics.yaml').read_text()),
            'robot_description_planning':yaml.safe_load((config/'joint_limits.yaml').read_text()),
            'ompl':{'planning_plugin':'ompl_interface/OMPLPlanner','request_adapters':'',
                    'planner_configs':{'RRTConnectkConfigDefault':{'type':'geometric::RRTConnect','range':.15}},
                    'arm':{'planner_configs':['RRTConnectkConfigDefault'],'longest_valid_segment_fraction':.001,'enforce_joint_model_state_space':True},
                    'simplify_solutions':True}}
    target=run/'planner_parameters.yaml';target.write_text(yaml.safe_dump({'mtc_motion_planner':{'ros__parameters':params}},sort_keys=False));(run/'planner_launcher.pid').write_text(str(os.getpid()))
    os.execvp('ros2',['ros2','run','mtc_motion_planning','planner','--ros-args','--params-file',str(target)])
if __name__=='__main__':main()
