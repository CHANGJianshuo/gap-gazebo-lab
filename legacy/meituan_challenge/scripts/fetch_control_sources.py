#!/usr/bin/env python3
"""Fetch pinned upstream control sources without resetting existing checkouts."""
from pathlib import Path
import subprocess
ROOT=Path(__file__).resolve().parents[1]
sources=[('gz_ros2_control','https://github.com/ros-controls/gz_ros2_control.git','c88a5fd9170af120c263c1201f0744a40f93d673'),('ros2_controllers','https://github.com/ros-controls/ros2_controllers.git','159e6298b2a308b1cb3596680e98b86eb80dedc3')]
for name,url,commit in sources:
    path=ROOT/'vendor_ws/src'/name
    if not path.exists():
        path.mkdir(parents=True)
        for command in [['git','init','-q'],['git','remote','add','origin',url],['git','fetch','--depth','1','origin',commit],['git','checkout','--detach','FETCH_HEAD']]:subprocess.run(command,cwd=path,check=True)
    actual=subprocess.check_output(['git','rev-parse','HEAD'],cwd=path,text=True).strip()
    if actual!=commit:raise SystemExit(f'{name}: expected {commit}, found {actual}; existing checkout was preserved')
    print(name,commit)
