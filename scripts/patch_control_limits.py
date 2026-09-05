#!/usr/bin/env python3
"""Small, auditable overlay patch: clamp effort commands to the SDF joint limit."""
from pathlib import Path
import subprocess

root=Path(__file__).resolve().parents[1]
repo=root/'vendor_ws/src/gz_ros2_control'
path=repo/'gz_ros2_control/src/gz_system.cpp'
text=path.read_text()
old='{this->dataPtr->joints_[i].joint_effort_cmd});'
new='''{std::clamp(this->dataPtr->joints_[i].joint_effort_cmd,
            -std::abs(this->dataPtr->joints_[i].joint_axis.Effort()),
             std::abs(this->dataPtr->joints_[i].joint_axis.Effort()))}); // MTC: finite actuator effort'''
if 'MTC: finite actuator effort' not in text:
    if text.count(old)!=1:raise RuntimeError('Unexpected upstream source; review patch before applying')
    text=text.replace(old,new)
    if '#include <algorithm>' not in text:text='#include <algorithm>\n'+text
    path.write_text(text)
patch=subprocess.check_output(['git','diff','--','gz_ros2_control/src/gz_system.cpp'],cwd=repo)
(root/'scripts/gz_effort_limits.patch').write_bytes(patch)
print('Effort clamp patch ready; rebuild vendor overlay before running.')
