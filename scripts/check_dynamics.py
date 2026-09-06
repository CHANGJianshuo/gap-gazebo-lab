#!/usr/bin/env python3
"""Cross-check the full-tree Newton-Euler implementation against Bullet RNEA."""
import json
from pathlib import Path
import site
import subprocess
import sys
import xml.etree.ElementTree as E
import numpy as np
from runtime_assets import runtime_asset
sys.path.append(site.getusersitepackages())  # Existing optional PyBullet installation.
import pybullet as bullet

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'data/validation';OUT.mkdir(parents=True,exist_ok=True)
NAMES=['shoulder_joint','upperArm_joint','foreArm_joint','wrist1_joint','wrist2_joint','wrist3_joint']
payload={'mass':.4,'com':[.005,-.008,.042],'inertia':[[.0003,.00002,0],[.00002,.0004,-.00001],[0,-.00001,.0002]]}
rng=np.random.default_rng(20260905)
rows=[]
for loaded in [False,True]:
    for i in range(32):
        row={'q':rng.uniform(-2.,2.,6).tolist(),'v':(np.zeros(6) if i<8 else rng.uniform(-2.,2.,6)).tolist(),'a':(np.zeros(6) if i<8 else rng.uniform(-4.,4.,6)).tolist()}
        if loaded:row['payload']=payload
        rows.append(row)
(OUT/'dynamics_input.json').write_text(json.dumps(rows))
probe=subprocess.run([str(ROOT/'install/mtc_motion_planning/lib/mtc_motion_planning/dynamics_probe'),str(ROOT/'src/mtc_description/urdf/s3_latch.urdf'),str(ROOT/'src/mtc_motion_planning/config/s3.srdf'),str(OUT/'dynamics_input.json')],capture_output=True,text=True,check=True)
result=json.loads(probe.stdout.strip().splitlines()[-1])
bullet.connect(bullet.DIRECT);bullet.setGravity(0,0,-9.81)
models=[]
for loaded in [False,True]:
    xml=E.parse(runtime_asset('src/mtc_description/urdf/s3_latch.urdf',resolve_packages=True));robot=xml.getroot()
    for link in robot.findall('link'):
        for tag in ['visual','collision']:
            for item in link.findall(tag):link.remove(item)
        if link.find('inertial') is None:
            inert=E.SubElement(link,'inertial');E.SubElement(inert,'mass',value='0');E.SubElement(inert,'inertia',ixx='0',iyy='0',izz='0',ixy='0',ixz='0',iyz='0')
    if loaded:
        link=E.SubElement(robot,'link',name='check_payload');inert=E.SubElement(link,'inertial');E.SubElement(inert,'origin',xyz=' '.join(map(str,payload['com'])));E.SubElement(inert,'mass',value=str(payload['mass']));I=payload['inertia'];E.SubElement(inert,'inertia',**{k:str(I[i][j]) for k,i,j in [('ixx',0,0),('iyy',1,1),('izz',2,2),('ixy',0,1),('ixz',0,2),('iyz',1,2)]});joint=E.SubElement(robot,'joint',name='check_payload_joint',type='fixed');E.SubElement(joint,'parent',link='tcp');E.SubElement(joint,'child',link='check_payload')
    path=OUT/f'bullet_model_{int(loaded)}.urdf';xml.write(path)
    model=bullet.loadURDF(str(path),useFixedBase=True,flags=bullet.URDF_USE_INERTIA_FROM_FILE);models.append(model)
errors=[];fk_errors=[]
for row,expected in zip(rows,result):
    model=models[int('payload' in row)];movable=[];tcp=None
    for j in range(bullet.getNumJoints(model)):
        info=bullet.getJointInfo(model,j);name=info[1].decode()
        if info[2]!=bullet.JOINT_FIXED:movable.append((j,name))
        if info[12].decode()=='tcp':tcp=j
    q=[];v=[];a=[]
    for j,name in movable:
        i=NAMES.index(name) if name in NAMES else -1
        q.append(row['q'][i] if i>=0 else .022);v.append(row['v'][i] if i>=0 else 0.);a.append(row['a'][i] if i>=0 else 0.);bullet.resetJointState(model,j,q[-1],v[-1])
    tau=bullet.calculateInverseDynamics(model,q,v,a)
    ordered=[tau[next(i for i,(_,n) in enumerate(movable) if n==name)] for name in NAMES]
    errors.append(float(np.max(np.abs(np.array(ordered)-expected['tau']))))
    fk_errors.append(float(np.linalg.norm(np.array(bullet.getLinkState(model,tcp,computeForwardKinematics=True)[4])-expected['tcp'])))
report={'pass':max(errors)<1e-7 and max(fk_errors)<1e-6,'cases':len(rows),'max_torque_difference_Nm':max(errors),'max_tcp_difference_m':max(fk_errors),'coverage':'full URDF tree, off-axis camera and open latch; 32 unloaded and 32 loaded static/dynamic states','reference':'PyBullet calculateInverseDynamics, URDF_USE_INERTIA_FROM_FILE; not a check of actual hardware parameters'}
(OUT/'dynamics.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2));bullet.disconnect()
raise SystemExit(0 if report['pass'] else 1)
