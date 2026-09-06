#!/usr/bin/env python3
"""Derive local simulation assets from pinned MoveIt Panda resources."""
from pathlib import Path
import xml.etree.ElementTree as E
import yaml,json,subprocess
R=Path(__file__).resolve().parents[1]; C=R/'config'; C.mkdir(exist_ok=True)
D=R/'deps/moveit_resources'
s=(D/'panda_description/urdf/panda.urdf.xacro').read_text().replace('$(find moveit_resources_panda_description)',str(D/'panda_description'))
r=E.fromstring(s)
for el in list(r):
 if el.tag.endswith('arg'): r.remove(el)
# Independent finger commands avoid simulator-version dependent mimic behavior.
m=r.find("joint[@name='panda_finger_joint2']/mimic")
if m is not None:r.find("joint[@name='panda_finger_joint2']").remove(m)
E.SubElement(r,'link',name='world')
j=E.SubElement(r,'joint',name='base_fixed',type='fixed');E.SubElement(j,'parent',link='world');E.SubElement(j,'child',link='panda_link0')
E.SubElement(r,'link',name='panda_tcp')
j=E.SubElement(r,'joint',name='tcp_fixed',type='fixed');E.SubElement(j,'parent',link='panda_hand');E.SubElement(j,'child',link='panda_tcp');E.SubElement(j,'origin',xyz='0 0 0.1034')
# Keep hand link for physics attachment and world-frame measurements.
g=E.SubElement(r,'gazebo',reference='panda_hand_joint');E.SubElement(g,'preserveFixedJoint').text='true'
arm=[f'panda_joint{i}' for i in range(1,8)]; fingers=['panda_finger_joint1','panda_finger_joint2']; initial=[0,-.785,0,-2.356,0,1.571,.785,.04,.04]
rc=E.SubElement(r,'ros2_control',name='PandaGazebo',type='system');h=E.SubElement(rc,'hardware');E.SubElement(h,'plugin').text='gz_ros2_control/GazeboSimSystem'
for name,pos in zip(arm+fingers,initial):
 j=E.SubElement(rc,'joint',name=name);E.SubElement(j,'command_interface',name='position');state=E.SubElement(j,'state_interface',name='position');E.SubElement(state,'param',name='initial_value').text=str(pos)
 for iface in ['velocity','effort']:E.SubElement(j,'state_interface',name=iface)
g=E.SubElement(r,'gazebo');p=E.SubElement(g,'plugin',filename='gz_ros2_control-system',name='gz_ros2_control::GazeboSimROS2ControlPlugin');E.SubElement(p,'parameters').text=str(C/'controllers.yaml');E.SubElement(p,'robot_param_node').text='robot_state_publisher';E.SubElement(p,'position_proportional_gain').text='0.2'
urdf=E.tostring(r,encoding='unicode');(C/'panda.urdf').write_text(urdf)
sr=E.fromstring((D/'panda_moveit_config/config/panda.srdf').read_text())
for v in sr.findall('virtual_joint'):sr.remove(v)
sr.find("group[@name='panda_arm']/chain").set('tip_link','panda_tcp')
srdf=E.tostring(sr,encoding='unicode');(C/'panda.srdf').write_text(srdf)
cm={'update_rate':250,'use_sim_time':True,'joint_state_broadcaster':{'type':'joint_state_broadcaster/JointStateBroadcaster'}}
ctrl={'controller_manager':{'ros__parameters':cm}}
for name,joints in [('arm_controller',arm),('hand_controller',fingers)]:
 cm[name]={'type':'joint_trajectory_controller/JointTrajectoryController'}
 ctrl[name]={'ros__parameters':{'joints':joints,'command_interfaces':['position'],'state_interfaces':['position','velocity'],'state_publish_rate':50.,'action_monitor_rate':20.,'constraints':{'goal_time':3.,'stopped_velocity_tolerance':.1},'use_sim_time':True}}
(C/'controllers.yaml').write_text(yaml.safe_dump(ctrl))
params={'robot_description':urdf,'robot_description_semantic':srdf,'use_sim_time':True,'robot_description_kinematics':{'panda_arm':{'kinematics_solver':'kdl_kinematics_plugin/KDLKinematicsPlugin','kinematics_solver_timeout':.1,'kinematics_solver_search_resolution':.005}},'robot_description_planning':{'joint_limits':{n:{'has_velocity_limits':True,'max_velocity':1.5 if n in arm else .15,'has_acceleration_limits':True,'max_acceleration':2.0 if n in arm else .5} for n in arm+fingers}},'planning_pipelines':['ompl'],'default_planning_pipeline':'ompl','ompl':{'planning_plugin':'ompl_interface/OMPLPlanner','request_adapters':'default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints','start_state_max_bounds_error':.1,'planner_configs':{'RRTConnect':{'type':'geometric::RRTConnect','range':.15}},'panda_arm':{'planner_configs':['RRTConnect'],'longest_valid_segment_fraction':.005}},'allow_trajectory_execution':False,'publish_robot_description_semantic':True}
params['qos_overrides']={'/clock':{'subscription':{'durability':'volatile','history':'keep_last','depth':1,'reliability':'best_effort'}}}
(C/'move_group.yaml').write_text(yaml.safe_dump({'move_group':{'ros__parameters':params}}))
(C/'rsp.yaml').write_text(yaml.safe_dump({'robot_state_publisher':{'ros__parameters':{'robot_description':urdf,'use_sim_time':True}}}))
(C/'sources.json').write_text(json.dumps({'moveit_resources_commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=D,text=True).strip(),'controller':'gz_ros2_control position command / idealized velocity servo','inertials':'upstream Panda identified parameters; not hardware calibration'},indent=2))

def box(name,xyz,size,color,static=True):
 mass='' if static else '<inertial><mass>0.08</mass><inertia><ixx>0.00002133</ixx><iyy>0.00002133</iyy><izz>0.00002133</izz></inertia></inertial>'
 geom=f'<geometry><box><size>{size}</size></box></geometry>'
 return f'<model name="{name}"><static>{str(static).lower()}</static><pose>{xyz} 0 0 0</pose><link name="body">{mass}<visual name="visual">{geom}<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual><collision name="collision">{geom}</collision></link></model>'
world='<sdf version="1.9"><world name="gap_lab"><physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor></physics>'
for short,name in [('physics','Physics'),('user-commands','UserCommands'),('scene-broadcaster','SceneBroadcaster'),('contact','Contact')]:world+=f'<plugin filename="gz-sim-{short}-system" name="gz::sim::systems::{name}"/>'
world+='<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin><plugin filename="libgap_world.so" name="gaplab::World"/><scene><ambient>.65 .65 .65 1</ambient><background>.08 .11 .17 1</background><shadows>true</shadows></scene><light name="sun" type="directional"><pose>0 0 4 0 0 0</pose><diffuse>.9 .9 .9 1</diffuse><direction>-.3 .3 -1</direction></light>'
world+=box('floor','0 0 -0.035','4 4 .05','.19 .23 .3 1')
world+=box('table','.55 0 .1','.75 1 .1','.6 .65 .7 1')
world+=box('barrier','.52 0 .265','.32 .065 .23','.8 .25 .2 1')
world+=box('cube','.5 -.24 .171','.04 .04 .04','.12 .62 .96 1',False)
# Target marker is visual only, not a hidden colliding box.
world+='<model name="target"><static>true</static><pose>.5 .24 .151 0 0 0</pose><link name="body"><visual name="marker"><geometry><box><size>.12 .12 .002</size></box></geometry><material><ambient>.16 .85 .57 1</ambient><diffuse>.16 .85 .57 1</diffuse></material></visual></link></model>'
world+='<model name="overview"><static>true</static><pose>1.55 -1.4 1.4 0 .50 2.25</pose><link name="camera"><sensor name="camera" type="camera"><topic>/gap/camera</topic><always_on>true</always_on><update_rate>8</update_rate><camera><horizontal_fov>1.05</horizontal_fov><image><width>720</width><height>480</height><format>R8G8B8</format></image><clip><near>.02</near><far>10</far></clip></camera></sensor></link></model></world></sdf>'
(C/'world.sdf').write_text(world)
print('Prepared',C)
