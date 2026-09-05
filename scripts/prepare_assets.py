#!/usr/bin/env python3
"""Rebuild CAD-derived assets and the parameterized S3 simulation from original sources."""
from pathlib import Path
import hashlib
import json
import math
import shutil
import subprocess
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
import yaml
from OCP.STEPControl import STEPControl_Reader
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp

ROOT = Path(__file__).resolve().parents[1]
DESC = ROOT / 'src/mtc_description'
SIM = ROOT / 'src/mtc_simulation'
PLAN = ROOT / 'src/mtc_motion_planning'
MESH = DESC / 'meshes'
TABLE_Z = .65
NAMES = ['shoulder_joint','upperArm_joint','foreArm_joint','wrist1_joint','wrist2_joint','wrist3_joint']
INITIAL = [0., .35, 1.3, -.35, 1.57, 0.]
MASS = .4

def numbers(values): return ' '.join(f'{x:.10g}' for x in values)
def node(parent, tag, **attrs): return ET.SubElement(parent, tag, {k:str(v) for k,v in attrs.items()})
def value(parent, tag, text):
    e=ET.SubElement(parent,tag);e.text=str(text);return e
def save_xml(root,path):
    ET.indent(root);path.parent.mkdir(parents=True,exist_ok=True);ET.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)
def inertia(link,mass,xyz=(0,0,0),size=(.03,.03,.03),matrix=None):
    e=node(link,'inertial');node(e,'origin',xyz=numbers(xyz),rpy='0 0 0');node(e,'mass',value=mass)
    if matrix is None:
        x,y,z=size;matrix=np.diag([mass*(y*y+z*z)/12,mass*(x*x+z*z)/12,mass*(x*x+y*y)/12])
    node(e,'inertia',ixx=matrix[0,0],iyy=matrix[1,1],izz=matrix[2,2],ixy=matrix[0,1],ixz=matrix[0,2],iyz=matrix[1,2])
def geom(link,kind,shape,xyz=(0,0,0),rgba='.17 .21 .28 1',name=None):
    e=node(link,kind,**({'name':name} if name else {}));node(e,'origin',xyz=numbers(xyz),rpy='0 0 0');g=node(e,'geometry')
    tag,attrs=shape;node(g,tag,**attrs)
    if kind=='visual': node(node(e,'material',name=''), 'color',rgba=rgba)
    return e
def fixed(robot,name,parent,child,xyz=(0,0,0),rpy=(0,0,0)):
    j=node(robot,'joint',name=name,type='fixed');node(j,'parent',link=parent);node(j,'child',link=child);node(j,'origin',xyz=numbers(xyz),rpy=numbers(rpy));return j

def cad():
    MESH.mkdir(parents=True,exist_ok=True)
    reader=STEPControl_Reader();result=reader.ReadFile(str(ROOT/'电池道具.stp'))
    if int(result)!=1: raise RuntimeError('STEP reader failed')
    reader.TransferRoots();shape=reader.OneShape();BRepMesh_IncrementalMesh(shape,.04,False,.15,True).Perform()
    native=ROOT/'data/cad/battery_native.stl';native.parent.mkdir(parents=True,exist_ok=True)
    writer=StlAPI_Writer();writer.ASCIIMode=False;writer.Write(shape,str(native))
    mesh=trimesh.load(native,force='mesh');assert mesh.is_watertight
    transform=np.diag([.001,-.001,-.001,1.]);transform[2,3]=.05
    mesh.apply_transform(transform);mesh.export(MESH/'battery.stl')
    assert np.allclose(mesh.extents,[.07,.08,.08],atol=1e-5)
    body=trimesh.convex.convex_hull(mesh.vertices[mesh.vertices[:,2]<=.050001]);body.export(MESH/'battery_body_collision.stl')
    properties=GProp_GProps();BRepGProp.VolumeProperties_s(shape,properties)
    vol=properties.Mass();c=np.array(properties.CentreOfMass().Coord());rot=np.diag([1,-1,-1]);com=rot@c*.001+[0,0,.05]
    m=properties.MatrixOfInertia();I=np.array([[m.Value(i+1,j+1) for j in range(3)] for i in range(3)])*MASS/vol*1e-6;I=rot@I@rot.T
    collision=[{'kind':'mesh','file':str(MESH/'battery_body_collision.stl'),'xyz':[0,0,0]}]
    collision += [{'kind':'box','size':[.01,.026,.020],'xyz':[x,0,.060]} for x in [-.03,.03]]
    collision += [{'kind':'box','size':[.07,.026,.012],'xyz':[0,0,.074]}]
    info={'source':'电池道具.stp','sha256':hashlib.sha256((ROOT/'电池道具.stp').read_bytes()).hexdigest(),
          'units':'m','native_units':'mm','native_to_model':transform.tolist(),'dimensions':mesh.extents.tolist(),
          'mass':MASS,'mass_source':'simulation assumption; physical mass not provided','volume':vol*1e-9,
          'center_of_mass':com.tolist(),'inertia':I.tolist(),'handle_center':[0,0,.074],
          'handle_bar_size':[.070,.026,.012],'collision':collision,'faces':len(mesh.faces)}
    (DESC/'config').mkdir(exist_ok=True);(DESC/'config/battery.json').write_text(json.dumps(info,indent=2,ensure_ascii=False)+'\n')
    return info

def half_ring(sign):
    # A split collar around the handle's X axis. Inner radius clears its 26 x 12 mm section.
    vertices=[];faces=[];angles=np.linspace(-math.pi/2,math.pi/2,25)
    for a in angles:
        for x,r in [(-.005,.0155),(.005,.0155),(.005,.0195),(-.005,.0195)]:
            vertices.append([x,sign*r*math.cos(a),r*math.sin(a)])
    for i in range(24):
        for j in range(4):
            a=4*i+j;b=4*i+(j+1)%4;c=b+4;d=a+4;faces += [[a,b,c],[a,c,d]]
    faces += [[0,2,1],[0,3,2],[96,97,98],[96,98,99]]
    mesh=trimesh.Trimesh(vertices=vertices,faces=faces,process=True);mesh.fix_normals()
    return mesh

def robot():
    shutil.copytree(ROOT/'third_party/aubo_description/upstream/meshes/aubo_S3',MESH/'aubo_S3',dirs_exist_ok=True)
    tree=ET.parse(ROOT/'third_party/aubo_description/upstream/urdf/aubo_S3.urdf');r=tree.getroot();r.set('name','mtc_s3')
    for child in list(r):
        if child.tag not in ['link','joint','material']:r.remove(child)
    for p in r.findall('.//property'): # vendor-only metadata is archived in the original
        for j in r.findall('joint'):
            if p in list(j): j.remove(p)
    for limit in r.findall('.//limit'):limit.attrib.pop('start_stop',None)
    for m in r.findall('.//mesh'):m.set('filename',m.get('filename').replace('package://aubo_description/', 'package://mtc_description/'))
    for i,j in enumerate(r.findall('joint')):
        if j.get('name')=='world_joint':j.find('origin').set('xyz',f'0 0 {TABLE_Z}')
        if j.get('type')=='revolute':node(j,'dynamics',damping='.025',friction='0')
    tool=node(r,'link',name='latch_mount');inertia(tool,.16,xyz=(0,0,.045),size=(.05,.065,.09))
    for kind in ['visual','collision']:
        geom(tool,kind,('cylinder',{'radius':'.018','length':'.020'}),xyz=(0,0,.005),rgba='.25 .29 .34 1')
        geom(tool,kind,('box',{'size':'.046 .065 .030'}),xyz=(0,0,.025),rgba='.12 .17 .23 1')
        # Supports remain above the closed collar; leave its approach opening clear.
        for y in [-.03,.03]:geom(tool,kind,('box',{'size':'.012 .010 .065'}),xyz=(0,y,.069),rgba='.35 .39 .43 1')
    fixed(r,'latch_mount_joint','wrist3_Link','latch_mount',xyz=(0,0,.005))
    tcp=node(r,'link',name='tcp');inertia(tcp,.003,size=(.005,.005,.005));fixed(r,'tcp_joint','latch_mount','tcp',xyz=(0,0,.125))
    for label,sign in [('left',1),('right',-1)]:
        name=f'latch_{label}';mesh=half_ring(sign);mesh.export(MESH/f'{name}.stl')
        link=node(r,'link',name=name);inertia(link,.035,size=(.01,.04,.04))
        geom(link,'visual',('mesh',{'filename':f'package://mtc_description/meshes/{name}.stl'}),rgba='.88 .66 .15 1')
        # Piecewise convex blocks approximate the ring without filling its central hole.
        for i,a in enumerate(np.linspace(-math.pi/2,math.pi/2,18)):
            e=geom(link,'collision',('box',{'size':'.010 .0045 .0045'}),xyz=(0,sign*.0175*math.cos(a),.0175*math.sin(a)),name=f'{name}_{i}')
            e.find('origin').set('rpy',numbers((sign*a,0,0)))
        j=node(r,'joint',name=f'{name}_joint',type='prismatic');node(j,'parent',link='latch_mount');node(j,'child',link=name);node(j,'origin',xyz='0 0 .125',rpy='0 0 0');node(j,'axis',xyz=f'0 {sign} 0');node(j,'limit',lower='0',upper='.022',velocity='.08',effort='25');node(j,'dynamics',damping='1',friction='0')
    camera_xml=subprocess.check_output(['/opt/ros/humble/bin/xacro','/opt/ros/humble/share/realsense2_description/urdf/test_d435i_camera.urdf.xacro','use_nominal_extrinsics:=true'],text=True)
    camera=ET.fromstring(camera_xml)
    for item in list(camera):
        if item.tag=='link' and item.get('name')=='base_link':continue
        if item.tag=='joint' and item.find('parent') is not None and item.find('parent').get('link')=='base_link':
            item.find('parent').set('link','wrist3_Link');item.find('origin').set('xyz','.065 0 .03');item.find('origin').set('rpy','0 -1.5707963267948966 0')
        r.append(item)
    camera_link=r.find("link[@name='camera_link']")
    if camera_link is not None:
        old=camera_link.find('inertial')
        if old is not None:camera_link.remove(old)
        inertia(camera_link,.075,size=(.025,.09,.025))
    control=node(r,'ros2_control',name='S3GazeboSystem',type='system');value(node(control,'hardware'),'plugin','gz_ros2_control/GazeboSimSystem')
    for name,initial in zip(NAMES+['latch_left_joint','latch_right_joint'],INITIAL+[.022,.022]):
        j=node(control,'joint',name=name);cmd=node(j,'command_interface',name='effort' if name in NAMES else 'position')
        if name in NAMES:
            lim=float(r.find(f"joint[@name='{name}']/limit").get('effort'));value(node(cmd,'param',name='min'),'unused','') if False else None
            node(cmd,'param',name='min').text=str(-lim);node(cmd,'param',name='max').text=str(lim)
        state=node(j,'state_interface',name='position');node(state,'param',name='initial_value').text=str(initial);node(j,'state_interface',name='velocity');node(j,'state_interface',name='effort')
    plugin=node(node(r,'gazebo'),'plugin',filename='gz_ros2_control-system',name='gz_ros2_control::GazeboSimROS2ControlPlugin')
    value(plugin,'parameters',str(PLAN/'config/controllers.yaml'));value(plugin,'robot_param_node','robot_state_publisher')
    # Preserve the named TCP for the mechanical latch constraint and its ground-truth pose.
    value(node(r,'gazebo',reference='tcp_joint'),'preserveFixedJoint','true')
    # A real simulated RGB-D sensor accompanies the physical D435i housing and optical TFs.
    gz=node(r,'gazebo',reference='camera_link');sensor=node(gz,'sensor',name='wrist_rgbd',type='rgbd_camera');value(sensor,'pose','0 0 0 0 0 0');value(sensor,'topic','/wrist_camera');value(sensor,'update_rate',10);value(sensor,'always_on','true')
    c=node(sensor,'camera');value(c,'horizontal_fov','1.211');im=node(c,'image');value(im,'width',640);value(im,'height',480);value(im,'format','R8G8B8');clip=node(c,'clip');value(clip,'near','.05');value(clip,'far','3');value(sensor,'gz_frame_id','camera_color_optical_frame')
    imu=node(gz,'sensor',name='wrist_imu',type='imu');value(imu,'topic','/wrist_camera/imu');value(imu,'always_on','true');value(imu,'update_rate',100);node(imu,'imu');value(imu,'gz_frame_id','camera_link')
    for link in r.findall('link'):
        gz=node(r,'gazebo',reference=link.get('name'));value(gz,'self_collide','true')
    save_xml(r,DESC/'urdf/s3_latch.urdf')
    # All file resources are resolved into a second URDF for direct Gazebo loading and external tools.
    raw=ET.tostring(r,encoding='unicode').replace('package://mtc_description/',str(DESC)+'/').replace('package://realsense2_description/','/opt/ros/humble/share/realsense2_description/')
    (DESC/'urdf/s3_latch_resolved.urdf').write_text(raw)
    srdf=ET.Element('robot',name='mtc_s3');g=node(srdf,'group',name='arm');node(g,'chain',base_link='base_link',tip_link='tcp')
    g=node(srdf,'group',name='latch');node(g,'joint',name='latch_left_joint');node(g,'joint',name='latch_right_joint')
    state=node(srdf,'group_state',name='home',group='arm')
    for name,pos in zip(NAMES,INITIAL):node(state,'joint',name=name,value=pos)
    # Only connected/fixed neighbours are exempted automatically. Other exclusions require evidence.
    parents={j.find('child').get('link'):j.find('parent').get('link') for j in r.findall('joint')}
    for child,parent in parents.items():node(srdf,'disable_collisions',link1=parent,link2=child,reason='Adjacent')
    fixed_parent={j.find('child').get('link'):j.find('parent').get('link') for j in r.findall('joint') if j.get('type')=='fixed'}
    def cluster(link):
        while link in fixed_parent:link=fixed_parent[link]
        return link
    links=[x.get('name') for x in r.findall('link')]
    for i,a in enumerate(links):
        for b in links[i+1:]:
            if cluster(a)==cluster(b) and parents.get(a)!=b and parents.get(b)!=a:node(srdf,'disable_collisions',link1=a,link2=b,reason='Rigid assembly')
    for finger in ['latch_left','latch_right']:
        node(srdf,'disable_collisions',link1=finger,link2='tcp',reason='Adjacent')
    node(srdf,'disable_collisions',link1='latch_left',link2='latch_right',reason='Closure contact')
    save_xml(srdf,PLAN/'config/s3.srdf')
    return r

def configs():
    PLAN.joinpath('config').mkdir(exist_ok=True)
    gains={n:{'p':float(p),'i':0.,'d':float(d),'i_clamp':0.,'ff_velocity_scale':0.} for n,p,d in zip(NAMES,[500,650,500,120,100,30],[10,12,8,1.2,.6,.12])}
    cfg={'controller_manager':{'ros__parameters':{'update_rate':500,'use_sim_time':True,'joint_state_broadcaster':{'type':'joint_state_broadcaster/JointStateBroadcaster'},'arm_controller':{'type':'joint_trajectory_controller/JointTrajectoryController'},'latch_controller':{'type':'joint_trajectory_controller/JointTrajectoryController'}}},
         'arm_controller':{'ros__parameters':{'joints':NAMES,'command_interfaces':['effort'],'state_interfaces':['position','velocity'],'state_publish_rate':100.,'action_monitor_rate':25.,'allow_partial_joints_goal':False,'gains':gains,'constraints':{'goal_time':2.,'stopped_velocity_tolerance':.05,**{n:{'trajectory':.18,'goal':.012} for n in NAMES}}}},
         'latch_controller':{'ros__parameters':{'joints':['latch_left_joint','latch_right_joint'],'command_interfaces':['position'],'state_interfaces':['position','velocity'],'constraints':{'goal_time':1.,'stopped_velocity_tolerance':.05}}}}
    for name in ['arm_controller','latch_controller']:cfg[name]['ros__parameters']['use_sim_time']=True
    (PLAN/'config/controllers.yaml').write_text(yaml.safe_dump(cfg,sort_keys=False))
    limits={n:{'has_velocity_limits':True,'max_velocity':v,'has_acceleration_limits':True,'max_acceleration':a,'has_jerk_limits':True,'max_jerk':j} for n,v,a,j in zip(NAMES,[2.3,2.3,2.3,2.8,2.8,2.8],[4.,4.,4.,6.,6.,6.],[28.,28.,28.,45.,45.,45.])}
    (PLAN/'config/joint_limits.yaml').write_text(yaml.safe_dump({'joint_limits':limits},sort_keys=False))
    (PLAN/'config/kinematics.yaml').write_text(yaml.safe_dump({'arm':{'kinematics_solver':'kdl_kinematics_plugin/KDLKinematicsPlugin','kinematics_solver_timeout':.035,'kinematics_solver_search_resolution':.005}}))

def plane_dae(task,geometry):
    out=SIM/'models';out.mkdir(exist_ok=True)
    source=ROOT/f'docs/competition/extracted/board_{task}.png';shutil.copy2(source,out/f'board_{task}.png')
    p=next(p for p in geometry['pages'] if p['task']==task);u,v=p['base_center_uv_mm'];width,height=p['size_uv_mm'];xmax=v/1000;xmin=(v-height)/1000;ymax=u/1000;ymin=(u-width)/1000
    positions=[xmax,ymax,0,xmin,ymax,0,xmin,ymin,0,xmax,ymin,0]
    dae=f'''<?xml version="1.0"?><COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1"><asset><unit meter="1"/><up_axis>Z_UP</up_axis></asset><library_images><image id="image"><init_from>board_{task}.png</init_from></image></library_images><library_effects><effect id="effect"><profile_COMMON><newparam sid="surface"><surface type="2D"><init_from>image</init_from></surface></newparam><newparam sid="sampler"><sampler2D><source>surface</source></sampler2D></newparam><technique sid="common"><lambert><emission><color>0.12 0.12 0.12 1</color></emission><diffuse><texture texture="sampler" texcoord="UVMap"/></diffuse></lambert></technique></profile_COMMON></effect></library_effects><library_materials><material id="material"><instance_effect url="#effect"/></material></library_materials><library_geometries><geometry id="board"><mesh><source id="positions"><float_array id="positions-array" count="12">{numbers(positions)}</float_array><technique_common><accessor source="#positions-array" count="4" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source><source id="uv"><float_array id="uv-array" count="8">0 1 0 0 1 0 1 1</float_array><technique_common><accessor source="#uv-array" count="4" stride="2"><param name="S" type="float"/><param name="T" type="float"/></accessor></technique_common></source><vertices id="vertices"><input semantic="POSITION" source="#positions"/></vertices><triangles count="2" material="mat"><input semantic="VERTEX" source="#vertices" offset="0"/><input semantic="TEXCOORD" source="#uv" offset="1" set="0"/><p>0 0 1 1 2 2 0 0 2 2 3 3</p></triangles></mesh></geometry></library_geometries><library_visual_scenes><visual_scene id="scene"><node><instance_geometry url="#board"><bind_material><technique_common><instance_material symbol="mat" target="#material"><bind_vertex_input semantic="UVMap" input_semantic="TEXCOORD" input_set="0"/></instance_material></technique_common></bind_material></instance_geometry></node></visual_scene></library_visual_scenes><scene><instance_visual_scene url="#scene"/></scene></COLLADA>'''
    (out/f'board_{task}.dae').write_text(dae)
    return p

def sdf_box(link,name,size,pose,color,collision=True):
    for kind in (['visual','collision'] if collision else ['visual']):
        e=node(link,kind,name=name);value(e,'pose',numbers(pose)+' 0 0 0');value(node(node(e,'geometry'),'box'),'size',numbers(size))
        if kind=='visual':
            mat=node(e,'material');value(mat,'ambient',color);value(mat,'diffuse',color)
        else:
            surf=node(e,'surface');fr=node(node(surf,'friction'),'ode');value(fr,'mu',.8);value(fr,'mu2',.8)

def world(task,battery,geometry):
    page=plane_dae(task,geometry);sdf=ET.Element('sdf',version='1.9');w=node(sdf,'world',name='mtc')
    physics=node(w,'physics',name='1ms',type='ignored');value(physics,'max_step_size',.001);value(physics,'real_time_factor',1.0)
    for filename,name in [('gz-sim-physics-system','Physics'),('gz-sim-user-commands-system','UserCommands'),('gz-sim-scene-broadcaster-system','SceneBroadcaster'),('gz-sim-contact-system','Contact')]:node(w,'plugin',filename=filename,name='gz::sim::systems::'+name)
    sensor=node(w,'plugin',filename='gz-sim-sensors-system',name='gz::sim::systems::Sensors');value(sensor,'render_engine','ogre2')
    node(w,'plugin',filename='gz-sim-imu-system',name='gz::sim::systems::Imu')
    node(w,'plugin',filename='libmtc_simulation_system.so',name='mtc::SimulationSystem')
    scene=node(w,'scene');value(scene,'ambient','.65 .65 .65 1');value(scene,'background','.90 .93 .97 1');value(scene,'shadows','true')
    light=node(w,'light',name='key',type='directional');value(light,'pose','0 0 4 0 0 0');value(light,'diffuse','.95 .95 .95 1');value(light,'specular','.2 .2 .2 1');value(light,'direction','-.3 .4 -1');value(light,'cast_shadows','true')
    floor=node(w,'model',name='floor');value(floor,'static','true');sdf_box(node(floor,'link',name='floor'), 'floor',[10,10,.05],[0,0,-.05],'.86 .89 .93 1')
    table=node(w,'model',name='table');value(table,'static','true');link=node(table,'link',name='table');sdf_box(link,'top',[.68,.86,.05],[.17,.043,TABLE_Z-.025],'.14 .19 .24 1')
    for x in [-.1,.43]:
        for y in [-.30,.39]:sdf_box(link,f'leg_{x}_{y}',[.035,.035,.60],[x,y,.30],'.18 .23 .28 1')
    u,v=page['base_center_uv_mm'];width,height=page['size_uv_mm']
    e=node(link,'visual',name='original_board');value(e,'pose',f'{(v-height/2)/1000} {(u-width/2)/1000} {TABLE_Z+.0003} 0 0 {-math.pi/2}');plane=node(node(e,'geometry'),'plane');value(plane,'normal','0 0 1');value(plane,'size',f'{width/1000} {height/1000}');value(e,'cast_shadows','false')
    mat=node(e,'material');value(mat,'ambient','1 1 1 1');value(mat,'diffuse','1 1 1 1');metal=node(node(mat,'pbr'),'metal');value(metal,'albedo_map',str(SIM/f'models/board_{task}.png'));value(metal,'roughness',1);value(metal,'metalness',0)
    colors={'A':'.85 .12 .10 1','B':'.98 .73 .05 1','C':'.06 .30 .82 1','D':'.10 .64 .34 1'}
    initial={}
    for shape in page['shapes']:
        name=shape['name']
        if name not in colors:continue
        pos=shape['center_board_xyz_m'][:2]+[TABLE_Z+.001];initial[name]=pos
        model=node(w,'model',name='battery_'+name);value(model,'pose',numbers(pos)+f' 0 0 {math.pi/2}');link=node(model,'link',name='body')
        inert=node(link,'inertial');value(inert,'pose',numbers(battery['center_of_mass'])+' 0 0 0');value(inert,'mass',MASS);ii=node(inert,'inertia');I=battery['inertia']
        for k,a,b in [('ixx',0,0),('iyy',1,1),('izz',2,2),('ixy',0,1),('ixz',0,2),('iyz',1,2)]:value(ii,k,I[a][b])
        vis=node(link,'visual',name='cad');value(node(node(vis,'geometry'),'mesh'),'uri',str(MESH/'battery.stl'));mat=node(vis,'material');value(mat,'ambient',colors[name]);value(mat,'diffuse',colors[name])
        for i,s in enumerate(battery['collision']):
            if s['kind']=='box':
                # Physical collision boxes; use the CAD mesh as the only visible shape.
                e=node(link,'collision',name=f'part_{i}');value(e,'pose',numbers(s['xyz'])+' 0 0 0');value(node(node(e,'geometry'),'box'),'size',numbers(s['size']))
            else:e=node(link,'collision',name=f'part_{i}');value(node(node(e,'geometry'),'mesh'),'uri',s['file'])
            fr=node(node(node(e,'surface'),'friction'),'ode');value(fr,'mu',.8);value(fr,'mu2',.8)
    # Gazebo's camera looks along its +X axis. Aim at the board and the full robot.
    model=node(w,'model',name='overview');value(model,'static','true');link=node(model,'link',name='camera');eye=np.array([1.12,-1.27,1.49]);target=np.array([.16,.01,.82]);d=target-eye;yaw=math.atan2(d[1],d[0]);pitch=-math.atan2(d[2],math.hypot(d[0],d[1]));value(model,'pose',numbers(eye)+f' 0 {pitch} {yaw}')
    s=node(link,'sensor',name='overview',type='camera');value(s,'topic','/overview/image');value(s,'always_on','true');value(s,'update_rate',25);c=node(s,'camera');value(c,'horizontal_fov',.95);im=node(c,'image');value(im,'width',1280);value(im,'height',720);value(im,'format','R8G8B8');clip=node(c,'clip');value(clip,'near',.02);value(clip,'far',20)
    save_xml(sdf,SIM/f'worlds/{task}.sdf')
    scene={'task':task,'table_z':TABLE_Z,'battery':battery,'initial_objects':initial,'yaw':math.pi/2,'initial_joints':dict(zip(NAMES,INITIAL)),
           'targets':{s['name']:s['center_board_xyz_m'][:2]+[TABLE_Z] for s in page['shapes'] if s['name']=='T0' or s['name'].endswith('_inner')},
           'assumptions':{'battery_mass_kg':MASS,'battery_friction':.8,'joint_acceleration_jerk':'simulation commissioning limits, not certified AUBO specifications','camera_mass_kg':.075,'latch':'parameterized split collar; gated rigid constraint when closed, no teleportation'}}
    (SIM/f'config/{task}.json').write_text(json.dumps(scene,ensure_ascii=False,indent=2)+'\n')

if __name__=='__main__':
    configs();battery=cad();robot();geometry=json.loads((ROOT/'docs/competition/extracted/board_geometry.json').read_text())
    for task in ['basic','sequence']:world(task,battery,geometry)
    print('Generated exact-scale CAD, official robot composition, controllers, SRDF and both board worlds.')
