#!/usr/bin/env python3
"""Read real simulated D435i RGB, depth, intrinsics and IMU messages."""
import argparse
import json
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image,CameraInfo,Imu

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',default='data/validation/sensors.json');args=p.parse_args()
    rclpy.init();node=Node('mtc_sensor_check');found={}
    def rgb(msg):found['rgb']={'width':msg.width,'height':msg.height,'encoding':msg.encoding,'bytes':len(msg.data),'frame_id':msg.header.frame_id}
    def depth(msg):
        if msg.encoding!='32FC1':return
        a=np.frombuffer(bytes(msg.data),dtype=np.float32);finite=a[np.isfinite(a)]
        found['depth']={'width':msg.width,'height':msg.height,'encoding':msg.encoding,'finite_fraction':float(len(finite)/len(a)),'min_m':float(finite.min()) if len(finite) else None,'max_m':float(finite.max()) if len(finite) else None}
    def intrinsics(msg):found['intrinsics']={'width':msg.width,'height':msg.height,'k':list(msg.k)}
    def imu(msg):found['imu']={'frame_id':msg.header.frame_id,'linear_acceleration':[msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z],'angular_velocity':[msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z]}
    subscriptions=[node.create_subscription(Image,'/wrist_camera/image',rgb,2),node.create_subscription(Image,'/wrist_camera/depth_image',depth,2),node.create_subscription(CameraInfo,'/wrist_camera/camera_info',intrinsics,2),node.create_subscription(Imu,'/wrist_camera/imu',imu,2)]
    deadline=time.monotonic()+45
    while len(found)<4 and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
    found['pass']=len(found)==4 and found.get('depth',{}).get('finite_fraction',0)>.01
    out=Path(args.output);out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(found,indent=2));print(json.dumps(found,indent=2));node.destroy_node();rclpy.shutdown()
    return 0 if found['pass'] else 1
if __name__=='__main__':raise SystemExit(main())
