#!/usr/bin/env python3
"""Capture actual Gazebo sensor images; preserve sensor-time playback in MP4."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image as ImageMessage
from PIL import Image, ImageDraw, ImageFont

ROOT=Path(__file__).resolve().parents[1]
class Recorder(Node):
    def __init__(self,args):
        super().__init__('mtc_recorder');self.args=args;self.output=Path(args.output);self.output.parent.mkdir(parents=True,exist_ok=True);self.frames=0;self.received=0;self.duplicates=0;self.max_gap=0.;self.previous=None;self.first=None;self.last=None;self.proc=None;self.wrist=None
        image_qos=QoSProfile(depth=2,reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(ImageMessage,'/overview/image',self.frame,image_qos)
        self.create_subscription(ImageMessage,'/wrist_camera/image',self.wrist_frame,image_qos)
        fonts=['/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc','/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf']
        font=next((x for x in fonts if Path(x).exists()),None)
        if font is None:raise RuntimeError('A Chinese font is required: install fonts-noto-cjk or fonts-droid-fallback')
        self.font=ImageFont.truetype(font,19)
    @staticmethod
    def decode(msg):
        mode='RGB' if msg.encoding in ['rgb8','bgr8'] else 'RGBA';raw='BGR' if msg.encoding=='bgr8' else mode
        return Image.frombytes(mode,(msg.width,msg.height),bytes(msg.data),'raw',raw,msg.step).convert('RGB')
    def wrist_frame(self,msg):
        try:self.wrist=self.decode(msg)
        except Exception:return
    def frame(self,msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if self.last is not None and stamp<=self.last+1e-5:return
        im=self.decode(msg)
        if self.first is None:
            self.first=stamp;im.save(self.output.with_suffix('.png'))
            if self.args.snapshot:self.frames+=1;self.last=stamp;return
            self.proc=subprocess.Popen(['ffmpeg','-hide_banner','-loglevel','error','-y','-f','rawvideo','-pix_fmt','rgb24','-s',f'{msg.width}x{msg.height}','-r','25','-i','pipe:0','-an','-c:v','libx264','-preset','veryfast','-crf','19','-pix_fmt','yuv420p','-movflags','+faststart',str(self.output)],stdin=subprocess.PIPE,start_new_session=True)
        if self.args.snapshot:return
        # Informative overlays never alter the simulation geometry or timing.
        draw=ImageDraw.Draw(im);draw.rectangle((0,0,im.width,58),fill=(16,28,46));draw.text((22,13),f'AUBO S3  |  Gazebo Harmonic  |  仿真时间 {stamp-self.first:5.1f} s',font=self.font,fill='white')
        state={}
        try:state=json.loads((ROOT/self.args.status_file).read_text())
        except (OSError,json.JSONDecodeError):pass
        draw.rectangle((0,im.height-44,im.width,im.height),fill=(16,28,46));draw.text((22,im.height-35),str(state.get('phase','规划实验 · ground truth 输入')),font=self.font,fill=(213,227,246))
        if self.wrist is not None:
            inset=self.wrist.resize((256,192));im.paste(inset,(im.width-272,76));draw.rectangle((im.width-273,75,im.width-15,269),outline=(180,197,222),width=2)
        # Sensor stamps determine video time. Fill missing frames with the previous
        # observation so transport drops cannot silently accelerate the playback.
        target_index=round((stamp-self.first)*25)
        data=im.tobytes()
        try:
            while self.frames<target_index:
                self.proc.stdin.write(self.previous if self.previous is not None else data)
                self.frames+=1;self.duplicates+=1
            self.proc.stdin.write(data)
        except BrokenPipeError:raise RuntimeError('ffmpeg stopped while recording')
        self.frames+=1;self.received+=1;self.previous=data
        if self.last is not None:self.max_gap=max(self.max_gap,stamp-self.last)
        self.last=stamp
        if self.args.ready_file and self.frames==1:Path(self.args.ready_file).write_text(str(stamp))
    def finish(self):
        if self.proc:
            self.proc.stdin.close();code=self.proc.wait(timeout=30)
            if code:raise RuntimeError(f'ffmpeg exited with {code}')
        report={'frames':self.frames,'received_frames':self.received,'duplicated_frames':self.duplicates,'maximum_sensor_gap_s':self.max_gap,'first_sensor_stamp':self.first,'last_sensor_stamp':self.last,'sensor_duration':None if self.last is None else self.last-self.first,'video_duration_s':self.frames/25,'fps':25,'source':'Gazebo overview camera; wrist-camera inset','timing':'sensor simulation time; gaps held with previous frame; no artificial speed-up'}
        self.output.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report),flush=True)

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',default='data/videos/s3_demo.mp4');p.add_argument('--snapshot',action='store_true');p.add_argument('--seconds',type=float,default=0);p.add_argument('--status-file',default='data/runs/live/demo_status.json');p.add_argument('--ready-file');args=p.parse_args()
    rclpy.init();node=Recorder(args);start=time.monotonic()
    try:
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.2)
            if args.snapshot and node.frames:break
            if args.seconds and time.monotonic()-start>args.seconds:break
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:node.finish();node.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
