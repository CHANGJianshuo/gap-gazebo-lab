"""Probe and fully decode a video, including systems that only provide ffmpeg."""
import json
import re
import shutil
import subprocess
def probe_video(path):
    if shutil.which('ffprobe'):
        info=json.loads(subprocess.check_output(['ffprobe','-v','error','-show_entries','format=duration:stream=codec_name,width,height,r_frame_rate,nb_frames','-of','json',str(path)],text=True))
    else:info={}
    result=subprocess.run(['ffmpeg','-hide_banner','-nostats','-i',str(path),'-map','0:v:0','-progress','pipe:1','-f','null','-'],capture_output=True,text=True,check=True)
    progress={}
    for line in result.stdout.splitlines():
        if '=' in line:
            key,value=line.split('=',1);progress[key]=value
    dimensions=re.search(r'Video:.*?\b(\d{3,5})x(\d{3,5})\b',result.stderr)
    info['decode_check']={'pass':True,'frames':int(progress['frame']),'duration_s':int(progress['out_time_us'])/1e6,'width':int(dimensions[1]) if dimensions else None,'height':int(dimensions[2]) if dimensions else None}
    return info
