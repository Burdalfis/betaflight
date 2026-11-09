#!/usr/bin/env python3
"""Record only an owned Gazebo window, encoding explicitly with Nvidia NVENC.

The visible GUI and encoder both use Nvidia hardware. Capture is 2560x1440 at 60 fps by
default and selects only the owned Gazebo drawable, never the root window or a
desktop rectangle. Software OpenGL and software encoding are rejected.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import time
import xml.etree.ElementTree as ET

from run import frame_viewer, native_environment
from runtime import start_child, stop_process

OVERLAY_ROWS=10
DEFAULT_WIDTH=2560
DEFAULT_HEIGHT=1440


def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def gpu_information(gpu=0):
    fields='index,name,uuid,driver_version,encoder.stats.sessionCount,utilization.encoder'
    process=subprocess.run(['nvidia-smi','-i',str(gpu),'--query-gpu='+fields,
                            '--format=csv,noheader,nounits'],capture_output=True,text=True,timeout=10)
    if process.returncode:raise RuntimeError('Nvidia GPU query failed: '+process.stderr.strip())
    return {'fields':fields.split(','),'values':[v.strip() for v in process.stdout.strip().split(',')]}


def probe_nvenc(directory,gpu=0):
    """Perform a real one-second encode, not merely an encoder-list check."""
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
    path=directory/'nvenc-probe.mp4';log=directory/'nvenc-probe.log'
    command=['ffmpeg','-hide_banner','-f','lavfi','-i','testsrc2=size=640x360:rate=60',
        '-t','1','-c:v','h264_nvenc','-gpu',str(gpu),'-preset','p1','-pix_fmt','yuv420p','-y',str(path)]
    info=gpu_information(gpu)
    with log.open('w') as stream:
        process=subprocess.run(command,env=native_environment(),stdout=stream,stderr=subprocess.STDOUT,timeout=30)
    if process.returncode:raise RuntimeError(f'Hardware NVENC probe failed; inspect {log}')
    result=inspect_video(path)
    record={'pass':True,'command':command,'gpu':info,'video':result,'video_sha256':sha(path),
            'encoder':'h264_nvenc','software_encoder_fallback':False}
    (directory/'nvenc-capability.json').write_text(json.dumps(record,indent=2)+'\n')
    return record


def inspect_video(path):
    process=subprocess.run(['ffprobe','-v','error','-select_streams','v:0','-show_entries',
        'stream=codec_name,width,height,r_frame_rate,nb_frames:stream_tags=encoder:format=duration',
        '-of','json',str(path)],capture_output=True,text=True,timeout=15)
    if process.returncode:raise RuntimeError('Cannot inspect encoded video: '+process.stderr)
    result=json.loads(process.stdout)
    if not result.get('streams') or float(result.get('format',{}).get('duration',0))<=0:
        raise RuntimeError('Encoded video has no positive-duration video stream')
    encoder=result['streams'][0].get('tags',{}).get('encoder','')
    if 'h264_nvenc' not in encoder:
        raise RuntimeError('Output video does not retain the required h264_nvenc encoder provenance')
    return result


def gui_config(path,width=DEFAULT_WIDTH,height=DEFAULT_HEIGHT,follow_target=None):
    """Use installed plugin configuration without touching user settings."""
    source=Path('/usr/share/gz/gz-sim/gui/gui.config')
    text=re.sub(r'<\?xml[^>]*\?>','',source.read_text())
    original=ET.fromstring('<root>'+text+'</root>')
    root=ET.Element('root')
    ET.SubElement(root,'dialog',{'name':'quick_start','show_again':'false'})
    window=ET.SubElement(root,'window')
    ET.SubElement(window,'width').text=str(width);ET.SubElement(window,'height').text=str(height)
    ET.SubElement(window,'dialog_on_exit').text='false'
    menus=ET.SubElement(window,'menus');ET.SubElement(menus,'drawer',{'default':'false'})
    wanted={'MinimalScene','GzSceneManager','InteractiveViewControl','CameraTracking','MarkerManager'}
    found=set()
    for plugin in original.findall('plugin'):
        filename=plugin.get('filename')
        if filename not in wanted:continue
        if filename=='MinimalScene':
            plugin.find('engine').text='ogre'
            clip=plugin.find('camera_clip')
            if clip is None:clip=ET.SubElement(plugin,'camera_clip')
            for key,value in (('near','.1'),('far','2000')):
                element=clip.find(key)
                if element is None:element=ET.SubElement(clip,key)
                element.text=value
        if filename=='CameraTracking' and follow_target:
            # The plugin waits for an XML-configured target to appear instead
            # of discarding an early topic request while visuals initialize.
            ET.SubElement(plugin,'follow_target').text=follow_target
        root.append(plugin);found.add(filename)
    if found!=wanted:raise RuntimeError('Installed Gazebo GUI configuration lacks required view/tracking plugins')
    Path(path).write_text('<?xml version="1.0"?>\n'+'\n'.join(ET.tostring(child,encoding='unicode') for child in root)+'\n')


def owned_window(process_group,display=':0'):
    tree=subprocess.run(['xwininfo','-display',display,'-root','-tree'],capture_output=True,text=True,timeout=5)
    if tree.returncode:raise RuntimeError('Cannot enumerate X11 windows: '+tree.stderr.strip())
    candidates=[]
    for match in re.finditer(r'^\s*(0x[0-9a-fA-F]+)\s+"([^"\n]*)"',tree.stdout,re.MULTILINE):
        xid,title=match.groups()
        if 'gazebo' not in title.lower():continue
        prop=subprocess.run(['xprop','-display',display,'-id',xid,'_NET_WM_PID'],capture_output=True,text=True,timeout=3)
        pid=re.search(r'=\s*(\d+)',prop.stdout)
        if not pid:continue
        try:
            if os.getpgid(int(pid[1]))!=process_group:continue
        except ProcessLookupError:continue
        geometry=subprocess.run(['xwininfo','-display',display,'-id',xid],capture_output=True,text=True,timeout=3)
        width=re.search(r'^\s*Width:\s*(\d+)',geometry.stdout,re.MULTILINE)
        height=re.search(r'^\s*Height:\s*(\d+)',geometry.stdout,re.MULTILINE)
        if width and height and int(width[1])>=320 and int(height[1])>=240:
            candidates.append({'window_id':int(xid,16),'pid':int(pid[1]),'title':title,
                               'width':int(width[1]),'height':int(height[1]),'process_group':process_group})
    if len(candidates)>1:candidates.sort(key=lambda row:row['width']*row['height'],reverse=True)
    return candidates[0] if candidates else None


def filter_path(path):
    return str(path).replace('\\','\\\\').replace(':','\\:').replace("'","'\\''")


def keep_capture_visible(window,environment,width=DEFAULT_WIDTH,height=DEFAULT_HEIGHT):
    """Require an owned, mapped drawable; desktop focus is best-effort metadata.

    Qt's Nvidia OpenGL render loop remains current while this window is
    unfocused. Window managers may refuse activation to prevent focus stealing,
    so global keyboard focus cannot determine whether capture is usable.
    """
    if os.getpgid(window['pid'])!=window['process_group']:
        raise RuntimeError('Capture window no longer belongs to this recorder')
    commands=[['wmctrl','-ir',str(window['window_id']),'-b','remove,maximized_vert,maximized_horz'],
              ['wmctrl','-ir',str(window['window_id']),'-e',f'0,80,80,{width},{height}'],
              ['wmctrl','-ir',str(window['window_id']),'-b','add,above'],
              ['xdotool','windowraise',str(window['window_id'])],
              ['wmctrl','-ia',str(window['window_id'])]]
    results=[]
    for command in commands:
        process=subprocess.run(command,env=environment,capture_output=True,text=True,timeout=5)
        results.append({'command':command,'returncode':process.returncode,'stdout':process.stdout,'stderr':process.stderr})
        if process.returncode and command[1:2]!=['-ia']:
            raise RuntimeError('Cannot expose owned Gazebo capture window: '+process.stderr)
    time.sleep(.75)
    actual=owned_window(window['process_group'],environment['DISPLAY'])
    if not actual or actual['window_id']!=window['window_id'] or (actual['width'],actual['height'])!=(width,height):
        raise RuntimeError(f'Owned capture window did not adopt its required {width}x{height} geometry')
    window.update(actual)
    geometry=subprocess.run(['xwininfo','-id',str(window['window_id'])],env=environment,
        capture_output=True,text=True,timeout=5)
    viewable=geometry.returncode==0 and bool(re.search(r'^\s*Map State:\s*IsViewable\s*$',geometry.stdout,re.MULTILINE))
    if not viewable:
        raise RuntimeError('Owned Gazebo capture window is not viewable')
    state=subprocess.run(['xprop','-id',str(window['window_id']),'_NET_WM_STATE'],
        env=environment,capture_output=True,text=True,timeout=5)
    active=subprocess.run(['xprop','-root','_NET_ACTIVE_WINDOW'],env=environment,
        capture_output=True,text=True,timeout=5)
    selected=re.search(r'0x[0-9a-fA-F]+',active.stdout)
    active_owned=bool(selected and int(selected[0],16)==window['window_id'])
    focused=state.returncode==0 and '_NET_WM_STATE_FOCUSED' in state.stdout
    return {'actions':results,'window_state':state.stdout,'active_window':active.stdout,
            'viewable':viewable,'viewability_output':geometry.stdout,
            'active':active_owned,'focused':focused,'focus_required':False,
            'reason':'The owned hardware-rendered drawable is viewable at the requested size; desktop focus is best-effort and scene cadence is audited separately.'}


def hardware_environment(partition,display):
    env=native_environment()
    for key in ('WAYLAND_DISPLAY','LIBGL_ALWAYS_SOFTWARE','LIBGL_ALWAYS_INDIRECT','MESA_LOADER_DRIVER_OVERRIDE'):
        env.pop(key,None)
    env.update(DISPLAY=display,GZ_PARTITION=partition,QT_QPA_PLATFORM='xcb',QT_SCALE_FACTOR='1',
        QT_AUTO_SCREEN_SCALE_FACTOR='0',__NV_PRIME_RENDER_OFFLOAD='1',__GLX_VENDOR_LIBRARY_NAME='nvidia',
        QSG_RHI_BACKEND='opengl',QSG_NO_VSYNC='1',__GL_SYNC_TO_VBLANK='0')
    return env


def read_gui_message(environment,topic):
    process=subprocess.run(['gz','topic','-t',topic,'-e','-d','0.25','--json-output'],
        env=environment,capture_output=True,text=True,timeout=3)
    decoder=json.JSONDecoder();text=process.stdout.strip();messages=[]
    while text:
        value,end=decoder.raw_decode(text);messages.append(value);text=text[end:].lstrip()
    return messages[-1] if messages else None


def camera_matches(status,pose,state,heading):
    if not status or not pose or not state:return False
    def field(camel,snake,default=None):return status.get(camel,status.get(snake,default))
    if field('trackMode','track_mode') not in ('FOLLOW_LOOK_AT',4):return False
    if field('followTarget','follow_target',{}).get('name')!=heading or field('trackTarget','track_target',{}).get('name')!='tricopter':return False
    if any(abs(float(field(camel,snake,0))-1)>1e-6 for camel,snake in
           (('followPgain','follow_pgain'),('trackPgain','track_pgain'))):return False
    offset=field('followOffset','follow_offset',{})
    if any(abs(float(offset.get(key,0))-value)>1e-6 for key,value in zip(('x','y','z'),(-1.2,0,.5))):return False
    yaw=math.radians(state['yaw_deg']);position=pose.get('position',{})
    expected=(state['x_m']-1.2*math.cos(yaw),state['y_m']-1.2*math.sin(yaw),state['z_m']+.5)
    return math.sqrt(sum((float(position.get(key,0))-value)**2 for key,value in zip(('x','y','z'),expected)))<.2


def configure_capture_camera(environment,directory,heading='tricopter_camera_heading'):
    """Require actual tracking-state and camera-position readback before capture."""
    directory=Path(directory)
    request=(f'track_mode: FOLLOW_LOOK_AT follow_target:{{name:{json.dumps(heading)}}} '
             'track_target:{name:"tricopter"} follow_offset:{x:-1.2 y:0 z:0.5} '
             'track_offset:{x:0 y:0 z:0} follow_pgain:1 track_pgain:1')
    command=['gz','topic','-t','/gui/track','-m','gz.msgs.CameraTrack','-p',request]
    record={'command':command,'pass':False,'attempts':[],
            'follow_offset_m':[-1.2,0,.5],'follow_pgain':1,'track_pgain':1,
            'follow_target':heading,'track_target':'tricopter','target_and_mode':'Explicit FOLLOW_LOOK_AT with readback'}
    deadline=time.monotonic()+15
    try:
        while time.monotonic()<deadline:
            process=subprocess.run(command,env=environment,capture_output=True,text=True,timeout=5)
            attempt={'returncode':process.returncode,'stdout':process.stdout,'stderr':process.stderr}
            try:
                attempt['status']=read_gui_message(environment,'/gui/currently_tracked')
                attempt['pose']=read_gui_message(environment,'/gui/camera/pose')
                state=None
                with (directory/'plant.csv').open() as stream:
                    for row in csv.DictReader(stream):
                        try:state={key:float(row[key]) for key in ('x_m','y_m','z_m','yaw_deg')}
                        except (ValueError,TypeError):continue
                attempt['plant_state']=state
                record['pass']=process.returncode==0 and camera_matches(attempt['status'],attempt['pose'],state,heading)
            except (OSError,ValueError,subprocess.TimeoutExpired) as error:attempt['readback_error']=repr(error)
            record['attempts'].append(attempt)
            if record['pass']:return record
            time.sleep(.1)
        raise RuntimeError('Capture chase camera did not pass target/pose readback; inspect video-camera.json')
    finally:(directory/'video-camera.json').write_text(json.dumps(record,indent=2)+'\n')


def overlay_line_path(path,index):return Path(path).with_name(Path(path).stem+f'-line{index:02d}.txt')


def encoder_command(window,path,overlay,display=':0',gpu=0,fps=60):
    if type(window.get('window_id')) is not int or not 0<window['window_id']<=0xffffffff:
        raise ValueError('A non-root owned X11 window ID is required')
    if any(not isinstance(window.get(key),int) or window[key]<2 for key in ('width','height')):
        raise ValueError('Positive window dimensions required')
    if not isinstance(gpu,int) or isinstance(gpu,bool) or gpu<0:raise ValueError('GPU index must be nonnegative')
    if fps not in (24,25,30,50,60):raise ValueError('Unsupported capture frame rate')
    # Separate newline-free files avoid the installed drawtext/Freetype path
    # rendering a visible missing-glyph box for every LF in multiline text.
    scale=window['height']/720
    font_size=max(12,round(20*scale));margin=max(6,round(12*scale));line_height=max(14,round(24*scale))
    lines=["drawtext=fontfile=/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf:textfile='"+
        filter_path(overlay_line_path(overlay,index))+f"':reload=1:expansion=none:fontcolor=white:fontsize={font_size}:box=1:boxcolor=black@0.55:x={margin}:y={margin+index*line_height}"
        for index in range(OVERLAY_ROWS)]
    vf=','.join(lines+['pad=ceil(iw/2)*2:ceil(ih/2)*2'])
    return ['ffmpeg','-hide_banner','-loglevel','info','-thread_queue_size','256','-f','x11grab',
        '-framerate',str(fps),'-draw_mouse','0','-window_id',str(window['window_id']),
        '-video_size',f"{window['width']}x{window['height']}",'-i',display,'-an','-vf',vf,
        '-c:v','h264_nvenc','-gpu',str(gpu),'-preset','p1','-tune','ll','-rc','vbr','-cq','23',
        '-b:v','0','-pix_fmt','yuv420p','-movflags','+faststart','-y',str(path)]


class FlightVideo:
    def __init__(self,directory,partition,gpu=0,fps=60,display=None,width=DEFAULT_WIDTH,height=DEFAULT_HEIGHT):
        if any(type(value) is not int or value<2 or value%2 for value in (width,height)):
            raise ValueError('Capture width and height must be positive even integers')
        self.directory=Path(directory);self.partition=partition;self.gpu=gpu;self.fps=fps
        self.width=width;self.height=height
        self.display=display or os.environ.get('DISPLAY',':0')
        self.gui=self.encoder=None;self.gui_log=self.encoder_log=None;self.record={'pass':False};self.closed=False

    def update(self,text):
        path=self.directory/'video-overlay.txt';temporary=path.with_suffix('.tmp')
        rows=str(text).splitlines()
        if len(rows)>OVERLAY_ROWS:raise ValueError(f'Video overlay supports at most {OVERLAY_ROWS} rows')
        temporary.write_text(str(text)+'\n');temporary.replace(path)
        for index in range(OVERLAY_ROWS):
            line=overlay_line_path(path,index);temporary=line.with_suffix('.tmp')
            temporary.write_text(rows[index] if index<len(rows) else '')
            temporary.replace(line)

    def start(self,frame=True):
        self.directory.mkdir(parents=True,exist_ok=True)
        self.record.update(gpu=gpu_information(self.gpu),display=self.display,
            encoder='h264_nvenc',software_encoder_fallback=False,
            capture_scope='Only the X11 drawable belonging to the owned Gazebo GUI process group',
            renderer='Nvidia PRIME GLX / Ogre1; Nvidia NVENC hardware encoding',
            display_server={'isolated':False,'physical':True,'managed':False},
            requested_capture_dimensions=[self.width,self.height],
            recording_started_wall_monotonic=time.monotonic())
        heading=(ET.parse(self.directory/'world.sdf').findtext(
            "./world/model/plugin[@name='tricopter::TricopterBridge']/camera_heading_model") if frame else None)
        if frame and not heading:raise RuntimeError('World has no camera heading target')
        configuration=self.directory/'video-gui.config'
        gui_config(configuration,width=self.width,height=self.height,follow_target=heading)
        self.update('Preparing simulator video')
        env=hardware_environment(self.partition,self.display)
        self.record['render_environment']={key:env[key] for key in ('DISPLAY','QT_QPA_PLATFORM',
            '__NV_PRIME_RENDER_OFFLOAD','__GLX_VENDOR_LIBRARY_NAME','QSG_RHI_BACKEND','QSG_NO_VSYNC','__GL_SYNC_TO_VBLANK')}
        renderer=subprocess.run(['glxinfo','-B'],env=env,capture_output=True,text=True,timeout=10)
        self.record['opengl_information']={'returncode':renderer.returncode,
            'stdout':renderer.stdout,'stderr':renderer.stderr}
        if renderer.returncode or 'OpenGL renderer string: NVIDIA' not in renderer.stdout or 'direct rendering: Yes' not in renderer.stdout:
            raise RuntimeError('Nvidia hardware OpenGL is required; no software renderer fallback attempted')
        self.gui_log=(self.directory/'video-gui.log').open('w')
        command=['gz','sim','-g','-v','2','--render-engine-gui','ogre','--gui-config',str(configuration)]
        self.record['gui_command']=command
        self.gui=start_child(command,env=env,cwd=self.directory,stdout=self.gui_log,stderr=subprocess.STDOUT)
        deadline=time.monotonic()+30;window=None;stable_since=None;previous=None
        while time.monotonic()<deadline:
            window=owned_window(self.gui.pid,self.display)
            # Qt first maps a temporary size, then applies its layout. Capturing
            # that temporary drawable makes x11grab terminate with BadMatch.
            signature=tuple(window.get(k) for k in ('window_id','width','height')) if window else None
            if signature!=previous:stable_since=time.monotonic();previous=signature
            if window and time.monotonic()-stable_since>=3:break
            time.sleep(.15)
        if not window:raise RuntimeError(f'Owned Gazebo capture window not found; inspect {self.directory/"video-gui.log"}')
        self.record['window']=window
        self.record['window_exposure']=keep_capture_visible(window,env,self.width,self.height)
        if frame:
            frame_viewer(env,self.directory,True)
            self.record['camera']=configure_capture_camera(env,self.directory,heading)
        self.encoder_log=(self.directory/'video-encoder.log').open('w')
        command=encoder_command(window,self.directory/'flight.mp4',self.directory/'video-overlay.txt',
                                self.display,self.gpu,self.fps)
        self.record['encoder_command']=command
        self.encoder=start_child(command,env=env,cwd=self.directory,stdin=subprocess.PIPE,
            stdout=self.encoder_log,stderr=subprocess.STDOUT,text=True)
        time.sleep(.75);self.check()
        return self

    def check(self):
        if self.encoder is not None and self.encoder.poll() is not None:
            raise RuntimeError(f'NVENC recorder exited ({self.encoder.returncode}); inspect {self.directory/"video-encoder.log"}')

    def close(self):
        if self.closed:return self.record
        error=None
        try:
            if self.encoder is not None:
                if self.encoder.poll() is not None:
                    raise RuntimeError('NVENC capture ended before recording was stopped; inspect video-encoder.log')
                if self.encoder.poll() is None:
                    try:self.encoder.communicate('q\n',timeout=15)
                    except subprocess.TimeoutExpired:stop_process(self.encoder);raise RuntimeError('NVENC recorder did not finalize normally')
                self.record['encoder_returncode']=self.encoder.returncode
                if self.encoder.returncode!=0:raise RuntimeError('NVENC recorder failed; no CPU fallback attempted')
                path=self.directory/'flight.mp4'
                self.record['video']=inspect_video(path);self.record['video_sha256']=sha(path)
                stream=self.record['video']['streams'][0]
                if (stream['width'],stream['height'])!=(self.width,self.height):
                    raise RuntimeError('Encoded dimensions differ from the verified owned-window capture geometry')
                self.record['gpu_after_recording']=gpu_information(self.gpu)
                self.record['pass']=True
        except Exception as caught:
            error=caught;self.record['error']=repr(caught)
        finally:
            stop_process(self.encoder);stop_process(self.gui)
            for stream in (self.encoder_log,self.gui_log):
                if stream:stream.close()
            self.record['recording_finished_wall_monotonic']=time.monotonic()
            self.record['all_owned_video_processes_stopped']=all(p is None or p.poll() is not None for p in (self.encoder,self.gui))
            (self.directory/'video-record.json').write_text(json.dumps(self.record,indent=2)+'\n')
            self.closed=True
        if error:raise error
        return self.record


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    parser.add_argument('--partition',help='Existing Gazebo server partition to view')
    parser.add_argument('--probe',action='store_true')
    parser.add_argument('--gpu',type=int,default=0)
    parser.add_argument('--duration',type=float,default=15.)
    parser.add_argument('--fps',type=int,choices=(24,25,30,50,60),default=60)
    parser.add_argument('--width',type=int,default=DEFAULT_WIDTH)
    parser.add_argument('--height',type=int,default=DEFAULT_HEIGHT)
    args=parser.parse_args()
    if args.probe:
        print(json.dumps(probe_nvenc(args.directory,args.gpu),indent=2));return
    if not args.partition:parser.error('--partition is required for recording')
    video=FlightVideo(args.directory.resolve(),args.partition,args.gpu,fps=args.fps,width=args.width,height=args.height)
    try:
        video.start();video.update('Gazebo flight recording — Nvidia NVENC')
        deadline=time.monotonic()+args.duration
        while time.monotonic()<deadline:video.check();time.sleep(.1)
    finally:video.close()


if __name__=='__main__':main()
