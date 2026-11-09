#!/usr/bin/env python3
"""Plot observed prefixes from a stress-summary.json trial ledger.

Accepts entries with directory, variant, profile, case, completed and error.
Actual RC packet timestamps define stage boundaries, including aborted trials.
Plotting never changes simulator state, configuration or controller gains.
"""
from __future__ import annotations

import argparse
from collections import defaultdict
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import tempfile

os.environ.setdefault('MPLCONFIGDIR',str(Path(tempfile.gettempdir())/'tricopter-matplotlib'))
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from compare_metrics import actual_yaw_rate


def read(path):return json.loads(Path(path).read_text())
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def slug(value):return re.sub(r'[^a-zA-Z0-9_.-]+','-',str(value)).strip('-') or 'trial'


def load_trace(path):
    data=np.atleast_1d(np.genfromtxt(path,delimiter=',',names=True,dtype=float))
    required=('sim_time_s','yaw_deg','roll_deg','pitch_deg','gyro_frd_z',
              'pwm0_us','pwm1_us','pwm2_us','servo_pwm_us','joint_angle_deg','servo_target_deg')
    if not data.dtype.names or not set(required)<=set(data.dtype.names):raise ValueError('Missing required plant fields')
    if len(data)<2 or not np.all(np.diff(data['sim_time_s'])>0):raise ValueError('Incomplete/nonmonotonic plant trace')
    if any(not np.all(np.isfinite(data[key])) for key in required):raise ValueError('Nonfinite plant trace')
    return data


def observed_stages(packets,case_name,native):
    """Packet evidence takes priority over planned or incomplete stage lists."""
    selected=[p for p in packets if p.get('case')==case_name and p.get('sim_time_s') is not None]
    stages=[];previous=None
    definitions={stage['name']:stage for case in native.get('cases',[]) if case.get('name')==case_name
                 for stage in case.get('stages',[])}
    for packet in selected:
        name=packet.get('stage','unknown')
        if name==previous:continue
        previous=name
        stages.append({**definitions.get(name,{}),'name':name,
            'actual_start_sim_time_s':float(packet['sim_time_s']),'channels':packet['channels']})
    if not stages:
        stages=[dict(stage) for stage in definitions.values() if stage.get('actual_start_sim_time_s') is not None]
    return stages


def load_trial(entry,summary_directory):
    directory=Path(entry['directory'])
    if not directory.is_absolute():directory=summary_directory/directory
    directory=directory.resolve();native_path=directory/'comparison-run.json'
    native=read(native_path) if native_path.exists() else {}
    case=entry.get('case',native.get('selected_case','unknown'))
    case_name=case.get('name','unknown') if isinstance(case,dict) else str(case)
    packets=read(directory/'rc-packets.json') if (directory/'rc-packets.json').exists() else []
    stages=observed_stages(packets,case_name,native)
    data=load_trace(directory/'plant.csv')
    start=float(stages[0]['actual_start_sim_time_s']) if stages else float(data['sim_time_s'][0])
    end=float(data['sim_time_s'][-1]);end_reason='last recorded plant sample'
    metadata=next((item for item in native.get('cases',[]) if item.get('name')==case_name),{})
    if metadata.get('end_sim_time_s') is not None:
        end=min(end,float(metadata['end_sim_time_s']));end_reason='recorded program completion'
    for packet in packets:
        time=packet.get('sim_time_s')
        if time is not None and float(time)>start and len(packet.get('channels',[]))>4 and packet['channels'][4]<1500:
            if float(time)<end:end=float(time);end_reason='first recorded disarm after program start'
            break
    selected=(data['sim_time_s']>=start)&(data['sim_time_s']<=end)
    data=data[selected]
    if len(data)<2:raise ValueError('No observed program prefix before completion/disarm')
    completed=entry.get('completed') is True
    status='completed' if completed else ('aborted' if entry.get('error') or native.get('error') else 'pending/incomplete')
    profile=entry.get('profile','unspecified')
    if isinstance(profile,dict):profile=profile.get('name',json.dumps(profile,sort_keys=True))
    return {'directory':directory,'variant':str(entry.get('variant',native.get('variant','unknown'))),
        'profile':str(profile),'case':case_name,'status':status,'error':entry.get('error',native.get('error')),
        'native':native,'data':data,'packets':packets,'stages':stages,'start_s':start,'end_s':end,
        'end_reason':end_reason,'plant_sha256':sha(directory/'plant.csv')}


def yaw_curves(data):
    """World CCW Euler yaw is negated to match positive native FRD/CW yaw."""
    heading=np.rad2deg(np.unwrap(np.deg2rad(data['yaw_deg'])))
    return -np.gradient(heading,data['sim_time_s']),np.rad2deg(data['gyro_frd_z'])


def draw(axis,trial,color,label,alpha=1):
    data=trial['data'];x=data['sim_time_s']-trial['start_s'];world,body=yaw_curves(data)
    axis[0].plot(x,world,color=color,lw=1.4,alpha=alpha,label=label+' world CW')
    axis[0].plot(x,body,color=color,lw=.9,ls=':',alpha=alpha,label=label+' body FRD')
    axis[1].plot(x,data['roll_deg'],color=color,lw=1.2,alpha=alpha,label=label+' roll')
    axis[1].plot(x,data['pitch_deg'],color=color,lw=1.0,ls='--',alpha=alpha,label=label+' pitch')
    motors=np.array([(data[f'pwm{i}_us']-1000)/10 for i in range(3)])
    axis[2].plot(x,np.mean(motors,axis=0),color=color,lw=1.6,alpha=alpha,label=label+' mean motor output')
    for index,style in enumerate((':','--','-.')):
        axis[2].plot(x,motors[index],color=color,lw=.65,ls=style,alpha=.55*alpha,
                     label=f'motor {index}' if len(axis[2].lines)<=3 else None)
    axis[3].plot(x,data['joint_angle_deg'],color=color,lw=1.4,alpha=alpha,label=label+' measured basket')
    axis[3].plot(x,data['servo_target_deg'],color=color,lw=.8,ls='--',alpha=.75*alpha,label=label+' servo target')
    packets=[packet for packet in trial['packets'] if packet.get('sim_time_s') is not None and
             trial['start_s']<=float(packet['sim_time_s'])<=trial['end_s']]
    if packets:
        t=np.array([float(p['sim_time_s'])-trial['start_s'] for p in packets])
        axis[2].step(t,[(p['channels'][2]-1000)/10 for p in packets],where='post',color=color,
                     ls='--',lw=1.1,alpha=alpha,label=label+' RC throttle')
        target_stages=[stage for stage in trial['stages'] if 'yaw_pid_target_frd_dps' in stage]
        settings=trial['native'].get('yaw_settings')
        if target_stages:
            times=[float(stage['actual_start_sim_time_s'])-trial['start_s'] for stage in target_stages]
            targets=[float(stage['yaw_pid_target_frd_dps']) for stage in target_stages]
            times.append(trial['end_s']-trial['start_s']);targets.append(targets[-1])
            axis[0].step(times,targets,where='post',color=color,ls='--',lw=.8,alpha=.6,
                         label=label+' recorded ANGLE body target')
        elif settings:
            command=[actual_yaw_rate(p['channels'][3],**settings) for p in packets]
            axis[0].step(t,command,where='post',color=color,ls='--',lw=.65,alpha=.55,
                         label=label+' yaw stick (before ANGLE transform)')
    if trial['status']!='completed':
        for ax,values in ((axis[0],world),(axis[1],data['pitch_deg']),(axis[2],np.mean(motors,axis=0)),(axis[3],data['joint_angle_deg'])):
            ax.scatter([x[-1]],[values[-1]],marker='x',s=28,color=color,zorder=4)


def figure(trials,path,title):
    fig,axis=plt.subplots(4,1,figsize=(13,11),sharex=True)
    palette=plt.get_cmap('tab10')
    for index,trial in enumerate(trials):
        label=f"{trial['variant']} / {trial['profile']}"
        draw(axis,trial,palette(index%10),label)
    if len(trials)==1:
        trial=trials[0]
        for stage in trial['stages']:
            x=float(stage['actual_start_sim_time_s'])-trial['start_s']
            if x<0 or x>trial['end_s']-trial['start_s']:continue
            for ax in axis:ax.axvline(x,color='#aaaaaa',lw=.6,alpha=.7)
            axis[0].text(x,.98,stage['name'],transform=axis[0].get_xaxis_transform(),rotation=90,
                         va='top',ha='right',fontsize=7,color='#555555')
        servo_axis=axis[3].twinx();data=trial['data']
        servo_axis.plot(data['sim_time_s']-trial['start_s'],data['servo_pwm_us'],color='#777777',lw=.6,alpha=.5)
        servo_axis.set_ylabel('Servo command (µs)',color='#666666');servo_axis.spines['top'].set_visible(False)
    labels=('Yaw rate (°/s, CW positive)','Physical roll / pitch (°)','RC throttle / motor output (%)','Basket angle (°)')
    for ax,label in zip(axis,labels):
        ax.set_ylabel(label);ax.grid(True,color='#dddddd',lw=.6)
        ax.spines[['top','right']].set_visible(False);ax.legend(fontsize=7,ncol=2,loc='best')
    axis[2].axhline(70,color='#999999',lw=.8,ls=':');axis[2].set_ylim(-3,103)
    axis[-1].set_xlabel('Simulation time from first observed program stage (s)')
    statuses='; '.join(f"{trial['variant']}/{trial['profile']}: {trial['status']}" for trial in trials)
    fig.suptitle(title,fontsize=13,weight='bold')
    foot=(statuses+'\nWorld yaw: derivative of unwrapped plant heading, CW positive. Body yaw: native FRD gyro. '
          'These differ during tilted flight.\nDashed yaw: recorded ANGLE body target where available, otherwise stick rate before ANGLE transformation. '
          'Motor output uses (PWM−1000)/10; RC throttle is separate. Crosses end aborted prefixes.')
    fig.text(.5,.012,foot,ha='center',fontsize=7.5)
    fig.tight_layout(rect=(0,.08,1,.96))
    fig.savefig(path.with_suffix('.png'),dpi=160);fig.savefig(path.with_suffix('.svg'));plt.close(fig)


def thumbnail(trial,path):
    source=trial['directory']/'flight.mp4'
    if not source.exists():return None
    probe=subprocess.run(['ffprobe','-v','error','-show_entries','format=duration','-of','json',str(source)],
                         capture_output=True,text=True,timeout=15,check=True)
    duration=float(json.loads(probe.stdout)['format']['duration']);at=duration*.5
    subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-y','-ss',str(at),'-i',str(source),
                    '-frames:v','1','-vf','scale=960:-2',str(path)],capture_output=True,text=True,timeout=30,check=True)
    return {'source':str(source),'source_sha256':sha(source),'video_time_s':at,'path':str(path),
            'note':'Midpoint of recorded wall-time video; not asserted to be yaw-event time'}


def analyze(summary_path,output=None,thumbnails=True):
    summary_path=Path(summary_path).resolve()
    if summary_path.is_dir():summary_path=summary_path/'stress-summary.json'
    summary_bytes=summary_path.read_bytes();summary=json.loads(summary_bytes)
    output=Path(output).resolve() if output else summary_path.parent/'plots'
    output.mkdir(parents=True,exist_ok=True)
    entries=summary.get('entries',summary.get('runs',[]))
    if not isinstance(entries,list):raise ValueError('Stress ledger entries must be a list')
    protocol=summary.get('protocol',{})
    scheduled=(len(protocol['cases'])*len(protocol['profiles'])*int(protocol.get('repetitions',1))*2
               if protocol.get('cases') and protocol.get('profiles') else None)
    prefix='PROVISIONAL — ' if scheduled is not None and len(entries)<scheduled else ''
    result={'ledger':str(summary_path),'ledger_sha256':hashlib.sha256(summary_bytes).hexdigest(),'ledger_entries':len(entries),
            'scheduled_trial_count':scheduled,'source_campaign_pass':summary.get('pass'),
            'boundary':'Analysis success describes this exact observed ledger snapshot, not campaign completion or stable flight.',
            'records':[],'figures':[],'analysis_errors':[]};trials=[]
    for index,entry in enumerate(entries):
        record={'entry_index':index,**entry}
        try:
            trial=load_trial(entry,summary_path.parent);trials.append(trial)
            stem=f"{index+1:02d}-{slug(trial['variant'])}-{slug(trial['profile'])}-{slug(trial['case'])}"
            figure([trial],output/stem,prefix+trial['case']+' — '+trial['variant']+' / '+trial['profile'])
            record.update({key:trial[key] for key in ('status','start_s','end_s','end_reason','plant_sha256')})
            record['figure']=str(output/(stem+'.png'));result['figures'].append(record['figure'])
            if thumbnails:
                try:record['thumbnail']=thumbnail(trial,output/(stem+'-video.png'))
                except Exception as error:record['thumbnail_error']=repr(error)
        except Exception as error:
            record['analysis_error']=repr(error);result['analysis_errors'].append({'entry_index':index,'error':repr(error)})
        result['records'].append(record)
    grouped=defaultdict(list)
    for trial in trials:grouped[trial['case']].append(trial)
    for case,members in grouped.items():
        if len(members)<2:continue
        stem='compare-'+slug(case);figure(members,output/stem,prefix+case+' — observed stress responses')
        result['figures'].append(str(output/(stem+'.png')))
        profiles=defaultdict(list)
        for trial in members:profiles[trial['profile']].append(trial)
        for profile,selected in profiles.items():
            if len(selected)<2:continue
            stem='compare-'+slug(case)+'-'+slug(profile)
            figure(selected,output/stem,prefix+case+' — '+profile+' — observed stress responses')
            result['figures'].append(str(output/(stem+'.png')))
    result['plotted_trials']=len(trials)
    result['pass']=bool(trials) and not result['analysis_errors']
    (output/'stress-plots.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('summary',type=Path)
    parser.add_argument('--output',type=Path)
    parser.add_argument('--no-thumbnails',action='store_true')
    args=parser.parse_args();result=analyze(args.summary,args.output,not args.no_thumbnails)
    print(json.dumps(result,indent=2))
    if not result['pass']:raise SystemExit(1)


if __name__=='__main__':main()
