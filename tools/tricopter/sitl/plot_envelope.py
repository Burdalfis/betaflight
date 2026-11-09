#!/usr/bin/env python3
"""Analyze the completed envelope ledger, retaining every aborted program.

Analysis pass means all scheduled outcomes were recognized, including recorded
flight failures. --allow-partial writes explicitly provisional artifacts while
the frozen runner is still active. No simulator/socket/configuration writes.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile

os.environ.setdefault('MPLCONFIGDIR',str(Path(tempfile.gettempdir())/'tricopter-matplotlib'))
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import envelope_metrics as metrics
import envelope_targets

COLORS={'stock':'#0072b2','modified':'#d55e00'}
POINTS=('hover','climb','forward')


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def read(path):return json.loads(path.read_text())


def load_trace(path):
    data=np.genfromtxt(path,delimiter=',',names=True,dtype=float)
    if data.size<2 or not np.all(np.diff(data['sim_time_s'])>0):raise ValueError('Incomplete/nonmonotonic trace')
    return data


def configuration(path):
    import xml.etree.ElementTree as ET
    plugin=ET.parse(path).find(".//plugin[@name='tricopter::TricopterBridge']")
    result={name:float(plugin.findtext(name)) for name in (
        'motor_current_limit_a','motor_brake_current_limit_a','motor_voltage_v','servo_max_torque_nm')}
    result.update({name:list(map(float,plugin.findtext(name).split())) for name in ('pwm_knots_us','angle_knots_deg')})
    return result


def flatten(value,prefix=''):
    if isinstance(value,dict):
        for key,child in value.items():yield from flatten(child,prefix+'.'+key if prefix else key)
    elif isinstance(value,list):
        for index,child in enumerate(value):yield from flatten(child,prefix+'.'+str(index))
    elif value is None or isinstance(value,(int,float)) and not isinstance(value,bool):yield prefix,value


def statistics(values):
    present=[float(value) for value in values if value is not None]
    if any(not math.isfinite(value) for value in present):raise ValueError('Nonfinite metric')
    return {'n':len(present),'n_observed':len(values),'n_censored':len(values)-len(present),
        'mean':float(np.mean(present)) if present else None,'min':min(present) if present else None,
        'max':max(present) if present else None}


def draw_repeats(axis,curves,color,label):
    if not curves:return
    for x,y in curves:axis.plot(x,y,color=color,alpha=.3,lw=.65)
    start=max(x[0] for x,_ in curves);end=min(x[-1] for x,_ in curves)
    if end<=start:return
    grid=np.arange(math.ceil(start*100),math.floor(end*100)+1)*.01
    if len(grid)<2:return
    values=np.array([np.interp(grid,x,y) for x,y in curves])
    axis.fill_between(grid,np.min(values,axis=0),np.max(values,axis=0),color=color,alpha=.12,lw=0)
    axis.plot(grid,np.median(values,axis=0),color=color,lw=1.9,label=label)


def _style(fig,title,foot):
    fig.suptitle(title,fontsize=13,weight='bold')
    for axis in fig.axes:
        axis.grid(True,color='#dddddd',lw=.6);axis.spines[['top','right']].set_visible(False)
    fig.text(.5,.012,foot,ha='center',fontsize=8)
    footer_space=.035+(foot.count('\n')+1)*.14/fig.get_size_inches()[1]
    fig.tight_layout(rect=(0,footer_space,1,.95))


def save(fig,path):
    fig.savefig(path.with_suffix('.png'),dpi=170);fig.savefig(path.with_suffix('.svg'));plt.close(fig)


def coordinate_context(protocol):
    identities=protocol.get('binary_identities',{})
    adapters=[record.get('sitl_adapter') for record in identities.values()]
    if any(adapters):
        if (set(identities)!=set(COLORS) or not all(adapters)
                or any(adapter.get('name')!='corrected-frame' for adapter in adapters)
                or len({adapter.get('patch_sha256') for adapter in adapters})!=1):
            raise ValueError('Cannot classify inconsistent declared common SITL adapters')
        return {'classification':'corrected-frame-common-SITL-adapter',
                'patch_sha256':adapters[0]['patch_sha256'],
                'figure_note':'Stock controller and modified controller use the identical common SITL coordinate adapter.'}
    return {'classification':'legacy-SITL-coordinate-path', 'patch_sha256':None,
            'figure_note':'Legacy gyro/IMU frame inconsistency limits physical and mixer attribution.'}


def make_figures(directory,prefix,records,pairs,aggregates,protocol):
    operating_points=tuple(point for point in POINTS if any(record['operating_point']==point for record in records))
    context=coordinate_context(protocol)
    counts={status:sum(record['status']==status for record in records) for status in ('completed','failed','pending')}
    ledger=f"Ledger: {counts['completed']} completed, {counts['failed']} aborted, {counts['pending']} pending / {len(records)} scheduled."
    def style(fig,title,foot):
        _style(fig,('PROVISIONAL / INCOMPLETE — ' if prefix else '')+title,ledger+'\n'+foot+'\n'+context['figure_note'])
    # Actual traces only; aborted observations terminate at their RC disarm
    # boundary and never acquire a fabricated full-step metric.
    fig,axes=plt.subplots(len(operating_points),2,figsize=(12,3*len(operating_points)+1),squeeze=False)
    for i,point in enumerate(operating_points):
        for j,direction in enumerate(('cw','ccw')):
            axis=axes[i,j]
            for variant,color in COLORS.items():
                selected=[r for r in records if(r['operating_point'],r['direction'],r['variant'])==(point,direction,variant)]
                completed=[];fail_label=False
                for record in selected:
                    if 'trace' not in record or record.get('analysis_error'):continue
                    if record['status']=='completed':
                        pulse=record['metrics']['windows']['pulse'];onset=pulse['start_s'];end=record['case']['end_sim_time_s']
                        mask=(record['trace']['sim_time_s']>=onset-.3)&(record['trace']['sim_time_s']<=end)
                        completed.append((record['trace']['sim_time_s'][mask]-onset,record['trace']['gyro_frd_z'][mask]*180/math.pi))
                        target=record['metrics']['targets']['requested_yaw_rate_dps'];release=pulse['end_s']-onset
                        axis.plot([-.3,0,0,release,release,end-onset],[0,0,target,target,0,0],
                                  color=color,ls='--',lw=.6,alpha=.35)
                    else:
                        observed=record.get('prefix') or {};onset=observed.get('yaw_onset_s');end=observed.get('observed_end_s')
                        if onset is None or end is None:continue
                        mask=(record['trace']['sim_time_s']>=onset-.3)&(record['trace']['sim_time_s']<=end)
                        x=record['trace']['sim_time_s'][mask]-onset;y=record['trace']['gyro_frd_z'][mask]*180/math.pi
                        axis.plot(x,y,color=color,lw=1,alpha=.75,label=variant+' aborted prefix' if not fail_label else None)
                        axis.scatter([x[-1]],[y[-1]],color=color,marker='x',s=35);fail_label=True
                        target=record['planned']['stages'][2]['requested_yaw_rate_dps']
                        axis.plot([-.3,0,0,end-onset],[0,0,target,target],color=color,ls='--',lw=.6,alpha=.35)
                draw_repeats(axis,completed,color,variant+' completed')
            axis.set(title=point+' / '+direction.upper(),xlabel='Time from actual yaw onset (s)',ylabel='Native body-FRD yaw rate (°/s)')
            if axis.lines:axis.legend(fontsize=7)
    style(fig,'Operating-envelope yaw responses; crosses mark aborted observed prefixes',
          'Dashed requests use actual onset/release and ANGLE earth-reference attenuation. Completed: median/min–max, not CI; failed: bounded individual prefixes.')
    save(fig,directory/(prefix+'envelope-yaw-rate'))
    fig,axes=plt.subplots(2,2,figsize=(11,7))
    for col,direction in enumerate(('cw','ccw')):
        for row,(metric,label,scale) in enumerate((('yaw.rise_time_10_90_s','10–90% body-rate rise (ms)',1000),
                                                ('yaw.overshoot_percent_of_command','Target overshoot (%)',1))):
            for variant,color in COLORS.items():
                x=[];y=[];lower=[];upper=[]
                for index,point in enumerate(operating_points):
                    item=next((a for a in aggregates if(a['variant'],a['case'],a['family'],a['metric'])==(variant,point+'_'+direction,'completed',metric)),None)
                    if item and item['mean'] is not None:
                        x.append(index+(-.06 if variant=='stock' else .06));y.append(item['mean']*scale)
                        lower.append((item['mean']-item['min'])*scale);upper.append((item['max']-item['mean'])*scale)
                if x:axes[row,col].errorbar(x,y,yerr=[lower,upper],fmt='o-',color=color,capsize=3,label=variant)
            axes[row,col].set_xticks(range(len(operating_points)),operating_points);axes[row,col].set(title=direction.upper(),ylabel=label)
            if any(record['status']=='failed' and record['direction']==direction for record in records):
                axes[row,col].text(.98,.97,'Aborted full responses\nhave no point',transform=axes[row,col].transAxes,ha='right',va='top',fontsize=7)
            if axes[row,col].lines:axes[row,col].legend(fontsize=8)
    style(fig,'Achieved-regime response comparison with unchanged hover gains','Failure counts and achieved speeds are retained separately; commands do not establish a steady operating point.')
    save(fig,directory/(prefix+'envelope-response-summary'))
    fig,axes=plt.subplots(2,len(operating_points),figsize=(max(10,4*len(operating_points)+1),7),squeeze=False)
    for col,point in enumerate(operating_points):
        for row,(field,label) in enumerate((('horizontal_speed_m_s','Pre-yaw horizontal ground speed (m/s)'),('vz_m_s','Pre-yaw vertical speed (m/s)'))):
            for variant,color in COLORS.items():
                labeled=False
                for index,direction in enumerate(('control','cw','ccw')):
                    selected=[r for r in records if(r['operating_point'],r['direction'],r['variant'])==(point,direction,variant)]
                    points=[]
                    for record in selected:
                        window=(record.get('metrics',{}).get('windows') or record.get('prefix',{}).get('windows',{})).get('pre_pulse')
                        if window:points.append(window['kinematics'][field]['mean'])
                    if points:
                        x=index+(-.08 if variant=='stock' else .08);center=float(np.mean(points))
                        axes[row,col].scatter([x]*len(points),points,color=color,s=16,alpha=.5)
                        axes[row,col].errorbar([x],[center],yerr=[[center-min(points)],[max(points)-center]],fmt='o',color=color,capsize=3,
                                                label=variant if not labeled else None)
                        labeled=True
            axes[row,col].set(title=point,xlabel='Planned yaw case',ylabel=label);axes[row,col].set_xticks(range(3),('control','CW','CCW'))
            if axes[row,col].lines:axes[row,col].legend(fontsize=8)
    style(fig,'Actually achieved operation during the final 0.3 seconds before yaw','Includes valid pre-yaw observations from later-aborted trials; all speeds are simulated ground-relative observations, not commanded/steady airspeed.')
    save(fig,directory/(prefix+'envelope-achieved-operation'))
    fig,axes=plt.subplots(len(operating_points),2,figsize=(12,3*len(operating_points)+1),squeeze=False)
    for i,point in enumerate(operating_points):
        for j,direction in enumerate(('cw','ccw')):
            for variant,color in COLORS.items():
                curves=[]
                for pair in pairs:
                    if(pair['operating_point'],pair['direction'],pair['variant'])!=(point,direction,variant):continue
                    matched=pair['metrics'];r=matched['curves']['roll_deg'];p=matched['curves']['pitch_deg']
                    curves.append((np.array(matched['time_from_yaw_onset_s']),np.hypot(r,p)))
                draw_repeats(axes[i,j],curves,color,variant)
            axes[i,j].set(title=point+' / '+direction.upper(),xlabel='Time from yaw onset (s)',ylabel='Matched Euler RP difference norm (°)')
            if axes[i,j].lines:axes[i,j].legend(fontsize=8)
            else:axes[i,j].text(.5,.5,'No completed yaw/control response\nAborted prefixes shown separately',transform=axes[i,j].transAxes,ha='center',fontsize=9)
    style(fig,'Yaw response minus matched no-yaw control at equal operating-lead age','Own pre-event means removed; pre-event offsets/trends retained. Raw body RP gyro during tilted yaw includes intended coordinate kinematics.')
    save(fig,directory/(prefix+'envelope-attitude-coupling'))
    attitude_point='forward' if 'forward' in operating_points else operating_points[-1]
    fig,axes=plt.subplots(2,2,figsize=(12,8))
    for col,direction in enumerate(('cw','ccw')):
        for row,axis_name in enumerate(('roll','pitch')):
            axis=axes[row,col]
            for variant,color in COLORS.items():
                for record in records:
                    if ((record['operating_point'],record['direction'],record['variant'])!=(attitude_point,direction,variant)
                            or 'trace' not in record or record.get('analysis_error')):continue
                    if record['status']=='completed':
                        onset=record['metrics']['windows']['pulse']['start_s'];end=record['case']['end_sim_time_s'];fw=record['firmware_attitude']
                    else:
                        observed=record.get('prefix',{});onset=observed.get('yaw_onset_s');end=observed.get('observed_end_s');fw=observed.get('firmware_attitude',{})
                    if onset is None or end is None:continue
                    data=record['trace'];mask=(data['sim_time_s']>=onset-.3)&(data['sim_time_s']<=end)
                    axis.plot(data['sim_time_s'][mask]-onset,data[axis_name+'_deg'][mask],color=color,alpha=.65,lw=1.1)
                    samples=[sample for sample in fw.get('samples',[]) if onset-.3<=sample['sim_time_s']<=end]
                    axis.plot([s['sim_time_s']-onset for s in samples],[s[axis_name+'_firmware_deg'] for s in samples],color=color,ls=':',marker='.',ms=3,lw=.8)
            axis.axhline(40,color='#999999',ls='--',lw=.7);axis.axhline(-40,color='#999999',ls='--',lw=.7)
            axis.set(title=attitude_point+' / '+direction.upper(),xlabel='Time from actual yaw onset (s)',ylabel=axis_name+' Euler angle (°)')
    for variant,color in COLORS.items():axes[0,0].plot([],[],color=color,label=variant+' ground truth');axes[0,0].plot([],[],color=color,ls=':',label=variant+' native estimate')
    axes[0,0].legend(fontsize=7)
    style(fig,attitude_point.capitalize()+' attitude: actual physical response versus native estimate','Solid: plant truth; dotted: native MSP attitude. No extrapolation after disarm; ±40° retained safety bound.')
    save(fig,directory/(prefix+('envelope-forward-attitude' if attitude_point=='forward' else 'envelope-attitude-estimation')))


def analyze(directory,allow_partial=False):
    directory=Path(directory).resolve();summary_path=directory/'envelope-summary.json';summary_bytes=summary_path.read_bytes()
    summary=json.loads(summary_bytes);summary_digest=hashlib.sha256(summary_bytes).hexdigest()
    complete=summary.get('all_owned_processes_stopped') is True
    if not complete and not allow_partial:raise ValueError('Campaign still running; use --allow-partial for provisional analysis')
    protocol=summary['protocol'];definitions={case['name']:case for case in protocol['cases']}
    records=[];pairs=[];flat=[];errors=[];groups={g['group_directory']:g for g in summary['runs']}
    for index,variant in enumerate(protocol['variant_order']):
        group_name=f'{index+1:02d}-{variant}';group=groups.get(group_name,{})
        done={case['name']:case for case in group.get('cases',[])};failed={item['case']:item for item in group.get('failures',[])}
        for name,planned in definitions.items():
            record={'group':group_name,'variant':variant,'case_name':name,'operating_point':planned['operating_point'],'direction':planned['direction']}
            if name not in done and name not in failed:
                records.append({**record,'status':'pending'});continue
            path=directory/group_name/name;record['directory']=str(path)
            record['status']='completed' if name in done else 'failed';record['failure']=failed.get(name)
            try:
                native=read(path/'comparison-run.json');data=load_trace(path/'plant.csv');config=configuration(path/'world.sdf')
                telemetry=read(path/'telemetry.json');record['trace']=data;record['trace_sha256']=sha(path/'plant.csv')
                record['input_sha256']={name:sha(path/name) for name in ('comparison-run.json','plant.csv','world.sdf','telemetry.json','rc-packets.json')}
                planned={**planned,'stages':[{**stage,**envelope_targets.angle_targets(native,*[stage['channels'][i] for i in (0,1,3)],
                    earth_ref=native.get('angle_earth_ref',protocol['angle_earth_ref']))} for stage in planned['stages']]}
                record['planned']=planned
                if name in done:
                    record['case']=done[name];record['metrics']=metrics.operating_metrics(data,done[name],config)
                    record['firmware_attitude']=metrics.firmware_attitude_metrics(data,telemetry,done[name])
                    for metric,value in flatten({'yaw':record['metrics']['yaw'],
                        'windows':record['metrics']['windows'],'full_program_observations':record['metrics']['full_program_observations'],
                        'firmware_windows':record['firmware_attitude']['windows']}):
                        flat.append({**{key:record[key] for key in ('group','variant','case_name')},'family':'completed','metric':metric,'value':value})
                else:
                    packets=read(path/'rc-packets.json')
                    record['prefix']=metrics.failed_prefix_metrics(data,planned,packets,telemetry,config,
                        angle_limit_deg=protocol['maximum_absolute_roll_pitch_deg'])
                pre=(record.get('metrics',{}).get('windows') or record.get('prefix',{}).get('windows',{})).get('pre_pulse')
                if pre:
                    for metric,value in flatten(pre['kinematics']):
                        flat.append({**{key:record[key] for key in ('group','variant','case_name')},'family':'achieved_pre_pulse','metric':metric,'value':value})
            except Exception as error:
                record['analysis_error']=repr(error);errors.append({'group':group_name,'case':name,'error':repr(error)})
            records.append(record)
    # Only successful complete programs acquire full matched-response metrics.
    candidates=[]
    for record in records:
        if record['status']!='completed' or 'metrics' not in record or record.get('analysis_error') or record['direction']=='control':continue
        control=next((r for r in records if r['group']==record['group'] and r['case_name']==record['planned']['matched_control'] and r['status']=='completed' and 'metrics' in r and not r.get('analysis_error')),None)
        if control is None:continue
        case=record['case'];base=control['case'];start=case['stages'][2]['actual_start_sim_time_s'];op=case['stages'][1]['actual_start_sim_time_s']
        equivalent=base['stages'][1]['actual_start_sim_time_s']+start-op
        candidates.append((record,control,min(case['end_sim_time_s']-start,base['end_sim_time_s']-equivalent)))
    windows={point:min(value for record,_,value in candidates if record['operating_point']==point) for point in POINTS if any(record['operating_point']==point for record,_,_ in candidates)}
    for record,control,_ in candidates:
        matched=metrics.matched_control_metrics(record['trace'],record['case'],control['trace'],control['case'],windows[record['operating_point']])
        pairs.append({'group':record['group'],'variant':record['variant'],'operating_point':record['operating_point'],'direction':record['direction'],'case_name':record['case_name'],'metrics':matched})
        for metric,value in flatten({'phases':matched['phases'],'pre_event_diagnostics':matched['pre_event_diagnostics']}):
            flat.append({**{key:record[key] for key in ('group','variant','case_name')},'family':'matched_control','metric':metric,'value':value})
    aggregates={}
    for row in flat:
        key=tuple(row[name] for name in ('variant','case_name','family','metric'));aggregates.setdefault(key,[]).append(row['value'])
    aggregated=[dict(zip(('variant','case','family','metric'),key),**statistics(values),n_expected=protocol['repetitions']) for key,values in sorted(aggregates.items())]
    prefix='' if complete else 'partial-';make_figures(directory,prefix,records,pairs,aggregated,protocol)
    serialized=[{key:value for key,value in record.items() if key!='trace'} for record in records]
    counts={status:sum(record['status']==status for record in records) for status in ('completed','failed','pending')}
    report={'pass':complete and counts['pending']==0 and not errors,'analysis_complete':complete,
        'all_flight_programs_passed':summary.get('pass',False),'classification':'complete outcome ledger including recognized flight failures' if complete else 'PROVISIONAL incomplete outcome ledger',
        'source_summary_sha256':summary_digest,'analysis_source_sha256':{name:sha(path) for name,path in (
            ('plot_envelope.py',Path(__file__)),('envelope_metrics.py',Path(metrics.__file__)),('envelope_targets.py',Path(envelope_targets.__file__)),
            ('compare_metrics.py',Path(metrics.yaw_response.__code__.co_filename)))},
        'protocol':protocol,'coordinate_context':coordinate_context(protocol),
        'outcome_counts':counts,'analysis_errors':errors,'records':serialized,
        'matched_pairs':pairs,'aggregate_metrics':aggregated,
        'boundary':'Flight failures are outcomes, not discarded repeats. Failed prefixes never acquire full-target metrics. Physical Euler tracking and native estimated attitude remain separate; no steady-speed or physical hardware claim.'}
    (directory/(prefix+'envelope-analysis.json')).write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    with (directory/(prefix+'envelope-aggregate.csv')).open('w') as stream:
        keys=('variant','case','family','metric','n','n_observed','n_censored','n_expected','mean','min','max')
        writer=csv.DictWriter(stream,fieldnames=keys);writer.writeheader();writer.writerows(aggregated)
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('directory',type=Path)
    parser.add_argument('--allow-partial',action='store_true');args=parser.parse_args()
    report=analyze(args.directory,args.allow_partial)
    print(json.dumps({'analysis_pass':report['pass'],'flight_programs_all_passed':report['all_flight_programs_passed'],
        'outcomes':report['outcome_counts'],'analysis_errors':report['analysis_errors']},indent=2))
    if report['analysis_errors'] or (not args.allow_partial and not report['pass']):raise SystemExit(1)


if __name__=='__main__':main()
