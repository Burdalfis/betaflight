"""Finite-window yaw stress screening from recorded plant/RC stages.

Patterns are descriptive screening flags, not proof of asymptotic PID stability.
Native yaw is body-FRD; physical world-yaw rate is reconstructed separately.
No flight outputs, recordings, or runtime settings are changed here.
"""
from __future__ import annotations

import csv
import json
import math
from pathlib import Path
import struct

from compare_metrics import _mean, _rms, _series, _window
from envelope_metrics import body_to_euler_rates
from envelope_targets import angle_targets

SCHEMA_VERSION = 5
DEFAULT_CONFIG = {
    'transient_exclusion_s': .5,
    'oscillation_band_dps': 5.,
    'minimum_full_cycles': 4,
    'minimum_oscillation_span_s': 1.,
    'sustained_amplitude_ratio': .8,
    'oscillation_tail_quiet_s': .5,
    'divergence_window_s': .5,
    'divergence_each_growth': 1.25,
    'divergence_total_growth': 2.,
    'divergence_final_rms_dps': 30.,
    'spin_world_rate_dps': 90.,
    'spin_duration_s': .5,
    'maximum_sample_gap_s': .05,
    'motor_lower_pwm_us': 1050.,
    'motor_upper_pwm_us': 2000.,
    'servo_lower_pwm_us': 1100.,
    'servo_upper_pwm_us': 1700.,
    'entry_window_s': 2.,
}


def _config(config):
    result = dict(DEFAULT_CONFIG)
    if config:
        unknown = set(config)-set(result)
        if unknown: raise ValueError('Unknown stress metric setting: '+str(sorted(unknown)))
        result.update(config)
    if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or v<=0
           for v in result.values()): raise ValueError('Stress thresholds must be positive and finite')
    if not isinstance(result['minimum_full_cycles'],int) or result['minimum_full_cycles']<4:
        raise ValueError('At least four complete oscillation cycles required')
    if not 0<result['sustained_amplitude_ratio']<=1: raise ValueError('Invalid sustained amplitude ratio')
    if result['motor_lower_pwm_us']>=result['motor_upper_pwm_us'] or result['servo_lower_pwm_us']>=result['servo_upper_pwm_us']:
        raise ValueError('Invalid actuator bounds')
    return result


def _peaks(series,band):
    """Alternating above-band lobe extrema; neutral samples do not add peaks."""
    result=[];current=None
    for t,value in series:
        sign=1 if value>band else -1 if value<-band else 0
        if not sign: continue
        if current is None: current=[t,value,sign]
        elif sign==current[2]:
            if abs(value)>abs(current[1]): current=[t,value,sign]
        else: result.append(current[:2]);current=[t,value,sign]
    if current is not None: result.append(current[:2])
    return result



def _confirmed_peak_runs(series,peaks):
    """Require a recorded reversal on both sides of each lobe extremum.

    An interval may begin after an extremum or end while a lobe is still
    growing. Such endpoint candidates cannot close a confirmed full cycle.
    Equal-valued plateaus are accepted only when both plateau edges are seen.
    """
    indices={t:i for i,(t,_) in enumerate(series)};evidence=[];runs=[];current=[]
    for stamp,value in peaks:
        i=indices[stamp];sign=1 if value>0 else -1;amplitude=abs(value)
        tolerance=1e-9*max(1.,amplitude);left=right=i
        while left>0 and abs(sign*series[left-1][1]-amplitude)<=tolerance:left-=1
        while right+1<len(series) and abs(sign*series[right+1][1]-amplitude)<=tolerance:right+=1
        rise=left>0 and sign*series[left-1][1]<amplitude-tolerance
        fall=right+1<len(series) and sign*series[right+1][1]<amplitude-tolerance
        confirmed=bool(rise and fall)
        evidence.append({'time_s':stamp,'value_dps':value,'confirmed':confirmed,
            'left_reversal_observed':bool(rise),'right_reversal_observed':bool(fall),
            'plateau_start_s':series[left][0],'plateau_end_s':series[right][0]})
        if confirmed:current.append([stamp,value])
        elif current:runs.append(current);current=[]
    if current:runs.append(current)
    return evidence,runs


def _cycles(peaks):
    return [{'start_s':peaks[i][0],'end_s':peaks[i+2][0],
        'amplitude_dps':sum(abs(peaks[j][1]) for j in (i,i+1,i+2))/3}
        for i in range(0,len(peaks)-2,2)]


def _longest_over(series,threshold):
    """Same-sign excursions, linearly interpolated at threshold crossings."""
    longest=0.;run=0.;previous_sign=0;previous_end=None
    for (a,x),(b,y) in zip(series,series[1:]):
        pieces=[]
        for sign in (-1,1):
            u,v=sign*x-threshold,sign*y-threshold
            if u<0 and v<0:continue
            first=a if u>=0 else a+(b-a)*(-u)/(v-u)
            last=b if v>=0 else a+(b-a)*u/(u-v)
            if last>first:pieces.append((first,last,sign))
        for first,last,sign in sorted(pieces):
            run=run+(last-first) if previous_sign==sign and previous_end==first else last-first
            longest=max(longest,run);previous_sign=sign;previous_end=last
    return longest


def classify_yaw_series(error_series,world_yaw_series=None,*,zero_yaw_command=True,config=None):
    """Screen one already-trimmed constant-input interval.

    Four full cycles require nine alternating extrema with recorded reversals
    on both sides; boundary-censored candidates are reported separately. Amplitude is the mean
    absolute value of a cycle's three extrema; the first/last two full cycles
    are compared over the entire supplied interval, without searching for a
    favorable subwindow. A quiet final tail makes the pattern decaying ringing.
    """
    cfg=_config(config)
    rows=[{'sim_time_s':float(t),'error':float(v)} for t,v in error_series]
    error=_series(rows,'error');duration=error[-1][0]-error[0][0]
    gap=max(b[0]-a[0] for a,b in zip(error,error[1:]))
    peak=_peaks(error,cfg['oscillation_band_dps'])
    evidence,runs=_confirmed_peak_runs(error,peak)
    candidates=_cycles(peak)
    cycles=[cycle for run in runs for cycle in _cycles(run)]
    span=cycles[-1]['end_s']-cycles[0]['start_s'] if cycles else 0.
    ratio=(sum(c['amplitude_dps'] for c in cycles[-2:])/sum(c['amplitude_dps'] for c in cycles[:2])) if len(cycles)>=2 else None
    above_band_times=[t for t,value in error if abs(value)>cfg['oscillation_band_dps']]
    tail_active=bool(above_band_times and error[-1][0]-above_band_times[-1]<=cfg['oscillation_tail_quiet_s']+1e-9)
    enough=len(cycles)>=cfg['minimum_full_cycles'] and span>=cfg['minimum_oscillation_span_s']
    sustained=bool(enough and tail_active and ratio is not None and ratio>=cfg['sustained_amplitude_ratio'])
    ringing=bool(len(cycles)>=1 and not sustained and (not tail_active or (ratio is not None and ratio<cfg['sustained_amplitude_ratio'])))
    bins=[];width=cfg['divergence_window_s'];i=0
    while error[0][0]+(i+1)*width<=error[-1][0]+1e-9:
        a=error[0][0]+i*width;b=min(error[-1][0],a+width)
        bins.append({'start_s':a,'end_s':b,'rms_dps':_rms(_window(error,a,b))});i+=1
    growth=[]
    for a,b,c in zip(bins,bins[1:],bins[2:]):
        x,y,z=[v['rms_dps'] for v in (a,b,c)]
        if (x>1e-9 and y>=cfg['divergence_each_growth']*x and z>=cfg['divergence_each_growth']*y
            and z>=cfg['divergence_total_growth']*x and z>=cfg['divergence_final_rms_dps']):
            growth.append({'start_s':a['start_s'],'end_s':c['end_s'],'rms_dps':[x,y,z]})
    spin_duration=None
    if world_yaw_series is not None:
        world=_series([{'sim_time_s':t,'rate':v} for t,v in world_yaw_series],'rate')
        world=_window(world,error[0][0],error[-1][0])
        spin_duration=_longest_over(world,cfg['spin_world_rate_dps'])
    available=gap<=cfg['maximum_sample_gap_s']+1e-9
    spin=bool(zero_yaw_command and spin_duration is not None and spin_duration>=cfg['spin_duration_s']-1e-9)
    label=('sustained_oscillation' if sustained else 'decaying_ringing' if ringing else 'no_oscillation_pattern')
    if growth: label='divergent_growth_candidate'
    if spin: label='unwanted_world_yaw_spin'
    if not available: label='sampling_gap_unclassified'
    return {'classification':label,'classification_available':available,'duration_s':duration,
        'maximum_sample_gap_s':gap,'native_yaw_error_mean_dps':_mean(error),
        'native_yaw_error_rms_dps':_rms(error),'native_yaw_error_peak_abs_dps':max(abs(v) for _,v in error),
        'alternating_peaks':peak,'peak_confirmation':evidence,
        'candidate_complete_cycles':candidates,'candidate_full_cycle_count':len(candidates),
        'confirmed_alternating_peaks':[p for run in runs for p in run],
        'complete_cycles':cycles,'full_cycle_count':len(cycles),
        'oscillation_span_s':span,'last_two_to_first_two_cycle_amplitude_ratio':ratio,'oscillation_active_at_tail':tail_active,
        'sustained_oscillation':sustained if available else None,'decaying_ringing':ringing if available else None,
        'divergent_growth_candidate':bool(growth) if available else None,'divergence_windows':growth,
        'unwanted_world_yaw_spin':spin if available else None,'longest_world_spin_excursion_s':spin_duration,
        'spin_test_applied':bool(zero_yaw_command and world_yaw_series is not None),
        'boundary':'Finite-window threshold screening; an overshoot or growing transient alone is not proof of unstable PID poles.'}


def _saturation(rows,start,end,cfg):
    chosen=[r for r in rows if start<=float(r['sim_time_s'])<=end]
    def occupancy(names,predicate):
        if not chosen or any(name not in row for row in chosen for name in names): return None
        flags=[any(predicate(float(row[name])) for name in names) for row in chosen]
        spans=[];a=None
        for i,flag in enumerate(flags):
            if flag and a is None:a=i
            if a is not None and (not flag or i==len(flags)-1):
                j=i if flag else i-1;spans.append(float(chosen[j]['sim_time_s'])-float(chosen[a]['sim_time_s']));a=None
        return {'sample_fraction':sum(flags)/len(flags),'longest_contiguous_sample_span_s':max(spans,default=0.)}
    return {'motor_upper':occupancy([f'pwm{i}_us' for i in range(3)],lambda v:v>=cfg['motor_upper_pwm_us']-1),
        'motor_lower':occupancy([f'pwm{i}_us' for i in range(3)],lambda v:v<=cfg['motor_lower_pwm_us']+1),
        'servo_endpoint':occupancy(['servo_pwm_us'],lambda v:v<=cfg['servo_lower_pwm_us']+1 or v>=cfg['servo_upper_pwm_us']-1)}


def _target(stage,record):
    for key in ('yaw_pid_target_frd_dps','requested_yaw_rate_dps'):
        if key in stage:
            value=float(stage[key])
            if not math.isfinite(value):raise ValueError('Nonfinite yaw reference')
            return value
    if record is None:raise ValueError('Held yaw target or retained MSP profile required')
    channels=stage['channels'];midrc=1500
    if '44' in record.get('common_msp_readback',{}):midrc=struct.unpack_from('<H',bytes.fromhex(record['common_msp_readback']['44']),3)[0]
    return angle_targets(record,channels[0],channels[1],channels[3],earth_ref=record['angle_earth_ref'],midrc=midrc)['yaw_pid_target_frd_dps']


def entry_statistics(rows,start,end,*,channels=None,record=None):
    """Achieved operation; no automatic proof that a high-throttle trim exists."""
    t=_series(rows,'vx_m_s');_window(t,start,end)
    fields={name:_window(_series(rows,name),start,end) for name in ('vx_m_s','vy_m_s','vz_m_s','roll_deg','pitch_deg','gyro_frd_x','gyro_frd_y','gyro_frd_z')}
    stamps=[a for a,_ in fields['vx_m_s']];selected=[]
    for i,stamp in enumerate(stamps):
        row={name:values[i][1] for name,values in fields.items()}
        rates=body_to_euler_rates(row['roll_deg'],row['pitch_deg'],tuple(row['gyro_frd_'+axis] for axis in 'xyz'))
        selected.append((stamp,row,math.hypot(row['vx_m_s'],row['vy_m_s']),rates))
    accel=[]
    for (a,x,_,_),(b,y,_,_) in zip(selected,selected[1:]):
        accel.append((b-a,math.sqrt(sum(((y[k]-x[k])/(b-a))**2 for k in ('vx_m_s','vy_m_s','vz_m_s')))))
    speeds=[(stamp,speed) for stamp,_,speed,_ in selected]
    motor={}
    for i in range(3):
        if all(f'pwm{i}_us' in r for r in rows):
            s=_window(_series(rows,f'pwm{i}_us'),start,end)
            motor[str(i)]={'pwm_mean_us':_mean(s),'pwm_min_us':min(v for _,v in s),'pwm_max_us':max(v for _,v in s),
                'modeled_duty_mean':_mean([(a,max(0,min(1,(b-1000)/1000))) for a,b in s])}
    minimum,maximum=min(v for _,v in speeds),max(v for _,v in speeds);mean=_mean(speeds)
    demand=None
    if channels is not None:
        mincheck=1050
        if record and '44' in record.get('common_msp_readback',{}):mincheck=struct.unpack_from('<H',bytes.fromhex(record['common_msp_readback']['44']),5)[0]
        demand={'rc_throttle_us':channels[2],'normalized_above_mincheck':max(0,min(1,(channels[2]-mincheck)/(2000-mincheck))),
            'definition':'Native pre-idle collective above retained min_check; modeled ESC duty is reported separately.'}
    return {'start_s':start,'end_s':end,'duration_s':end-start,'throttle_demand':demand,'motors':motor,
        'horizontal_speed_m_s':{'mean':mean,'min':minimum,'max':maximum,'range_over_mean':(maximum-minimum)/mean if mean>1e-9 else None},
        'vz_m_s':{'mean':_mean(fields['vz_m_s']),'min':min(v for _,v in fields['vz_m_s']),'max':max(v for _,v in fields['vz_m_s'])},
        'finite_difference_3d_acceleration_rms_m_s2':math.sqrt(sum(dt*a*a for dt,a in accel)/(end-start)),
        'finite_difference_3d_acceleration_peak_m_s2':max(a for _,a in accel),
        'maximum_abs_euler_roll_pitch_rate_dps':max(max(abs(r[0]),abs(r[1])) for _,_,_,r in selected),
        'maximum_abs_world_yaw_rate_dps':max(abs(r[2]) for _,_,_,r in selected),
        'maximum_abs_native_yaw_rate_dps':max(abs(math.degrees(row['gyro_frd_z'])) for _,row,_,_ in selected),
        'boundary':'Recorded ground velocity/finite differences, not airspeed or an imposed speed hold. Steadiness requires a separately declared acceptance rule.'}


def stress_metrics(rows,case,*,record=None,config=None,observed_end_s=None):
    cfg=_config(config);clock=_series(rows,'gyro_frd_z')
    stages=case['stages']
    if not stages:raise ValueError('Actual stage markers required')
    starts=[float(s['actual_start_sim_time_s']) for s in stages]
    if any(not math.isfinite(t) for t in starts) or any(b<=a for a,b in zip(starts,starts[1:])):raise ValueError('Stage starts must increase strictly')
    planned_end=float(case.get('end_sim_time_s',clock[-1][0]));end=min(planned_end,clock[-1][0]) if observed_end_s is None else min(float(observed_end_s),planned_end,clock[-1][0])
    if not math.isfinite(end) or end<=starts[0] or starts[0]<clock[0][0]:raise ValueError('Invalid observed coverage')
    holds=[]
    for i,stage in enumerate(stages):
        a=starts[i];b=min(starts[i+1] if i+1<len(starts) else end,end)
        if a>=end:break
        channels=list(stage['channels'][:4])
        if len(channels)!=4 or any(not math.isfinite(float(v)) or not 1000<=v<=2000 for v in channels):raise ValueError('Four finite RC channels required')
        target=_target(stage,record)
        if holds and holds[-1]['channels']==channels and holds[-1]['native_yaw_target_dps']==target:
            holds[-1]['end_s']=b;holds[-1]['stage_names'].append(stage['name'])
        else:holds.append({'start_s':a,'end_s':b,'channels':channels,'native_yaw_target_dps':target,'stage_names':[stage['name']]})
    # A safety abort can leave a later tumbling/disarmed tail in plant.csv.
    # Transform only the observed prefix, including interpolated boundaries;
    # an unrelated post-disarm Euler singularity must not erase valid evidence.
    bounded={name:_window(_series(rows,name),starts[0],end)
             for name in ('roll_deg','pitch_deg','gyro_frd_x','gyro_frd_y','gyro_frd_z')}
    world=[(stamp,body_to_euler_rates(roll,bounded['pitch_deg'][i][1],
             tuple(bounded['gyro_frd_'+axis][i][1] for axis in 'xyz'))[2])
           for i,(stamp,roll) in enumerate(bounded['roll_deg'])]
    for hold in holds:
        a=hold['start_s']+cfg['transient_exclusion_s'];b=hold['end_s']
        hold['saturation']=_saturation(rows,hold['start_s'],b,cfg)
        hold['observed_end_censored']=bool(b==end and (end<planned_end or case.get('partial_program',False)))
        hold['assessment_start_s']=a
        if b-a<.05:
            hold['yaw']={'classification':'insufficient_constant_input_observation','classification_available':False}
            continue
        rates=_window(clock,a,b);error=[(t,math.degrees(v)-hold['native_yaw_target_dps']) for t,v in rates]
        hold['yaw']=classify_yaw_series(error,_window(world,a,b),zero_yaw_command=abs(hold['native_yaw_target_dps'])<1e-9,config=cfg)
        saturated=any(v and v['sample_fraction']>0 for v in hold['saturation'].values())
        hold['authority_limit_observed']=bool(saturated)
        hold['interpretation']='Pattern coincides with actuator limits; do not label it unsaturated PID instability.' if saturated else 'No sampled actuator endpoint in this hold; screening is not causal PID identification.'
    marker=next((s for s in stages if s['name'] in ('pulse','probe','test')),None)
    entry=None;entry_reason=None
    if marker is None:
        entry_reason='No observed pulse/probe/test marker; operating-point entry was not reached.'
    elif float(marker['actual_start_sim_time_s'])>end:
        entry_reason='Observed pulse/probe/test marker lies beyond the active observed prefix.'
    else:
        entry_end=float(marker['actual_start_sim_time_s'])
        entry_start=max(starts[0],entry_end-cfg['entry_window_s'])
        if entry_end-entry_start>.05:
            before=next((s for s in reversed(stages) if float(s['actual_start_sim_time_s'])<entry_end),stages[0])
            entry=entry_statistics(rows,entry_start,entry_end,channels=before['channels'],record=record)
        else:
            entry_reason='Insufficient recorded coverage before the observed entry marker.'
    flags={name:any(h['yaw'].get(name) is True for h in holds) for name in ('sustained_oscillation','decaying_ringing','divergent_growth_candidate','unwanted_world_yaw_spin')}
    return {'schema_version':SCHEMA_VERSION,'available':True,'case_name':case.get('name'),'config':cfg,
        'observed_start_s':starts[0],'observed_end_s':end,'observed_end_censored':bool(end<planned_end or case.get('partial_program',False)),'holds':holds,
        'flags':flags,'entry_statistics':entry,'entry_statistics_reason':entry_reason,'target_scope':'Held native ANGLE body-yaw reference; smoothing and feedforward transients excluded from target reconstruction.',
        'boundary':'All attempted flights must remain in the failure ledger. Screening flags, overshoot and actuator authority are separate from a proof of controller instability.'}


def analyze_trial(directory,*,config=None):
    directory=Path(directory)
    path=next((directory/name for name in ('stress-run.json','comparison-run.json') if (directory/name).is_file()),None)
    if path is None:raise FileNotFoundError('No recorded stress/comparison trial')
    record=json.loads(path.read_text())
    with (directory/'plant.csv').open() as stream:rows=[{k:float(v) for k,v in row.items()} for row in csv.DictReader(stream)]
    cases=record.get('cases',[])
    if not cases:return {'schema_version':SCHEMA_VERSION,'available':False,'run_pass':record.get('pass'),'reason':'No actual completed/observed stage markers; no fabricated stability metric.'}
    packets_path=directory/'rc-packets.json'
    packets=json.loads(packets_path.read_text()) if packets_path.is_file() else []
    results=[]
    for case in cases:
        if not case.get('stages'):
            results.append({'available':False,'reason':'No actual stage markers.'});continue
        observed_end=None
        if not record.get('pass') or case.get('partial_program'):
            start=float(case['stages'][0]['actual_start_sim_time_s'])
            disarms=[float(p['sim_time_s']) for p in packets if p.get('sim_time_s') is not None
                and float(p['sim_time_s'])>=start and len(p.get('channels',[]))>4 and p['channels'][4]<1700]
            if disarms:observed_end=min(disarms)
            if observed_end is not None and observed_end<=start:
                results.append({'available':False,'reason':'Disarm at or before first recorded program sample.'});continue
        result=stress_metrics(rows,case,record=record,config=config,observed_end_s=observed_end)
        result['observed_end_reason']='first_recorded_disarm_cap' if observed_end is not None else 'recorded_case_end'
        results.append(result)
    names=('sustained_oscillation','decaying_ringing','divergent_growth_candidate','unwanted_world_yaw_spin')
    return {'schema_version':SCHEMA_VERSION,'available':any(r.get('available') for r in results),
        'run_pass':record.get('pass'),'cases':results,
        'flags':{name:any(result.get('flags',{}).get(name) is True for result in results) for name in names}}
