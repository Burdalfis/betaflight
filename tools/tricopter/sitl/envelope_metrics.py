"""Recorded climb/forward-flight measurements without changing the plant.

Gyro columns are sensor/body FRD. Gazebo Euler angles use body FLU and world
ENU: positive roll is right bank, positive pitch nose down, yaw world CCW.
Euler-rate reconstruction removes the body roll rate required for a tilted
yaw turn; raw body rates remain available and are not labelled coupling.
"""
from __future__ import annotations

import math

from compare_metrics import _at, _mean, _rms, _series, _unwrap, _window, yaw_response

SCHEMA_VERSION = 1
BASELINE_S = .3
DEFAULT_CONFIG = {
    'pwm_knots_us': [1100., 1509., 1700.],
    'angle_knots_deg': [-47.8, 0., 21.6],
    'motor_lower_pwm_us': 1050., 'motor_upper_pwm_us': 2000.,
    'motor_current_limit_a': 35.22, 'motor_brake_current_limit_a': 35.22,
    'motor_voltage_v': 22.2, 'servo_max_torque_nm': .45,
}


def body_to_euler_rates(roll_deg, pitch_deg, gyro_frd_rad_s):
    """Return Gazebo Euler roll/pitch/world-yaw rates in degrees/second.

    The bridge explicitly maps FLU gyro to FRD as (x,-y,-z). Inverting
    Rz(yaw)Ry(pitch)Rx(roll) therefore requires the signs below. Euler rates
    are undefined at pitch +/-90 degrees; do not turn a singularity into a
    fictitious large coupling measurement.
    """
    values = (roll_deg, pitch_deg, *gyro_frd_rad_s)
    if len(gyro_frd_rad_s) != 3 or any(not math.isfinite(float(v)) for v in values):
        raise ValueError('Finite roll/pitch and three FRD gyro components required')
    roll, pitch = math.radians(roll_deg), math.radians(pitch_deg)
    cosine = math.cos(pitch)
    if abs(cosine) < 1e-4:
        raise ValueError('Euler-rate reconstruction is singular near +/-90 degree pitch')
    p, q, r = gyro_frd_rad_s
    transverse = q*math.sin(roll)+r*math.cos(roll)
    return tuple(value*180/math.pi for value in (
        p-transverse*math.tan(pitch),
        -q*math.cos(roll)+r*math.sin(roll),
        -transverse/cosine))


def _definition(case):
    stages = {stage['name']: stage for stage in case['stages']}
    if len(stages) != len(case['stages']): raise ValueError('Duplicate stage names')
    for name in ('baseline', 'operating_lead', 'pulse', 'recovery'):
        stage = stages[name]
        if not math.isfinite(float(stage['actual_start_sim_time_s'])):
            raise ValueError('Nonfinite stage boundary')
    start, operating, onset, release = [float(stages[name]['actual_start_sim_time_s'])
                                       for name in ('baseline','operating_lead','pulse','recovery')]
    end = float(case['end_sim_time_s'])
    if not math.isfinite(end) or not start < operating <= onset-BASELINE_S < onset < release < end:
        raise ValueError('Invalid program/pre-pulse coverage')
    return stages, start, operating, onset, release, end


def _prepare(rows, reference_heading):
    required = ('sim_time_s','x_m','y_m','z_m','vx_m_s','vy_m_s','vz_m_s',
                'roll_deg','pitch_deg','yaw_deg','gyro_frd_x','gyro_frd_y','gyro_frd_z')
    result = []
    reference = math.radians(reference_heading)
    for source in rows:
        row = {name: float(source[name]) for name in required}
        if any(not math.isfinite(v) for v in row.values()): raise ValueError('Nonfinite plant state')
        roll_rate, pitch_rate, yaw_rate = body_to_euler_rates(row['roll_deg'],row['pitch_deg'],
            tuple(row['gyro_frd_'+axis] for axis in ('x','y','z')))
        yaw = math.radians(row['yaw_deg']);pitch = math.radians(row['pitch_deg'])
        vx, vy, vz = (row[name] for name in ('vx_m_s','vy_m_s','vz_m_s'))
        row.update(horizontal_speed_m_s=math.hypot(vx,vy), speed_3d_m_s=math.sqrt(vx*vx+vy*vy+vz*vz),
            body_forward_ground_speed_m_s=math.cos(pitch)*(vx*math.cos(yaw)+vy*math.sin(yaw))-vz*math.sin(pitch),
            initial_heading_forward_ground_speed_m_s=vx*math.cos(reference)+vy*math.sin(reference),
            initial_heading_left_ground_speed_m_s=-vx*math.sin(reference)+vy*math.cos(reference),
            euler_roll_rate_dps=roll_rate,euler_pitch_rate_dps=pitch_rate,world_yaw_rate_dps=yaw_rate,
            body_roll_rate_frd_dps=row['gyro_frd_x']*180/math.pi,
            body_pitch_rate_frd_dps=row['gyro_frd_y']*180/math.pi,
            body_yaw_rate_frd_dps=row['gyro_frd_z']*180/math.pi)
        result.append(row)
    _series(result,'z_m') # Strict timestamp/count validation, shared with earlier metrics.
    return result


def _stats(series, start, end):
    selected = _window(series,start,end); values = [value for _,value in selected]
    return {'mean':_mean(selected),'rms':_rms(selected),'sampled_min':min(values),'sampled_max':max(values),
            'sampled_peak_abs':max(map(abs,values)),'start':values[0],'end':values[-1],
            'change':values[-1]-values[0]}


def _kinematics(rows,start,end):
    fields = ('horizontal_speed_m_s','speed_3d_m_s','body_forward_ground_speed_m_s',
              'initial_heading_forward_ground_speed_m_s','initial_heading_left_ground_speed_m_s',
              'vz_m_s','euler_roll_rate_dps','euler_pitch_rate_dps','world_yaw_rate_dps',
              'body_roll_rate_frd_dps','body_pitch_rate_frd_dps','body_yaw_rate_frd_dps')
    result = {name:_stats(_series(rows,name),start,end) for name in fields}
    for name in ('horizontal_speed_m_s','vz_m_s'):
        result[name]['net_change_rate_m_s2'] = result[name]['change']/(end-start)
    position = {name:_window(_series(rows,name),start,end) for name in ('x_m','y_m','z_m')}
    delta = {name:values[-1][1]-values[0][1] for name,values in position.items()}
    heading = _window(_unwrap(_series(rows,'yaw_deg')),start,end)
    result.update(height_m=_stats(_series(rows,'z_m'),start,end),height_change_m=delta['z_m'],
        horizontal_displacement_m=math.hypot(delta['x_m'],delta['y_m']),
        displacement_3d_m=math.sqrt(sum(value*value for value in delta.values())),
        horizontal_path_m=result['horizontal_speed_m_s']['mean']*(end-start),
        path_3d_m=result['speed_3d_m_s']['mean']*(end-start),
        world_heading_change_ccw_deg=heading[-1][1]-heading[0][1])
    return result


def _attitude(rows,start,end,stage):
    errors = {}; result = {}
    for axis in ('roll','pitch'):
        target = float(stage['requested_'+axis+'_deg'])
        if not math.isfinite(target): raise ValueError('Nonfinite requested ANGLE target')
        actual = _series(rows,axis+'_deg')
        errors[axis] = [(t,value-target) for t,value in _window(actual,start,end)]
        result[axis+'_target_deg']=target
        result[axis+'_actual_deg']=_stats(actual,start,end)
        result[axis+'_tracking_error_deg']=_stats(errors[axis],start,end)
    norm = [(roll[0],math.hypot(roll[1],pitch[1])) for roll,pitch in zip(errors['roll'],errors['pitch'])]
    result['roll_pitch_tracking_error_norm_deg']=_stats(norm,start,end)
    rates = [_window(_series(rows,'euler_'+axis+'_rate_dps'),start,end) for axis in ('roll','pitch')]
    result['euler_roll_pitch_rate_norm_dps']=_stats([(a[0],math.hypot(a[1],b[1])) for a,b in zip(*rates)],start,end)
    return result


def _map_three(pwm,config):
    x,y = config['pwm_knots_us'],config['angle_knots_deg']; pwm = min(x[-1],max(x[0],pwm))
    i = 0 if pwm <= x[1] else 1
    return y[i]+(pwm-x[i])*(y[i+1]-y[i])/(x[i+1]-x[i])


def _configuration(config):
    result = dict(DEFAULT_CONFIG);result.update(config or {})
    for name in ('pwm_knots_us','angle_knots_deg'):
        values = result[name]
        if len(values)!=3 or any(not math.isfinite(float(value)) for value in values) or not values[0]<values[1]<values[2]:
            raise ValueError('Invalid three-point '+name)
    for name,value in result.items():
        if name not in ('pwm_knots_us','angle_knots_deg') and (not math.isfinite(float(value)) or value<0):
            raise ValueError('Invalid actuator parameter '+name)
    return result


def _authority(rows,start,end,baseline,config):
    def field(name,scale=1): return _stats(_series(rows,name,scale),start,end)
    selected = [row for row in rows if start<=float(row['sim_time_s'])<=end]
    if len(selected)<2: raise ValueError('Insufficient actuator samples')
    motors = []
    for i in range(3):
        motors.append({'pwm_us':field(f'pwm{i}_us'), 'rpm':field(f'rotor{i}_omega_rad_s',60/(2*math.pi)),
            'thrust_n':field(f'thrust{i}_n'),'terminal_current_a':field(f'rotor{i}_terminal_current_a'),
            'battery_current_a':field(f'rotor{i}_battery_current_a'),
            'drive_limit_sample_fraction':sum(float(row[f'rotor{i}_terminal_current_a'])>=config['motor_current_limit_a']-1e-8 for row in selected)/len(selected),
            'brake_limit_sample_fraction':(sum(float(row[f'rotor{i}_terminal_current_a'])<=-config['motor_brake_current_limit_a']+1e-8 for row in selected)/len(selected)
                                           if config['motor_brake_current_limit_a']>0 else 0.)})
    actual = _series(rows,'joint_angle_deg')
    command = [(t,_map_three(value,config)) for t,value in _series(rows,'servo_pwm_us')]
    base_actual = _mean(_window(actual,*baseline));base_command = _mean(_window(command,*baseline))
    servo = {'pwm_us':field('servo_pwm_us'),'actual_angle_deg':field('joint_angle_deg'),
        'actual_rate_dps':field('joint_velocity_dps'),'joint_torque_nm':field('servo_torque_nm'),
        'command_angle_deg':_stats(command,start,end),
        'command_minus_actual_deg':_stats([(a[0],a[1]-b[1]) for a,b in zip(command,actual)],start,end),
        'actual_excursion_from_pre_pulse_deg':max(abs(value-base_actual) for _,value in _window(actual,start,end)),
        'command_excursion_from_pre_pulse_deg':max(abs(value-base_command) for _,value in _window(command,start,end)),
        'endpoint_sample_fraction':sum(float(row['servo_pwm_us'])<=config['pwm_knots_us'][0]+1 or float(row['servo_pwm_us'])>=config['pwm_knots_us'][-1]-1 for row in selected)/len(selected),
        'torque_limit_sample_fraction':sum(abs(float(row['servo_torque_nm']))>=.999*config['servo_max_torque_nm'] for row in selected)/len(selected)}
    total_current = [(float(row['sim_time_s']),sum(float(row[f'rotor{i}_battery_current_a']) for i in range(3))) for row in rows]
    bus = _stats(total_current,start,end)
    bus['integral_as']=bus['mean']*(end-start)
    bus['ideal_fixed_voltage_energy_j']=config['motor_voltage_v']*bus['integral_as']
    return {'motors':motors,'servo':servo,'modeled_motor_bus_current_a':bus,
            'motor_lower_sample_fraction':sum(any(float(row[f'pwm{i}_us'])<=config['motor_lower_pwm_us']+1 for i in range(3)) for row in selected)/len(selected),
            'motor_upper_sample_fraction':sum(any(float(row[f'pwm{i}_us'])>=config['motor_upper_pwm_us']-1 for i in range(3)) for row in selected)/len(selected)}


def operating_metrics(rows,case,config=None):
    """Measure each phase and the achieved pre-yaw operating point.

    Recorded stage RP targets must use the bridge's world Euler convention;
    requested_yaw_rate_dps is the held native body target after earth-reference
    attenuation. Controller target smoothing remains part of measured response.
    """
    stages,start,operating,onset,release,end = _definition(case)
    config = _configuration(config)
    heading = _at(_unwrap(_series(rows,'yaw_deg')),start)
    prepared = _prepare(rows,heading)
    baseline = (onset-BASELINE_S,onset)
    definitions = {'baseline':(start,operating,stages['baseline']),
        'operating_lead':(operating,onset,stages['operating_lead']),
        'pre_pulse':(*baseline,stages['operating_lead']),
        'pulse':(onset,release,stages['pulse']),'recovery':(release,end,stages['recovery']),
        'full_program':(start,end,None)}
    windows = {}
    for name,(a,b,stage) in definitions.items():
        windows[name]={'start_s':a,'end_s':b,'duration_s':b-a,'kinematics':_kinematics(prepared,a,b),
                       'attitude_tracking':None if stage is None else _attitude(prepared,a,b,stage),
                       'authority':_authority(rows,a,b,baseline,config)}
    target=float(stages['pulse']['requested_yaw_rate_dps'])
    yaw = yaw_response(rows,onset,release,end,target) if target else None
    full = _window(_series(rows,'actuator_valid'),start,end)
    observations={'actuator_valid_sample_fraction':sum(float(row['actuator_valid'])==1 for row in rows if start<=float(row['sim_time_s'])<=end)/sum(start<=float(row['sim_time_s'])<=end for row in rows),
                  'maximum_state_sample_gap_s':max(b[0]-a[0] for a,b in zip(full,full[1:]))}
    errors=[]
    for index,stage in enumerate(case['stages']):
        a=float(stage['actual_start_sim_time_s']);duration=float(stage['duration_s'])
        b=float(case['stages'][index+1]['actual_start_sim_time_s']) if index+1<len(case['stages']) else end
        if not math.isfinite(duration) or duration<=0 or b<=a:raise ValueError('Invalid stage duration')
        errors.append(abs(b-a-duration))
    observations['maximum_stage_duration_error_s']=max(errors)
    return {'schema_version':SCHEMA_VERSION,'operating_point':case['operating_point'],'direction':case['direction'],
        'reference_heading_world_ccw_deg':heading,'pre_pulse_baseline_s':BASELINE_S,'actuator_config':config,
        'targets':{key:stages['pulse'].get(key) for key in ('requested_roll_deg','requested_pitch_deg',
            'stick_requested_yaw_rate_dps','yaw_pid_target_frd_dps','requested_yaw_rate_dps')},
        'windows':windows,'yaw':yaw,'full_program_observations':observations,
        'runner_program_health':case.get('program_health'),
        'boundary':'Achieved simulated ground speed/climb, not commanded or steady airspeed. Tilted-yaw body RP rates include intentional coordinate kinematics. All actuator extrema are nominal 100 Hz sampled observations.'}


def _firmware_samples(rows,telemetry,stages,start,end):
    actual={axis:_series(rows,axis+'_deg') for axis in ('roll','pitch')}
    observations=[]
    for sample in telemetry:
        stamp=sample.get('sim_time_s')
        if stamp is None or not start<=float(stamp)<=end:continue
        if any('firmware_'+axis+'_deg' not in sample for axis in ('roll','pitch')):continue
        stamp=float(stamp)
        stage=next(item for item in reversed(stages) if stamp>=float(item['actual_start_sim_time_s']))
        point={'sim_time_s':stamp}
        for axis in ('roll','pitch'):
            estimated=float(sample['firmware_'+axis+'_deg']);truth=_at(actual[axis],stamp)
            requested=float(stage['requested_'+axis+'_deg'])
            if not all(math.isfinite(value) for value in (estimated,truth,requested)):
                raise ValueError('Nonfinite native attitude observation')
            point.update({axis+'_firmware_deg':estimated,axis+'_ground_truth_deg':truth,
                axis+'_firmware_minus_ground_truth_deg':estimated-truth,
                axis+'_firmware_minus_commanded_deg':estimated-requested,
                axis+'_ground_truth_minus_commanded_deg':truth-requested})
        observations.append(point)
    return observations


def _firmware_windows(observations,definitions):
    windows={}
    for name,a,b in definitions:
        selected=[row for row in observations if a<=row['sim_time_s']<=b]
        values={}
        if selected:
            for field in selected[0]:
                if field=='sim_time_s':continue
                series=[row[field] for row in selected]
                values[field]={'sample_mean':sum(series)/len(series),
                    'sample_rms':math.sqrt(sum(value*value for value in series)/len(series)),
                    'sampled_min':min(series),'sampled_max':max(series),'sampled_peak_abs':max(map(abs,series))}
        windows[name]={'samples':len(selected),'observations':values or None}
    return windows


def firmware_attitude_metrics(rows,telemetry,case):
    """Compare quantized native attitude with truth near 10 Hz state times."""
    stages,start,operating,onset,release,end=_definition(case)
    observations=_firmware_samples(rows,telemetry,case['stages'],start,end)
    windows=_firmware_windows(observations,[('pre_pulse',onset-BASELINE_S,onset),
        ('operating_lead',operating,onset),('pulse',onset,release),
        ('recovery',release,end),('full_program',start,end)])
    return {'available':bool(observations),'windows':windows,'samples':observations,
        'boundary':'MSP108 attitude at approximate recorded state times, nominal 10 Hz/0.1 degree resolution. Separates estimator tracking from physical Euler tracking; no direct-ground-truth attitude-input assumption.'}


def failed_prefix_metrics(rows,case,packets,telemetry=None,config=None,angle_limit_deg=40):
    """Measure an aborted program's observed prefix, never a fabricated trial.

    ``case`` contains the planned stages with reconstructed held targets. First
    transmitted RC markers supply actual starts. The prefix ends at the first
    subsequent disarm packet's state timestamp (or last observed planned packet
    if no such boundary exists), excluding later unpowered falling motion.
    No full-step rise, overshoot or settling metrics are produced.
    """
    planned={stage['name']:stage for stage in case['stages']}
    sent=[packet for packet in packets if packet.get('case')==case['name'] and
          packet.get('stage') in planned and packet.get('sim_time_s') is not None]
    if not sent:return {'available':False,'reason':'No recorded planned-program RC transmission','yaw':None}
    actual=[]
    for stage in case['stages']:
        first=next((packet for packet in sent if packet['stage']==stage['name']),None)
        if first is not None:actual.append({**stage,'actual_start_sim_time_s':float(first['sim_time_s'])})
    start=actual[0]['actual_start_sim_time_s'];last=sent[-1]
    disarm=next((packet for packet in packets if packet.get('sim_time_s') is not None and
        float(packet['wall_monotonic'])>float(last['wall_monotonic']) and
        (packet.get('case')=='disarmed' or (len(packet.get('channels',[]))>4 and packet['channels'][4]<1500))),None)
    end=min(float(disarm['sim_time_s']) if disarm else float(last['sim_time_s']),float(rows[-1]['sim_time_s']))
    if end<=start:return {'available':False,'reason':'No positive observed program interval','yaw':None}
    heading=_at(_unwrap(_series(rows,'yaw_deg')),start);prepared=_prepare(rows,heading)
    by_name={stage['name']:stage for stage in actual};onset=by_name.get('pulse',{}).get('actual_start_sim_time_s')
    windows={'observed_program_prefix':{'start_s':start,'end_s':end,'duration_s':end-start,
        'kinematics':_kinematics(prepared,start,end)}}
    if onset is not None and onset-BASELINE_S>=start and end>onset:
        config=_configuration(config);baseline=(onset-BASELINE_S,onset)
        for name,a,b,stage in [('pre_pulse',*baseline,by_name.get('operating_lead',by_name['pulse'])),
                              ('observed_yaw_prefix',onset,end,by_name['pulse'])]:
            windows[name]={'start_s':a,'end_s':b,'duration_s':b-a,'kinematics':_kinematics(prepared,a,b),
                'attitude_tracking':_attitude(prepared,a,b,stage),'authority':_authority(rows,a,b,baseline,config)}
    observations=_firmware_samples(rows,telemetry or [],actual,start,end)
    firmware=_firmware_windows(observations,[(name,value['start_s'],value['end_s']) for name,value in windows.items()])
    limit=next((row['sim_time_s'] for row in prepared if start<=row['sim_time_s']<=end and
                max(abs(row['roll_deg']),abs(row['pitch_deg']))>=angle_limit_deg),None)
    return {'available':True,'schema_version':SCHEMA_VERSION,'completed_program':False,
        'actual_observed_stages':actual,'observed_start_s':start,'observed_end_s':end,
        'end_rule':'first subsequent disarm-packet state time' if disarm else 'last recorded planned-packet state time',
        'yaw_onset_s':onset,'observed_yaw_duration_s':None if onset is None else end-onset,
        'planned_yaw_duration_s':float(planned['pulse']['duration_s']),
        'observed_angle_limit_first_time_s':limit,'angle_limit_deg':angle_limit_deg,
        'windows':windows,'firmware_attitude':{'available':bool(observations),'windows':firmware,'samples':observations},
        'yaw':None,'full_step_metrics_available':False,
        'boundary':'Aborted, bounded observed prefix only. No full-target rise/overshoot/settling or completed-program metrics; no extrapolation after disarm.'}


def _curve_metrics(grid,curves,start,end):
    selected = {key:_window(list(zip(grid,values)),start,end) for key,values in curves.items()}
    result = {key:_stats(values,start,end) for key,values in selected.items()}
    norm = [(a[0],math.hypot(a[1],b[1])) for a,b in zip(selected['roll_deg'],selected['pitch_deg'])]
    result['roll_pitch_norm_deg']=_stats(norm,start,end)
    rate_norm=[(a[0],math.hypot(a[1],b[1])) for a,b in zip(selected['euler_roll_rate_dps'],selected['euler_pitch_rate_dps'])]
    result['euler_roll_pitch_rate_norm_dps']=_stats(rate_norm,start,end)
    return result


def matched_control_metrics(rows,case,control_rows,control_case,common_duration_s=None):
    """Describe yawcase minus its same-regime no-yaw control at equal age.

    Both traces keep their own clocks. The control no-op pulse timestamp is
    metadata, while alignment follows actual age since operating_lead began.
    Own pre-event means are removed; raw state mismatches/slopes are retained.
    """
    stages,start,op,onset,release,end=_definition(case)
    ctl,cs,cop,conset,crelease,cend=_definition(control_case)
    if case['operating_point']!=control_case['operating_point']:raise ValueError('Unmatched operating point')
    if float(ctl['pulse']['requested_yaw_rate_dps'])!=0:raise ValueError('Matched control must have zero yaw')
    for name in ('operating_lead','pulse','recovery'):
        if stages[name]['channels'][:3]!=ctl[name]['channels'][:3]:raise ValueError('Non-yaw RC programs differ')
        if any(abs(float(stages[name]['requested_'+axis+'_deg'])-float(ctl[name]['requested_'+axis+'_deg']))>1e-9 for axis in ('roll','pitch')):
            raise ValueError('Non-yaw attitude targets differ')
    equivalent=cop+(onset-op)
    duration=min(end-onset,cend-equivalent)
    if common_duration_s is not None:duration=min(duration,float(common_duration_s))
    duration=math.floor((duration+1e-9)*100)/100
    if not math.isfinite(duration) or duration<=0:raise ValueError('No common matched response window')
    target=_prepare(rows,_at(_unwrap(_series(rows,'yaw_deg')),start))
    control=_prepare(control_rows,_at(_unwrap(_series(control_rows,'yaw_deg')),cs))
    grid=[i*.01 for i in range(-30,round(duration/.01)+1)]
    fields=('roll_deg','pitch_deg','euler_roll_rate_dps','euler_pitch_rate_dps','body_roll_rate_frd_dps',
            'body_pitch_rate_frd_dps','body_yaw_rate_frd_dps','world_yaw_rate_dps',
            'horizontal_speed_m_s','body_forward_ground_speed_m_s','vz_m_s')
    curves={};diagnostics={}
    for field in fields:
        sequences=[_series(data,field) for data in (target,control)]
        bases=[_mean(_window(sequence,t-BASELINE_S,t)) for sequence,t in zip(sequences,(onset,equivalent))]
        raw=[[_at(sequence,t+offset) for t in grid] for sequence,offset in zip(sequences,(onset,equivalent))]
        curves[field]=[(a-bases[0])-(b-bases[1]) for a,b in zip(*raw)]
        pre=[(t,a-b) for t,a,b in zip(grid,*raw) if t<=0]
        mean_t=sum(t for t,_ in pre)/len(pre);mean_v=sum(v for _,v in pre)/len(pre)
        slope=sum((t-mean_t)*(v-mean_v) for t,v in pre)/sum((t-mean_t)**2 for t,_ in pre)
        diagnostics[field]={'target_pre_mean':bases[0],'control_pre_mean':bases[1],
                            'raw_event_difference':pre[-1][1],
                            'raw_pre_peak_abs_difference':max(abs(v) for _,v in pre),'pre_differential_slope_per_s':slope}
    before=[i for i,t in enumerate(grid) if t<=0]
    diagnostics['raw_pre_roll_pitch_norm_peak_deg']=max(math.hypot(
        _at(_series(target,'roll_deg'),onset+grid[i])-_at(_series(control,'roll_deg'),equivalent+grid[i]),
        _at(_series(target,'pitch_deg'),onset+grid[i])-_at(_series(control,'pitch_deg'),equivalent+grid[i])) for i in before)
    pulse_end=min(release-onset,duration)
    phases={'whole_response':_curve_metrics(grid,curves,0,duration),
            'pulse':_curve_metrics(grid,curves,0,pulse_end),
            'recovery':_curve_metrics(grid,curves,pulse_end,duration) if duration>pulse_end+1e-9 else None}
    return {'schema_version':SCHEMA_VERSION,'operating_point':case['operating_point'],'direction':case['direction'],
        'alignment':{'target_onset_s':onset,'control_equivalent_onset_s':equivalent,
            'control_recorded_noop_pulse_s':conset,'control_age_alignment_delta_s':equivalent-conset,
            'reference_stage':'operating_lead','actual_yaw_release_relative_s':release-onset},
        'common_duration_s':duration,'pre_event_diagnostics':diagnostics,'phases':phases,
        'time_from_yaw_onset_s':grid,'curves':curves,
        'boundary':'Matched closed-loop trajectory differences, not identified physical coupling. Raw body RP rate differences include tilted-yaw coordinate kinematics; Euler-rate differences describe attitude changes.'}
