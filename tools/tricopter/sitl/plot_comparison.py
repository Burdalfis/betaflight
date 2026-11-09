#!/usr/bin/env python3
"""Aggregate completed controlled comparisons and export standalone figures.

Matched-control subtraction is within each run, with each trajectory referenced
to its own pre-event baseline. It describes these closed-loop simulations; it
does not identify aircraft aerodynamic coefficients or physical performance.
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

os.environ.setdefault('MPLCONFIGDIR', str(Path(tempfile.gettempdir())/'tricopter-matplotlib'))
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

TRAPEZOID = np.trapezoid if hasattr(np,'trapezoid') else np.trapz
COLORS = {'stock':'#0072b2', 'modified':'#d55e00'}
PAIRS = {'collective_up':'sham', 'collective_down':'sham',
         'yaw_cw_collective_up':'yaw_cw', 'yaw_ccw_collective_up':'yaw_ccw'}
BASELINE = .3
GRID_STEP = .01


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def statistics(values):
    present = [float(value) for value in values if value is not None]
    if any(not math.isfinite(value) for value in present):
        raise ValueError('Nonfinite response metric')
    return {'n':len(present), 'n_total':len(values), 'n_missing_or_censored':len(values)-len(present),
            'mean':None if not present else float(np.mean(present)),
            'median':None if not present else float(np.median(present)),
            'min':None if not present else min(present), 'max':None if not present else max(present),
            'values':values}


def load_trace(path):
    data = np.genfromtxt(path, delimiter=',', names=True, dtype=float)
    fields = ('sim_time_s','roll_deg','pitch_deg','yaw_deg','gyro_frd_z')
    if data.size < 2 or any(not np.all(np.isfinite(data[name])) for name in fields):
        raise ValueError(f'Insufficient or nonfinite trace: {path}')
    if not np.all(np.diff(data['sim_time_s']) > 0):
        raise ValueError(f'Non-increasing trace timestamps: {path}')
    return data


def stage(case, name):
    return next(item for item in case['stages'] if item['name'] == name)


def coupling_duration(case):
    end = stage(case,'recovery')['actual_start_sim_time_s'] if case['kind'] in ('yaw','combined') else case['end_sim_time_s']
    return end-stage(case,'pulse')['actual_start_sim_time_s']


def pair_onsets(target, control):
    """Match elapsed command age, not an independently jittered no-op marker."""
    reference='yaw_lead' if target['kind']=='combined' else 'baseline'
    onset=stage(target,'pulse')['actual_start_sim_time_s']
    age=onset-stage(target,reference)['actual_start_sim_time_s']
    equivalent=stage(control,reference)['actual_start_sim_time_s']+age
    recorded=stage(control,'pulse')['actual_start_sim_time_s']
    return {'target_onset_s':onset,'control_equivalent_onset_s':equivalent,
            'control_recorded_pulse_onset_s':recorded,'control_age_alignment_delta_s':equivalent-recorded,
            'reference_stage':reference,'target_age_s':age}


def pair_duration(target, control):
    aligned=pair_onsets(target,control)
    def end(case):
        return stage(case,'recovery')['actual_start_sim_time_s'] if target['kind']=='combined' else case['end_sim_time_s']
    return min(end(target)-aligned['target_onset_s'],end(control)-aligned['control_equivalent_onset_s'])


def grid_until(duration, start=0):
    end = math.floor((duration+1e-9)/GRID_STEP)*GRID_STEP
    if end <= start:
        raise ValueError('Insufficient common trajectory interval')
    return np.arange(round(start/GRID_STEP), round(end/GRID_STEP)+1)*GRID_STEP


def interpolate(data, values, times):
    if times[0] < data['sim_time_s'][0]-1e-8 or times[-1] > data['sim_time_s'][-1]+1e-8:
        raise ValueError('Resampling would extrapolate outside recorded state')
    return np.interp(times, data['sim_time_s'], values)


def baseline(data, values, onset):
    time = data['sim_time_s']
    points = np.concatenate(([onset-BASELINE], time[(time>onset-BASELINE)&(time<onset)], [onset]))
    return float(TRAPEZOID(interpolate(data,values,points),points)/BASELINE)


def paired_difference(data, target, control, grid, control_data=None):
    """Return Δtarget−Δcontrol, not a difference of already positive norms."""
    control_data=data if control_data is None else control_data
    aligned=pair_onsets(target,control)
    onsets = [aligned['target_onset_s'],aligned['control_equivalent_onset_s']]
    result = {}
    for output, field, scale in [('roll_deg','roll_deg',1), ('pitch_deg','pitch_deg',1),
                                  ('yaw_rate_dps','gyro_frd_z',180/math.pi)]:
        curves = []
        for source,onset in zip((data,control_data),onsets):
            values=source[field]*scale
            curves.append(interpolate(source,values,grid+onset)-baseline(source,values,onset))
        result[output] = curves[0]-curves[1]
    result['roll_pitch_norm_deg'] = np.hypot(result['roll_deg'],result['pitch_deg'])
    return result


def pair_pre_event_diagnostics(data, target, control, control_data=None):
    """Probe raw state mismatch and differential slopes before the step."""
    control_data=data if control_data is None else control_data
    aligned=pair_onsets(target,control);grid=np.arange(-30,1)*GRID_STEP
    onsets=(aligned['target_onset_s'],aligned['control_equivalent_onset_s'])
    result=dict(aligned);raw={}
    for axis in ('roll','pitch'):
        field=axis+'_deg'
        values=[interpolate(source,source[field],grid+onset)
                for source,onset in zip((data,control_data),onsets)]
        raw[axis]=values[0]-values[1]
        result[axis+'_raw_event_difference_deg']=float(raw[axis][-1])
        result[axis+'_raw_pre_event_peak_abs_difference_deg']=float(np.max(np.abs(raw[axis])))
        result[axis+'_pre_event_differential_slope_dps']=float(np.polyfit(grid,raw[axis],1)[0])
    result['raw_pre_event_peak_norm_difference_deg']=float(np.max(np.hypot(raw['roll'],raw['pitch'])))
    return result


def difference_metrics(grid, curves):
    result = {'window_duration_s':float(grid[-1])}
    for field, values in curves.items():
        units = 'dps' if field.endswith('dps') else 'deg'
        prefix = field[:-len(units)-1]
        result[prefix+'_peak_abs_difference_'+units] = float(np.max(np.abs(values)))
        result[prefix+'_rms_difference_'+units] = float(np.sqrt(TRAPEZOID(values**2,grid)/(grid[-1]-grid[0])))
        result[prefix+'_end_difference_'+units] = float(values[-1])
    return result


def pretrend_diagnostics(pre_grid, pre_curves, post_grid, post_curves, baseline_difference=None):
    """Sensitivity to extending the preceding 0.3 s trend, not a correction.

    A local line need not remain valid during yaw. The purpose is to reveal
    whether existing state/history trends are material to the paired response.
    """
    fits={axis:np.polyfit(pre_grid,pre_curves[axis],1) for axis in ('roll_deg','pitch_deg')}
    residual={axis:post_curves[axis]-np.polyval(fits[axis],post_grid) for axis in fits}
    pre_residual={axis:pre_curves[axis]-np.polyval(fits[axis],pre_grid) for axis in fits}
    post_norm=np.hypot(residual['roll_deg'],residual['pitch_deg'])
    pre_norm=np.hypot(pre_residual['roll_deg'],pre_residual['pitch_deg'])
    result = {
        'roll_pre_event_differential_slope_dps':float(fits['roll_deg'][0]),
        'pitch_pre_event_differential_slope_dps':float(fits['pitch_deg'][0]),
        'roll_fitted_intercept_at_event_deg':float(fits['roll_deg'][1]),
        'pitch_fitted_intercept_at_event_deg':float(fits['pitch_deg'][1]),
        'event_initial_difference_norm_deg':float(post_curves['roll_pitch_norm_deg'][0]),
        'pre_event_line_fit_rms_residual_norm_deg':float(np.sqrt(TRAPEZOID(pre_norm**2,pre_grid)/BASELINE)),
        'linear_pretrend_projected_end_norm_deg':float(math.hypot(
            np.polyval(fits['roll_deg'],post_grid[-1]),np.polyval(fits['pitch_deg'],post_grid[-1]))),
        'linear_pretrend_subtracted_peak_norm_deg':float(np.max(post_norm)),
        'linear_pretrend_subtracted_rms_norm_deg':float(np.sqrt(TRAPEZOID(post_norm**2,post_grid)/(post_grid[-1]-post_grid[0]))),
    }
    if baseline_difference is not None:
        raw={axis:pre_curves[axis]+baseline_difference[axis] for axis in ('roll_deg','pitch_deg')}
        for axis,field in [('roll','roll_deg'),('pitch','pitch_deg')]:
            result[axis+'_raw_pre_event_mean_difference_deg']=baseline_difference[field]
            result[axis+'_raw_pre_event_peak_abs_difference_deg']=float(np.max(np.abs(raw[field])))
            result[axis+'_raw_event_difference_deg']=float(raw[field][-1])
        result['raw_pre_event_peak_norm_difference_deg']=float(np.max(np.hypot(raw['roll_deg'],raw['pitch_deg'])))
    return result


def numeric_metrics(case):
    result = {}
    for group in ('yaw','coupling'):
        for key,value in case.get(group,{}).items():
            if value is None or (isinstance(value,(int,float)) and not isinstance(value,bool)):
                result[group+'.'+key] = value
    for key in ('motor_upper_saturation_fraction','servo_endpoint_fraction','actuator_valid_fraction',
                'maximum_measurement_rc_send_gap_s','sampled_feedback_valid_fraction',
                'sampled_yaw_allocator_saturation_fraction','sampled_vertical_allocator_saturation_fraction'):
        if key in case:result['health.'+key] = case[key]
    return result


def draw_repeats(axis, curves, color, label):
    common = grid_until(min(curve[0][-1] for curve in curves),start=max(curve[0][0] for curve in curves))
    values = np.asarray([np.interp(common,x,y) for x,y in curves])
    for x,y in curves:axis.plot(x,y,color=color,lw=.7,alpha=.3)
    axis.fill_between(common,np.min(values,axis=0),np.max(values,axis=0),color=color,alpha=.15,lw=0)
    axis.plot(common,np.median(values,axis=0),color=color,lw=2,label=label)


def figure_style(fig, title):
    fig.suptitle(title,fontsize=14,fontweight='bold',y=.98)
    fig.subplots_adjust(left=.09,right=.98,bottom=.13,top=.90,hspace=.35,wspace=.27)
    fig.text(.5,.035,'Thin: individual repeats  •  Thick: median  •  Shading: min–max, not confidence intervals\n'
             'Same supplied plant; normal RC inputs only; setup pilot overridden during measurement.',
             ha='center',va='center',fontsize=9)
    for axis in fig.axes:
        axis.grid(True,color='#dddddd',linewidth=.6)
        axis.spines[['top','right']].set_visible(False)


def make_figures(directory, runs, pair_grids, paired, classification):
    prefix='Exploratory timing-varied: ' if classification!='timing-controlled' else ''
    yaw_fig,axes=plt.subplots(2,2,figsize=(12,8.3))
    for row,(name,label) in enumerate([('yaw_cw','Clockwise'),('yaw_ccw','Counterclockwise')]):
        for variant in COLORS:
            rate_curves=[];heading_curves=[];release_offsets=[];targets=[]
            selected=[run for run in runs if run['variant']==variant]
            for run in selected:
                case=run['cases'][name];data=run['traces'][name]
                onset=stage(case,'yaw_lead')['actual_start_sim_time_s']
                release=stage(case,'recovery')['actual_start_sim_time_s']
                end=case['end_sim_time_s'];target=case['yaw']['target_yaw_rate_frd_dps']
                grid=grid_until(end-onset,start=-BASELINE)
                rate_curves.append((grid,interpolate(data,data['gyro_frd_z']*180/math.pi,onset+grid)))
                release_offsets.append(release-onset);targets.append(target)
                grid=grid_until(end-release)
                heading=np.rad2deg(np.unwrap(np.deg2rad(data['yaw_deg'])))
                relative=interpolate(data,heading,release+grid)-interpolate(data,heading,np.array([release]))[0]
                heading_curves.append((grid,-math.copysign(1,target)*relative))
            draw_repeats(axes[row,0],rate_curves,COLORS[variant],variant.capitalize())
            draw_repeats(axes[row,1],heading_curves,COLORS[variant],variant.capitalize())
            # Use actual transmitted stage boundaries, including every repeat's
            # release variation; a nominal 1.8 s line could mimic braking lag.
            end=max(curve[0][-1] for curve in rate_curves)
            for offset,target in zip(release_offsets,targets):
                axes[row,0].step([-BASELINE,0,offset,end],[0,target,0,0],where='post',
                    color=COLORS[variant],ls='--',lw=.6,alpha=.25)
            axes[row,0].step([-BASELINE,0,float(np.median(release_offsets)),end],
                [0,float(np.median(targets)),0,0],where='post',color=COLORS[variant],
                ls='--',lw=1.2,alpha=.8,label=variant.capitalize()+' RC target')
            axes[row,0].axvspan(min(release_offsets),max(release_offsets),color=COLORS[variant],alpha=.07,lw=0)
        axes[row,0].set(title=label+' yaw-rate pulse',xlabel='Time from yaw command (s)',ylabel='Body-FRD yaw rate (°/s)')
        axes[row,1].set(title=label+' extra heading after release',xlabel='Time from yaw release (s)',ylabel='Angle continuing commanded turn (°)')
        for axis in axes[row]:axis.legend(fontsize=9,loc='best')
    figure_style(yaw_fig,prefix+'Stock vs modified: yaw response and braking')
    for suffix in ('png','svg'):yaw_fig.savefig(directory/('comparison-yaw.'+suffix),dpi=220)
    plt.close(yaw_fig)

    coupling_fig,axes=plt.subplots(2,2,figsize=(12,8.3))
    labels={'collective_up':'Collective +25 µs, neutral yaw', 'collective_down':'Collective −25 µs, neutral yaw',
            'yaw_cw_collective_up':'Collective +25 µs during clockwise yaw',
            'yaw_ccw_collective_up':'Collective +25 µs during counterclockwise yaw'}
    for axis,name in zip(axes.flat,PAIRS):
        for variant in COLORS:
            curves=[(r['plot_grid'],r['plot_curves']['roll_pitch_norm_deg']) for r in paired
                    if r['variant']==variant and r['case']==name]
            draw_repeats(axis,curves,COLORS[variant],variant.capitalize())
        axis.axvspan(0,.4,color='#777777',alpha=.08,label='Scheduled collective pulse')
        axis.axvline(0,color='#555555',lw=.9,ls=':')
        axis.set(title=labels[name],xlabel='Time from collective step (s)',
                 ylabel='Paired roll/pitch difference norm (°)',ylim=(0,None))
        axis.legend(fontsize=8.5,loc='best')
    fresh=all(run['fresh_case_reset'] for run in runs)
    figure_style(coupling_fig,prefix+('Fresh-case matched responses around collective changes' if fresh else
                                    'History-confounded matched trajectory differences'))
    coupling_fig.subplots_adjust(bottom=.17)
    warning=('Fresh resets remove prior-case history; own baselines are subtracted. Raw pre-event state/trend diagnostics still matter.' if fresh else
             'Pre-event history is shown: these differences do not isolate the causal effect of throttle.')
    coupling_fig.text(.5,.085,warning,
                      ha='center',va='center',fontsize=9,color='#8b2a1b')
    for suffix in ('png','svg'):coupling_fig.savefig(directory/('comparison-coupling.'+suffix),dpi=220)
    plt.close(coupling_fig)


def analyze(directory):
    directory=Path(directory).resolve()
    summary_path=directory/'comparison-summary.json'
    summary=json.loads(summary_path.read_text())
    if not summary.get('pass'):
        raise ValueError('Comparison is incomplete or failed; refusing a final analysis')
    protocol=summary['protocol'];repeats=protocol['repetitions']
    required_cases=set(PAIRS)|set(PAIRS.values())
    if not required_cases.issubset(case['name'] for case in protocol['cases']):
        raise ValueError('Final comparison plots require the complete seven-case suite, not a probe subset')
    if len(summary['runs'])!=2*repeats:
        raise ValueError('Unexpected number of completed runs')
    varied_gains=protocol.get('intentionally_varied_yaw_gains')
    readback_key='common_msp_readback';mask_helper_sha256=None;verified_yaw_profiles=[]
    if varied_gains is not None:
        if not isinstance(varied_gains,dict) or set(varied_gains)!=set(COLORS):
            raise ValueError('Intentionally varied yaw gains must be explicitly declared')
        import tune_profiles
        mask_helper_sha256=sha(Path(tune_profiles.__file__))
        if protocol.get('common_msp_mask')!=tune_profiles.MASKED_BYTE_RANGES:
            raise ValueError('Declared MSP mask differs from the fixed yaw PID/F mask')
        feedforward=protocol.get('feedforward_yaw');hold=protocol.get('feedforward_yaw_hold_gain')
        if isinstance(feedforward,bool) or not isinstance(feedforward,int) or not 0<=feedforward<=1000:
            raise ValueError('Declared yaw feedforward must be an explicit integer')
        if isinstance(hold,bool) or not isinstance(hold,int) or not 0<=hold<=100:
            raise ValueError('Declared yaw hold gain must be an explicit integer')
        readback_key='shared_msp_readback'
        variant_raw={}
        for record in summary['runs']:
            canonical=tune_profiles.shared_msp_readback(record)
            if record.get(readback_key)!=canonical['readback'] or record.get('shared_msp_mask')!=canonical['masked_byte_ranges']:
                raise ValueError('Recorded shared MSP reply differs from canonical yaw-only mask')
            variant=record['variant']
            expected=varied_gains[variant]
            if (not isinstance(expected,dict) or set(expected)!={'p','i','d'} or
                any(isinstance(value,bool) or not isinstance(value,int) or not 0<=value<=250 for value in expected.values())):
                raise ValueError('Declared yaw PID gains must contain integer p/i/d')
            actual=dict(zip(('p','i','d'),bytes.fromhex(record['common_msp_readback']['112'])[6:9]))
            actual_f=int.from_bytes(bytes.fromhex(record['common_msp_readback']['94'])[36:38],'little')
            if actual!=expected or actual_f!=feedforward:
                raise ValueError('Raw yaw PID/F differs from declared '+variant+' profile')
            if record.get('requested_yaw_gains',expected)!=expected:
                raise ValueError('Requested and protocol yaw gains differ')
            verified_yaw_profiles.append({'variant':variant,'gains':actual,'feedforward':actual_f})
            if variant in variant_raw and record['common_msp_readback']!=variant_raw[variant]:
                raise ValueError('Yaw profile/readback changed across repeats of '+variant)
            variant_raw.setdefault(variant,record['common_msp_readback'])
    runs=[];seen={variant:0 for variant in COLORS};per_run=[]
    for i,record in enumerate(summary['runs']):
        variant=record['variant'];seen[variant]+=1
        if not record['pass'] or record['final_status']['armed']:
            raise ValueError('Run failed or did not finish disarmed')
        if variant!=protocol['variant_order'][i] or record['protocol_sha256']!=protocol['sha256']:
            raise ValueError('Run ordering or protocol hash mismatch')
        for name in ('world_sha256','common_config_sha256',readback_key,'yaw_settings'):
            if record[name]!=summary['runs'][0][name]:raise ValueError('Unmatched setting: '+name)
        name=record.get('group_directory',f'{i+1:02d}-{variant}');path=directory/name
        cases={case['name']:case for case in record['cases']}
        if set(cases)!=set(case['name'] for case in protocol['cases']):
            raise ValueError('Incomplete measurement cases')
        if record.get('case_directories'):
            if record.get('case_directory_base')!='group_directory':
                raise ValueError('Unknown per-case trace directory convention')
            traces={case:load_trace(path/record['case_directories'][case]/'plant.csv') for case in cases}
        else:
            common_trace=load_trace(path/'plant.csv');traces={case:common_trace for case in cases}
        runs.append({'name':name,'variant':variant,'repetition':seen[variant],
                     'cases':cases,'traces':traces,'fresh_case_reset':record.get('fresh_case_reset',False)})
        for case_name,case in cases.items():
            for metric,value in numeric_metrics(case).items():
                per_run.append({'run':name,'variant':variant,'repetition':seen[variant],
                                'case':case_name,'family':'gross','control':'','metric':metric,'value':value})
    if any(n!=repeats for n in seen.values()):raise ValueError('Unbalanced variants')
    errors=[]
    for run in runs:
        for case in run['cases'].values():
            for i,item in enumerate(case['stages']):
                end=case['stages'][i+1]['actual_start_sim_time_s'] if i+1<len(case['stages']) else case['end_sim_time_s']
                errors.append(abs(end-item['actual_start_sim_time_s']-item['duration_s']))
    timing_limit=protocol.get('maximum_stage_duration_error_s',.03)
    controlled=bool(protocol.get('execution_timing')) and max(errors)<=timing_limit+1e-6
    classification='timing-controlled' if controlled else 'timing-varied-exploratory'
    pair_grids={name:grid_until(min(pair_duration(run['cases'][name],run['cases'][control])
                    for run in runs)) for name,control in PAIRS.items()}
    paired=[]
    for run in runs:
        for name,control in PAIRS.items():
            grid=pair_grids[name]
            data,control_data=run['traces'][name],run['traces'][control]
            aligned=pair_onsets(run['cases'][name],run['cases'][control])
            curves=paired_difference(data,run['cases'][name],run['cases'][control],grid,control_data)
            metrics=difference_metrics(grid,curves)
            pre_grid=np.arange(-30,1)*GRID_STEP
            pre_curves=paired_difference(data,run['cases'][name],run['cases'][control],pre_grid,control_data)
            baseline_difference={field:baseline(data,data[field],aligned['target_onset_s'])-
                baseline(control_data,control_data[field],aligned['control_equivalent_onset_s'])
                for field in ('roll_deg','pitch_deg')}
            diagnostics=pretrend_diagnostics(pre_grid,pre_curves,grid,curves,baseline_difference)
            plot_grid=np.concatenate((pre_grid[:-1],grid))
            plot_curves={key:np.concatenate((pre_curves[key][:-1],curves[key])) for key in curves}
            paired.append({'run':run['name'],'variant':run['variant'],'case':name,'control':control,
                           'repetition':run['repetition'],'metrics':metrics,'curves':curves,
                           'diagnostics':diagnostics,'alignment':aligned,'plot_grid':plot_grid,'plot_curves':plot_curves})
            for metric,value in metrics.items():
                per_run.append({'run':run['name'],'variant':run['variant'],'repetition':run['repetition'],
                                'case':name,'family':'paired','control':control,'metric':metric,'value':value})
            for metric,value in diagnostics.items():
                per_run.append({'run':run['name'],'variant':run['variant'],'repetition':run['repetition'],
                                'case':name,'family':'paired_pretrend','control':control,'metric':metric,'value':value})
    aggregate={}
    for row in per_run:
        key=(row['family'],row['variant'],row['case'],row['control'],row['metric'])
        aggregate.setdefault(key,[]).append(row['value'])
    aggregated=[dict(zip(('family','variant','case','control','metric'),key),**statistics(values))
                for key,values in sorted(aggregate.items())]
    report={'pass':True,'directory':str(directory),'summary_sha256':sha(summary_path),
            'classification':classification,'maximum_stage_duration_error_s':max(errors),
            'stage_duration_tolerance_s':timing_limit,
            'numpy_version':np.__version__,'matplotlib_version':matplotlib.__version__,
            'protocol_sha256':protocol['sha256'],'plotter_sha256':sha(Path(__file__)),
            'intentionally_varied_yaw_gains':varied_gains,
            'msp_comparison':{'key':readback_key,'mask_helper_sha256':mask_helper_sha256,
                'declared_mask':protocol.get('common_msp_mask'),
                'declared_feedforward_yaw':protocol.get('feedforward_yaw'),
                'declared_feedforward_yaw_hold_gain':protocol.get('feedforward_yaw_hold_gain'),
                'verified_raw_yaw_profiles':verified_yaw_profiles,
                'hold_gain_boundary':'No hold field exists in these MSP replies; runner CLI verification is retained in each case record/provision.log.',
                'boundary':('Only explicitly declared yaw PID/F bytes are masked; all other provided replies compare exactly.'
                    if varied_gains is not None else 'All raw supplied MSP replies compare exactly.'),
                'readbacks':[{'run':record.get('group_directory',f'{i+1:02d}-{record["variant"]}'),
                    'variant':record['variant'],'common_msp_readback':record['common_msp_readback'],
                    'shared_msp_readback':record.get('shared_msp_readback'),
                    'shared_msp_mask':record.get('shared_msp_mask')}
                    for i,record in enumerate(summary['runs'])]},
            'repetitions_per_variant':repeats,'source_run_order':[run['name'] for run in runs],
            'fresh_case_reset':all(run['fresh_case_reset'] for run in runs),
            'coupling_classification':('fresh-case-age-aligned-matched-responses' if all(run['fresh_case_reset'] for run in runs) else
                                      'history-confounded-matched-trajectory-differences'),
            'boundary':'Matched closed-loop responses on the approximate plant, not identified physics or physical flight validation.',
            'statistics':'Mean/median/min/max of repeats. Null settling is finite-window censored; n excludes nulls. Shading is min–max, not CI.',
            'paired_difference':'(target minus its pre-event mean) minus (matched control minus its pre-event mean), within the same run.',
            'coupling_attribution':('Fresh case resets remove prior-case history; remaining pre-event trajectory differences and timing uncertainty are reported.'
                if all(run['fresh_case_reset'] for run in runs) else
                'Paired trajectory differences include pre-existing state/controller-history trends; they are not isolated causal throttle-response estimates.'),
            'pair_alignment':'Control time equals control yaw/baseline onset plus target elapsed command age at its collective onset; the no-op pulse marker is retained only as metadata.',
            'pair_alignment_records':[dict(run=item['run'],variant=item['variant'],case=item['case'],control=item['control'],**item['alignment']) for item in paired],
            'pre_event_reference_rule':'Prospective diagnostic reference only, not an exclusion or run-acceptance gate: raw pre-event roll/pitch difference norm <=0.05 deg and each differential slope <=0.1 deg/s. Every trial is retained.',
            'pre_event_reference_checks':[
                {'run':item['run'],'variant':item['variant'],'case':item['case'],'control':item['control'],
                 'raw_pre_event_peak_norm_deg':item['diagnostics']['raw_pre_event_peak_norm_difference_deg'],
                 'maximum_abs_differential_slope_dps':max(abs(item['diagnostics'][axis+'_pre_event_differential_slope_dps']) for axis in ('roll','pitch')),
                 'within_state_reference':item['diagnostics']['raw_pre_event_peak_norm_difference_deg']<=.05+1e-9,
                 'within_slope_reference':all(abs(item['diagnostics'][axis+'_pre_event_differential_slope_dps'])<=.1+1e-9 for axis in ('roll','pitch'))}
                for item in paired],
            'pretrend_sensitivity':'Least-squares line on the preceding 0.3 s (31 points) extrapolated through the post-event window and subtracted. This is a sensitivity diagnostic, not a physically validated correction.',
            'paired_baseline_s':BASELINE,'resampling_interval_s':GRID_STEP,
            'paired_windows_s':{name:float(grid[-1]) for name,grid in pair_grids.items()},
            'paired_windows_rule':'Shortest available target/control interval across all runs, rounded down to the 10 ms grid; no extrapolation.',
            'aggregate_metrics':aggregated,'per_run_metrics':per_run}
    (directory/'comparison-analysis.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    with (directory/'comparison-aggregate.csv').open('w') as stream:
        keys=['family','variant','case','control','metric','n','n_total','n_missing_or_censored','mean','median','min','max']
        writer=csv.DictWriter(stream,fieldnames=keys);writer.writeheader()
        writer.writerows({key:row[key] for key in keys} for row in aggregated)
    with (directory/'comparison-per-run.csv').open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(per_run[0]));writer.writeheader();writer.writerows(per_run)
    with (directory/'comparison-paired-trajectories.csv').open('w') as stream:
        writer=csv.writer(stream);writer.writerow(['run','variant','case','control','time_s','roll_difference_deg',
                                                  'pitch_difference_deg','roll_pitch_norm_difference_deg','yaw_rate_difference_dps'])
        for item in paired:
            curves=item['curves']
            for i,stamp in enumerate(pair_grids[item['case']]):
                writer.writerow([item['run'],item['variant'],item['case'],item['control'],stamp,
                                 curves['roll_deg'][i],curves['pitch_deg'][i],curves['roll_pitch_norm_deg'][i],curves['yaw_rate_dps'][i]])
    with (directory/'comparison-paired-prehistory.csv').open('w') as stream:
        writer=csv.writer(stream);writer.writerow(['run','variant','case','control','time_s','roll_difference_deg',
                                                  'pitch_difference_deg','roll_pitch_norm_difference_deg','yaw_rate_difference_dps'])
        for item in paired:
            curves=item['plot_curves']
            for i,stamp in enumerate(item['plot_grid']):
                writer.writerow([item['run'],item['variant'],item['case'],item['control'],stamp,
                                 curves['roll_deg'][i],curves['pitch_deg'][i],curves['roll_pitch_norm_deg'][i],curves['yaw_rate_dps'][i]])
    make_figures(directory,runs,pair_grids,paired,classification)
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    args=parser.parse_args()
    report=analyze(args.directory)
    print(f"Analyzed {2*report['repetitions_per_variant']} completed runs; JSON, CSV, PNG and SVG saved in {args.directory}")


if __name__=='__main__':main()
