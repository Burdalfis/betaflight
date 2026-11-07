#!/usr/bin/env python3
"""Offline sampled-horizon lexicographic allocation replay and independent audits."""
import argparse
import copy
import csv
import gzip
import json
import os
from pathlib import Path
import numpy as np
from dynamic_native import (ROOT,PROTOCOL,SOURCES,NativeDynamic,load_protocol,dynamic_parameters,run_configuration,legacy,sha)
from dynamic_audit import snapshot,validate_priority_every_row
from dynamic_legacy import verify_legacy
from dynamic_reference import static_request_classification


def export_matlab_audit(native,p,output):
    policy=p['audit_policy']['matlab_export'];base=legacy.load_protocol()
    airframe=next(a for a in base['airframes'] if a['name']==policy['airframe'])
    scenario=next(s for s in p['scenarios'] if s['name']==policy['scenario'])
    experiment=next(e for e in p['experiments'] if e['name']=='dynamic_priority')
    _,c,q,inputs=run_configuration(p,airframe,scenario,experiment);d=dynamic_parameters(p)
    end=int(max(policy['times_us'])//p['dt_us'])+1;inputs={k:v[:end] for k,v in inputs.items()}
    result=native.run(c,q,d,**inputs);validate_priority_every_row(result,d)
    snapshots=[]
    for time in policy['times_us']:
        row=int(time//p['dt_us']);record=snapshot(result,row,time,c,d,{'airframe':airframe['name'],'scenario':scenario['name'],'experiment':experiment['name']})
        if not record['priority_available']:raise ValueError('Predeclared MATLAB audit row lacks dynamic authority/state')
        snapshots.append(record)
    document={'schema_version':1,'snapshots':snapshots,'protocol_sha256':sha(PROTOCOL),'base_protocol_sha256':p['base_protocol_sha256'],
              'native_build':native.provenance,'source_paths':list(native.provenance['sources']),
              'source_sha256':list(native.provenance['sources'].values()),'controller_parameters':c.tolist(),
              'plant_parameters':q.tolist(),'dynamic_parameters':d.tolist(),
              'scope':'Predeclared synthetic candidate audit, effective float32 model values promoted to double; independent box-LS and published-command forward costs'}
    path=output/'candidate-audit.json';path.write_text(json.dumps(document,indent=2,allow_nan=False)+'\n');return path


def score(result,scales,mask=None):
    n=result['numeric'];mask=np.ones(len(n),bool) if mask is None else mask
    error=n[mask,4:8]-n[mask,:4];normalized=error/np.asarray(scales)
    return {'rows':int(mask.sum()),'all_selected_actual_rows_scored':True,
        'rmse_by_axis':np.sqrt(np.mean(error**2,axis=0)).tolist(),
        'normalized_high_group_rms':float(np.sqrt(np.mean(np.sum(normalized[:,:3]**2,axis=1)))),
        'normalized_yaw_rms':float(np.sqrt(np.mean(normalized[:,3]**2)))}


def metrics(p,base,inputs,result,mode,controller):
    n,b,u,d,di=(result[key] for key in ('numeric','flags','integer','dynamic_numeric','dynamic_integer'))
    scales=base['allocator_limits']['response_scales'];time=inputs['time_us'];used=di[:,8].astype(bool)
    windows=[]
    for window in p['transition_windows']:
        mask=(time>=window['start_us'])&(time<window['end_us'])
        effective=np.asarray(controller,np.float32).astype(float);request=inputs['requested_response'][mask][0].astype(float)
        classification=static_request_classification(effective[:16].reshape(4,4),request,effective[16:19],effective[19],effective[20])
        classification['request_constant_in_window']=bool(np.all(inputs['requested_response'][mask]==request))
        windows.append({**window,'actual_error':score(result,scales,mask),
            'static_request_classification':classification,
            'dynamic_used_rows':int(used[mask].sum()),'predicted_high_minimum_within_numeric_tolerance_rows':int(np.count_nonzero(used[mask]&(d[mask,2]<=p['dynamic']['high_cost_tolerance'])))})
    available=b[:,9].astype(bool);prediction_error=n[available,43:47]-n[available,4:8]
    flag_bits={str(bit):int(np.count_nonzero(di[:,0]&bit)) for bit in (1,2,4,8,16,32,64,128,256,512)}
    fallback_bits={str(bit):int(np.count_nonzero(((di[:,0]&bit)!=0)&(~used))) for bit in (1,2,4,8,16,32,64,128,256,512)}
    return {'whole_wave':score(result,scales),'declared_windows':windows,
        'current_total_prediction_error':{'available_rows':int(available.sum()),'total_rows':len(n),
            'coverage_fraction':float(available.mean()),'rmse_by_axis':np.sqrt(np.mean(prediction_error**2,axis=0)).tolist() if available.any() else None},
        'nominal_soft_planner':{'valid_rows':int(b[:,1].sum()),'not_converged_rows':int(np.count_nonzero(~b[:,2].astype(bool))),
            'bound_active_rows':int(np.count_nonzero(u[:,2])),'scope':'These original flags describe the nominal soft planner, not dynamic optimality'},
        'dynamic':{'enabled':bool(mode&4),'used_rows':int(used.sum()),'priority_available_rows':int(di[:,5].sum()),
            'coverage_fraction':float(used.mean()) if mode&4 else None,'static_fallback_rows':int((~used).sum()) if mode&4 else None,
            'status_rows_by_bit':flag_bits if mode&4 else {},'fallback_status_rows_by_bit':fallback_bits if mode&4 else {}},
        'controller_valid_rows':int(b[:,0].sum()),'servo_matched_instance_max_counts_error':float(np.max(abs(n[:,14]-n[:,35])))}


def summary_labels():
    f=['']*43
    for index,name in {0:'selected_pwm_us',1:'selected_predicted_direction_h',2:'minimum_primary_cost',3:'minimum_yaw_within_primary_band',
        4:'selected_primary_cost',5:'selected_yaw_cost',6:'selected_movement_cost',36:'selected_counts_h',41:'minimum_pwm_us',42:'maximum_pwm_us'}.items():f[index]=name
    for start,prefix,count in [(7,'reachable_min_effort',3),(10,'reachable_max_effort',3),(13,'minimum_command_rpm',3),
        (16,'maximum_command_rpm',3),(19,'selected_effort_h',3),(22,'selected_rpm_h',3),(25,'selected_response_h',4),
        (29,'selected_residual_h',4),(33,'selected_command_rpm',3),(37,'net_request',4)]:f[start:start+count]=[f'{prefix}_{i}' for i in range(count)]
    return f,['dynamic_status','candidate_count','valid_candidate_count','selected_index','dynamic_valid','priority_available','dynamic_calibrated','dynamic_for_flight','dynamic_used']


def write_trace(path,inputs,result):
    names=legacy.labels();names=copy.deepcopy(names);names['numeric'][33]='nominal_static_goal_direction_rad'
    names['numeric'][32]='actual_issued_pwm_us';names['flags'][1:4]=['nominal_allocator_valid','nominal_allocator_converged','nominal_allocator_exact']
    names['integer'][1:4]=['nominal_allocator_status','nominal_allocator_constraints','nominal_allocator_iterations']
    dn,di=summary_labels()
    with gzip.open(path,'wt') as stream:
        header=['time_us','mode',*names['numeric'],*names['integer'],*names['flags'],*dn,*di]
        if len(header)!=len(set(header)):raise ValueError('Ambiguous duplicate dynamic trace columns')
        writer=csv.writer(stream);writer.writerow(header)
        for row,time in enumerate(inputs['time_us']):writer.writerow([int(time),int(inputs['modes'][row]),
            *result['numeric'][row],*result['integer'][row],*result['flags'][row],*result['dynamic_numeric'][row],*result['dynamic_integer'][row]])


def plot_group(airframe,scenario,p,inputs,results,output,controller):
    import matplotlib.pyplot as plt
    time=inputs['time_us']/1e6;colors=('#3275af','#c58220','#57945a');scales=np.asarray(legacy.load_protocol()['allocator_limits']['response_scales'])
    fig,axes=plt.subplots(2,2,figsize=(13,8),sharex=True,layout='constrained')
    for axis,(plot,name) in enumerate(zip(axes.flat,('collective','roll','pitch','yaw'))):
        plot.plot(time,inputs['requested_response'][:,axis],'--',color='.35',label='requested',linewidth=1)
        for experiment,result,color in zip(p['experiments'],results,colors):plot.plot(time,result['numeric'][:,4+axis],label=experiment['name'],color=color,linewidth=.9)
        plot.set_ylabel(name+' response proxy');plot.grid(alpha=.2)
    axes[0,0].legend(fontsize=8)
    for axis in axes[-1]:axis.set_xlabel('Synthetic seconds; whole wave includes infeasible demand')
    fig.suptitle(airframe+' / '+scenario+': actual response, every row scored')
    fig.savefig(output/(airframe+'-'+scenario+'-whole-wave.png'),dpi=140,bbox_inches='tight');plt.close(fig)
    windows=[w for w in p['transition_windows'] if w['plot_transition']]
    fig,axes=plt.subplots(len(windows),2,figsize=(13,8),layout='constrained')
    for row,window in enumerate(windows):
        mask=(inputs['time_us']>=window['start_us'])&(inputs['time_us']<window['end_us']);t=(inputs['time_us'][mask]-window['start_us'])/1e6
        for experiment,result,color in zip(p['experiments'],results,colors):
            error=(result['numeric'][mask,4:8]-result['numeric'][mask,:4])/scales
            axes[row,0].plot(t,np.linalg.norm(error[:,:3],axis=1),label=experiment['name'],color=color,linewidth=1)
            axes[row,1].plot(t,result['numeric'][mask,7],label=experiment['name'],color=color,linewidth=1)
        axes[row,1].plot(t,inputs['requested_response'][mask,3],'--',color='.3',label='requested')
        effective=np.asarray(controller,np.float32).astype(float)
        classification=static_request_classification(effective[:16].reshape(4,4),inputs['requested_response'][mask][0],effective[16:19],effective[19],effective[20])
        label=window['name']+' (controller-B full4 '+('feasible' if classification['full4_actuator_feasible'] else 'infeasible')+')'
        axes[row,0].set_ylabel('Actual normalized high3 error norm');axes[row,1].set_ylabel('Actual yaw response proxy')
        for axis in axes[row]:axis.set_title(label,fontsize=10);axis.grid(alpha=.2);axis.set_xlabel('Seconds from declared transition')
    axes[0,0].legend(fontsize=8)
    fig.suptitle(airframe+' / '+scenario+': predeclared yaw windows; controller-map static feasibility varies, reaction excluded')
    fig.savefig(output/(airframe+'-'+scenario+'-transitions.png'),dpi=140,bbox_inches='tight');plt.close(fig)
    dynamic=results[-1];n,d,u=dynamic['numeric'],dynamic['dynamic_numeric'],dynamic['dynamic_integer'];used=u[:,8].astype(bool)
    fig,axes=plt.subplots(2,2,figsize=(13,8),sharex=True,layout='constrained')
    axes[0,0].plot(time,n[:,33],label='nominal static goal',linewidth=.8);axes[0,0].plot(time,n[:,15],label='actual current direction',linewidth=.9)
    horizon=d[:,1].copy();horizon[~used]=np.nan;axes[0,0].plot(time,horizon,label='selected predicted direction at H',linewidth=.9)
    axes[0,0].set_ylabel('Mathematical tail direction (rad)')
    axes[1,0].plot(time,n[:,32],label='actually issued PWM');axes[1,0].set_ylabel('Issued synthetic servo command (us)')
    high=d[:,4].copy();high[~used]=np.nan;yaw=d[:,5].copy();yaw[~used]=np.nan
    axes[0,1].plot(time,high,label='selected primary cost');axes[0,1].plot(time,yaw,label='selected yaw cost');axes[0,1].set_yscale('symlog',linthresh=1e-8)
    axes[0,1].set_ylabel('Predicted H normalized costs')
    axes[1,1].plot(time,used.astype(int),label='dynamic used');axes[1,1].plot(time,dynamic['flags'][:,19:22].min(axis=1),label='all RPM fresh',linestyle='--')
    axes[1,1].set_ylabel('Quality / opt-in policy availability')
    for axis in axes.flat:axis.grid(alpha=.2);axis.legend(fontsize=7)
    for axis in axes[-1]:axis.set_xlabel('Synthetic seconds; missing H curves mean explicit fallback')
    fig.suptitle(airframe+' / '+scenario+': nominal goal, issued command and horizon prediction are different quantities')
    fig.savefig(output/(airframe+'-'+scenario+'-dynamic-policy.png'),dpi=140,bbox_inches='tight');plt.close(fig)


def full_report(native,p,output):
    os.environ.setdefault('MPLCONFIGDIR',str(output/'matplotlib-cache'))
    import matplotlib
    matplotlib.use('Agg')
    base=legacy.load_protocol();d=dynamic_parameters(p);runs=[];audits=[]
    for airframe in base['airframes']:
        for scenario in p['scenarios']:
            results=[]
            for experiment in p['experiments']:
                b,c,q,inputs=run_configuration(p,airframe,scenario,experiment);result=native.run(c,q,d,**inputs)
                guarantee=validate_priority_every_row(result,d) if experiment['mode']&4 else {'enabled':False}
                if experiment['mode']&4:
                    identity={'airframe':airframe['name'],'scenario':scenario['name'],'experiment':experiment['name']}
                    for time in p['audit_policy']['times_us']:audits.append(snapshot(result,int(time//p['dt_us']),time,c,d,identity))
                # Full-time archives intentionally exclude 19-candidate arrays.
                result.pop('candidate_numeric');result.pop('candidate_integer')
                name='-'.join((airframe['name'],scenario['name'],experiment['name']));write_trace(output/(name+'.csv.gz'),inputs,result)
                runs.append({'name':name,'airframe':airframe['name'],'scenario':scenario['name'],'experiment':experiment['name'],
                    'mode':experiment['mode'],'rows':len(inputs['time_us']),'controller_parameters':c.tolist(),'plant_parameters':q.tolist(),
                    'dynamic_parameters':d.tolist(),'metrics':metrics(p,b,inputs,result,experiment['mode'],c),
                    'predicted_sampled_priority_validation':guarantee,'csv':name+'.csv.gz'})
                results.append(result)
            plot_group(airframe['name'],scenario['name'],p,inputs,results,output,c)
    return {'runs':runs,'sampled_candidate_audits':audits}


def markdown(summary):
    lines=['# Offline dynamic priority allocation','',
        'The opt-in controller evaluates copied servo-predictor state at a declared horizon for bounded PWM samples. Explicit motor lag/slew gives reachable effort boxes; each accepted direction has a full-rank primary three-axis box least-squares solution over 27 active faces. Bounded RPM commands are inverted and forwarded again, and the resulting published-command costs determine selection.','',
        'Selection uses global passes: high3 collective/roll/pitch cost first, yaw within the global primary tolerance band, then movement within both bands. This guarantee covers accepted sampled candidate primary optima; it is not a continuous-PWM optimum, rank-deficient-map solution or yaw optimization over the entire primary-tolerance motor sublevel set. Numerical tie bands are explicit.','',
        'All configurations remain synthetic and need no CAD/CG/body-inertia model. Mathematical direction is not a calibrated rotor angle. The current observer state and copied predictor history are the only dynamic state inputs; future plant truth is never supplied. State/authority loss degrades explicitly to the static nominal policy. Original soft-planner flags remain nominal diagnostics, distinct from dynamic used/priority/status fields.','',
        'Whole-wave errors include high demands and every actual row. Predeclared yaw transition windows have no blanket feasibility label. Each frame is classified independently by its unique controller-B inverse of the raw actuator-only request, with the separate reaction contribution excluded. This is not actual total-response feasibility under reaction or plant-map mismatch, and it does not establish motor/servo horizon reachability. High3 static feasibility is separately proved by a witness or a collective support bound; otherwise it is unknown. Actual performance under mismatch/faults is descriptive and is not guaranteed by the predicted sampled-priority contract. Lower predicted horizon primary cost does not imply lower actual primary RMS at every transition or every intermediate instant; the tables retain those tradeoffs.','',
        '| Controller map | Predeclared yaw transition | Pure actuator full4 feasible | Required direction (rad) | Required tail / FR / FL effort | High3 static proof |',
        '|---|---|---|---:|---|---|']
    seen=set()
    for run in summary['runs']:
        for window in run['metrics']['declared_windows']:
            key=(run['airframe'],window['start_us'])
            if key in seen or not window['plot_transition']:continue
            seen.add(key);classification=window['static_request_classification']
            lines.append(f'| {run["airframe"]} | {window["name"]} | {classification["full4_actuator_feasible"]} | {classification["required_mathematical_direction_rad"]:.6g} | '+
                ' / '.join(f'{v:.6g}' for v in classification['required_effort_logical'])+f' | {classification["high3_static_feasibility"]["status"]} |')
    lines+=['',
        '| Airframe / scenario / policy | Whole high3 RMS | Positive-yaw window high3 RMS | Reverse-yaw window high3 RMS | Dynamic used coverage |',
        '|---|---:|---:|---:|---:|']
    for run in summary['runs']:
        m=run['metrics'];coverage=m['dynamic']['coverage_fraction'];windows=m['declared_windows']
        lines.append(f'| {run["airframe"]} / {run["scenario"]} / {run["experiment"]} | {m["whole_wave"]["normalized_high_group_rms"]:.5g} | {windows[0]["actual_error"]["normalized_high_group_rms"]:.5g} | {windows[1]["actual_error"]["normalized_high_group_rms"]:.5g} | '+('off' if coverage is None else f'{coverage:.1%}')+' |')
    lines+=['','Full-time traces contain original state/response fields plus dynamic summaries. Full candidate arrays are retained only at predeclared audit times. Independent NumPy SVD/face enumeration, box-gap certificates, motor forward equations and global selection checks audit accepted candidates; the compact `candidate-audit.json` additionally supports independent MATLAB lsqlin checks.','',
        'The original simulation ABI and default callback paths reproduce the frozen 701-row fixture and mode0/1/3 traces bit-for-bit, including signed zero. The earlier e7608a372 evidence is preserved; this opt-in version has separate source/protocol pins. The old nominal direction goal is labeled separately from issued PWM and the selected predicted horizon direction.','',
        'Reproduce with `python3 tools/tricopter/dynamic_replay.py --output /path/to/output` and `python3 -m unittest discover -s tools/tricopter -p test_dynamic_replay.py -v`. `--audit-only` exports the compact independent optimization audit. No hardware, PID, mixer, scheduler or active actuator integration is involved.','']
    return '\n'.join(lines)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',required=True);parser.add_argument('--audit-only',action='store_true')
    args=parser.parse_args();output=Path(args.output).resolve();output.mkdir(parents=True,exist_ok=True)
    protocol_hash=sha(PROTOCOL);p=load_protocol();native=NativeDynamic(output/'build');audit=export_matlab_audit(native,p,output)
    (output/'native-build.json').write_text(json.dumps(native.provenance,indent=2)+'\n')
    if args.audit_only:print(json.dumps({'audit':str(audit),'sha256':sha(audit)},indent=2));return
    summary={'protocol_sha256':protocol_hash,'protocol':p,'native_build':native.provenance,
             'legacy_regression':verify_legacy(native,dynamic_parameters(p)),**full_report(native,p,output),
             'matlab_candidate_audit_sha256':sha(audit),'calibrated':False,'for_flight':False}
    if sha(PROTOCOL)!=protocol_hash or native.provenance['sources']!={str(path.relative_to(ROOT)):sha(path) for path in SOURCES}:
        raise RuntimeError('Frozen dynamic protocol/core/header/bridge changed during replay')
    scripts=[path for path in Path(__file__).parent.glob('dynamic_*') if path.is_file()]
    scripts += [Path(__file__).with_name(name) for name in ('test_dynamic_replay.py','simulation_native.py','simulation_replay.py','allocator_native.py','rpm_data.py')]
    summary['script_sha256']={str(path.relative_to(ROOT)):sha(path) for path in scripts}
    (output/'dynamic-summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    (output/'README.md').write_text(markdown(summary))
    files=sorted(path for path in output.iterdir() if path.is_file() and path.name!='checksums.sha256')
    (output/'checksums.sha256').write_text(''.join(f'{sha(path)}  {path.name}\n' for path in files))
    print(json.dumps({'output':str(output),'runs':len(summary['runs']),'rows':sum(r['rows'] for r in summary['runs']),
                      'audit_rows':len(summary['sampled_candidate_audits']),'legacy_bit_exact':True},indent=2))


if __name__=='__main__':main()
