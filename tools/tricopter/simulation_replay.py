#!/usr/bin/env python3
"""Synthetic actuator-to-response integration using the shared C batch bridge."""
import argparse
import gzip
import csv
import json
import os
from pathlib import Path
import numpy as np

from simulation_native import (ROOT, PROTOCOL, SOURCES, NativeSimulation, load_protocol, pack_run, tick_inputs, labels, sha)


def fixture_inputs(protocol):
    controller,plant,mode=pack_run(protocol,protocol['airframes'][0],protocol['scenarios'][0],protocol['experiments'][2])
    inputs=tick_inputs(protocol,protocol['scenarios'][0],mode)
    inputs={key:value[:701].copy() for key,value in inputs.items()}
    time=inputs['time_us']
    inputs['drop_generation'][(time>=350000)&(time<520000),0]=1
    inputs['outlier_generation'][(time>=600000)&(time<620000),1]=1
    inputs['requested_response'][time>=600000]=[3,.8,-.6,1.2]
    return controller,plant,inputs


def export_fixture(native,protocol,output):
    controller,plant,inputs=fixture_inputs(protocol);result=native.run(controller,plant,**inputs)
    fixture={'schema_version':1,'controller_parameters':controller.tolist(),'plant_parameters':plant.tolist(),
             **{key:value.tolist() for key,value in inputs.items()},
             **{'expected_'+key:value.tolist() for key,value in result.items()},
             'column_labels':labels(),'native_build':native.provenance,'protocol_sha256':sha(PROTOCOL),
             'source_paths':list(native.provenance['sources']), 'source_sha256':list(native.provenance['sources'].values()),
             'scope':'Same C bridge parity fixture only, with capture-to-software-delivery delay; not an independent plant implementation'}
    path=output/'matlab-parity-fixture.json';path.write_text(json.dumps(fixture,separators=(',',':'),allow_nan=False)+'\n')
    return path


def error_statistics(error,mask=None):
    mask=np.ones(len(error),bool) if mask is None else np.asarray(mask,bool)
    value=np.asarray(error)[mask]
    return {'available_rows':int(mask.sum()),'total_rows':len(mask),'coverage_fraction':float(mask.mean()),
            'rmse_by_axis':np.sqrt(np.mean(value**2,axis=0)).tolist() if len(value) else None,
            'mean_signed_by_axis':np.mean(value,axis=0).tolist() if len(value) else None,
            'p95_absolute_by_axis':np.percentile(abs(value),95,axis=0).tolist() if len(value) else None}


def summarize(protocol,inputs,result):
    n,u,b=result['numeric'],result['integer'],result['flags'];time=inputs['time_us']
    actual_error=n[:,4:8]-n[:,:4]
    scales=np.asarray(protocol['allocator_limits']['response_scales'])
    score=error_statistics(actual_error)
    score['all_actual_rows_scored']=True
    score['normalized_rmse_by_axis']=np.sqrt(np.mean((actual_error/scales)**2,axis=0)).tolist()
    score['normalized_vector_rms']=float(np.sqrt(np.mean(np.sum((actual_error/scales)**2,axis=1))))
    phases=[]
    for segment in protocol['request_segments']:
        mask=(time>=segment['start_us'])&(time<segment['end_us'])
        if segment['end_us']==protocol['duration_us']:mask|=time==segment['end_us']
        phases.append({'start_us':segment['start_us'],'end_us':segment['end_us'],'actual_error':error_statistics(actual_error,mask)})
    rpm=[]
    for motor in range(3):
        seen=b[:,13+motor].astype(bool)
        rpm.append({'logical_motor':motor,'accepted_rows':int(b[:,16+motor].sum()),'fresh_rows':int(b[:,19+motor].sum()),
                    'acceleration_valid_rows':int(b[:,10+motor].sum()),'invalid_observation_rows':int(np.count_nonzero(u[:,6+motor]&16)),
                    'maximum_seen_observation_age_us':int(u[seen,9+motor].max()) if seen.any() else None,
                    'packet_deliveries':int(b[:,31+motor].sum()),'dropped_generation_events':int(b[:,34+motor].sum()),
                    'outlier_generation_events':int(b[:,37+motor].sum()),'maximum_queue_depth':int(u[:,24+motor].max()),
                    'plant_rpm_clipped_rows':int(b[:,44+motor].sum())})
    return {'actual_response_error_all_rows':score,'phase_errors':phases,
        'predicted_actuator_response_error':error_statistics(n[:,39:43]-n[:,8:12],b[:,8]),
        'predicted_total_response_error':error_statistics(n[:,43:47]-n[:,4:8],b[:,9]),
        'predicted_corrected_counts_error':error_statistics((n[:,35]-n[:,14])[:,None],b[:,4]),
        'estimated_reaction_nm_error':error_statistics((n[:,37]-n[:,12])[:,None],b[:,5]),
        'quality':{'controller_valid_rows':int(b[:,0].sum()),'allocator_valid_rows':int(b[:,1].sum()),
            'allocator_not_converged_rows':int(np.count_nonzero(~b[:,2].astype(bool))),
            'allocator_exact_rows':int(b[:,3].sum()),'any_allocator_bound_active_rows':int(np.count_nonzero(u[:,2])),
            'reaction_available_rows':int(b[:,5].sum()),'reaction_compensation_clipped_rows':int(np.count_nonzero(u[:,0]&128)),
            'tail_effort_coordination_clipped_rows':int(np.count_nonzero(u[:,0]&2048)),
            'tail_cosine_guard_rows':int(np.count_nonzero(u[:,0]&1024)),'rpm':rpm}}


def write_csv(path,inputs,result):
    names=labels()
    with gzip.open(path,'wt') as stream:
        writer=csv.writer(stream);writer.writerow(['time_us','mode',*names['numeric'],*names['integer'],*names['flags']])
        for row,time in enumerate(inputs['time_us']):
            writer.writerow([int(time),int(inputs['modes'][row]),*result['numeric'][row],*result['integer'][row],*result['flags'][row]])


def plot_tracking(airframe,scenario,experiments,results,inputs,output):
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,2,figsize=(13,8),sharex=True,layout='constrained')
    time=inputs['time_us']/1e6
    for axis,(plot,name) in enumerate(zip(axes.flat,('collective','roll','pitch','yaw'))):
        plot.plot(time,inputs['requested_response'][:,axis],'--',color='.35',label='requested',linewidth=1)
        for experiment,result,color in zip(experiments,results,('#3275af','#c58220','#57945a')):
            plot.plot(time,result['numeric'][:,4+axis],label=experiment['name'],color=color,linewidth=.9)
        plot.set_ylabel(name+' response proxy');plot.grid(alpha=.2)
    axes[0,0].legend(fontsize=8)
    for axis in axes[-1]:axis.set_xlabel('Synthetic elapsed seconds')
    fig.suptitle(airframe+' / '+scenario+': actual plant response; every row scored, including faults and saturation')
    fig.savefig(output/(airframe+'-'+scenario+'-tracking.png'),dpi=150,bbox_inches='tight');plt.close(fig)


def plot_diagnostics(name,inputs,result,output):
    import matplotlib.pyplot as plt
    n,b=result['numeric'],result['flags'];time=inputs['time_us']/1e6
    fig,axes=plt.subplots(3,2,figsize=(14,11),sharex=True,layout='constrained')
    for column,label in [(33,'target'),(34,'C predictor'),(15,'synthetic truth')]:axes[0,0].plot(time,n[:,column],label=label,linewidth=1)
    axes[0,0].set_ylabel('Mathematical tail direction (rad)')
    axes[0,1].plot(time,n[:,14],label='truth instance');axes[0,1].plot(time,n[:,35],'--',label='predictor instance')
    axes[0,1].set_ylabel('Synthetic corrected ADC counts')
    for motor,color in enumerate(('#3275af','#c58220','#57945a')):
        axes[1,0].plot(time,n[:,17+motor],color=color,label=f'motor {motor} truth',linewidth=.8)
        axes[1,0].plot(time,n[:,69+motor],color=color,linestyle='--',linewidth=.7)
    axes[1,0].set_ylabel('Synthetic mechanical RPM')
    axes[1,1].plot(time,n[:,20],label='motor 0 analytic derivative',linewidth=.8)
    axes[1,1].plot(time,n[:,72],label='motor 0 C observer',linewidth=.8)
    axes[1,1].set_ylabel('Synthetic RPM/s (known truth here)')
    axes[2,0].plot(time,n[:,13],label='true additive disturbance',linewidth=.8)
    estimate=n[:,82].copy();estimate[~b[:,5].astype(bool)]=np.nan
    axes[2,0].plot(time,estimate,label='estimated disturbance',linewidth=.8)
    axes[2,0].plot(time,n[:,38],label='applied compensation',linewidth=.8)
    axes[2,0].set_ylabel('Yaw response proxy')
    axes[2,1].plot(time,n[:,7],label='actual total yaw',linewidth=.8)
    prediction=n[:,46].copy();prediction[~b[:,9].astype(bool)]=np.nan
    axes[2,1].plot(time,prediction,label='predicted total yaw',linewidth=.8)
    axes[2,1].plot(time,n[:,3],'--',color='.4',label='requested',linewidth=.8)
    axes[2,1].set_ylabel('Yaw response proxy')
    for axis in axes.flat:
        axis.fill_between(time,0,1,where=~b[:,5].astype(bool),transform=axis.get_xaxis_transform(),color='#e8ae45',alpha=.23)
        axis.grid(alpha=.2);axis.legend(fontsize=7,loc='upper right')
    for axis in axes[-1]:axis.set_xlabel('Synthetic elapsed seconds; shaded = reaction unavailable')
    fig.suptitle(name+': separate C servo instances, synthetic motor plant, actual C observer/reaction/allocator')
    fig.savefig(output/(name+'-diagnostics.png'),dpi=150,bbox_inches='tight');plt.close(fig)


def full_report(native,protocol,output):
    os.environ.setdefault('MPLCONFIGDIR',str(output/'matplotlib-cache'))
    import matplotlib
    matplotlib.use('Agg')
    runs=[]
    for airframe in protocol['airframes']:
        for scenario in protocol['scenarios']:
            results=[]
            for experiment in protocol['experiments']:
                controller,plant,mode=pack_run(protocol,airframe,scenario,experiment)
                inputs=tick_inputs(protocol,scenario,mode);result=native.run(controller,plant,**inputs)
                if not result['flags'][:,0].all() or not result['flags'][:,43].all():raise ValueError('Invalid controller or missing synthetic truth reaction')
                name='-'.join((airframe['name'],scenario['name'],experiment['name']));write_csv(output/(name+'.csv.gz'),inputs,result)
                runs.append({'name':name,'airframe':airframe['name'],'scenario':scenario['name'],'experiment':experiment['name'],
                    'rows':len(inputs['time_us']),'controller_parameters':controller.tolist(),'plant_parameters':plant.tolist(),
                    'csv':name+'.csv.gz','metrics':summarize(protocol,inputs,result)})
                results.append(result)
                if scenario['name']=='telemetry_faults' and experiment['name']=='reaction_only':plot_diagnostics(name,inputs,result,output)
            plot_tracking(airframe['name'],scenario['name'],protocol['experiments'],results,inputs,output)
    comparisons=[]
    for airframe in protocol['airframes']:
        for scenario in protocol['scenarios']:
            selected=[r for r in runs if r['airframe']==airframe['name'] and r['scenario']==scenario['name']]
            base=next(r for r in selected if r['experiment']=='baseline')['metrics']['actual_response_error_all_rows']
            comparisons.append({'airframe':airframe['name'],'scenario':scenario['name'],
                'actual_rmse_by_axis':{r['experiment']:r['metrics']['actual_response_error_all_rows']['rmse_by_axis'] for r in selected},
                'normalized_vector_rms':{r['experiment']:r['metrics']['actual_response_error_all_rows']['normalized_vector_rms'] for r in selected},
                'interpretation':'Descriptive synthetic errors only; no assertion that compensation improves every axis or scenario'})
    return {'runs':runs,'comparisons':comparisons}


def markdown(summary):
    lines=['# Offline integrated actuator-to-response simulation','',
        'This time-stepped simulation executes the actual C servo predictor, RPM observer, reaction equations, allocator and offline coordinator. A separate synthetic plant has its own C servo instance, analytic motor lag/slew, effectiveness map and coefficient values. It integrates no vehicle attitude or body dynamics and contains no PID, mixer, hardware or active control path.','',
        'The controller requires no CAD, CG, vehicle mass or body inertia. Its effectiveness map uses a declared global virtual-tail effort basis, not an arbitrary local motor/servo Jacobian. Mathematical direction and corrected PB0-PC0-equivalent counts are linked only by an explicit synthetic mapping, with no physical-angle calibration. Both servo instances use the same model family; matched agreement tests integration, not independent servo physics.','',
        'At each boundary the plant advances under prior commands, captures RPM and delivers queued observations, then the coordinator emits commands for the following interval. Decoder capture timestamps survive delayed software delivery, so observer age includes that queue delay. The capture is not an ESC-internal timing measurement. Cached/equal-zero packets, dropouts, outliers and reacquisition retain explicit identity/quality flags.','',
        'Plant actual response, direction, counts, derivative, map and reaction never enter the controller after its initial corrected-position reset; RPM packet metadata is the sole simulated measurement stream. Suffix perturbation and hidden-truth counterfactual tests verify that separation.','',
        'The full Q+Jalpha law is an explicitly synthetic additive yaw disturbance, independent of mathematical tail direction. Both maps must declare that they exclude it, preventing double counting. It is a scalar surrogate, without tilted-rotor, gyroscopic or aircraft torque-accuracy claims. The plant uses its analytic RPM derivative; the controller uses qualified observer acceleration. Reaction-only compensation and optional bounded virtual-tail-vertical coordination are separate experiments. The latter does not preserve arbitrary matrix collective response.','',
        'Every actual-response row is scored, including telemetry faults, saturation and estimator unavailability. Prediction errors are availability-masked with explicit counts/coverage. All four response errors are reported; improvement is not assumed, especially under mismatch. Plant RPM limits are real caps in this synthetic plant, with separate clip flags.','',
        '| Airframe / scenario | Experiment | Actual RMSE collective / roll / pitch / yaw | Predicted total coverage |','|---|---|---|---:|']
    for run in summary['runs']:
        m=run['metrics'];rmse=m['actual_response_error_all_rows']['rmse_by_axis']
        lines.append(f'| {run["airframe"]} / {run["scenario"]} | {run["experiment"]} | '+ ' / '.join(f'{v:.4g}' for v in rmse)+f' | {m["predicted_total_response_error"]["coverage_fraction"]:.1%} |')
    lines+=['','All numerical coefficients, maps, lags, limits and counts mappings are arbitrary synthetic placeholders. Error numbers describe this defined plant only and establish neither flight stability nor real-aircraft performance.','',
        'The MATLAB parity fixture executes the same C batch bridge; matching it is an interface/parity check, not independent model validation. MATLAB/Simulink analysis is maintained separately. `simulation-summary.json`, `native-build.json`, compressed traces and source hashes preserve reproduction details.','',
        'Reproduce with `python3 tools/tricopter/simulation_replay.py --output /path/to/output` and `python3 -m unittest discover -s tools/tricopter -p test_simulation_replay.py -v`. `--fixture-only` exports the compact shared-C parity input/output fixture.','']
    return '\n'.join(lines)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',required=True);parser.add_argument('--fixture-only',action='store_true')
    args=parser.parse_args();output=Path(args.output).resolve();output.mkdir(parents=True,exist_ok=True)
    protocol_hash=sha(PROTOCOL);protocol=load_protocol();native=NativeSimulation(output/'build');fixture=export_fixture(native,protocol,output)
    (output/'native-build.json').write_text(json.dumps(native.provenance,indent=2)+'\n')
    if args.fixture_only:
        print(json.dumps({'fixture':str(fixture),'sha256':sha(fixture),'rows':701,'shapes':[85,27,47]},indent=2));return
    summary={'protocol_sha256':protocol_hash,'protocol':protocol,'native_build':native.provenance,
             'column_labels':labels(),**full_report(native,protocol,output),'fixture_sha256':sha(fixture),'calibrated':False,'for_flight':False}
    if sha(PROTOCOL)!=protocol_hash or native.provenance['sources']!={str(path.relative_to(ROOT)):sha(path) for path in SOURCES}:
        raise RuntimeError('Simulation protocol/core/header/bridge changed during replay')
    scripts=[path for path in Path(__file__).parent.glob('simulation_*') if path.is_file()]
    scripts += [Path(__file__).with_name(name) for name in ('test_simulation_replay.py','allocator_native.py','rpm_data.py')]
    summary['script_sha256']={str(path.relative_to(ROOT)):sha(path) for path in scripts}
    (output/'simulation-summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    (output/'README.md').write_text(markdown(summary))
    files=sorted(path for path in output.iterdir() if path.is_file() and path.name!='checksums.sha256')
    (output/'checksums.sha256').write_text(''.join(f'{sha(path)}  {path.name}\n' for path in files))
    print(json.dumps({'output':str(output),'runs':len(summary['runs']),'rows':sum(r['rows'] for r in summary['runs']),
                      'fixture_sha256':summary['fixture_sha256']},indent=2))


if __name__=='__main__':main()
