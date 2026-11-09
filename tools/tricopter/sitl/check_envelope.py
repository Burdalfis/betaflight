#!/usr/bin/env python3
"""Fresh-world climb/forward-flight comparisons with frozen hover yaw gains."""
from __future__ import annotations

import argparse
import copy
import csv
import json
import math
from pathlib import Path
import shutil
import time

from autotune_yaw import IdentityError, ProfileTrial, fingerprint, write_json
from build import ADAPTER_PATCH_SHA256, ADAPTER_PREIMAGES, ADAPTER_POSTIMAGES
from check_comparison import analyze, sha
from check_comparison_fresh import freeze_sources, source_inputs, verify_case, verify_frozen_sources
from check_flight import PILOT, angle_channel
from envelope_targets import angle_targets, read_angle_earth_ref
from run import build_plugin, verified_binary
from runtime import ARTIFACTS, HERE, SessionLock, fresh_run_directory
from tune_profiles import overlay_text, verify_profile

REFERENCE = ARTIFACTS/'runs/20261108T235059-both-yaw-autotune-338473'
VARIANTS = ('stock', 'modified')
EXTRA_INPUTS = ('check_envelope.py', 'envelope_targets.py',
                'autotune_yaw.py', 'tune_metrics.py', 'tune_profiles.py',
                'build.py', 'patches/corrected-frame.patch')


def inputs():
    return dict(source_inputs(), **{name: HERE/name for name in EXTRA_INPUTS})


def read_limits(record):
    rates = bytes.fromhex(record['common_msp_readback']['111'])
    advanced = bytes.fromhex(record['common_msp_readback']['94'])
    if rates[22] != 3: raise ValueError('ACTUAL rate profile required')
    return {'angle_limit_deg': advanced[17], 'center': [rates[0]*10, rates[12]*10],
            'max_rate': [rates[2]*10, rates[3]*10], 'expo': [rates[1]/100, rates[13]/100]}


def cases(reference, climb_throttle=1400, forward_lead=3.):
    if not isinstance(climb_throttle,int) or not 1345<climb_throttle<=1500:
        raise ValueError('Climb collective must be an integer in 1346..1500 us')
    if (not isinstance(forward_lead,(int,float)) or isinstance(forward_lead,bool)
        or not math.isfinite(forward_lead) or not 3<=forward_lead<=8):
        raise ValueError('Forward lead must be finite and in 3..8 seconds')
    pitch = angle_channel(20., read_limits(reference), 1)
    result = []
    for point, throttle, pitch_pwm in [('hover',1345,1500), ('climb',climb_throttle,1500),
                                       ('forward',1380,pitch)]:
        for direction, yaw in [('control',1500), ('cw',1368), ('ccw',1632)]:
            result.append({'name':f'{point}_{direction}', 'operating_point':point,
                'kind':'control' if direction=='control' else 'yaw', 'direction':direction,
                'matched_control':f'{point}_control',
                'stages':[
                    {'name':'baseline', 'duration_s':1., 'channels':[1500,1500,1345,1500]},
                    {'name':'operating_lead', 'duration_s':float(forward_lead) if point=='forward' else 3.,
                     'channels':[1500,pitch_pwm,throttle,1500]},
                    {'name':'pulse', 'duration_s':2., 'channels':[1500,pitch_pwm,throttle,yaw]},
                    {'name':'recovery', 'duration_s':1.5, 'channels':[1500,pitch_pwm,throttle,1500]}]})
    return result


class EnvelopeTrial(ProfileTrial):
    maximum_horizontal_distance_m = 200.

    def make_pilot(self, *args):
        pilot=super().make_pilot(*args)
        pilot.maximum_height_m=200.
        return pilot

    def recapture(self, first=False):
        if not self.profile_verified:
            try:
                self.record['angle_earth_ref']=read_angle_earth_ref((self.directory/'provision.log').read_text(errors='replace'))
            except (ValueError,AssertionError,OSError) as error:
                raise IdentityError(str(error)) from error
            if self.record['angle_earth_ref']!=100:
                raise IdentityError('Native ANGLE earth reference differs from the frozen hover configuration')
        return super().recapture(first)

    def measurement(self, case):
        stages,end=super().measurement(case)
        for stage in stages:
            roll,pitch,_,yaw=stage['channels']
            stage.update(angle_targets(self.record,roll,pitch,yaw,
                earth_ref=self.record['angle_earth_ref']))
        return stages,end

    def finish(self):
        # A fresh disposable simulator world needs no long return/landing from
        # the climb. Retain a normal RC disarm and stop the owned processes.
        self.record['termination']='simulated_in_air_disarm_then_process_stop'
        self.record['pretermination_state']=self.pilot.snapshot()
        self.pilot.command('disarmed',active=False)
        self.receiver.select('disarmed','disarmed',[1500,1500,1000,1500])
        self.receiver.set(4,1000);self.receiver.set(2,1000)
        self.armed=False;time.sleep(.5)
        self.record['final_status']=self.client.status()
        if self.record['final_status']['armed']:raise AssertionError('Simulated termination did not disarm')


def program_health(directory, record):
    case=record['cases'][0]
    start=case['stages'][0]['actual_start_sim_time_s'];end=case['end_sim_time_s']
    with (directory/'plant.csv').open() as stream:
        rows=[{k:float(v) for k,v in row.items()} for row in csv.DictReader(stream)]
    selected=[r for r in rows if start<=r['sim_time_s']<=end]
    packets=json.loads((directory/'rc-packets.json').read_text())
    packets=[r for r in packets if r['sim_time_s'] is not None and start<=r['sim_time_s']<=end]
    telemetry=json.loads((directory/'telemetry.json').read_text())
    telemetry=[r for r in telemetry if r['sim_time_s'] is not None and start<=r['sim_time_s']<=end]
    observations={'plant_samples':len(selected),'status_samples':len(telemetry),
        'actuator_valid_fraction':sum(r['actuator_valid']==1 for r in selected)/len(selected),
        'all_status_samples_armed_without_fault':bool(telemetry) and all(
            r['armed'] and not r['arming_disable_flags'] for r in telemetry),
        'maximum_rc_gap_s':max(b['wall_monotonic']-a['wall_monotonic'] for a,b in zip(packets,packets[1:])),
        'maximum_state_gap_s':max(b['sim_time_s']-a['sim_time_s'] for a,b in zip(selected,selected[1:]))}
    if record['variant']=='modified':
        observations['feedback_valid_fraction']=sum(r['debug'][6]&3==3 for r in telemetry)/len(telemetry)
    if (observations['actuator_valid_fraction']!=1 or not observations['all_status_samples_armed_without_fault']
        or observations['maximum_rc_gap_s']>=.05 or observations['maximum_state_gap_s']>.05
        or observations.get('feedback_valid_fraction',1)!=1):
        raise AssertionError(f'Unhealthy planned program: {observations}')
    return rows,observations


class Campaign:
    def __init__(self, reference, directory, repetitions, selected, artifacts=None, climb_throttle=1400, forward_lead=3.):
        self.directory=directory
        self.artifacts=Path(artifacts).resolve() if artifacts is not None else ARTIFACTS
        self.reference_path=reference
        campaign=json.loads((reference/'autotune-summary.json').read_text())
        self.reference=json.loads((reference/'tuned-evaluation/comparison-summary.json').read_text())
        if not campaign['pass'] or not self.reference['pass']:raise ValueError('Completed hover tune required')
        self.gains=campaign['selected_gains']
        self.reference_records={run['variant']:run for run in self.reference['runs']}
        if set(self.gains)!=set(VARIANTS):raise ValueError('Both selected profiles required')
        self.binary_identities={}
        for variant in VARIANTS:
            binary=verified_binary(self.artifacts,variant)
            manifest=self.artifacts/variant/'build-manifest.json'
            metadata=json.loads(manifest.read_text())
            old=json.loads((ARTIFACTS/variant/'build-manifest.json').read_text())
            if old['binary_sha256']!=self.reference_records[variant]['binary_sha256']:
                raise IdentityError('Original hover binary provenance changed')
            adapter=metadata.get('provenance',{}).get('sitl_adapter')
            if adapter:
                if adapter['name']!='corrected-frame':raise IdentityError('Unrecognized common SITL correction')
                if (adapter['patch_sha256']!=ADAPTER_PATCH_SHA256
                    or adapter['allowed_preimage_sha256']!=ADAPTER_PREIMAGES
                    or adapter['applied_postimage_sha256']!=ADAPTER_POSTIMAGES
                    or adapter['changed_paths']!=sorted(ADAPTER_PREIMAGES)):
                    raise IdentityError('Common SITL correction differs from the four approved changes')
                before=adapter['allowed_preimage_sha256'];after=adapter['applied_postimage_sha256']
                old_sources=old['source_sha256'];new_sources=metadata['source_sha256']
                if set(old_sources)!=set(new_sources):raise IdentityError('Build source input set changed')
                for name,expected in old_sources.items():
                    if name in before:
                        if expected!=before[name] or new_sources[name]!=after[name]:
                            raise IdentityError('Incorrect shared adapter pre/postimage: '+name)
                    elif new_sources[name]!=expected:
                        raise IdentityError('Flight-control source changed outside the shared SITL correction: '+name)
            elif metadata['binary_sha256']!=old['binary_sha256']:
                raise IdentityError('Undeclared firmware binary change')
            self.binary_identities[variant]={'binary_sha256':sha(binary),
                'original_hover_binary_sha256':old['binary_sha256'],
                'build_manifest':str(manifest),'build_manifest_sha256':sha(manifest),
                'sitl_adapter':adapter}
            retained=directory/'firmware'/variant;retained.mkdir(parents=True)
            shutil.copy2(binary,retained/binary.name)
            shutil.copy2(manifest,retained/manifest.name)
        adapters=[value['sitl_adapter'] for value in self.binary_identities.values()]
        if any(adapters) and (not all(adapters) or any(
            a[key]!=adapters[0][key] for a in adapters[1:]
            for key in ('name','patch_sha256','allowed_preimage_sha256','applied_postimage_sha256'))):
            raise IdentityError('Stock and modified do not use the same common SITL correction')
        # Executable harness may grow bounds/hooks, but plant/config/assets and
        # the exact per-variant binaries must stay identical to the hover study.
        previous=self.reference['protocol']['source_sha256']
        current=inputs()
        for name,digest in previous.items():
            if name.startswith(('gazebo/','cad/','configs/')):
                if name not in current or sha(current[name])!=digest:
                    raise AssertionError('Hover plant/config changed: '+name)
        definitions=cases(self.reference_records['stock'],climb_throttle,forward_lead)
        if selected:
            names=selected.split(',')
            if len(set(names))!=len(names) or any(name not in {c['name'] for c in definitions} for name in names):
                raise ValueError('Unknown or duplicate selected case')
            definitions=[c for c in definitions if c['name'] in names]
        self.frozen=freeze_sources(directory,inputs())
        self.protocol={'cases':definitions,'repetitions':repetitions,
            'hover_reference_directory':str(reference),'hover_reference_summary_sha256':sha(reference/'autotune-summary.json'),
            'requested_yaw_gains':self.gains,'feedforward_yaw':0,'feedforward_yaw_hold_gain':0,
            'climb_throttle_us':climb_throttle,
            'forward_lead_s':float(forward_lead),
            'firmware_artifacts_directory':str(self.artifacts),'binary_identities':self.binary_identities,
            'stock_label':'stock controller with common SITL coordinate correction' if any(adapters) else 'unmodified stock',
            'angle_earth_ref':100,'gains_frozen_no_retuning':True,
            'fresh_case_reset':True,'measurement_rc_rate_hz':50,'plant_trace_rate_hz':100,
            'maximum_stage_duration_error_s':.03,'maximum_measurement_rc_send_gap_s':.05,
            'setup_pilot':PILOT.copy(),'setup_gate':self.reference['protocol']['initial_gate'],
            'maximum_horizontal_distance_m':200,'maximum_height_above_origin_m':200,
            'maximum_absolute_roll_pitch_deg':40,'measurement_minimum_body_height_m':.35,
            'measurement_first_four_channels_override_all_external_pilot_outputs':True,
            'no_native_autopilot_modes':True,'no_speed_hold_during_measurement':True,
            'termination':'normal_RC_disarm_in_air_then_stop_disposable_simulator',
            'variant_order':[v for i in range(repetitions) for v in
                (VARIANTS if i%2==0 else tuple(reversed(VARIANTS)))],
            'source_sha256':self.frozen,
            'operating_point_policy':'Fixed identical RC and stage times; report achieved speeds/acceleration, do not infer steady flight from stick commands',
            'forward_turn_policy':'Fixed body pitch/roll and yaw stick produce a curved path; ANGLE earth-reference yaw attenuation and coordinated body-rate terms retained',
            'failure_policy':'Retain every failure and continue other cases; no selective retry or gain changes'}
        self.protocol['sha256']=fingerprint(self.protocol)
        write_json(directory/'protocol.json',self.protocol)
        self.summary={'pass':False,'directory':str(directory),'protocol':self.protocol,'runs':[]}
        self.plugin=None

    def guard(self):
        verify_frozen_sources(self.directory,self.frozen,inputs())
        if self.plugin and sha(self.plugin)!=self.summary['plugin_sha256']:raise AssertionError('Frozen plugin changed')
        for identity in self.binary_identities.values():
            if sha(Path(identity['build_manifest']))!=identity['build_manifest_sha256']:
                raise IdentityError('Build manifest changed during comparison')

    def run(self):
        try:
            with SessionLock():
                built=build_plugin(ARTIFACTS,4);self.guard()
                self.plugin=self.directory/'plugin'/built.name;self.plugin.parent.mkdir()
                shutil.copy2(built,self.plugin)
                self.summary['plugin_sha256']=sha(self.plugin)
                if self.summary['plugin_sha256']!=self.reference_records['stock']['plugin_sha256']:
                    raise AssertionError('Plugin differs from hover study')
                shutil.copy2(built.parent/'build-manifest.json',self.plugin.parent/'build-manifest.json')
                profiles=self.directory/'profiles';profiles.mkdir()
                for variant in VARIANTS:
                    (profiles/f'{variant}.cli').write_text(overlay_text(self.gains[variant])+'get angle_earth_ref\n')
                for index,variant in enumerate(self.protocol['variant_order']):
                    group_path=self.directory/f'{index+1:02d}-{variant}';group_path.mkdir()
                    group={'variant':variant,'repetition':index//2+1,'pass':False,
                        'group_directory':group_path.name,'cases':[],'failures':[],'case_directories':{}}
                    self.summary['runs'].append(group)
                    for definition in self.protocol['cases']:
                        self.guard();path=group_path/definition['name']
                        single=copy.deepcopy(self.protocol);single['cases']=[copy.deepcopy(definition)]
                        trial=EnvelopeTrial(path,variant,self.plugin,
                            self.directory/'source/gazebo/worlds/tricopter.sdf',single,
                            gains=self.gains[variant],extra_config=profiles/f'{variant}.cli',
                            artifacts=getattr(self,'artifacts',ARTIFACTS))
                        trial.record.update(selected_case=definition['name'],fresh_case_reset=True,
                                            requested_yaw_gains=self.gains[variant])
                        try:
                            trial.execute();record=analyze(path);verify_case(record,self.protocol)
                            try:
                                record['yaw_profile_verification']=verify_profile(record,self.gains[variant],directory=path)
                            except (ValueError,AssertionError) as error:
                                raise IdentityError(str(error)) from error
                            reference=self.reference_records[variant]
                            if record['binary_sha256']!=self.binary_identities[variant]['binary_sha256']:
                                raise IdentityError('Selected firmware binary changed')
                            for field in ('plugin_sha256','common_config_sha256','common_msp_readback','yaw_settings'):
                                if record[field]!=reference[field]:raise IdentityError('Hover identity drift: '+field)
                            _,health=program_health(path,record)
                            case=record['cases'][0]
                            case['program_health']=health
                            write_json(path/'comparison-metrics.json',record)
                            group['cases'].append(case);group['case_directories'][definition['name']]=definition['name']
                            print(f"COMPLETE {group_path.name}/{definition['name']}",flush=True)
                        except Exception as error:
                            group['failures'].append({'case':definition['name'],'directory':str(path),'error':repr(error)})
                            print(f"FAILED {group_path.name}/{definition['name']}: {error!r}",flush=True)
                            if isinstance(error,IdentityError):raise
                        finally:
                            self.guard();write_json(self.directory/'envelope-summary.json',self.summary)
                    group['pass']=not group['failures'] and len(group['cases'])==len(self.protocol['cases'])
                self.summary['pass']=all(group['pass'] for group in self.summary['runs'])
                self.summary['all_owned_processes_stopped']=True
        except Exception as error:
            self.summary['error']=repr(error);raise
        finally:write_json(self.directory/'envelope-summary.json',self.summary)
        print(f"Envelope comparison finished: pass={self.summary['pass']}",flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference',type=Path,default=REFERENCE)
    parser.add_argument('--artifacts',type=Path,default=ARTIFACTS/'corrected-frame',
                        help='Firmware builds, optionally with the identical declared common SITL coordinate correction')
    parser.add_argument('--repetitions',type=int,default=3)
    parser.add_argument('--climb-throttle',type=int,default=1400,
                        help='Fixed climb collective, 1346..1500 us; default 1400, rapid-climb comparison 1450')
    parser.add_argument('--forward-lead',type=float,default=3.,
                        help='Fixed forward acceleration lead, 3..8 s; default 3, higher-speed comparison 6')
    parser.add_argument('--cases',help='Optional comma-separated case subset')
    parser.add_argument('--label',default='frozen-hover-envelope')
    parser.add_argument('--dry-run',action='store_true')
    args=parser.parse_args()
    if not 1<=args.repetitions<=5:parser.error('Repetitions must be 1..5')
    if not 1345<args.climb_throttle<=1500:parser.error('Climb collective must be 1346..1500 us')
    if not math.isfinite(args.forward_lead) or not 3<=args.forward_lead<=8:
        parser.error('Forward lead must be finite and in 3..8 seconds')
    if args.dry_run:
        reference=json.loads((args.reference/'tuned-evaluation/comparison-summary.json').read_text())
        print(json.dumps(cases(reference['runs'][0],args.climb_throttle,args.forward_lead),indent=2));return
    PILOT['max_angle_deg']=12.
    directory=fresh_run_directory(ARTIFACTS,'both',args.label)
    print(f'Envelope comparison: {directory}',flush=True)
    campaign=Campaign(args.reference.resolve(),directory,args.repetitions,args.cases,args.artifacts,args.climb_throttle,args.forward_lead)
    campaign.run()
    if not campaign.summary['pass']:raise SystemExit(1)


if __name__=='__main__':main()
