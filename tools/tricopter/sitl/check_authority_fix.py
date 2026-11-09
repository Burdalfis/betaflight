#!/usr/bin/env python3
"""Matched stock/before/fixed comparisons after the declared common plant revision.

Historical linear-drag campaigns remain immutable. This runner pins the new
plant and controller identities rather than accepting them as the old baseline.
It reuses the same RC sender, setup gates, timing checks and response metrics.
"""
from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
import shutil

from autotune_yaw import IdentityError, fingerprint, write_json
from build import ADAPTER_PATCH_SHA256, ADAPTER_PREIMAGES, ADAPTER_POSTIMAGES
from check_comparison import analyze, sha
from check_comparison_fresh import freeze_sources, verify_case, verify_frozen_sources
from check_envelope import program_health
from check_flight import PILOT
from check_yaw_stress import PROFILES, StressTrial, extended_world, inputs as stress_inputs, native_integer, stage, steady_entry_gate, stress_cases
from run import build_plugin, verified_binary
from runtime import ARTIFACTS, HERE, SessionLock, fresh_run_directory
from stress_metrics import analyze_trial
from tune_profiles import overlay_text, verify_profile

ALLOWED_CONTROLLER_CHANGES={
    'src/main/flight/tricopter_control.c','src/main/flight/tricopter_control.h',
    'src/main/flight/mixer_tricopter.c','src/main/flight/mixer_tricopter.h',
    'src/main/flight/mixer.c',
}


def inputs(video):
    return dict(stress_inputs(video), **{'check_authority_fix.py':HERE/'check_authority_fix.py'})


def definitions(args):
    cases=stress_cases(args.forward_lead)
    # Keep the old throttle/yaw waveforms. Pitch and roll for the forward point
    # are an explicit shared trim for the revised drag law, not per-controller tuning.
    for case in cases:
        if case['operating_point']!='steady_forward_70':continue
        case['forward_trim_rc_us']=[args.forward_roll,args.forward_pitch]
        for item in case['stages']:
            if item['name'].startswith('ramp_'):
                fraction=int(item['name'].split('_')[1])/20
                item['channels'][0]=round(1500+(args.forward_roll-1500)*fraction)
                item['channels'][1]=round(1500+(args.forward_pitch-1500)*fraction)
            elif item['name'] in ('operating_lead','pulse','recovery'):
                item['channels'][0]=args.forward_roll;item['channels'][1]=args.forward_pitch
    for direction,yaw in (('cw',1368),('ccw',1632)):
        cases.append({'name':f'hover_{direction}','kind':'yaw','direction':direction,
            'operating_point':'hover','stages':[stage('baseline',1),stage('pulse',2,1345,yaw),stage('recovery',2)]})
    selected=args.cases.split(',')
    known={case['name']:case for case in cases}
    if len(selected)!=len(set(selected)) or any(name not in known for name in selected):
        raise ValueError('Unknown or duplicate case selection')
    return [known[name] for name in selected]


def controller_catalog(before,after):
    catalog={}
    for name,variant,artifacts in (('stock','stock',before),('before','modified',before),('modified','modified',after)):
        binary=verified_binary(artifacts,variant)
        manifest=artifacts/variant/'build-manifest.json'
        metadata=json.loads(manifest.read_text())
        adapter=metadata['provenance']['sitl_adapter']
        if adapter['name']!='corrected-frame' or adapter['patch_sha256']!=ADAPTER_PATCH_SHA256:
            raise IdentityError('Unexpected common SITL adapter')
        if (adapter.get('allowed_preimage_sha256')!=ADAPTER_PREIMAGES
            or adapter.get('applied_postimage_sha256')!=ADAPTER_POSTIMAGES
            or adapter.get('changed_paths')!=sorted(ADAPTER_PREIMAGES)
            or any(metadata['source_sha256'].get(path)!=digest for path,digest in ADAPTER_POSTIMAGES.items())):
            raise IdentityError('Common SITL adapter pre/postimage or path mismatch')
        catalog[name]={'variant':variant,'artifacts':str(artifacts),'binary_sha256':sha(binary),
            'manifest':str(manifest),'manifest_sha256':sha(manifest),'metadata':metadata}
    old=catalog['before']['metadata']['source_sha256'];new=catalog['modified']['metadata']['source_sha256']
    changed=sorted(name for name in set(old)|set(new) if old.get(name)!=new.get(name))
    if not changed or set(changed)-ALLOWED_CONTROLLER_CHANGES:
        raise IdentityError('Unexpected firmware changes: '+str(changed))
    return catalog,changed


class AuthorityTrial(StressTrial):
    def recapture(self,first=False):
        log=(self.directory/'provision.log').read_text(errors='replace')
        # Native lookup fields print ON/OFF. Retain the original log while
        # reusing the strict final-query/later-setting check for its ON value.
        first_arm=log.replace('gyro_cal_on_first_arm = ON','gyro_cal_on_first_arm = 1')
        if native_integer(first_arm,'gyro_cal_on_first_arm')!=1:
            raise IdentityError('Native first-arm gyro calibration is not enabled')
        self.record['gyro_calibration_policy']='Native gyro_cal_on_first_arm=ON after ground settling'
        if self.variant=='modified':
            actual=native_integer(log,'tri_yaw_trim')
            expected=self.protocol['trim_deci_deg']
            if actual!=expected:raise IdentityError('Physical yaw trim readback differs')
            self.record['physical_yaw_trim_deci_deg']=actual
        forward=self.protocol['cases'][0]['operating_point']=='steady_forward_70'
        previous_ascent=PILOT['ascent_m_s']
        try:
            if first and forward:
                self.target=self.pilot.origin[2]+self.protocol['forward_start_height_m']
                PILOT['ascent_m_s']=2.
                self.record['forward_setup']={'target_height_above_origin_m':self.protocol['forward_start_height_m'],
                    'ascent_m_s':2.,'same_initial_hover_gate':True}
            return super().recapture(first)
        finally:PILOT['ascent_m_s']=previous_ascent


class Campaign:
    def __init__(self,args,directory):
        self.args=args;self.directory=directory
        self.cases=definitions(args)
        self.controllers=args.controllers.split(',');self.profiles=args.profiles.split(',')
        if len(set(self.controllers))!=len(self.controllers) or any(c not in ('stock','before','modified') for c in self.controllers):raise ValueError('Invalid controllers')
        if any(p not in PROFILES for p in self.profiles):raise ValueError('Invalid profiles')
        self.catalog,changed=controller_catalog(args.before_build.resolve(),args.after_build.resolve())
        self.frozen=freeze_sources(directory,inputs(args.video))
        self.world=directory/'recording-world.sdf'
        self.world_record=extended_world(directory/'source/gazebo/worlds/tricopter.sdf',self.world)
        self.protocol={'study':'Yaw-authority fix on a new shared quadratic-drag baseline',
            'cases':self.cases,'profiles':{p:PROFILES[p] for p in self.profiles},
            'controllers':self.controllers,'controller_identities':self.catalog,
            'declared_native_controller_changes':changed,'trim_deci_deg':args.trim,
            'trim_policy':'Signed physical basket trim for both modified binaries; physical PWM/ADC zero unchanged. Stock retains original native servo configuration.',
            'repetitions':args.repetitions,'source_sha256':self.frozen,'world_change':self.world_record,
            'gyro_only_diagnostic':args.gyro_only,'forward_lead_s':args.forward_lead,
            'forward_trim_rc_us':[args.forward_roll,args.forward_pitch],
            'forward_start_height_m':args.forward_start_height,
            'forward_setup_policy':'Common elevated hover with2m/s setup ascent; same gate; all external pilot channels overridden during measurement',
            'angle_limit_deg':80,'measurement_rc_rate_hz':50,'plant_trace_rate_hz':100,
            'gyro_calibration_policy':'gyro_cal_on_first_arm=ON for every controller; calibrate after the simulated model settles',
            'maximum_stage_duration_error_s':.03,'maximum_measurement_rc_send_gap_s':.05,
            'setup_pilot':PILOT.copy(),'fresh_world_eeprom_per_case':True,
            'no_external_pilot_during_measurement':True,
            'physics_policy':'Every selected controller uses the identical frozen revised plant; historical linear-drag studies are separate.',
            'failure_policy':'All attempts and guards retained; no selective gain changes or dropped failures',
            'video_policy':'QHD60 hardware capture on first repetition when requested'}
        self.protocol['sha256']=fingerprint(self.protocol)
        write_json(directory/'protocol.json',self.protocol)
        self.summary={'pass':False,'directory':str(directory),'protocol':self.protocol,'entries':[]}
        self.plugin=None

    def guard(self):
        verify_frozen_sources(self.directory,self.frozen,inputs(self.args.video))
        if sha(self.world)!=self.world_record['extended_sha256']:raise IdentityError('Frozen world changed')
        if self.plugin and sha(self.plugin)!=self.summary['plugin_sha256']:raise IdentityError('Frozen plugin changed')
        for item in self.catalog.values():
            if sha(Path(item['manifest']))!=item['manifest_sha256']:raise IdentityError('Build manifest changed')
            if sha(verified_binary(Path(item['artifacts']),item['variant']))!=item['binary_sha256']:raise IdentityError('Firmware binary changed')

    def run(self):
        common={}
        try:
            with SessionLock():
                built=build_plugin(ARTIFACTS,4);self.guard()
                self.plugin=self.directory/'plugin'/built.name;self.plugin.parent.mkdir()
                shutil.copy2(built,self.plugin)
                self.summary['plugin_sha256']=sha(self.plugin)
                shutil.copy2(built.parent/'build-manifest.json',self.plugin.parent/'build-manifest.json')
                for repetition in range(1,self.args.repetitions+1):
                    for profile_name in self.profiles:
                        profile=PROFILES[profile_name]
                        for definition in self.cases:
                            order=self.controllers if repetition%2 else list(reversed(self.controllers))
                            for controller in order:
                                self.guard();identity=self.catalog[controller];variant=identity['variant']
                                overlay=self.directory/f'{profile_name}-{controller}.cli'
                                overlay.write_text(overlay_text(profile['gains'],profile['feedforward'],profile['hold_gain'])+
                                    'set angle_limit = 80\nget angle_limit\nget angle_earth_ref\n'+
                                    'set gyro_cal_on_first_arm = ON\nget gyro_cal_on_first_arm\n'+
                                    (f'set tri_yaw_trim = {self.args.trim}\nget tri_yaw_trim\n' if variant=='modified' else '')+
                                    ('set imu_dcm_kp = 0\nset imu_dcm_ki = 0\nget imu_dcm_kp\nget imu_dcm_ki\n' if self.args.gyro_only else ''))
                                path=self.directory/f'{repetition:02d}-{profile_name}-{definition["name"]}-{controller}'
                                entry={'directory':str(path),'variant':controller,'platform_variant':variant,'profile':profile_name,
                                    'case':definition['name'],'repetition':repetition,'completed':False}
                                self.summary['entries'].append(entry)
                                single=copy.deepcopy(self.protocol);single['cases']=[copy.deepcopy(definition)]
                                trial=AuthorityTrial(path,variant,self.plugin,self.world,single,profile=profile,
                                    extra_config=overlay,artifacts=Path(identity['artifacts']),video=self.args.video and repetition==1)
                                trial.record.update(profile_name=profile_name,profile=profile,controller_label=controller,
                                    selected_case=definition['name'],fresh_case_reset=True)
                                try:
                                    trial.execute();record=analyze(path);verify_case(record,self.protocol)
                                    _,entry['program_health']=program_health(path,record)
                                    verify_profile(record,profile['gains'],profile['feedforward'],profile['hold_gain'],directory=path)
                                    matched={key:record[key] for key in ('world_sha256','plugin_sha256','common_config_sha256','common_msp_readback','yaw_settings')}
                                    if profile_name in common and matched!=common[profile_name]:raise IdentityError('Unmatched shared settings')
                                    common[profile_name]=matched;entry['completed']=True
                                except Exception as error:
                                    entry['error']=repr(error);print(f'FAILED {path.name}: {error!r}',flush=True)
                                    if isinstance(error,IdentityError):raise
                                finally:
                                    if (path/'comparison-run.json').is_file():
                                        try:
                                            entry['metrics']=analyze_trial(path)
                                            if definition.get('steady_entry_required') and entry['completed']:
                                                entry['steady_entry_gate']=steady_entry_gate(entry['metrics'])
                                        except Exception as error:entry['metrics_error']=repr(error)
                                    if (path/'video-record.json').is_file():entry['video']=json.loads((path/'video-record.json').read_text())
                                    print(f'RECORDED {path.name}: completed={entry["completed"]}',flush=True)
                                    self.guard();write_json(self.directory/'stress-summary.json',self.summary)
                self.summary['pass']=all(e['completed'] and 'metrics_error' not in e for e in self.summary['entries'])
                steady={c['name'] for c in self.cases if c.get('steady_entry_required')}
                gates=[bool(e['completed'] and e.get('steady_entry_gate',{}).get('qualified',False)) for e in self.summary['entries'] if e['case'] in steady]
                self.summary['all_requested_steady_entries_qualified']=all(gates) if gates else None
        finally:write_json(self.directory/'stress-summary.json',self.summary)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--before-build',type=Path,required=True)
    parser.add_argument('--after-build',type=Path,default=ARTIFACTS/'corrected-frame')
    parser.add_argument('--controllers',default='stock,modified')
    parser.add_argument('--profiles',default='factory_yaw,hover_tuned')
    parser.add_argument('--cases',default='punch_70,punch_85,punch_100,hold_90_control,hold_95_control,hold_100_control')
    parser.add_argument('--repetitions',type=int,default=1)
    parser.add_argument('--trim',type=int,default=-61)
    parser.add_argument('--forward-roll',type=int,default=1332)
    parser.add_argument('--forward-pitch',type=int,default=1903)
    parser.add_argument('--forward-start-height',type=float,default=20.)
    parser.add_argument('--forward-lead',type=float,default=25.)
    parser.add_argument('--gyro-only',action='store_true')
    parser.add_argument('--video',action='store_true')
    parser.add_argument('--label',default='quadratic-authority-fix')
    parser.add_argument('--dry-run',action='store_true')
    args=parser.parse_args()
    if not 1<=args.repetitions<=3 or not -478<=args.trim<=216:parser.error('Invalid repetitions or calibrated trim')
    if not 1000<=args.forward_roll<=2000 or not 1000<=args.forward_pitch<=2000:parser.error('Invalid forward trim RC')
    if not 1.5<=args.forward_start_height<=30:parser.error('Forward setup height must be1.5..30m')
    if args.dry_run:print(json.dumps(definitions(args),indent=2));return
    PILOT['max_angle_deg']=12.
    directory=fresh_run_directory(ARTIFACTS,'both',args.label)
    print(f'Authority comparison: {directory}',flush=True)
    campaign=Campaign(args,directory)
    campaign.run()
    if not campaign.summary['pass']:raise SystemExit(1)


if __name__=='__main__':main()
