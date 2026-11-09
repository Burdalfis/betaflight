#!/usr/bin/env python3
"""Matched high-collective yaw stress tests in disposable Gazebo worlds.

No aerodynamic coefficients or flight-control code are changed. Factory yaw
gains are tested with the same aircraft roll/pitch configuration as the frozen
hover tune. Every attempted flight, including guards and timing failures, stays
in the ledger. A completed flight is not necessarily a stable yaw response.
"""
from __future__ import annotations

import argparse
import copy
import json
import math
import os
from pathlib import Path
import re
import shutil
import time
import xml.etree.ElementTree as ET

from autotune_yaw import IdentityError, fingerprint, write_json
from check_comparison import Trial, analyze, sha
from check_comparison_fresh import freeze_sources, verify_case, verify_frozen_sources
from check_envelope import Campaign as IdentityCampaign, EnvelopeTrial, REFERENCE, inputs as envelope_inputs, program_health
from check_flight import PILOT
from envelope_metrics import body_to_euler_rates
from envelope_targets import angle_targets, read_angle_earth_ref
from run import build_plugin, materialize_world
from runtime import ARTIFACTS, HERE, SessionLock, fresh_run_directory
from tune_profiles import overlay_text, verify_profile

PROFILES = {
    'factory_yaw': {'gains': {'p':45,'i':80,'d':0}, 'feedforward':120, 'hold_gain':15,
        'label':'Betaflight factory yaw gains; aircraft roll/pitch gains retained'},
    'hover_tuned': {'gains': {'p':31,'i':20,'d':10}, 'feedforward':0, 'hold_gain':0,
        'label':'Frozen hover-tuned yaw gains; aircraft roll/pitch gains retained'},
    'aircraft_original': {'gains': {'p':50,'i':10,'d':20}, 'feedforward':47, 'hold_gain':15,
        'label':'Aircraft original yaw gains, not Betaflight factory defaults'},
}


def stage(name, duration, throttle=1345, yaw=1500, roll=1500, pitch=1500):
    return {'name':name,'duration_s':float(duration),'channels':[roll,pitch,throttle,yaw]}


def stress_cases(forward_lead=25.):
    if not math.isfinite(forward_lead) or not 25<=forward_lead<=45:
        raise ValueError('Forward lead must be25..45 seconds')
    cases=[]
    # Native min_check=1050 and integer RC normalization are retained.
    # 1715/1858/2000 produce exactly .70/.85/1.0 native collective.
    for percent, throttle in ((70,1715),(85,1858),(100,2000)):
        cases.append({'name':f'punch_{percent}', 'kind':'collective', 'direction':'control',
            'operating_point':'punchout', 'native_collective_requested':((throttle-1050)*1000//950)/1000,
            'stages':[stage('baseline',1),stage('pulse',1,throttle),stage('recovery',4)]})
    for percent, throttle in ((70,1715),(85,1858),(90,1905),(95,1953),(100,2000)):
        for direction,yaw in (('control',1500),('cw',1368),('ccw',1632)):
            cases.append({'name':f'hold_{percent}_{direction}',
                'kind':'collective' if direction=='control' else 'combined',
                'direction':direction,'operating_point':'high_collective_hold',
                'stages':[stage('baseline',1),stage('operating_lead',.5,throttle),
                          stage('pulse',4 if percent<=85 else 3,throttle,yaw),stage('recovery',4)]})
    for direction,yaw in (('control',1500),('cw',1368),('ccw',1632)):
        # A common smooth setup is followed by a long fixed RC hold. No speed
        # or altitude controller contributes to the measured RC program.
        stages=[stage('baseline',1)]
        for i in range(1,21):
            f=i/20
            stages.append(stage(f'ramp_{i:02d}',.2,round(1345+(1715-1345)*f),
                                roll=round(1500+(1440-1500)*f),pitch=round(1500+(1986-1500)*f)))
        stages += [stage('operating_lead',forward_lead,1715,roll=1440,pitch=1986),
                   stage('pulse',3,1715,yaw,1440,1986),
                   stage('recovery',4,1715,1500,1440,1986)]
        cases.append({'name':f'forward_70_{direction}', 'kind':'control' if direction=='control' else 'yaw',
            'direction':direction,'operating_point':'steady_forward_70',
            'steady_entry_required':True,'stages':stages})
    return cases


def inputs(video=True):
    names=('check_yaw_stress.py','stress_metrics.py','envelope_metrics.py')
    if video:names+=('record_flight_video.py',)
    return dict(envelope_inputs(), **{name:HERE/name for name in names})


def steady_entry_gate(metrics):
    entry=metrics['cases'][0].get('entry_statistics')
    if not entry:return {'qualified':False,'reason':'No observed pre-probe window'}
    duty=sum(m['modeled_duty_mean'] for m in entry['motors'].values())/3
    checks={
        'two_second_window':entry['duration_s']>=1.99,
        'native_collective_at_least_70_percent':entry['throttle_demand']['normalized_above_mincheck']>=.7-1e-9,
        'mean_motor_duty_at_least_70_percent':duty>=.7,
        'speed_range_under_one_percent':entry['horizontal_speed_m_s']['range_over_mean'] is not None and entry['horizontal_speed_m_s']['range_over_mean']<.01,
        'vertical_speed_within_0_3_m_s':max(abs(entry['vz_m_s'][k]) for k in ('min','max'))<=.3,
        'acceleration_rms_under_0_1_m_s2':entry['finite_difference_3d_acceleration_rms_m_s2']<.1,
        'roll_pitch_rates_under_1_dps':entry['maximum_abs_euler_roll_pitch_rate_dps']<1,
        'world_heading_rate_under_2_dps':entry['maximum_abs_world_yaw_rate_dps']<2,
    }
    return {'qualified':all(checks.values()),'checks':checks,'mean_motor_duty':duty,
            'window':'Final2 seconds before pulse, fixed common input and settling time'}


def native_integer(log, name):
    clean=re.sub(r'\x1b\[[0-?]*[ -/]*[@-~]','',log).replace('\r','')
    commands=list(re.finditer(r'^\s*#\s*(?P<command>[^\n]*)$',clean,re.MULTILINE))
    queries=[i for i,c in enumerate(commands) if c['command'].strip()==f'get {name}']
    if not queries:raise IdentityError('Missing native query: '+name)
    i=queries[-1];end=commands[i+1].start() if i+1<len(commands) else len(clean)
    matches=re.findall(rf'^\s*{re.escape(name)}\s*=\s*(-?\d+)\s*$',clean[commands[i].end():end],re.MULTILINE)
    if len(matches)!=1:raise IdentityError('Ambiguous native query: '+name)
    if any(re.match(rf'set\s+{re.escape(name)}\b',c['command'].strip()) for c in commands[i+1:]):
        raise IdentityError('Setting changed after native query: '+name)
    return int(matches[0])


def extended_world(source, destination):
    """Only extend ground; the complete aircraft subtree must be byte-identical."""
    materialize_world(source,destination)
    tree=ET.parse(destination)
    before=ET.tostring(tree.find("./world/model[@name='tricopter']"))
    ground=tree.find("./world/model[@name='ground']")
    for size in ground.findall('.//plane/size'):size.text='20000 20000'
    # One textured surface supplies world references without coplanar strips
    # or crossings. Ground collision and aircraft physics stay identical.
    visual=ground.find('./link/visual[@name="ground_visual"]')
    visual.remove(visual.find('geometry'))
    material=visual.find('material')
    if material is not None:visual.remove(material)
    mesh_path=source.resolve().parent.parent/'meshes/review_ground.dae'
    texture_path=mesh_path.with_suffix('.png')
    if not mesh_path.is_file() or not texture_path.is_file():
        raise FileNotFoundError('Review ground mesh and texture must accompany the source world')
    geometry=ET.SubElement(visual,'geometry');mesh=ET.SubElement(geometry,'mesh')
    ET.SubElement(mesh,'uri').text=str(mesh_path)
    ET.SubElement(visual,'cast_shadows').text='false'
    world=tree.find('./world')
    grid=world.find('./scene/grid')
    if grid is None:grid=ET.SubElement(world.find('scene'),'grid')
    grid.text='false'
    # Scene transport must exceed capture cadence to avoid duplicate-frame
    # aliasing between separate60Hz simulation-pose and screen-capture clocks.
    broadcaster=world.find("plugin[@name='gz::sim::systems::SceneBroadcaster']")
    for name in ('state_hertz','dynamic_pose_hertz'):
        setting=broadcaster.find(name)
        if setting is None:setting=ET.SubElement(broadcaster,name)
        setting.text='240'
    if ET.tostring(tree.find("./world/model[@name='tricopter']"))!=before:
        raise IdentityError('Aircraft changed while extending ground')
    tree.write(destination,encoding='utf-8',xml_declaration=True)
    return {'change':'Ground collision size100 to20000 metres; one textured surface and no GUI grid; visual transport240Hz',
            'ground_mesh_sha256':sha(mesh_path),'ground_texture_sha256':sha(texture_path),
            'visual_transport_hz':240,'physics_step_unchanged':True,
            'aircraft_subtree_unchanged':True,'source_sha256':sha(source),'extended_sha256':sha(destination)}


class StressTrial(Trial):
    maximum_horizontal_distance_m=10000.

    def __init__(self,*args,profile,video=False,**kwargs):
        super().__init__(*args,**kwargs)
        self.profile=profile;self.video_requested=video;self.video=None;self.next_video=0

    def make_pilot(self,*args):
        if self.video_requested:
            from record_flight_video import FlightVideo
            partition=f'tricopter-compare-{os.getpid()}-{self.directory.name}'
            self.video=FlightVideo(self.directory,partition)
            self.video.start()
        pilot=super().make_pilot(*args)
        pilot.maximum_height_m=800.;pilot.maximum_attitude_deg=85.
        return pilot

    def health(self):
        state=super().health()
        if self.video:
            self.video.check()
            if state and time.monotonic()>self.next_video:
                packet=self.receiver.sent[-1] if self.receiver.sent else {'stage':'setup','channels':[1500]*4}
                world_yaw=body_to_euler_rates(state['roll_deg'],state['pitch_deg'],
                    tuple(state['gyro_frd_'+a] for a in ('x','y','z')))[2]
                self.video.update(f"{self.record.get('controller_label',self.variant).upper()} | {self.record['profile_name']} | {self.protocol['cases'][0]['name']}\n"
                    f"stage: {packet['stage']}   sim: {state['sim_time_s']:.2f} s   throttle RC: {packet['channels'][2]} us\n"
                    f"yaw body CW: {math.degrees(state['gyro_frd_z']):+.1f} deg/s   world CCW: {world_yaw:+.1f} deg/s\n"
                    f"roll: {state['roll_deg']:+.1f}  pitch: {state['pitch_deg']:+.1f} deg   height: {state['z_m']:.1f} m\n"
                    f"horizontal speed: {math.hypot(state['vx_m_s'],state['vy_m_s']):.1f} m/s   climb: {state['vz_m_s']:+.1f} m/s\n"
                    f"servo: {state['servo_pwm_us']:.0f} us   basket: {state['joint_angle_deg']:+.1f} deg\n"
                    +( "GYRO-ONLY AHRS DIAGNOSTIC (Kp=Ki=0)\n" if self.protocol.get('gyro_only_diagnostic') else "Native AHRS correction\n")+
                    "Shared synthetic plant | fixed RC during test | Nvidia NVENC")
                self.next_video=time.monotonic()+.1
        return state

    def recapture(self,first=False):
        self.record['yaw_profile_verification']=verify_profile(self.record,
            self.profile['gains'],self.profile['feedforward'],self.profile['hold_gain'],directory=self.directory)
        self.record['angle_earth_ref']=read_angle_earth_ref((self.directory/'provision.log').read_text(errors='replace'))
        if self.record['angle_earth_ref']!=100:raise IdentityError('ANGLE earth reference changed')
        if bytes.fromhex(self.record['common_msp_readback']['94'])[17]!=80:
            raise IdentityError('Native ANGLE limit is not the declared80 degrees')
        if self.protocol.get('gyro_only_diagnostic'):
            log=(self.directory/'provision.log').read_text(errors='replace')
            values={name:native_integer(log,name) for name in ('imu_dcm_kp','imu_dcm_ki')}
            if any(values.values()):raise IdentityError('Gyro-only diagnostic AHRS readback mismatch')
            self.record['ahrs_configuration']={'gyro_only_diagnostic':True,'native_readback':values,
                'boundary':'No accelerometer correction; synthetic gyro drift is not representative of real flight'}
        return super().recapture(first)

    def measurement(self,case):
        self.receiver.program(case)
        complete=(case['name'],'complete')
        deadline=time.monotonic()+max(30,sum(s['duration_s'] for s in case['stages'])*2.5)
        while complete not in self.receiver.transitions:
            self.health()
            if time.monotonic()>deadline:raise TimeoutError('Stress program wall timeout')
            time.sleep(.01)
        return self.actual_stages(case),self.receiver.transitions[complete]['sim_time_s']

    def actual_stages(self,case):
        stages=[]
        for definition in case['stages']:
            marker=self.receiver.transitions.get((case['name'],definition['name']))
            if marker is None:break
            stages.append({**definition,'actual_start_sim_time_s':marker['sim_time_s'],
                'first_send_wall_monotonic':marker['wall_monotonic'],
                **angle_targets(self.record,*definition['channels'][:2],definition['channels'][3],earth_ref=100)})
        return stages

    def finish(self):
        EnvelopeTrial.finish(self)
        if self.video:self.record['video']=self.video.close()

    def execute(self):
        try:return super().execute()
        finally:
            if self.record['cases'] and self.receiver:
                case=self.record['cases'][0]
                if not case['stages']:
                    case['stages']=self.actual_stages(self.protocol['cases'][0])
                    state=self.pilot.snapshot() if self.pilot else None
                    case['end_sim_time_s']=state['sim_time_s'] if state else None
                    case['partial_program']=True
            if self.video:
                try:self.record['video']=self.video.close()
                except Exception as error:self.record['video_error']=repr(error)
            if self.directory.is_dir():write_json(self.directory/'comparison-run.json',self.record)


class Campaign:
    def __init__(self,args,directory):
        self.args=args;self.directory=directory
        # Reuse the established immutable firmware/common-adapter/plant audit.
        identity_dir=directory/'identity-reference';identity_dir.mkdir()
        self.identity=IdentityCampaign(REFERENCE,identity_dir,1,None,ARTIFACTS/'corrected-frame')
        self.definitions=[c for c in stress_cases(args.forward_lead) if not args.cases or c['name'] in args.cases.split(',')]
        if not self.definitions:raise ValueError('No selected cases')
        self.profiles=args.profiles.split(',')
        if any(p not in PROFILES for p in self.profiles):raise ValueError('Unknown profile')
        known={c['name'] for c in stress_cases()}
        if args.cases and any(n not in known for n in args.cases.split(',')):raise ValueError('Unknown case')
        self.frozen=freeze_sources(directory,inputs(args.video))
        self.world=directory/'extended-ground.sdf'
        world_change=extended_world(directory/'source/gazebo/worlds/tricopter.sdf',self.world)
        self.protocol={'cases':self.definitions,'profiles':{p:PROFILES[p] for p in self.profiles},
            'repetitions':args.repetitions,'source_sha256':self.frozen,
            'binary_identities':self.identity.binary_identities,'shared_sitl_adapter':'corrected-frame',
            'physics_policy':'Unchanged aircraft, propulsion, drag, servo and PB0-PC0 model',
            'world_change':world_change,'maximum_stage_duration_error_s':.03,
            'maximum_measurement_rc_send_gap_s':.05,'angle_limit_deg':80,
            'maximum_horizontal_distance_m':10000,'maximum_height_m':800,'maximum_attitude_deg':85,
            'roll_pitch_profile':'Aircraft roll35/16/15/F47, pitch35/17/15/F49, unchanged across all cases',
            'gyro_only_diagnostic':args.gyro_only,'forward_lead_s':args.forward_lead,
            'steady_entry_policy':f'Fixed common4s ramp +{args.forward_lead}s settling; independently qualify actual final2s before yaw. No external pilot during program',
            'steady_entry_limits':{'window_s':2,'native_collective_min':.7,'mean_motor_duty_min':.7,
                'speed_range_over_mean_max':.01,'absolute_vz_max_m_s':.3,'acceleration_rms_max_m_s2':.1,
                'absolute_euler_roll_pitch_rate_max_dps':1,'absolute_world_yaw_rate_max_dps':2},
            'failure_policy':'Retain every attempt, never interpret guard/timing failure alone as yaw instability',
            'video_policy':'First repetition of each selected case/profile/variant, explicit h264_nvenc; no software fallback',
            'setup_pilot':PILOT.copy()}
        self.protocol['sha256']=fingerprint(self.protocol)
        write_json(directory/'protocol.json',self.protocol)
        self.summary={'pass':False,'directory':str(directory),'protocol':self.protocol,'entries':[]}

    def guard(self):
        self.identity.guard();verify_frozen_sources(self.directory,self.frozen,inputs(self.args.video))
        if sha(self.world)!=self.protocol['world_change']['extended_sha256']:raise IdentityError('World changed')
        if getattr(self,'plugin',None) and sha(self.plugin)!=self.summary['plugin_sha256']:
            raise IdentityError('Retained plugin changed')

    def run(self):
        from stress_metrics import analyze_trial
        common={}
        try:
            with SessionLock():
                plugin=build_plugin(ARTIFACTS,4);self.guard()
                if sha(plugin)!=self.identity.reference_records['stock']['plugin_sha256']:
                    raise IdentityError('Plant plugin differs from hover reference')
                retained=self.directory/'plugin'/plugin.name;retained.parent.mkdir();shutil.copy2(plugin,retained)
                self.plugin=retained
                self.summary['plugin_sha256']=sha(retained)
                for repetition in range(1,self.args.repetitions+1):
                    for profile_name in self.profiles:
                        profile=PROFILES[profile_name]
                        overlay=self.directory/f'{profile_name}.cli'
                        overlay.write_text(overlay_text(profile['gains'],profile['feedforward'],profile['hold_gain'])+
                            'set angle_limit = 80\nget angle_limit\nget angle_earth_ref\n'+
                            ('set imu_dcm_kp = 0\nset imu_dcm_ki = 0\nget imu_dcm_kp\nget imu_dcm_ki\n' if self.args.gyro_only else ''))
                        for definition in self.definitions:
                            for variant in (('stock','modified') if repetition%2 else ('modified','stock')):
                                self.guard()
                                path=self.directory/f'{repetition:02d}-{profile_name}-{definition["name"]}-{variant}'
                                entry={'directory':str(path),'variant':variant,'profile':profile_name,
                                    'case':definition['name'],'repetition':repetition,'completed':False}
                                self.summary['entries'].append(entry)
                                single=copy.deepcopy(self.protocol);single['cases']=[copy.deepcopy(definition)]
                                trial=StressTrial(path,variant,retained,self.world,single,
                                    profile=profile,video=self.args.video and repetition==1,
                                    extra_config=overlay,artifacts=ARTIFACTS/'corrected-frame')
                                trial.record.update(profile_name=profile_name,profile=profile,
                                    selected_case=definition['name'],fresh_case_reset=True)
                                try:
                                    trial.execute();record=analyze(path);verify_case(record,self.protocol)
                                    _,health=program_health(path,record)
                                    entry['program_health']=health
                                    verify_profile(record,profile['gains'],profile['feedforward'],profile['hold_gain'],directory=path)
                                    key=profile_name
                                    matched={k:record[k] for k in ('world_sha256','plugin_sha256','common_config_sha256','common_msp_readback','yaw_settings')}
                                    if key in common and matched!=common[key]:raise IdentityError('Unmatched within-profile configuration')
                                    common[key]=matched
                                    entry['completed']=True
                                except Exception as error:
                                    entry['error']=repr(error)
                                    print(f'FAILED {path.name}: {error!r}',flush=True)
                                    if isinstance(error,IdentityError):raise
                                finally:
                                    if (path/'comparison-run.json').exists():
                                        try:
                                            entry['metrics']=analyze_trial(path)
                                            if definition.get('steady_entry_required') and entry['completed']:
                                                entry['steady_entry_gate']=steady_entry_gate(entry['metrics'])
                                        except Exception as error:entry['metrics_error']=repr(error)
                                    if (path/'video-record.json').exists():
                                        entry['video']=json.loads((path/'video-record.json').read_text())
                                    print(f'RECORDED {path.name}: completed={entry["completed"]}',flush=True)
                                    self.guard();write_json(self.directory/'stress-summary.json',self.summary)
                self.summary['pass']=all(e['completed'] and 'metrics_error' not in e for e in self.summary['entries'])
                steady_names={c['name'] for c in self.definitions if c.get('steady_entry_required')}
                gates=[bool(e['completed'] and e.get('steady_entry_gate',{}).get('qualified',False))
                       for e in self.summary['entries'] if e['case'] in steady_names]
                self.summary['all_requested_steady_entries_qualified']=all(gates) if gates else None
        finally:write_json(self.directory/'stress-summary.json',self.summary)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profiles',default='factory_yaw,hover_tuned')
    parser.add_argument('--cases')
    parser.add_argument('--repetitions',type=int,default=1)
    parser.add_argument('--video',action='store_true')
    parser.add_argument('--gyro-only',action='store_true',help='Explicit common AHRS isolation diagnostic; not normal native sensor fusion')
    parser.add_argument('--forward-lead',type=float,default=25.)
    parser.add_argument('--label',default='yaw-stress')
    parser.add_argument('--dry-run',action='store_true')
    args=parser.parse_args()
    if not 1<=args.repetitions<=3:parser.error('Repetitions must be1..3')
    if args.dry_run:print(json.dumps(stress_cases(args.forward_lead),indent=2));return
    PILOT['max_angle_deg']=12.
    directory=fresh_run_directory(ARTIFACTS,'both',args.label)
    print(f'Stress comparison: {directory}',flush=True)
    Campaign(args,directory).run()


if __name__=='__main__':main()
