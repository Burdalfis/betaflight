#!/usr/bin/env python3
"""Controlled comparisons with a fresh world/EEPROM for every individual case."""
from __future__ import annotations
import argparse
import copy
import hashlib
import json
from pathlib import Path
import shutil

from check_comparison import Trial, analyze, cases, sha
from check_flight import PILOT
from run import build_plugin
from runtime import ARTIFACTS, HERE, ROOT, SessionLock, fresh_run_directory


COMMON_KEYS=('world_sha256','plugin_sha256','common_config_sha256',
             'common_msp_readback','yaw_settings','mass_kg','total_maximum_thrust_n')
VARIANT_KEYS=('binary_sha256','firmware_config_sha256')
EXECUTION_HELPERS=('check_comparison_fresh.py','check_comparison.py','compare_metrics.py',
                   'check_flight.py','runtime.py','run.py',
                   'configs/common.cli','configs/modified.cli')


def source_inputs(here=HERE, project_root=ROOT):
    """Keep the world, its relative assets, and all plugin/model inputs together."""
    inputs={name:here/name for name in EXECUTION_HELPERS}
    inputs.update({str(path.relative_to(here)):path for path in sorted((here/'gazebo').rglob('*'))
                   if path.is_file() and '__pycache__' not in path.parts})
    # Generated geometry is sufficient to run. Preserve the optional original
    # CAD input as provenance when it is present, without making it a prerequisite.
    cad=project_root/'Tricopter_motors.step'
    if cad.is_file():inputs['cad/Tricopter_motors.step']=cad
    return inputs


def freeze_sources(directory, inputs):
    frozen={name:sha(path) for name,path in inputs.items()}
    for name,path in inputs.items():
        destination=directory/'source'/name
        destination.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(path,destination)
    verify_frozen_sources(directory,frozen,inputs)
    return frozen


def verify_frozen_sources(directory, frozen, inputs):
    if set(inputs)!=set(frozen):
        raise AssertionError('Execution source/config input set changed after protocol freeze')
    for name,expected in frozen.items():
        for path in (inputs[name],directory/'source'/name):
            if not path.is_file() or sha(path)!=expected:
                raise AssertionError(f'Execution source/config changed after protocol freeze: {name}')


def hover_throttle(value):
    value=int(value)
    # Keep both +/-25 us pulse commands inside the ordinary actuator range.
    if not 1025<=value<=1975:
        raise argparse.ArgumentTypeError('hover throttle must be 1025..1975 us')
    return value


def verify_case(record, protocol):
    if not record['pass'] or record['final_status']['armed'] or not record['all_owned_processes_stopped']:
        raise AssertionError('Fresh case did not finish healthy, disarmed and cleaned up')
    if len(record['cases'])!=1:raise AssertionError('Fresh trial executed more than one case')
    case=record['cases'][0]
    for index,stage in enumerate(case['stages']):
        end=(case['stages'][index+1]['actual_start_sim_time_s'] if index+1<len(case['stages'])
             else case['end_sim_time_s'])
        if abs(end-stage['actual_start_sim_time_s']-stage['duration_s'])>protocol['maximum_stage_duration_error_s']+1e-6:
            raise AssertionError('Fresh case exceeds 30 ms stage-duration tolerance')
    if case['actuator_valid_fraction']!=1 or not case['all_status_samples_armed_without_fault']:
        raise AssertionError('Unhealthy fresh-case measurement window')
    if case['maximum_measurement_rc_send_gap_s']>=protocol['maximum_measurement_rc_send_gap_s']:
        raise AssertionError('Fresh-case RC gap exceeds execution tolerance')
    if record['variant']=='modified' and case['sampled_feedback_valid_fraction']!=1:
        raise AssertionError('Invalid measured feedback during fresh case')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repetitions',type=int,default=3)
    parser.add_argument('--cases',help='Comma-separated case names; default is all seven')
    parser.add_argument('--hover-throttle',type=hover_throttle,default=1350,
                        help='Measured-window collective PWM in us (default: 1350); pulses remain +/-25 us')
    parser.add_argument('--label',default='fresh-case-comparison')
    args=parser.parse_args()
    if not 1<=args.repetitions<=5:parser.error('repetitions must be 1..5')
    definitions=cases(args.hover_throttle);known={case['name']:case for case in definitions}
    selected=list(known) if not args.cases else [name.strip() for name in args.cases.split(',')]
    if not selected or len(set(selected))!=len(selected) or any(name not in known for name in selected):
        parser.error('--cases must contain distinct names from: '+', '.join(known))
    PILOT['max_angle_deg']=12.
    directory=fresh_run_directory(ARTIFACTS,'both',args.label)
    inputs=source_inputs()
    frozen=freeze_sources(directory,inputs)
    protocol={'cases':definitions,'selected_cases':selected,'repetitions':args.repetitions,
        'fresh_case_reset':True,'case_timestamps_are_case_local':True,
        'hover_throttle_us':args.hover_throttle,
        'measurement_rc_rate_hz':50,'plant_trace_rate_hz':100,'setup_pilot':PILOT.copy(),
        'initial_gate':{'altitude_error_m':.1,'vertical_speed_m_s':.12,'horizontal_speed_m_s':.07,
            'horizontal_position_error_m':.35,'absolute_roll_pitch_deg':3,'roll_pitch_rate_dps':1,
            'yaw_rate_dps':2,'heading_error_deg':1,'continuous_duration_s':.5},
        'measurement_minimum_body_height_m':.35,
        'matched_no_collective_controls':{'collective_up':'sham','collective_down':'sham',
            'yaw_cw_collective_up':'yaw_cw','yaw_ccw_collective_up':'yaw_ccw'},
        'execution_timing':'RC sender owns absolute simulation-time plan; independent 100 Hz plant trace clock',
        'maximum_stage_duration_error_s':.03,'maximum_measurement_rc_send_gap_s':.05,
        'setup_uses_external_ground_truth_rc_pilot':True,
        'measurement_first_four_channels_override_all_external_pilot_outputs':True,
        'same_measurement_values_and_scheduled_durations_actual_send_times_retained':True,
        'variant_order':[variant for index in range(args.repetitions) for variant in
                         (['stock','modified'] if index%2==0 else ['modified','stock'])],
        'source_sha256':frozen,
        'selection_hash_contract':'Each trial retains this global protocol hash and records selected_case separately.'}
    protocol['sha256']=hashlib.sha256(json.dumps(protocol,sort_keys=True).encode()).hexdigest()
    (directory/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
    summary={'pass':False,'directory':str(directory),'protocol':protocol,'fresh_case_reset':True,'runs':[]}
    common_reference=None;variant_references={}
    print(f'Fresh-case comparison: {directory}',flush=True)
    try:
        with SessionLock():
            built_plugin=build_plugin(ARTIFACTS,4)
            verify_frozen_sources(directory,frozen,source_inputs())
            plugin=directory/'plugin'/built_plugin.name;plugin.parent.mkdir()
            shutil.copy2(built_plugin,plugin)
            plugin_sha=sha(plugin)
            manifest=built_plugin.parent/'build-manifest.json'
            shutil.copy2(manifest,directory/'plugin/build-manifest.json')
            summary['plugin_build_manifest']=json.loads(manifest.read_text())
            if summary['plugin_build_manifest']['plugin_sha256']!=plugin_sha:
                raise AssertionError('Built plugin differs from its source manifest')
            summary['plugin_sha256']=plugin_sha
            for index,variant in enumerate(protocol['variant_order']):
                group_name=f'{index+1:02d}-{variant}';group_path=directory/group_name;group_path.mkdir()
                group={'variant':variant,'pass':False,'fresh_case_reset':True,
                    'group_directory':group_name,'case_directory_base':'group_directory',
                    'case_timestamps_are_case_local':True,'case_directories':{},'case_final_states':{},
                    'protocol_sha256':protocol['sha256'],'cases':[]}
                summary['runs'].append(group)
                for name in selected:
                    verify_frozen_sources(directory,frozen,source_inputs())
                    if sha(plugin)!=plugin_sha:raise AssertionError('Frozen plugin binary changed')
                    single=copy.deepcopy(protocol);single['cases']=[copy.deepcopy(known[name])]
                    trial=Trial(group_path/name,variant,plugin,
                        directory/'source/gazebo/worlds/tricopter.sdf',single)
                    trial.record.update(selected_case=name,fresh_case_reset=True)
                    trial.execute();record=analyze(group_path/name);verify_case(record,protocol)
                    verify_frozen_sources(directory,frozen,source_inputs())
                    if sha(plugin)!=plugin_sha:raise AssertionError('Frozen plugin binary changed')
                    common={key:record[key] for key in COMMON_KEYS}
                    if common_reference is None:common_reference=common
                    if common!=common_reference:raise AssertionError('Unmatched fresh-case world/config/readback')
                    identity={key:record[key] for key in VARIANT_KEYS}
                    if variant not in variant_references:variant_references[variant]=identity
                    if identity!=variant_references[variant]:raise AssertionError('Firmware/config identity changed within variant')
                    group.update(common);group.update(identity)
                    group['case_directories'][name]=name
                    group['case_final_states'][name]={'final_status':record['final_status'],
                        'all_owned_processes_stopped':record['all_owned_processes_stopped'],
                        'maximum_rc_send_gap_s':record['maximum_rc_send_gap_s']}
                    group['cases'].append(record['cases'][0])
                    (directory/'comparison-summary.json').write_text(json.dumps(summary,indent=2)+'\n')
                group.update(pass_=True)
                del group['pass_']
                group['pass']=True
                group['final_status']={'armed':False,'all_fresh_cases_disarmed':True,'aggregate_of_fresh_cases':True}
                group['all_owned_processes_stopped']=True
                group['maximum_rc_send_gap_s']=max(value['maximum_rc_send_gap_s'] for value in group['case_final_states'].values())
                (directory/'comparison-summary.json').write_text(json.dumps(summary,indent=2)+'\n')
            summary['pass']=True
    except Exception as error:
        summary['error']=repr(error)
        raise
    finally:
        (directory/'comparison-summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print('All fresh-case comparisons complete; processes stopped.',flush=True)


if __name__=='__main__':main()
