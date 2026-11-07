"""Execute the shared C simulation bridge. Python supplies configs/requests only."""
import copy
import ctypes
import json
from pathlib import Path
import subprocess
import numpy as np

from allocator_native import virtual_parameters
from rpm_data import ROOT, sha

PROTOCOL = Path(__file__).with_name('simulation_protocol.json')
CORE_NAMES = ('tricopter_simulation', 'tricopter_servo_model', 'tricopter_rpm_observer', 'tricopter_reaction_torque', 'tricopter_allocator', 'tricopter_dynamic_allocator')
SOURCES = [ROOT/'src/main/flight'/(name+suffix) for name in CORE_NAMES for suffix in ('.c', '.h')]
SOURCES += [Path(__file__).with_name('simulation_bridge.c'), Path(__file__).with_name('simulation_bridge.h')]
COUNTS = (75, 74, 85, 27, 47)

def labels():
    numeric = ['']*85
    for start, prefix, count in [(0,'requested_response',4),(4,'actual_response',4),(8,'actual_actuator_response',4),
        (17,'true_rpm',3),(20,'true_acceleration_rpm_s',3),(23,'true_effort',3),(26,'command_rpm',3),(29,'command_effort',3),
        (39,'predicted_actuator_response',4),(43,'predicted_total_response',4),(47,'coordinated_actuator_response',4),
        (51,'coordinated_total_response',4),(55,'post_coordination_residual',4),(59,'allocation_response',4),(63,'allocation_residual',4),
        (69,'observer_rpm',3),(72,'observer_acceleration_rpm_s',3),(75,'packet_rpm',3),(78,'predicted_controls',4)]:
        numeric[start:start+count] = [f'{prefix}_{i}' for i in range(count)]
    for index, name in {12:'true_reaction_nm',13:'true_reaction_yaw_response',14:'true_servo_corrected_counts',15:'true_mathematical_direction_rad',
        16:'true_servo_derivative_counts_s',32:'command_pwm_us',33:'target_direction_rad',34:'predicted_direction_rad',35:'predicted_servo_counts',
        36:'predicted_servo_derivative_counts_s',37:'estimated_reaction_nm',38:'applied_reaction_response',67:'allocation_cost',
        68:'allocation_gradient_norm',82:'predicted_reaction_yaw_response',83:'true_tail_vertical_effort',84:'true_tail_rightward_effort'}.items(): numeric[index] = name
    integer = ['controller_status','allocator_status','allocator_constraints','allocator_iterations','true_servo_status','reaction_status']
    for prefix in ('observer_status','observer_age_us','observer_sequence','packet_observed_at_us','packet_delivered_at_us','packet_sequence','packet_queue_depth'):
        integer += [f'{prefix}_{i}' for i in range(3)]
    flags = ['controller_valid','allocator_valid','allocator_converged','allocator_exact','servo_prediction_available','reaction_available',
             'coordinated_actuator_available','coordinated_total_available','predicted_actuator_available','predicted_total_available']
    for prefix in ('acceleration_valid','rpm_seen','rpm_accepted','rpm_fresh','rpm_valid','packet_seen','packet_generated','packet_delivered','packet_dropped','packet_outlier'):
        flags += [f'{prefix}_{i}' for i in range(3)]
    flags += ['true_servo_valid','calibrated','for_flight','true_reaction_valid']
    flags += [f'plant_rpm_clipped_{i}' for i in range(3)]
    assert all(numeric) and (len(numeric),len(integer),len(flags)) == COUNTS[2:]
    return {'numeric':numeric,'integer':integer,'flags':flags}


def load_protocol():
    p = json.loads(PROTOCOL.read_text())
    if p['source'] != 'SYNTHETIC_PLACEHOLDER' or p['calibrated'] is not False or p['for_flight'] is not False:
        raise ValueError('Only uncalibrated synthetic simulation inputs are permitted')
    if p['effectiveness_excludes_reaction'] is not True or any(a['effectiveness_excludes_reaction'] is not True for a in p['airframes']):
        raise ValueError('Both effectiveness maps must explicitly exclude the separately added full reaction disturbance')
    if p['direction_map']['physical_angle_calibrated'] is not False:
        raise ValueError('No physical angle calibration exists in this simulation')
    if p['acceleration_qualification']['minimum_distinct_fresh_timestamps'] != 2:
        raise ValueError('Acceleration gate must match the actual C coordinator policy')
    return p


def servo_values(p):
    pwm, counts = np.asarray(p['pwm_us']), np.asarray(p['corrected_counts'])
    if pwm.shape != (3,) or counts.shape != (3,): raise ValueError('Simulation transport requires explicit three-knot servo maps')
    return np.r_[pwm,counts,p['backlash_us'],p['delay_us'],p['tau_increasing_pwm_s'],p['tau_decreasing_pwm_s'],
                 p['rate_increasing_pwm_counts_s'],p['rate_decreasing_pwm_counts_s'],p['max_gap_us']]


def reaction_values(p):
    parts = [np.asarray(p[key]) for key in ('k_q_nm_per_rad_s_squared','rotor_inertia_kg_m2','reaction_sign')]
    if any(part.shape != (3,) for part in parts): raise ValueError('Expected three reaction coefficients/inertias/signs')
    return np.r_[*parts,p['max_mechanical_rpm'],p['max_abs_acceleration_rpm_s'],p['max_abs_torque_nm'],p['max_rpm_age_us']]


def pack_run(p, airframe, scenario, experiment):
    if p['effectiveness_excludes_reaction'] is not True or airframe['effectiveness_excludes_reaction'] is not True:
        raise ValueError('Effectiveness plus full reaction would double count unless explicitly excluded')
    limits, solver, mapping = p['allocator_limits'],p['allocator_limits'],p['direction_map']
    allocator = {'effectiveness_matrix':airframe['effectiveness_matrix'],'maximum_effort':limits['maximum_effort'],
                 'tail_direction_min_rad':limits['tail_direction_min_rad'],'tail_direction_max_rad':limits['tail_direction_max_rad'],
                 'response_scales':limits['response_scales'],'solver':solver}
    obs = p['rpm_observer']; motor=p['motor_plant']; telemetry=p['telemetry']; guard=p['coordinator_guards']
    omega_scale = np.asarray(p['rpm_per_sqrt_effort'],float)*np.pi/30
    if omega_scale.shape != (3,) or np.any(omega_scale <= 0): raise ValueError('Explicit positive RPM/effort coefficients required')
    effort_coefficient = 1/omega_scale**2
    counts = mapping['zero_corrected_counts']+mapping['counts_per_direction']*np.asarray([limits['tail_direction_min_rad'],limits['tail_direction_max_rad']])
    controller = np.r_[virtual_parameters(allocator),servo_values(p['servo_predictor']),
        [obs[key] for key in ('tracking_tau_s','stale_acceleration_decay_tau_s','max_rpm','max_acceleration_rpm_s','stale_after_us','max_prediction_us','max_call_gap_us')],
        reaction_values(p['reaction']),effort_coefficient,counts,p['reaction_nm_to_yaw_response'],guard['max_abs_reaction_compensation_response'],
        guard['min_tail_support_cosine'],1,mapping['initial_corrected_counts'],mapping['initial_pwm_us'],0,1].astype(np.float64)
    plant_servo = copy.deepcopy(p['servo_predictor'])
    for key in ('tau_increasing_pwm_s','tau_decreasing_pwm_s'): plant_servo[key] *= scenario.get('plant_servo_tau_factor',1)
    plant_servo['delay_us'] = scenario.get('plant_servo_delay_us',plant_servo['delay_us'])
    plant_reaction=copy.deepcopy(p['reaction'])
    plant_reaction['k_q_nm_per_rad_s_squared'] = (np.asarray(plant_reaction['k_q_nm_per_rad_s_squared'])*scenario.get('plant_reaction_kq_factor',1)).tolist()
    plant_reaction['rotor_inertia_kg_m2'] = (np.asarray(plant_reaction['rotor_inertia_kg_m2'])*scenario.get('plant_reaction_inertia_factor',1)).tolist()
    plant_matrix=np.asarray(airframe['effectiveness_matrix'])*np.asarray(scenario.get('plant_matrix_row_factors',[1]*4))[:,None]
    plant = np.r_[plant_matrix.ravel(),servo_values(plant_servo),
        np.asarray(motor['tau_increasing_s'])*scenario.get('plant_motor_tau_factor',1),
        np.asarray(motor['tau_decreasing_s'])*scenario.get('plant_motor_tau_factor',1),motor['slew_increasing_rpm_s'],motor['slew_decreasing_rpm_s'],
        effort_coefficient,reaction_values(plant_reaction),counts,p['reaction_nm_to_yaw_response'],motor['maximum_rpm'],
        telemetry['period_us'],telemetry['delay_us'],telemetry['phase_us'],telemetry['outlier_rpm'],1,
        mapping['initial_corrected_counts'],mapping['initial_pwm_us'],0,p['initial_motor_rpm']].astype(np.float64)
    mode = int(experiment['reaction_compensation'])+2*int(experiment['tail_vertical_coordination'])
    if controller.shape != (75,) or plant.shape != (74,): raise ValueError('Simulation packed parameter layout mismatch')
    return controller,plant,mode


def tick_inputs(p, scenario, mode):
    dt,duration=p['dt_us'],p['duration_us']
    if not isinstance(dt,int) or dt <= 0 or duration%dt: raise ValueError('Require explicit integral uniform simulation tick')
    time=np.arange(0,duration+1,dt,dtype=np.uint64); request=np.zeros((len(time),4),np.float32)
    covered=np.zeros(len(time),bool)
    for segment in p['request_segments']:
        mask=(time>=segment['start_us'])&(time<segment['end_us'])
        if segment['end_us']==duration: mask|=time==duration
        fraction=(time[mask].astype(float)-segment['start_us'])/(segment['end_us']-segment['start_us'])
        request[mask]=np.asarray(segment['start'])+fraction[:,None]*(np.asarray(segment['end'])-segment['start']);covered[mask]=True
    if not covered.all(): raise ValueError('Request protocol leaves uncovered simulation ticks')
    masks=[]
    for key in ('dropouts','outliers'):
        mask=np.zeros((len(time),3),np.uint8)
        for event in scenario[key]:
            active=(time>=event['start_us'])&(time<event['end_us'])
            for motor in event['motors']:mask[active,motor]=1
        masks.append(mask)
    return {'time_us':time,'requested_response':request,'modes':np.full(len(time),mode,np.uint8),
            'drop_generation':masks[0],'outlier_generation':masks[1]}


class NativeSimulation:
    def __init__(self,build_dir,compiler='cc'):
        folder=Path(build_dir).resolve();folder.mkdir(parents=True,exist_ok=True);library=folder/'libtricopter_simulation.so'
        command=[compiler,'-std=c11','-O2','-fPIC','-shared','-Wall','-Wextra','-Werror','-I'+str(ROOT/'src/main'),
                 *[str(path) for path in SOURCES if path.suffix=='.c'],'-lm','-o',str(library)]
        before={str(path.relative_to(ROOT)):sha(path) for path in SOURCES}
        subprocess.run(command,check=True,capture_output=True,text=True)
        if before!={str(path.relative_to(ROOT)):sha(path) for path in SOURCES}:raise RuntimeError('Simulation source changed during compilation')
        self.provenance={'sources':before,'command':command,'binary_sha256':sha(library),
            'compiler':subprocess.run([compiler,'--version'],check=True,capture_output=True,text=True).stdout.splitlines()[0]}
        (folder/'native-build.json').write_text(json.dumps(self.provenance,indent=2)+'\n')
        self.library=ctypes.CDLL(str(library));self.library.tricopterSimulationReplay.argtypes=[ctypes.c_void_p,ctypes.c_void_p,ctypes.c_size_t]+[ctypes.c_void_p]*8
        self.library.tricopterSimulationReplay.restype=ctypes.c_int

    def run(self,controller_parameters,plant_parameters,**inputs):
        controller,plant=[np.ascontiguousarray(value,np.float64) for value in (controller_parameters,plant_parameters)]
        if controller.shape!=(75,) or plant.shape!=(74,) or any(not np.isfinite(a).all() or np.any(abs(a)>np.finfo(np.float32).max) for a in (controller,plant)):
            raise ValueError('Expected finite packed simulation parameter vectors')
        time=np.asarray(inputs['time_us']);request=np.asarray(inputs['requested_response']);modes=np.asarray(inputs['modes'])
        drop,outlier=np.asarray(inputs['drop_generation']),np.asarray(inputs['outlier_generation'])
        if time.ndim!=1 or not len(time) or request.shape!=(len(time),4) or modes.shape!=(len(time),) or drop.shape!=(len(time),3) or outlier.shape!=drop.shape:
            raise ValueError('Simulation tick inputs have inconsistent shapes')
        if (not np.isfinite(time).all() or np.any(time<0) or np.any(time>2**53-1) or np.any(time!=np.floor(time))
            or np.any(np.diff(time.astype(float))<=0)):raise ValueError('Expected increasing exact uint64 microseconds within MATLAB double integer range')
        if not np.isfinite(request).all() or np.any(abs(request)>1e8):raise ValueError('Finite bounded response requests required')
        if not np.isin(modes,[0,1,2,3]).all() or not np.isin(drop,[0,1]).all() or not np.isin(outlier,[0,1]).all():raise ValueError('Invalid mode/fault mask')
        dt=np.diff(time.astype(np.uint64))
        if len(dt) and (np.any(dt!=dt[0]) or plant[61]%int(dt[0]) or plant[62]%int(dt[0]) or np.any(plant[63:66]%int(dt[0]))):
            raise ValueError('Packet period/delay/phases must align with the declared uniform simulation grid')
        arrays=[np.ascontiguousarray(time,np.uint64),np.ascontiguousarray(request,np.float32),np.ascontiguousarray(modes,np.uint8),
                np.ascontiguousarray(drop,np.uint8),np.ascontiguousarray(outlier,np.uint8)]
        numeric=np.empty((len(time),85),np.float64);integer=np.empty((len(time),27),np.uint32);flags=np.empty((len(time),47),np.uint8)
        ptr=lambda a:ctypes.c_void_p(a.ctypes.data)
        rc=self.library.tricopterSimulationReplay(ptr(controller),ptr(plant),len(time),*[ptr(a) for a in arrays],ptr(numeric),ptr(integer),ptr(flags))
        if rc!=1 or not np.isfinite(numeric).all():raise ValueError(f'Native simulation failed config/time/model/queue/flight guard ({rc})')
        if flags[:,41:43].any():raise ValueError('Simulation incorrectly claims calibration or flight readiness')
        return {'numeric':numeric,'integer':integer,'flags':flags}
