"""Actual-C opt-in dynamic simulation transport; legacy simulation is unchanged."""
import copy
import ctypes
import json
from pathlib import Path
import subprocess
import numpy as np

import simulation_native as legacy
from simulation_native import ROOT,sha

PROTOCOL=Path(__file__).with_name('dynamic_protocol.json')
SOURCES=list(legacy.SOURCES)+[Path(__file__).with_name('dynamic_bridge.c'),Path(__file__).with_name('dynamic_bridge.h')]


def load_protocol():
    p=json.loads(PROTOCOL.read_text())
    if p['source']!='SYNTHETIC_PLACEHOLDER' or p['calibrated'] is not False or p['for_flight'] is not False:
        raise ValueError('Dynamic replay accepts only uncalibrated synthetic configurations')
    if sha(ROOT/p['base_protocol'])!=p['base_protocol_sha256']:raise ValueError('Pinned legacy simulation protocol changed')
    if p['high_group_axes']!=[0,1,2] or p['yaw_axis']!=3:raise ValueError('Dynamic high/yaw policy axis contract changed')
    return p


def dynamic_parameters(p):
    d=p['dynamic'];grid=np.asarray(d['pwm_grid_us'],float)
    parts=[np.asarray(d[key],float) for key in ('motor_tau_increasing_s','motor_tau_decreasing_s','motor_slew_increasing_rpm_s','motor_slew_decreasing_rpm_s')]
    if grid.ndim!=1 or len(grid)>17 or any(part.shape!=(3,) for part in parts):raise ValueError('Explicit motor triplets and at most17 PWM samples required')
    values=np.r_[d['horizon_us'],d['max_rpm_age_us'],*parts,np.pad(grid,(0,17-len(grid))),len(grid),d['max_high_matrix_condition'],
                 d['high_cost_tolerance'],d['yaw_cost_tolerance'],d['movement_cost_tolerance']].astype(np.float64)
    if values.shape!=(36,) or not np.isfinite(values).all() or np.any(abs(values)>np.finfo(np.float32).max):raise ValueError('Malformed finite dynamic parameter vector')
    for index in (0,1):
        if values[index]<0 or values[index]>np.iinfo(np.uint32).max or values[index]!=np.floor(values[index]):raise ValueError('Dynamic times require exact uint32 transport')
    return values


def run_configuration(p,airframe,scenario,experiment):
    base=legacy.load_protocol();base=copy.deepcopy(base)
    for key in ('dt_us','duration_us','request_segments'):base[key]=copy.deepcopy(p[key])
    if scenario.get('zero_reaction'):
        base['reaction']['k_q_nm_per_rad_s_squared']=[0]*3;base['reaction']['rotor_inertia_kg_m2']=[0]*3
    for target,key in [('period_us','telemetry_period_us'),('delay_us','telemetry_delay_us'),('phase_us','telemetry_phase_us')]:
        if key in scenario:base['telemetry'][target]=scenario[key]
    if 'observer_tracking_tau_s' in scenario:base['rpm_observer']['tracking_tau_s']=scenario['observer_tracking_tau_s']
    mode=experiment['mode']
    old_experiment={'reaction_compensation':bool(mode&1),'tail_vertical_coordination':bool(mode&2)}
    controller,plant,_=legacy.pack_run(base,airframe,scenario,old_experiment)
    inputs=legacy.tick_inputs(base,scenario,mode)
    return base,controller,plant,inputs


def angle_knots(profile):
    if (not isinstance(profile,dict) or profile.get('schema_version')!=1
        or profile.get('kind')!='tricopter_basket_measured_angle_mean_map'
        or profile.get('source_position')!='signed PB0-PC0 only'
        or profile.get('angle_units')!='radians' or profile.get('operator_defined_reference') is not True):
        raise ValueError('Explicit operator-referenced PB0-PC0 angle profile required')
    try:
        values=np.asarray([[p[key] for key in ('corrected_counts','angle_rad','half_branch_separation_rad')]
                           for p in profile['knots']],dtype=np.float64)
    except (KeyError,TypeError,ValueError) as error:
        raise ValueError('Malformed measured angle knots') from error
    if (values.ndim!=2 or values.shape[1]!=3 or not 2<=len(values)<=64
        or not np.isfinite(values).all() or np.any(abs(values)>np.finfo(np.float32).max)):
        raise ValueError('Require 2..64 finite measured angle triplets')
    return np.ascontiguousarray(values)


class NativeDynamic:
    def __init__(self,build_dir,compiler='cc'):
        folder=Path(build_dir).resolve();folder.mkdir(parents=True,exist_ok=True);library=folder/'libtricopter_dynamic.so'
        command=[compiler,'-std=c11','-O2','-fPIC','-shared','-Wall','-Wextra','-Werror','-I'+str(ROOT/'src/main'),
                 *[str(path) for path in SOURCES if path.suffix=='.c'],'-lm','-o',str(library)]
        before={str(path.relative_to(ROOT)):sha(path) for path in SOURCES};subprocess.run(command,check=True,capture_output=True,text=True)
        if before!={str(path.relative_to(ROOT)):sha(path) for path in SOURCES}:raise RuntimeError('Dynamic source changed during compilation')
        self.provenance={'sources':before,'command':command,'binary_sha256':sha(library),
            'compiler':subprocess.run([compiler,'--version'],check=True,capture_output=True,text=True).stdout.splitlines()[0]}
        (folder/'native-build.json').write_text(json.dumps(self.provenance,indent=2)+'\n')
        self.library=ctypes.CDLL(str(library));self.library.tricopterDynamicSimulationReplay.argtypes=[ctypes.c_void_p]*3+[ctypes.c_size_t]+[ctypes.c_void_p]*12
        self.library.tricopterDynamicSimulationReplay.restype=ctypes.c_int
        self.library.tricopterSimulationReplay.argtypes=[ctypes.c_void_p,ctypes.c_void_p,ctypes.c_size_t]+[ctypes.c_void_p]*8
        self.library.tricopterSimulationReplay.restype=ctypes.c_int
        self.library.tricopterMeasuredDynamicSimulationReplay.argtypes=[ctypes.c_void_p]*4+[ctypes.c_size_t]*2+[ctypes.c_void_p]*14
        self.library.tricopterMeasuredDynamicSimulationReplay.restype=ctypes.c_int
        self.library.tricopterMeasuredAngleEvaluate.argtypes=[ctypes.c_void_p,ctypes.c_size_t,ctypes.c_void_p,ctypes.c_size_t]+[ctypes.c_void_p]*3
        self.library.tricopterMeasuredAngleEvaluate.restype=ctypes.c_int

    def run(self,controller,plant,dynamic,angle_profile=None,**inputs):
        controller,plant,dynamic=[np.ascontiguousarray(value,np.float64) for value in (controller,plant,dynamic)]
        if (controller.shape,plant.shape,dynamic.shape)!=((75,),(74,),(36,)) or any(not np.isfinite(v).all() or np.any(abs(v)>np.finfo(np.float32).max) for v in (controller,plant,dynamic)):
            raise ValueError('Malformed finite dynamic/base packed configuration')
        time=np.asarray(inputs['time_us']);request=np.asarray(inputs['requested_response']);mode=np.asarray(inputs['modes'])
        drop,outlier=np.asarray(inputs['drop_generation']),np.asarray(inputs['outlier_generation'])
        n=len(time) if time.ndim==1 else 0
        if not n or request.shape!=(n,4) or mode.shape!=(n,) or drop.shape!=(n,3) or outlier.shape!=(n,3):raise ValueError('Dynamic input shape mismatch')
        if not np.isfinite(time).all() or np.any(time<0) or np.any(time>2**53-1) or np.any(time!=np.floor(time)) or np.any(np.diff(time.astype(float))<=0):raise ValueError('Increasing exact uint64 times required')
        if not np.isfinite(request).all() or np.any(abs(request)>1e8) or not np.isin(mode,[0,1,2,3,4,5]).all() or not np.isin(drop,[0,1]).all() or not np.isin(outlier,[0,1]).all():raise ValueError('Invalid request/mode/fault transport')
        arrays=[np.ascontiguousarray(time,np.uint64),np.ascontiguousarray(request,np.float32),np.ascontiguousarray(mode,np.uint8),np.ascontiguousarray(drop,np.uint8),np.ascontiguousarray(outlier,np.uint8)]
        result={'numeric':np.empty((n,85),float),'integer':np.empty((n,27),np.uint32),'flags':np.empty((n,47),np.uint8),
                'dynamic_numeric':np.empty((n,43),float),'dynamic_integer':np.empty((n,9),np.uint32),
                'candidate_numeric':np.empty((n,19,31),float),'candidate_integer':np.empty((n,19,3),np.uint32)}
        ptr=lambda a:ctypes.c_void_p(a.ctypes.data)
        outputs=list(result.values())
        if angle_profile is None:
            rc=self.library.tricopterDynamicSimulationReplay(ptr(controller),ptr(plant),ptr(dynamic),n,*[ptr(a) for a in arrays],*[ptr(a) for a in outputs])
        else:
            knots=angle_knots(angle_profile)
            result['angle_numeric']=np.zeros((n,2),float)
            result['angle_flags']=np.zeros((n,2),np.uint8)
            rc=self.library.tricopterMeasuredDynamicSimulationReplay(ptr(controller),ptr(plant),ptr(dynamic),ptr(knots),len(knots),n,
                *[ptr(a) for a in arrays],*[ptr(a) for a in outputs],ptr(result['angle_numeric']),ptr(result['angle_flags']))
        if rc!=1 or any(not np.isfinite(result[key]).all() for key in ('numeric','dynamic_numeric','candidate_numeric')):raise ValueError(f'Native dynamic replay failed config/time/model guard ({rc})')
        if result['flags'][:,41:43].any() or result['dynamic_integer'][:,6:8].any():raise ValueError('Dynamic replay falsely claims calibration/flight readiness')
        return result

    def angles(self,profile,counts):
        """Execute the same portable C map on canonical measured/predicted counts.

        Unavailable samples have valid=False and zero numeric outputs. The
        half-separation describes major branches, not a guaranteed error bound.
        """
        knots=angle_knots(profile)
        values=np.ascontiguousarray(counts,np.float32)
        if values.ndim!=1 or not len(values):raise ValueError('Expected a nonempty count vector')
        angles=np.empty_like(values);half=np.empty_like(values);valid=np.empty(len(values),np.uint8)
        ptr=lambda a:ctypes.c_void_p(a.ctypes.data)
        rc=self.library.tricopterMeasuredAngleEvaluate(ptr(knots),len(knots),ptr(values),len(values),ptr(angles),ptr(half),ptr(valid))
        if rc!=1:raise ValueError('Native measured angle profile rejected')
        return {'angle_rad':angles,'half_branch_separation_rad':half,'valid':valid.astype(bool)}

    def original(self,controller,plant,**inputs):
        """Original ABI entrypoint, solely for frozen trace regression."""
        return legacy.NativeSimulation.run(self,controller,plant,**inputs)
