"""Native synthetic reaction-torque evaluation. No Python runtime torque model."""

import ctypes
import json
from pathlib import Path
import subprocess

import numpy as np

from rpm_data import ROOT, sha

SOURCES = [ROOT/'src/main/flight/tricopter_reaction_torque.c',
           ROOT/'src/main/flight/tricopter_reaction_torque.h',
           Path(__file__).with_name('reaction_model_bridge.c')]


class NativeReaction:
    def __init__(self, build_dir, compiler='cc'):
        folder = Path(build_dir).resolve()
        folder.mkdir(parents=True, exist_ok=True)
        library = folder/'libtricopter_reaction.so'
        command = [compiler, '-std=c11', '-O2', '-fPIC', '-shared', '-Wall', '-Wextra', '-Werror',
                   '-I'+str(ROOT/'src/main'), str(SOURCES[0]), str(SOURCES[2]), '-lm', '-o', str(library)]
        before = {str(path.relative_to(ROOT)): sha(path) for path in SOURCES}
        subprocess.run(command, check=True, capture_output=True, text=True)
        if before != {str(path.relative_to(ROOT)): sha(path) for path in SOURCES}:
            raise RuntimeError('Reaction core/header/bridge changed during compilation')
        self.provenance = {'sources': before, 'command': command, 'binary_sha256': sha(library),
                           'compiler': subprocess.run([compiler, '--version'], check=True, capture_output=True, text=True).stdout.splitlines()[0]}
        (folder/'native-build.json').write_text(json.dumps(self.provenance, indent=2)+'\n')
        self.library = ctypes.CDLL(str(library))
        self.library.tricopterReactionReplay.argtypes = [ctypes.c_void_p, ctypes.c_uint32, ctypes.c_size_t]+[ctypes.c_void_p]*18
        self.library.tricopterReactionReplay.restype = ctypes.c_int

    def evaluate(self, parameters, mechanical_rpm, acceleration_rpm_s, age_us, seen, fresh, valid,
                 acceleration_valid, coefficient_source='SYNTHETIC_PLACEHOLDER'):
        parts = [np.asarray(parameters[key], np.float64) for key in
                 ('k_q_nm_per_rad_s_squared', 'rotor_inertia_kg_m2', 'reaction_sign')]
        if any(part.shape != (3,) for part in parts):
            raise ValueError('Expected exactly three coefficients/inertias/reaction signs')
        guards = np.asarray([parameters[key] for key in
                             ('max_mechanical_rpm', 'max_abs_acceleration_rpm_s', 'max_abs_torque_nm', 'max_rpm_age_us')], np.float64)
        if guards.shape != (4,):
            raise ValueError('Expected four scalar software guards')
        params = np.concatenate(parts+[guards])
        if (not np.all(np.isfinite(params)) or np.any(abs(params) > np.finfo(np.float32).max)
                or np.any(params[6:9] != np.floor(params[6:9])) or np.any(abs(params[6:9]) > 127)
                or params[12] < 0 or params[12] > np.iinfo(np.uint32).max or params[12] != np.floor(params[12])):
            raise ValueError('Reaction parameters exceed finite/native conversion bounds')
        source = {'UNSPECIFIED': 0, 'SYNTHETIC_PLACEHOLDER': 1}.get(coefficient_source)
        if source is None:
            raise ValueError('Unsupported coefficient provenance; no calibrated source is defined')
        rpm, acceleration, age = [np.asarray(value) for value in (mechanical_rpm, acceleration_rpm_s, age_us)]
        if rpm.ndim != 2 or rpm.shape[1:] != (3,) or not len(rpm) or acceleration.shape != rpm.shape or age.shape != rpm.shape:
            raise ValueError('Expected nonempty matching N x 3 mechanical RPM, acceleration and age')
        for value in (rpm, acceleration):
            if np.any(np.isfinite(value) & (abs(value) > np.finfo(np.float32).max)):
                raise ValueError('Finite motor input exceeds float32 transport')
        if (not np.all(np.isfinite(age)) or np.any(age < 0) or np.any(age > np.iinfo(np.uint32).max)
                or np.any(age != np.floor(age))):
            raise ValueError('Age transport must be exact uint32')
        flags = [np.asarray(value) for value in (seen, fresh, valid, acceleration_valid)]
        if any(value.shape != rpm.shape or not np.all(np.isin(value, [0, 1])) for value in flags):
            raise ValueError('Metadata must be four matching boolean N x 3 matrices')
        inputs = [np.ascontiguousarray(rpm, np.float32), np.ascontiguousarray(acceleration, np.float32),
                  np.ascontiguousarray(age, np.uint32)]+[np.ascontiguousarray(value, np.uint8) for value in flags]
        n = len(rpm)
        output = {name: np.empty((n, 3), np.float32) for name in ('omega_rad_s', 'alpha_rad_s2', 'aerodynamic_nm', 'inertial_nm', 'total_nm', 'summed_nm')}
        for name, dtype in [('status', np.uint32), ('invalid_motor_mask', np.uint8), ('valid', np.uint8),
                            ('calibrated', np.uint8), ('for_flight', np.uint8)]:
            output[name] = np.empty(n, dtype)
        ptr = lambda value: ctypes.c_void_p(value.ctypes.data)
        rc = self.library.tricopterReactionReplay(ptr(params), source, n, *[ptr(value) for value in inputs],
                                                *[ptr(value) for value in output.values()])
        if rc != 1:
            raise ValueError('Native reaction bridge rejected transport')
        for name in ('omega_rad_s', 'alpha_rad_s2', 'aerodynamic_nm', 'inertial_nm', 'total_nm', 'summed_nm'):
            if not np.all(np.isfinite(output[name])):
                raise ValueError('Native reaction core emitted nonfinite output')
        for name in ('valid', 'calibrated', 'for_flight'):
            output[name] = output[name].astype(bool)
        if np.any(output['calibrated']) or np.any(output['for_flight']):
            raise ValueError('Synthetic coefficient calibration/flight contract violated')
        return output
