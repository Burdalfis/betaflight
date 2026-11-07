"""Execute the actual portable C allocator; no Python allocation algorithm."""

import ctypes
import json
from pathlib import Path
import subprocess

import numpy as np

from rpm_data import ROOT, sha

SOURCES = [ROOT/'src/main/flight/tricopter_allocator.c', ROOT/'src/main/flight/tricopter_allocator.h',
           Path(__file__).with_name('allocator_model_bridge.c')]


def native_parameters(protocol):
    geometry, limits, solver = protocol['geometry_m'], protocol['limits'], protocol['solver']
    positions = np.asarray(geometry['rotor_positions'], np.float64)
    cg = np.asarray(geometry['center_of_gravity'], np.float64)
    arrays = [np.asarray(limits['maximum_thrust_n'], np.float64),
              np.asarray(protocol['thrust_coefficients_n_per_rad_s_squared'], np.float64),
              np.asarray(protocol['wrench_scales'], np.float64)]
    if positions.shape != (3, 3) or cg.shape != (3,) or [part.shape for part in arrays] != [(3,), (3,), (4,)]:
        raise ValueError('Expected 3 rotor vectors, one CG vector, three thrust/kT limits and four scales')
    scalars = np.asarray([limits['tail_angle_min_rad'], limits['tail_angle_max_rad'], solver['low_tail_thrust_n'],
                          solver['projected_gradient_tolerance'], solver['max_condition_number'], solver['max_iterations']], np.float64)
    if scalars.shape != (6,):
        raise ValueError('Expected scalar angle and solver settings')
    values = np.concatenate(((positions-cg).ravel(), arrays[0], arrays[1], scalars[:2], arrays[2], scalars[2:]))
    if (values.shape != (25,) or not np.all(np.isfinite(values)) or np.any(abs(values) > np.finfo(np.float32).max)
            or values[24] < 0 or values[24] > 65535 or values[24] != np.floor(values[24])):
        raise ValueError('Allocator parameters exceed finite/native transport bounds')
    return values


def virtual_parameters(config):
    matrix = np.asarray(config['effectiveness_matrix'], np.float64)
    maximum, scales = np.asarray(config['maximum_effort'], np.float64), np.asarray(config['response_scales'], np.float64)
    solver = config['solver']
    scalars = np.asarray([config['tail_direction_min_rad'], config['tail_direction_max_rad'], solver['low_tail_effort'],
                          solver['projected_gradient_tolerance'], solver['max_condition_number'], solver['max_iterations']], np.float64)
    if matrix.shape != (4, 4) or maximum.shape != (3,) or scales.shape != (4,) or scalars.shape != (6,):
        raise ValueError('Expected direct 4x4 map, three maximum efforts, four response scales and scalar solver settings')
    values = np.r_[matrix.ravel(), maximum, scalars[:2], scales, scalars[2:]]
    if (values.shape != (29,) or not np.all(np.isfinite(values)) or np.any(abs(values) > np.finfo(np.float32).max)
            or values[28] < 0 or values[28] > 65535 or values[28] != np.floor(values[28])):
        raise ValueError('Direct map parameters exceed finite/native transport bounds')
    return values


class NativeAllocator:
    def __init__(self, build_dir, compiler='cc'):
        folder = Path(build_dir).resolve(); folder.mkdir(parents=True, exist_ok=True)
        library = folder/'libtricopter_allocator.so'
        command = [compiler, '-std=c11', '-O2', '-fPIC', '-shared', '-Wall', '-Wextra', '-Werror',
                   '-I'+str(ROOT/'src/main'), str(SOURCES[0]), str(SOURCES[2]), '-lm', '-o', str(library)]
        before = {str(path.relative_to(ROOT)): sha(path) for path in SOURCES}
        subprocess.run(command, check=True, capture_output=True, text=True)
        if before != {str(path.relative_to(ROOT)): sha(path) for path in SOURCES}:
            raise RuntimeError('Allocator core/header/bridge changed during compilation')
        self.provenance = {'sources': before, 'command': command, 'binary_sha256': sha(library),
                           'compiler': subprocess.run([compiler, '--version'], check=True, capture_output=True, text=True).stdout.splitlines()[0]}
        (folder/'native-build.json').write_text(json.dumps(self.provenance, indent=2)+'\n')
        self.library = ctypes.CDLL(str(library))
        self.library.tricopterAllocatorReplay.argtypes = [ctypes.c_void_p, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_size_t]+[ctypes.c_void_p]*9
        self.library.tricopterAllocatorReplay.restype = ctypes.c_int
        self.library.tricopterAllocatorVirtualReplay.argtypes = [ctypes.c_void_p, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_size_t]+[ctypes.c_void_p]*6
        self.library.tricopterAllocatorVirtualReplay.restype = ctypes.c_int

    def run(self, protocol, requests=None, thrust=None, angles=None, reaction_yaw=None, use_reaction=None,
            reaction_valid=None, coefficient_source=None):
        values = native_parameters(protocol)
        source = {'UNSPECIFIED': 0, 'SYNTHETIC_PLACEHOLDER': 1}.get(protocol['coefficient_source'] if coefficient_source is None else coefficient_source)
        if source is None:
            raise ValueError('No calibrated geometry source is defined')
        forward = requests is None
        if forward:
            thrust = np.asarray(thrust)
            if thrust.ndim != 2 or thrust.shape[1:] != (3,) or not len(thrust):
                raise ValueError('Expected nonempty N x 3 forward thrust array')
            n = len(thrust); request = np.zeros((n, 4)); angle = np.asarray(angles)
        else:
            request = np.asarray(requests)
            if request.ndim != 2 or request.shape[1:] != (4,) or not len(request):
                raise ValueError('Expected nonempty N x 4 wrench request array')
            n = len(request); thrust = np.zeros((n, 3)); angle = np.zeros(n)
        reaction = np.zeros(n) if reaction_yaw is None else np.asarray(reaction_yaw)
        use = np.zeros(n, np.uint8) if use_reaction is None else np.asarray(use_reaction)
        known = np.zeros(n, np.uint8) if reaction_valid is None else np.asarray(reaction_valid)
        if any(value.shape != (n,) for value in (angle, reaction, use, known)):
            raise ValueError('Angle/reaction metadata must be matching N vectors')
        if any(not np.all(np.isfinite(value)) or np.any(abs(value) > np.finfo(np.float32).max)
               for value in (request, thrust, angle, reaction)):
            raise ValueError('Allocator runtime transport must be finite float32 values')
        if not np.all(np.isin(use, [0, 1])) or not np.all(np.isin(known, [0, 1])):
            raise ValueError('Reaction metadata must be boolean')
        inputs = [np.ascontiguousarray(value, np.float32) for value in (request, thrust, angle, reaction)]
        inputs += [np.ascontiguousarray(value, np.uint8) for value in (use, known)]
        numeric, integer, flags = np.empty((n, 32), np.float32), np.empty((n, 3), np.uint32), np.empty((n, 7), np.uint8)
        ptr = lambda value: ctypes.c_void_p(value.ctypes.data)
        rc = self.library.tricopterAllocatorReplay(ptr(values), source, int(forward), n,
                                                 *[ptr(value) for value in inputs], ptr(numeric), ptr(integer), ptr(flags))
        if rc != 1 or not np.all(np.isfinite(numeric)):
            raise ValueError('Native allocator rejected transport or emitted nonfinite outputs')
        out = {name: numeric[:, start:end].copy() for name, start, end in
               [('requested', 0, 4), ('achieved', 4, 8), ('residual', 8, 12), ('force_n', 12, 15),
                ('thrust_n', 15, 18), ('omega_rad_s', 18, 21), ('mechanical_rpm', 21, 24), ('controls_n', 24, 28)]}
        for column, name in enumerate(('tail_angle_rad', 'cost', 'projected_gradient_norm', 'condition_number'), 28):
            out[name] = numeric[:, column].copy()
        for column, name in enumerate(('status', 'constraints', 'iterations')):
            out[name] = integer[:, column].copy()
        for column, name in enumerate(('valid', 'converged', 'exact', 'tail_angle_defined', 'tail_thrust_low', 'calibrated', 'for_flight')):
            out[name] = flags[:, column].astype(bool)
        if out['calibrated'].any() or out['for_flight'].any():
            raise ValueError('Synthetic allocator provenance contract violated')
        return out

    def virtual_run(self, config, requests=None, effort=None, directions=None, coefficient_source=None):
        values = virtual_parameters(config)
        source = {'UNSPECIFIED': 0, 'SYNTHETIC_PLACEHOLDER': 1}.get(config['coefficient_source'] if coefficient_source is None else coefficient_source)
        if source is None:
            raise ValueError('No calibrated direct-effectiveness source is defined')
        forward = requests is None
        if forward:
            effort = np.asarray(effort)
            if effort.ndim != 2 or effort.shape[1:] != (3,) or not len(effort):
                raise ValueError('Expected nonempty N x 3 forward effort array')
            n = len(effort); request = np.zeros((n, 4)); direction = np.asarray(directions)
        else:
            request = np.asarray(requests)
            if request.ndim != 2 or request.shape[1:] != (4,) or not len(request):
                raise ValueError('Expected nonempty N x 4 direct response request')
            n = len(request); effort = np.zeros((n, 3)); direction = np.zeros(n)
        if direction.shape != (n,) or any(not np.all(np.isfinite(value)) or np.any(abs(value) > np.finfo(np.float32).max)
                                          for value in (request, effort, direction)):
            raise ValueError('Direct runtime inputs must have matching shape and finite float32 transport')
        inputs = [np.ascontiguousarray(value, np.float32) for value in (request, effort, direction)]
        numeric, integer, flags = np.empty((n, 23), np.float32), np.empty((n, 3), np.uint32), np.empty((n, 7), np.uint8)
        ptr = lambda value: ctypes.c_void_p(value.ctypes.data)
        rc = self.library.tricopterAllocatorVirtualReplay(ptr(values), source, int(forward), n,
                                                        *[ptr(value) for value in inputs], ptr(numeric), ptr(integer), ptr(flags))
        if rc != 1 or not np.all(np.isfinite(numeric)):
            raise ValueError('Native direct allocator rejected transport or emitted nonfinite output')
        out = {name: numeric[:, start:end].copy() for name, start, end in
               [('requested', 0, 4), ('achieved', 4, 8), ('residual', 8, 12), ('effort', 12, 15), ('controls', 15, 19)]}
        for column, name in enumerate(('tail_direction_rad', 'cost', 'projected_gradient_norm', 'condition_number'), 19):
            out[name] = numeric[:, column].copy()
        for column, name in enumerate(('status', 'constraints', 'iterations')):
            out[name] = integer[:, column].copy()
        for column, name in enumerate(('valid', 'converged', 'exact', 'tail_direction_defined', 'tail_effort_low', 'calibrated', 'for_flight')):
            out[name] = flags[:, column].astype(bool)
        if out['calibrated'].any() or out['for_flight'].any():
            raise ValueError('Direct synthetic allocator provenance contract violated')
        return out
