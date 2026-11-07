"""Synthetic-only requests and independent certificates; no allocation mirror."""
import json
from pathlib import Path
import numpy as np

from allocator_native import native_parameters, virtual_parameters
from allocator_reference import certificate, forward_geometry, geometric_matrix

PROTOCOL = Path(__file__).with_name('allocator_synthetic_protocol.json')


def load_protocol():
    protocol = json.loads(PROTOCOL.read_text())
    for config in (protocol, protocol['direct_effectiveness']):
        if config['coefficient_source'] != 'SYNTHETIC_PLACEHOLDER' or config['calibrated'] is not False or config['for_flight'] is not False:
            raise ValueError('Only explicit uncalibrated synthetic allocator sources are accepted')
    if protocol['logical_motor_order'] != ['rear', 'front_right', 'front_left'] or protocol['wrench_order'] != ['Fz_N', 'Mx_Nm', 'My_Nm', 'Mz_Nm']:
        raise ValueError('Allocator axis/motor order changed')
    if protocol['units'] != {'position': 'm', 'thrust': 'N', 'angle': 'rad', 'moment': 'N*m', 'angular_speed': 'rad/s', 'mechanical_speed': 'RPM'}:
        raise ValueError('Physical adapter SI units changed')
    if protocol['direct_effectiveness']['units']['actuator'] != 'normalized effort proxy':
        raise ValueError('Direct effectiveness must retain proxy effort units')
    native_parameters(protocol); virtual_parameters(protocol['direct_effectiveness'])
    return protocol


def physical_problem(protocol):
    p = native_parameters(protocol).astype(np.float32).astype(np.float64)
    positions = p[:9].reshape(3, 3)
    return {'positions': positions, 'matrix': geometric_matrix(positions), 'maximum': p[9:12],
            'minimum_angle': p[15], 'maximum_angle': p[16], 'scales': p[17:21]}


def virtual_problem(config):
    p = virtual_parameters(config).astype(np.float32).astype(np.float64)
    return {'matrix': p[:16].reshape(4, 4), 'maximum': p[16:19], 'minimum_angle': p[19],
            'maximum_angle': p[20], 'scales': p[21:25]}


def controls_from_effort(effort, direction):
    return np.asarray([effort[1], effort[2], effort[0]*np.cos(direction), effort[0]*np.sin(direction)])


def physical_requests(protocol):
    problem = physical_problem(protocol)
    requests = []
    for case in protocol['cases']:
        if case['kind'] == 'wrench':
            target = np.asarray(case['request'])
        else:
            _, target = forward_geometry(problem['positions'], np.asarray([case['thrust_n']]), np.asarray([case['tail_angle_rad']]), case['reaction_yaw_nm'])
            target = target[0]
        requests.append(target)
    return np.asarray(requests, np.float32)


def virtual_requests(config):
    matrix = virtual_problem(config)['matrix']
    return np.asarray([case['request'] if case['kind'] == 'response' else matrix@controls_from_effort(case['effort'], case['tail_direction_rad'])
                       for case in config['cases']], np.float32)


def check_output(problem, requested, controls, achieved, offset=None):
    result = certificate(problem['matrix'], requested, problem['scales'], controls,
                         problem['maximum'], problem['minimum_angle'], problem['maximum_angle'], offset)
    offset = np.zeros(4) if offset is None else np.asarray(offset)
    result['published_response_error_by_axis'] = abs(problem['matrix']@controls+offset-achieved).tolist()
    return result
