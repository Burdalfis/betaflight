"""Frozen synthetic inputs and independent float64 analytic *test oracles*.

This reference algebra is only a test oracle, not an alternate runtime model.
All reported predictions execute the portable C core through reaction_native.
"""

import json
from pathlib import Path

import numpy as np

PROTOCOL = Path(__file__).with_name('reaction_synthetic_protocol.json')


def validate_protocol(protocol):
    if (protocol.get('coefficient_source') != 'SYNTHETIC_PLACEHOLDER'
            or protocol.get('calibrated') is not False or protocol.get('for_flight') is not False):
        raise ValueError('Only explicitly uncalibrated synthetic placeholder coefficients are allowed')
    if protocol.get('logical_motor_order') != ['rear', 'front_right', 'front_left']:
        raise ValueError('Expected fixed aircraft logical motor order')
    parameters = protocol['parameters']
    required = {'k_q_nm_per_rad_s_squared', 'rotor_inertia_kg_m2', 'reaction_sign', 'max_mechanical_rpm',
                'max_abs_acceleration_rpm_s', 'max_rpm_age_us', 'max_abs_torque_nm'}
    if set(parameters) != required or parameters['reaction_sign'] != [-1, 1, -1]:
        raise ValueError('Explicit complete SI parameters and aircraft reaction signs are required')
    units = protocol['units']
    expected_units = {
        'mechanical_rpm': 'revolutions per minute, nonnegative magnitude',
        'acceleration_rpm_s': 'signed change of RPM magnitude per second',
        'omega_rad_s': 'mechanical_rpm * pi/30',
        'alpha_rad_s2': 'acceleration_rpm_s * pi/30',
        'aerodynamic_body_torque_nm': 'reaction_sign * kQ * omega^2',
        'inertial_body_torque_nm': 'reaction_sign * J * alpha',
        'total_body_torque_nm': 'sum of aerodynamic and inertial terms over logical motors',
    }
    if units != expected_units:
        raise ValueError('Synthetic SI units/equation convention changed')
    return protocol


def load_protocol():
    return validate_protocol(json.loads(PROTOCOL.read_text()))


def analytic_oracle(parameters, rpm, acceleration):
    """Independent double-precision equation oracle for analytic inputs only."""
    omega = np.asarray(rpm, np.float64)*(2*np.pi)/60
    alpha = np.asarray(acceleration, np.float64)*(2*np.pi)/60
    aerodynamic = np.asarray(parameters['reaction_sign'])*np.asarray(parameters['k_q_nm_per_rad_s_squared'])*omega**2
    inertial = np.asarray(parameters['reaction_sign'])*np.asarray(parameters['rotor_inertia_kg_m2'])*alpha
    return {'omega_rad_s': omega, 'alpha_rad_s2': alpha, 'aerodynamic_nm': aerodynamic,
            'inertial_nm': inertial, 'total_nm': aerodynamic+inertial,
            'summed_nm': np.column_stack((aerodynamic.sum(axis=1), inertial.sum(axis=1), (aerodynamic+inertial).sum(axis=1)))}


def build_cases(protocol):
    parameters = protocol['parameters']
    output = []
    for case in protocol['cases']:
        kind = case['kind']
        time = np.asarray([0.]) if kind == 'cancellation_snapshot' else np.arange(round(case['duration_s']/case['dt_s'])+1)*case['dt_s']
        rpm = np.zeros((len(time), 3))
        acceleration = np.zeros_like(rpm)
        if kind == 'steady':
            rpm[:] = case['rpm']
            acceleration[:] = case['acceleration_rpm_s']
            if np.any(acceleration):
                raise ValueError('A steady RPM trajectory must have zero acceleration')
        elif kind == 'ramp':
            motor = case['motor']
            rpm[:, motor] = case['initial_rpm']+case['acceleration_rpm_s']*time
            acceleration[:, motor] = case['acceleration_rpm_s']
        elif kind == 'braking':
            motor = case['motor']
            rpm[:, motor] = case['initial_rpm']*np.maximum(0, 1-time/case['stop_time_s'])
            acceleration[:, motor] = np.where(time < case['stop_time_s'], -case['initial_rpm']/case['stop_time_s'], 0)
        elif kind == 'cancellation_snapshot':
            rpm[0] = np.asarray([1, np.sqrt(2), 1])*case['rpm_base']
            acceleration[0] = np.asarray([1, 2, 1])*case['acceleration_base_rpm_s']
        elif kind == 'free_coast':
            motor = case['motor']
            ratio = parameters['k_q_nm_per_rad_s_squared'][motor]/parameters['rotor_inertia_kg_m2'][motor]
            omega = case['initial_omega_rad_s']/(1+ratio*case['initial_omega_rad_s']*time)
            rpm[:, motor] = omega*60/(2*np.pi)
            acceleration[:, motor] = -ratio*omega**2*60/(2*np.pi)
        else:
            raise ValueError(f'Unknown synthetic case {kind}')
        output.append({'name': case['name'], 'kind': kind, 'time_s': time, 'rpm': rpm,
                       'acceleration_rpm_s': acceleration, 'oracle': analytic_oracle(parameters, rpm, acceleration)})
    return output


def evaluate_case(native, protocol, case):
    shape = case['rpm'].shape
    return native.evaluate(protocol['parameters'], case['rpm'], case['acceleration_rpm_s'],
                           np.zeros(shape, np.uint32), *[np.ones(shape, np.uint8) for _ in range(4)],
                           coefficient_source=protocol['coefficient_source'])
