"""Steady native ANGLE targets for the pinned 2026.6.2 comparison profile.

This reads MSP replies; it does not infer a requested yaw rate from the plant's
response. Source: fc/rc.c updateRcCommands/applyActualRates/getMaxRcRate and
flight/pid.c pidLevel plus ANGLE yaw earth-reference attenuation. RC smoothing,
angle-target filters, acceleration limits and the preceding-loop angle target
make transient setpoints different; these are held-command reference values.
"""
from __future__ import annotations

import math
from numbers import Integral, Real
import re
import struct

from compare_metrics import actual_yaw_rate


def _finite(value, name):
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError(f'{name} must be finite')
    return float(value)


def _payloads(record):
    result = {}
    for command, length in (('111', 24), ('94', 61), ('125', 5)):
        try:
            value = bytes.fromhex(record['common_msp_readback'][command])
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f'Malformed or missing MSP{command} readback') from error
        if len(value) != length:
            raise ValueError(f'Pinned MSP{command} must contain {length} bytes')
        result[command] = value
    if result['111'][22] != 3:
        raise ValueError('Envelope target reconstruction requires ACTUAL rates')
    return result


def angle_targets(record, roll_pwm, pitch_pwm, yaw_pwm, *, earth_ref=100, midrc=1500):
    """Return native ANGLE roll/pitch and body-FRD yaw references, in degrees.

    High roll/pitch PWM gives positive native angle. High yaw PWM gives negative
    body-FRD rate unless the retained yaw_settings says reversed=True. ``earth_ref``
    must be verified separately from native CLI because MSP94 omits this field.
    ``midrc`` defaults to the shared profile's 1500 us and may be supplied from
    MSP44's uint16 offset3. Requested angles use the unconstrained maximum rate
    as denominator, while each current raw rate is clipped by its rate limit.
    """
    if isinstance(earth_ref, bool) or not isinstance(earth_ref, Integral) or not 0 <= earth_ref <= 100:
        raise ValueError('angle_earth_ref must be an integer in 0..100')
    midrc = _finite(midrc, 'midrc')
    if not 1000 <= midrc <= 2000:
        raise ValueError('midrc must be in 1000..2000 us')
    commands = [_finite(value, name) for value, name in
                ((roll_pwm, 'roll PWM'), (pitch_pwm, 'pitch PWM'), (yaw_pwm, 'yaw PWM'))]
    if any(not 1000 <= value <= 2000 for value in commands):
        raise ValueError('RC PWM must be in 1000..2000 us')
    payloads = _payloads(record)
    rates, advanced, deadbands = (payloads[key] for key in ('111', '94', '125'))
    angle_limit = advanced[17]
    if not 0 < angle_limit <= 90:
        raise ValueError('ANGLE limit must be in 1..90 degrees')
    if deadbands[0] >= 500 or deadbands[1] >= 500:
        raise ValueError('Invalid RC deadband')
    centers = [10*rates[index] for index in (0, 12, 11)]
    maxima = [10*rates[index] for index in (2, 3, 4)]
    expos = [rates[index]/100 for index in (1, 13, 10)]
    limits = list(struct.unpack_from('<3H', rates, 16))
    if any(expo > 1 for expo in expos):
        raise ValueError('ACTUAL expo must be in 0..100 percent')
    angles = []
    for axis in (0, 1):
        denominator = max(centers[axis], maxima[axis])
        if denominator <= 0:
            raise ValueError('ANGLE normalization requires a nonzero full-stick rate')
        # Reuse the source-backed ACTUAL curve with reversal disabled, so it
        # has roll/pitch's positive PWM sign. Native first clips rc-midrc.
        effective_pwm = 1500+max(-500, min(500, commands[axis]-midrc))
        rate = actual_yaw_rate(effective_pwm, centers[axis], maxima[axis], expos[axis],
                               deadband=deadbands[0], reversed=True, limit=limits[axis])
        angles.append(max(-angle_limit, min(angle_limit, angle_limit*rate/denominator)))
    yaw_settings = dict(center=centers[2], max=maxima[2], expo=expos[2],
                        deadband=deadbands[1], reversed=False, limit=limits[2])
    supplied = record.get('yaw_settings')
    if supplied is not None:
        if not isinstance(supplied, dict) or any(supplied.get(key) != value for key, value in
                yaw_settings.items() if key != 'reversed'):
            raise ValueError('yaw_settings disagree with native MSP replies')
        if not isinstance(supplied.get('reversed'), bool):
            raise ValueError('yaw_settings.reversed must be boolean')
        yaw_settings['reversed'] = supplied['reversed']
    effective_yaw = 1500+max(-500, min(500, commands[2]-midrc))
    stick_rate = actual_yaw_rate(effective_yaw, **yaw_settings)
    attenuation = math.cos(math.radians(earth_ref/100*max(abs(value) for value in angles)))
    body_rate = stick_rate*attenuation
    return {'requested_roll_deg': angles[0], 'requested_pitch_deg': angles[1],
            'stick_requested_yaw_rate_dps': stick_rate,
            'yaw_pid_target_frd_dps': body_rate, 'requested_yaw_rate_dps': body_rate,
            'angle_earth_ref': int(earth_ref), 'angle_yaw_attenuation': attenuation,
            'midrc_us': midrc,
            'reference_definition': 'held-command ANGLE target; transient smoothing/filtering excluded'}


def read_angle_earth_ref(log):
    """Read the last unambiguous native CLI get, rejecting a later setting."""
    clean = re.sub(r'\x1b\[[0-?]*[ -/]*[@-~]', '', log).replace('\r\n', '\n').replace('\r', '\n')
    commands = list(re.finditer(r'^[ \t]*#[ \t]*(?P<command>[^\r\n]*)$', clean, re.MULTILINE))
    queries = [(index, command) for index, command in enumerate(commands)
               if command['command'].strip().lower() == 'get angle_earth_ref']
    if not queries:
        raise AssertionError('Missing native get angle_earth_ref')
    index, query = queries[-1]
    end = commands[index+1].start() if index+1 < len(commands) else len(clean)
    values = re.findall(r'^[ \t]*angle_earth_ref[ \t]*=[ \t]*(\d+)[ \t]*$',
                        clean[query.end():end], re.MULTILINE)
    if len(values) != 1 or not 0 <= int(values[0]) <= 100:
        raise AssertionError('Last get angle_earth_ref lacks one valid native value')
    if any(re.match(r'set[ \t]+angle_earth_ref(?:[ \t]|$)',
                    command['command'].strip(), re.IGNORECASE) for command in commands[index+1:]):
        raise AssertionError('angle_earth_ref was set after its final get')
    return int(values[0])
