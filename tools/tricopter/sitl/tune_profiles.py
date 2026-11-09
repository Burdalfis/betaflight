"""Runner-local yaw PID overlays and strict readback checks for pinned SITL.

No sockets, firmware writes or shared configuration mutations occur here.
MSP offsets follow Betaflight 2026.6.2 msp.c: MSP_PID serializes five P/I/D
triples; MSP_PID_ADVANCED puts roll/pitch/yaw F at offsets32/34/36.
Yaw-hold gain has no field in those replies and must be queried in the CLI log.
"""
from __future__ import annotations

import copy
import hashlib
from numbers import Integral
from pathlib import Path
import re
import struct


# Zero-based, half-open byte intervals. Deliberately exclude yaw D-max, filters,
# roll/pitch P/I/D/F and every other supplied MSP field from these masks.
MASKED_BYTE_RANGES = {'112': [[6, 9]], '94': [[36, 38]]}
PAYLOAD_LENGTHS = {'112': 15, '94': 61}
HOLD_FIELD = 'feedforward_yaw_hold_gain'
ANSI_ESCAPE = re.compile(r'\x1b\[[0-?]*[ -/]*[@-~]')
COMMAND_ECHO = re.compile(r'^[ \t]*#[ \t]*(?P<command>[^\r\n]*)$', re.MULTILINE)


def _integer(value, name, maximum):
    if isinstance(value, bool) or not isinstance(value, Integral) or not 0 <= value <= maximum:
        raise ValueError(f'{name} must be an integer in 0..{maximum}')
    return int(value)


def _gains(values):
    if not isinstance(values, dict) or set(values) != {'p', 'i', 'd'}:
        raise ValueError('Yaw gains must contain exactly p, i and d')
    return {name: _integer(values[name], name, 250) for name in ('p', 'i', 'd')}


def overlay_text(gains, feedforward=0, hold_gain=0):
    """Return a profile0 CLI overlay; EOF provisioning saves it automatically.

    Only yaw P/I/D/F and yaw hold gain are set. Get commands retain native CLI
    readbacks, including hold gain which MSP94 does not serialize. There is no
    early save/exit command that could prevent the trailing queries executing.
    """
    gains = _gains(gains)
    feedforward = _integer(feedforward, 'feedforward', 1000)
    hold_gain = _integer(hold_gain, 'hold_gain', 100)
    settings = {f'{name}_yaw': value for name, value in gains.items()}
    settings.update(f_yaw=feedforward, **{HOLD_FIELD: hold_gain})
    lines = ['profile 0']
    lines.extend(f'set {name} = {value}' for name, value in settings.items())
    lines.extend(f'get {name}' for name in settings)
    return '\n'.join(lines)+'\n'


def _readback(record):
    values = record.get('common_msp_readback')
    if not isinstance(values, dict):
        raise ValueError('Record lacks common_msp_readback')
    payloads = {}
    for command, value in values.items():
        if not isinstance(command, str) or not isinstance(value, str):
            raise ValueError('MSP readbacks must map command strings to hex strings')
        try:
            payloads[command] = bytes.fromhex(value)
        except ValueError as error:
            raise ValueError(f'Malformed MSP{command} hex readback') from error
    for command, length in PAYLOAD_LENGTHS.items():
        if command not in payloads or len(payloads[command]) != length:
            raise ValueError(f'Pinned MSP{command} reply must contain {length} bytes')
    return payloads


def shared_msp_readback(record):
    """Return canonical shared bytes plus an explicit mask specification.

    The input record and its raw replies remain unchanged. All provided common
    replies participate in comparison, including any filter/config readbacks
    the caller records beyond Trial's normal set. Unrecorded settings cannot be
    checked here. Only the declared yaw PID/F bytes are replaced by zero.
    """
    payloads = _readback(record)
    shared = {command: bytearray(payload) for command, payload in payloads.items()}
    for command, ranges in MASKED_BYTE_RANGES.items():
        for start, end in ranges:
            shared[command][start:end] = bytes(end-start)
    return {'readback': {command: payload.hex() for command, payload in shared.items()},
            'masked_byte_ranges': copy.deepcopy(MASKED_BYTE_RANGES),
            'byte_range_convention': 'zero-based half-open'}


def _hold_get(log):
    """Require a native echoed get and its value, rather than a set-command echo."""
    clean = ANSI_ESCAPE.sub('', log).replace('\r\n', '\n').replace('\r', '\n')
    commands = list(COMMAND_ECHO.finditer(clean))
    queries = [(index, command) for index, command in enumerate(commands)
               if command['command'].strip().lower() == f'get {HOLD_FIELD}']
    if not queries:
        raise AssertionError(f'provision.log lacks native get {HOLD_FIELD} readback')
    index, query = queries[-1]
    end = commands[index+1].start() if index+1 < len(commands) else len(clean)
    values = re.findall(rf'^[ \t]*{HOLD_FIELD}[ \t]*=[ \t]*(\d+)[ \t]*$',
                        clean[query.end():end], re.MULTILINE)
    if len(values) != 1:
        raise AssertionError(f'Last get {HOLD_FIELD} lacks one unambiguous native value')
    if any(re.match(rf'set[ \t]+{HOLD_FIELD}(?:[ \t]|$)', command['command'].strip(), re.IGNORECASE)
           for command in commands[index+1:]):
        raise AssertionError(f'{HOLD_FIELD} was set after its final CLI get')
    return {'field': HOLD_FIELD, 'value': int(values[0]),
            'query_line': clean.count('\n', 0, query.start())+1}


def verify_profile(record, expected_gains, feedforward=0, hold_gain=0, *, directory, reference=None):
    """Verify yaw values and optionally every unmasked provided common byte.

    Returns a JSON-safe audit. Malformed inputs raise ValueError; observed
    configuration mismatches raise AssertionError. Reference is another Trial
    record, not a profile specification. No metadata or replies are changed.
    """
    expected_gains = _gains(expected_gains)
    feedforward = _integer(feedforward, 'feedforward', 1000)
    hold_gain = _integer(hold_gain, 'hold_gain', 100)
    payloads = _readback(record)
    actual_gains = dict(zip(('p', 'i', 'd'), payloads['112'][6:9]))
    actual_feedforward = struct.unpack_from('<H', payloads['94'], 36)[0]
    if actual_gains != expected_gains:
        raise AssertionError(f'Yaw PID readback {actual_gains} != requested {expected_gains}')
    if actual_feedforward != feedforward:
        raise AssertionError(f'Yaw F readback {actual_feedforward} != requested {feedforward}')
    path = Path(directory)/'provision.log'
    data = path.read_bytes()
    hold = _hold_get(data.decode('utf-8', errors='replace'))
    if hold['value'] != hold_gain:
        raise AssertionError(f'Yaw hold gain readback {hold["value"]} != requested {hold_gain}')
    shared = shared_msp_readback(record)
    if reference is not None and shared != shared_msp_readback(reference):
        raise AssertionError('Unexpected non-tuned MSP readback drift outside yaw PID/F bytes')
    return {'pass': True, 'gains': actual_gains, 'feedforward': actual_feedforward,
            'hold_gain': hold['value'],
            'yaw_pid_readback': {'command': 112, 'byte_range': [6, 9]},
            'yaw_feedforward_readback': {'command': 94, 'byte_range': [36, 38], 'encoding': 'uint16 little-endian'},
            'hold_cli_readback': {**hold, 'path': str(path.resolve()),
                                  'sha256': hashlib.sha256(data).hexdigest()},
            'reference_checked': reference is not None,
            'shared_msp_readback': shared}
