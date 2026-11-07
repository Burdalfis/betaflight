"""Build/load the real portable C servo core. No Python dynamics implementation."""
import ctypes
import json
from pathlib import Path
import subprocess

import numpy as np
from servo_data import ROOT, sha

SOURCES = [ROOT/'src/main/flight/tricopter_servo_model.c',
           ROOT/'src/main/flight/tricopter_servo_model.h',
           Path(__file__).with_name('servo_model_bridge.c')]


class NativeModel:
    def __init__(self, build_dir, compiler='cc'):
        folder = Path(build_dir).resolve()
        folder.mkdir(parents=True, exist_ok=True)
        self.provenance = {'sources': {str(p.relative_to(ROOT)): sha(p) for p in SOURCES}}
        self.library_path = folder/'libtricopter_servo.so'
        command = [compiler, '-std=c11', '-O2', '-fPIC', '-shared', '-Wall', '-Wextra', '-Werror',
                   '-I'+str(ROOT/'src/main'), str(SOURCES[0]), str(SOURCES[2]), '-lm', '-o', str(self.library_path)]
        subprocess.run(command, check=True, capture_output=True, text=True)
        if self.provenance['sources'] != {str(p.relative_to(ROOT)): sha(p) for p in SOURCES}:
            raise ValueError('C model source changed during compilation; rebuild from a stable source')
        self.provenance.update(command=command, binary_sha256=sha(self.library_path),
                               compiler=subprocess.run([compiler, '--version'], capture_output=True, text=True, check=True).stdout.splitlines()[0])
        (folder/'native-build.json').write_text(json.dumps(self.provenance, indent=2)+'\n')
        self.lib = ctypes.CDLL(str(self.library_path))
        pointer = ctypes.c_void_p
        self.lib.tricopterServoReplay.argtypes = [pointer, ctypes.c_double, ctypes.c_double, ctypes.c_int,
                                                  ctypes.c_size_t, pointer, pointer, pointer, pointer, pointer, pointer]
        self.lib.tricopterServoReplay.restype = ctypes.c_int

    def replay(self, params, unit):
        p = np.ascontiguousarray(params, dtype=np.float64)
        if p.shape != (9,) or not np.all(np.isfinite(p)):
            raise ValueError('Expected nine finite model parameters')
        if not isinstance(unit.time_us, np.ndarray) or not isinstance(unit.command_us, np.ndarray):
            raise ValueError('Time and command inputs must be NumPy arrays')
        if unit.time_us.ndim != 1 or unit.command_us.ndim != 1 or not len(unit.time_us) or len(unit.time_us) != len(unit.command_us):
            raise ValueError('Time and command arrays must be nonempty, one-dimensional and equal length')
        if unit.time_us.dtype != np.float64 or unit.command_us.dtype != np.float64:
            raise ValueError('Native time/command transport requires float64 arrays')
        if not np.all(np.isfinite(unit.time_us)) or np.any(unit.time_us < 0) or np.any(unit.time_us > 2**53-1):
            raise ValueError('Timestamps must be finite/nonnegative within exact float64 integer range')
        if not 0 <= p[4] <= np.iinfo(np.uint32).max or np.any(abs(p) > np.finfo(np.float32).max):
            raise ValueError('Parameters exceed native conversion bounds')
        initial = np.asarray(unit.position_counts)
        if initial.ndim != 1 or len(initial) != len(unit.time_us) or not np.isfinite(initial[0]):
            raise ValueError('Position initialization must be finite with matching one-dimensional target length')
        if not np.isfinite(unit.initial_command_us) or not np.all(np.isfinite(unit.command_us)) or np.any(abs(unit.command_us) > np.finfo(np.float32).max):
            raise ValueError('Commands must be finite and representable as float32')
        if abs(initial[0]) > np.finfo(np.float32).max or abs(unit.initial_command_us) > np.finfo(np.float32).max:
            raise ValueError('Initial values exceed native conversion bounds')
        if unit.initial_direction not in (-1, 0, 1):
            raise ValueError('Initial command direction must be -1,0,+1')
        n = len(unit.time_us)
        position = np.empty(n, np.float64)
        velocity = np.empty(n, np.float64)
        status = np.empty(n, np.uint32)
        valid = np.empty(n, np.uint8)
        arrays = [p, unit.time_us, unit.command_us, position, velocity, status, valid]
        if any(not a.flags.c_contiguous for a in arrays):
            raise ValueError('Native input arrays must be contiguous')
        ptr = lambda a: ctypes.c_void_p(a.ctypes.data)
        rc = self.lib.tricopterServoReplay(ptr(p), float(unit.position_counts[0]), unit.initial_command_us,
                                          unit.initial_direction, n, ptr(unit.time_us), ptr(unit.command_us),
                                          ptr(position), ptr(velocity), ptr(status), ptr(valid))
        if rc != 1:
            raise ValueError(f'Native reset/angle contract rejected parameters ({rc})')
        return {'position': position, 'velocity': velocity, 'status': status, 'valid': valid.astype(bool)}
