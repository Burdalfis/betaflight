"""Build/load the actual portable C RPM observer; no Python observer mirror."""

import ctypes
import json
from pathlib import Path
import subprocess

import numpy as np

from rpm_data import ROOT, sha

SOURCES = [ROOT/'src/main/flight/tricopter_rpm_observer.c',
           ROOT/'src/main/flight/tricopter_rpm_observer.h',
           Path(__file__).with_name('rpm_observer_bridge.c')]
PARAMETERS = ('tracking_tau_s', 'stale_acceleration_decay_tau_s', 'max_rpm',
              'max_acceleration_rpm_s', 'stale_after_us', 'max_prediction_us', 'max_call_gap_us')


class NativeObserver:
    def __init__(self, build_dir, compiler='cc'):
        folder = Path(build_dir).resolve()
        folder.mkdir(parents=True, exist_ok=True)
        self.library_path = folder/'libtricopter_rpm.so'
        command = [compiler, '-std=c11', '-O2', '-fPIC', '-shared', '-Wall', '-Wextra', '-Werror',
                   '-I'+str(ROOT/'src/main'), str(SOURCES[0]), str(SOURCES[2]), '-lm', '-o', str(self.library_path)]
        before = {str(path.relative_to(ROOT)): sha(path) for path in SOURCES}
        subprocess.run(command, check=True, capture_output=True, text=True)
        after = {str(path.relative_to(ROOT)): sha(path) for path in SOURCES}
        if after != before:
            raise RuntimeError('RPM core/header/bridge changed during native compilation; refusing mismatched provenance')
        self.provenance = {
            'sources': before,
            'command': command, 'binary_sha256': sha(self.library_path),
            'compiler': subprocess.run([compiler, '--version'], check=True, capture_output=True, text=True).stdout.splitlines()[0],
        }
        (folder/'native-build.json').write_text(json.dumps(self.provenance, indent=2)+'\n')
        self.library = ctypes.CDLL(str(self.library_path))
        self.library.tricopterRpmReplay.argtypes = [ctypes.c_void_p, ctypes.c_size_t]+[ctypes.c_void_p]*14
        self.library.tricopterRpmReplay.restype = ctypes.c_int

    def replay(self, parameters, now_us, observed_us, sequence, seen, mechanical_rpm):
        params = np.asarray([parameters[key] for key in PARAMETERS], dtype=np.float64)
        if (params.shape != (7,) or not np.all(np.isfinite(params)) or np.any(np.abs(params[:4]) > np.finfo(np.float32).max)
                or np.any(params[4:] < 0) or np.any(params[4:] > np.iinfo(np.uint32).max)
                or np.any(params[4:] != np.floor(params[4:]))):
            raise ValueError('Observer parameters exceed native finite/conversion bounds')
        now = np.asarray(now_us)
        matrices = [np.asarray(value) for value in (observed_us, sequence, seen, mechanical_rpm)]
        if now.ndim != 1 or not len(now) or any(value.shape != (len(now), 3) for value in matrices):
            raise ValueError('Expected nonempty time vector and four matching N x 3 motor matrices')
        for value in (now, matrices[0], matrices[1]):
            if (not np.all(np.isfinite(value)) or np.any(value < 0) or np.any(value > np.iinfo(np.uint32).max)
                    or np.any(value != np.floor(value))):
                raise ValueError('Time and sequence transport must be exact uint32 values')
        if not np.all(np.isin(matrices[2], [0, 1])):
            raise ValueError('Seen transport must be boolean')
        if np.any(np.isfinite(matrices[3]) & (np.abs(matrices[3]) > np.finfo(np.float32).max)):
            raise ValueError('Finite RPM exceeds float32 transport')
        inputs = [np.ascontiguousarray(now, np.uint32), np.ascontiguousarray(matrices[0], np.uint32),
                  np.ascontiguousarray(matrices[1], np.uint32), np.ascontiguousarray(matrices[2], np.uint8),
                  np.ascontiguousarray(matrices[3], np.float32)]
        shape = (len(now), 3)
        output = {name: np.empty(shape, dtype) for name, dtype in
                  [('rpm', np.float32), ('acceleration_rpm_s', np.float32), ('age_us', np.uint32),
                   ('sequence', np.uint32), ('status', np.uint32), ('seen', np.uint8),
                   ('accepted', np.uint8), ('fresh', np.uint8), ('valid', np.uint8)]}
        ptr = lambda value: ctypes.c_void_p(value.ctypes.data)
        rc = self.library.tricopterRpmReplay(ptr(params), len(now), *[ptr(value) for value in inputs],
                                           *[ptr(value) for value in output.values()])
        if rc != 1:
            raise ValueError('Native observer rejected configuration/reset')
        for key in ('seen', 'accepted', 'fresh', 'valid'):
            output[key] = output[key].astype(bool)
        if not np.all(np.isfinite(output['rpm'])) or not np.all(np.isfinite(output['acceleration_rpm_s'])):
            raise ValueError('Native observer emitted nonfinite state')
        return output
