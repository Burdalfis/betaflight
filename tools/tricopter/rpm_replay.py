#!/usr/bin/env python3
"""Offline real-C RPM observer replay with declared historical timing proxies."""

import argparse
import csv
import gzip
import json
import os
from pathlib import Path

import numpy as np

from rpm_data import ROOT, decode_inputs, observation_proxy, read_csv, sha
from rpm_native import NativeObserver, SOURCES

SETTINGS = Path(__file__).with_name('rpm_replay_settings.json')


def intervals(mask, times, ages):
    padded = np.r_[False, mask, False]
    starts = np.flatnonzero(np.diff(padded.astype(int)) == 1)
    ends = np.flatnonzero(np.diff(padded.astype(int)) == -1)
    return [{'start_s': float(times[start]), 'end_s': float(times[end-1]),
             'rows': int(end-start), 'max_native_age_lower_us': int(np.max(ages[start:end]))}
            for start, end in zip(starts, ends)]


def evaluate_record(native, settings, record, output):
    data = read_csv(record['csv'])
    time = (data['time (us)']-data['time (us)'][0])/1e6
    logged = np.stack([data[f'eRPM[{motor}]']*100.0/7.0 for motor in range(3)], axis=1)
    native_age = np.stack([data[f'debug[{4+motor}]']*100 for motor in range(3)], axis=1)
    cases, display = [], None
    params = settings['parameters']
    with gzip.open(output/(record['name']+'-replay.csv.gz'), 'wt') as stream:
        writer = csv.writer(stream)
        writer.writerow(['phase_allowance_us', 'blackbox_time_us', 'logical_motor', 'logged_cached_mechanical_rpm',
                         'native_age_floor_us', 'metadata_usable', 'proxy_interval_lower_us', 'proxy_interval_upper_us',
                         'proxy_sequence', 'proxy_observation_time_us', 'proxy_new', 'observer_rpm',
                         'observer_acceleration_rpm_s', 'observer_age_us', 'observer_sequence', 'observer_accepted_new',
                         'observer_fresh', 'effective_fresh', 'observer_valid', 'observer_status'])
        for phase in settings['phase_allowance_sensitivity_us']:
            proxy = observation_proxy(data, phase)
            observed = native.replay(params, proxy.now_us, proxy.observed_us, proxy.sequence, proxy.seen, proxy.mechanical_rpm)
            effective_fresh = (observed['fresh'] & proxy.metadata_usable
                               & (proxy.native_age_upper_us <= params['stale_after_us']))
            # These assertions establish stale handling, not acceleration truth.
            beyond = proxy.native_age_upper_us > params['max_prediction_us']
            if np.any(observed['accepted'] & beyond) or np.any(effective_fresh & beyond):
                raise ValueError('An old cached observation refreshed the native C observer')
            if np.any(observed['fresh'] & (observed['age_us'] > params['stale_after_us'])):
                raise ValueError('C observer freshness contradicts its accepted observation age')
            motors = []
            for motor in range(3):
                usable = proxy.metadata_usable[:, motor]
                fresh = effective_fresh[:, motor]
                discrepancy = observed['rpm'][fresh, motor]-logged[fresh, motor]
                motors.append({
                    'logical_motor': motor,
                    'proxy': proxy.diagnostics['motors'][motor],
                    'c_observer_accepted_proxy_observations': int(observed['accepted'][:, motor].sum()),
                    'effective_fresh_rows': int(fresh.sum()),
                    'not_effectively_fresh_rows': int((~fresh).sum()),
                    'prediction_valid_rows': int(observed['valid'][:, motor].sum()),
                    'horizon_exceeded_rows': int(np.count_nonzero(observed['status'][:, motor] & 512)),
                    'rejected_observation_rows': int(np.count_nonzero(observed['status'][:, motor] & 16)),
                    'maximum_native_age_lower_us': int(np.max(native_age[:, motor])),
                    'maximum_observer_age_us': int(np.max(observed['age_us'][observed['seen'][:, motor], motor])),
                    'maximum_logged_cached_mechanical_rpm': float(np.max(logged[:, motor])),
                    'maximum_absolute_estimated_acceleration_rpm_s': float(np.max(abs(observed['acceleration_rpm_s'][:, motor]))),
                    'fresh_cached_speed_discrepancy_rmse_rpm_not_accuracy': float(np.sqrt(np.mean(discrepancy**2))) if len(discrepancy) else None,
                    'native_age_over_10ms_intervals': intervals(usable & (native_age[:, motor] > 10000), time, native_age[:, motor]),
                    'accepted_when_native_age_beyond_horizon': int(np.count_nonzero(observed['accepted'][:, motor] & beyond[:, motor])),
                })
            cases.append({'phase_allowance_us': phase, 'motors': motors})
            if phase == settings['primary_display_phase_allowance_us']:
                display = (proxy, observed, effective_fresh)
            for i in range(len(time)):
                for motor in range(3):
                    writer.writerow([phase, int(data['time (us)'][i]), motor, logged[i, motor],
                                     int(native_age[i, motor]), int(proxy.metadata_usable[i, motor]),
                                     int(proxy.interval_lower_us[i, motor]), int(proxy.interval_upper_us[i, motor]),
                                     int(proxy.sequence[i, motor]), int(proxy.observed_us[i, motor]), int(proxy.proxy_new[i, motor]),
                                     float(observed['rpm'][i, motor]), float(observed['acceleration_rpm_s'][i, motor]),
                                     int(observed['age_us'][i, motor]), int(observed['sequence'][i, motor]),
                                     int(observed['accepted'][i, motor]), int(observed['fresh'][i, motor]),
                                     int(effective_fresh[i, motor]), int(observed['valid'][i, motor]), int(observed['status'][i, motor])])
    if display is None:
        raise ValueError('Primary display phase is absent from sensitivity settings')
    plot_record(record['name'], time, logged, native_age, display, settings, output)
    return {'name': record['name'], 'rows': len(time), 'duration_s': float(time[-1]), 'cases': cases}


def plot_record(name, time, logged, native_age, display, settings, output):
    os.environ.setdefault('MPLCONFIGDIR', str(output/'matplotlib-cache'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    proxy, observed, fresh = display
    fig, axes = plt.subplots(3, 3, figsize=(17, 10), sharex=True, layout='constrained')
    for motor in range(3):
        axes[motor, 0].plot(time, logged[:, motor], color='.65', linewidth=1, label='logged cached RPM')
        axes[motor, 0].plot(time, observed['rpm'][:, motor], color='#1664a5', linewidth=.9, label='C observer prediction')
        axes[motor, 0].set_ylabel(f'Motor {motor}\nmechanical RPM')
        axes[motor, 1].plot(time, observed['acceleration_rpm_s'][:, motor], color='#1664a5', linewidth=.8)
        axes[motor, 1].set_ylabel('Estimated RPM/s\n(no acceleration truth)')
        axes[motor, 2].plot(time, native_age[:, motor]/1000, color='.5', linewidth=.9, label='logged age floor')
        axes[motor, 2].plot(time, observed['age_us'][:, motor]/1000, color='#1664a5', linewidth=.7, label='proxy observer age')
        axes[motor, 2].axhline(settings['parameters']['stale_after_us']/1000, color='#c67c00', linestyle='--', linewidth=.8, label='freshness limit')
        axes[motor, 2].axhline(settings['parameters']['max_prediction_us']/1000, color='#b23535', linestyle=':', linewidth=.8, label='prediction horizon')
        axes[motor, 2].set_yscale('symlog', linthresh=1)
        axes[motor, 2].set_ylabel('Age (ms)')
        for column in range(3):
            axis = axes[motor, column]
            axis.fill_between(time, 0, 1, where=~fresh[:, motor], color='#e8ae45', alpha=.23,
                              transform=axis.get_xaxis_transform(), label='not effectively fresh' if motor == 0 and column == 0 else None)
            axis.grid(alpha=.2)
    axes[0, 0].legend(loc='upper right', fontsize=8)
    axes[0, 2].legend(loc='upper right', fontsize=8)
    for axis in axes[-1]:
        axis.set_xlabel('Blackbox elapsed seconds')
    fig.suptitle(f'{name}: real C observer; phase allowance {proxy.diagnostics["phase_allowance_us"]} us is an unmeasured sensitivity assumption')
    fig.savefig(output/(name+'-observer.png'), dpi=150, bbox_inches='tight')
    plt.close(fig)


def synthetic_report(native, parameters):
    times = np.arange(0, 500001, 1000, dtype=np.uint32)
    truth = 1000+2000*times.astype(float)/1e6
    observed = np.repeat(times[:, None], 3, axis=1)
    sequence = np.repeat(np.arange(1, len(times)+1, dtype=np.uint32)[:, None], 3, axis=1)
    output = native.replay(parameters, times, observed, sequence, np.ones((len(times), 3), np.uint8),
                           np.repeat(truth[:, None], 3, axis=1))
    tail = times > 300000
    return {'source': 'Independent analytic ramp: RPM=1000+2000*t; exact synthetic timestamps and identities',
            'tail_after_s': .3, 'maximum_tail_speed_error_rpm': float(np.max(abs(output['rpm'][tail, 0]-truth[tail]))),
            'maximum_tail_acceleration_error_rpm_s': float(np.max(abs(output['acceleration_rpm_s'][tail, 0]-2000))),
            'applies_to_real_trace_acceleration_accuracy': False}


def markdown_report(summary):
    params = summary['settings']['parameters']
    lines = ['# Offline RPM observer replay', '',
             'The real portable C observer is exercised independently for all three motors. These historical recordings establish compatibility and stale handling; they do not establish true acceleration accuracy or identify motor, voltage, load or torque coefficients.', '',
             f'Settings were frozen before evaluation. Tracking tau is {params["tracking_tau_s"]*1000:g} ms; acceleration decays after a {params["stale_after_us"]/1000:g} ms freshness limit with a {params["stale_acceleration_decay_tau_s"]*1000:g} ms time constant. Prediction stops at a {params["max_prediction_us"]/1000:g} ms horizon. RPM and acceleration limits are software guards.', '',
             'Blackbox ages are quantized and their within-loop phase is unmeasured. Replay uses interval-overlap cache identities and conservative representative timestamps. Phase allowances 0/250/500 us are sensitivity assumptions, not measured bounds. Equal-valued new receptions, including zero, can refresh when their plausible intervals separate; cached zeros retain their old source time.', '',
             'A cached representative timestamp can precede a later narrowed interval intersection. Keeping its original timestamp makes age more conservative; it does not establish an exact source age.', '',
             '| Record / phase allowance | Motor | Accepted proxy inputs | Not effectively fresh rows | Horizon exceeded rows | Maximum logged age (ms) |',
             '|---|---:|---:|---:|---:|---:|']
    for record in summary['records']:
        for case in record['cases']:
            for motor in case['motors']:
                lines.append(f'| {record["name"]} / {case["phase_allowance_us"]} us | {motor["logical_motor"]} | {motor["c_observer_accepted_proxy_observations"]} | {motor["not_effectively_fresh_rows"]} | {motor["horizon_exceeded_rows"]} | {motor["maximum_native_age_lower_us"]/1000:.1f} |')
    lines += ['', 'Counts above are proxy-adapter outcomes, not exact packet counts. Under the zero-phase sensitivity, the aborted gap contains a few contradictory or shifted ancient candidate intervals; they do not reset age to the current time. No observations are accepted in rows whose native age exceeds the prediction horizon.', '',
              'An independent exact-timestamp synthetic ramp checks the executable C derivative state. Its tail speed/acceleration errors are recorded in `rpm-replay-summary.json`; these errors do not apply to the historical bench trace.', '',
              'Plots show all three motors, predicted speed, estimated acceleration and stale gaps. The historical raw PB0 channel is unused. Host command times lack FC alignment, so no command-to-RPM delay is inferred.', '',
              'See `input-provenance.json`, `native-build.json`, `rpm-replay-summary.json` and `checksums.sha256` for pinned sources, compiler, raw/decoded data and outputs. No firmware integration, flashing or actuator action is performed.', '']
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--decoder', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    settings_hash = sha(SETTINGS)
    settings = json.loads(SETTINGS.read_text())
    inputs = decode_inputs(args.decoder, output/'decoded')
    (output/'input-provenance.json').write_text(json.dumps(inputs, indent=2)+'\n')
    native = NativeObserver(output/'build')
    (output/'native-build.json').write_text(json.dumps(native.provenance, indent=2)+'\n')
    summary = {'settings_sha256': settings_hash, 'settings': settings, 'native_build': native.provenance,
               'validation_boundary': settings['real_data_validation_scope'],
               'synthetic_exact_source_check': synthetic_report(native, settings['parameters']),
               'records': [evaluate_record(native, settings, record, output) for record in inputs['records']]}
    if sha(SETTINGS) != settings_hash or native.provenance['sources'] != {str(path.relative_to(ROOT)): sha(path) for path in SOURCES}:
        raise RuntimeError('Frozen settings/native source changed during replay; rerun before publishing artifacts')
    scripts = [path for path in Path(__file__).parent.glob('rpm_*') if path.is_file()]
    scripts.append(Path(__file__).with_name('test_rpm_replay.py'))
    summary['script_sha256'] = {path.name: sha(path) for path in scripts}
    (output/'rpm-replay-summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')
    (output/'README.md').write_text(markdown_report(summary))
    outputs = sorted(path for path in output.iterdir() if path.is_file() and path.name != 'checksums.sha256')
    (output/'checksums.sha256').write_text(''.join(f'{sha(path)}  {path.name}\n' for path in outputs))
    print(json.dumps({'output': str(output), 'records': [(record['name'], record['rows']) for record in summary['records']],
                      'synthetic': summary['synthetic_exact_source_check']}, indent=2))


if __name__ == '__main__':
    main()
