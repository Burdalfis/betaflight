#!/usr/bin/env python3
"""Bounded independent yaw tuning in the shared Gazebo plant; never uses a FC."""
from __future__ import annotations

import argparse
import copy
import csv
import hashlib
import json
from pathlib import Path
import shutil

from check_comparison import Trial, analyze, cases, sha
from check_comparison_fresh import (COMMON_KEYS, VARIANT_KEYS, freeze_sources,
                                    source_inputs, verify_case, verify_frozen_sources)
from check_flight import PILOT
from run import build_plugin
from runtime import ARTIFACTS, HERE, SessionLock, fresh_run_directory
from tune_metrics import (DEFAULT_LIMITS, DEFAULT_WEIGHTS, failed_trial,
                          score_candidate, score_trial)
from tune_profiles import overlay_text, shared_msp_readback, verify_profile

VARIANTS = ('stock', 'modified')
ORIGINAL = {'p': 50, 'i': 10, 'd': 20}
BOUNDS = {'p': (15, 150), 'i': (3, 40), 'd': (0, 50)}
SCORING_LIMITS = dict(DEFAULT_LIMITS, maximum_rc_send_gap_s=.05)
PAIRS_PER_VARIANT = 24
EXTRA_SOURCES = ('autotune_yaw.py', 'tune_metrics.py', 'tune_profiles.py')


class IdentityError(RuntimeError):
    """A configuration/model mismatch invalidates the experiment, not a PID."""


def write_json(path, value):
    temporary = path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')
    temporary.replace(path)


def fingerprint(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True).encode()).hexdigest()


def inputs():
    return dict(source_inputs(), **{name: HERE/name for name in EXTRA_SOURCES})


def key(gains): return tuple(gains[axis] for axis in ('p', 'i', 'd'))


def coarse_candidates():
    grid = [dict(p=p, i=10, d=d) for p in (25, 50, 100) for d in (0, 20, 40)]
    return [dict(ORIGINAL)]+[point for point in grid if point != ORIGINAL]


def training_case(direction, hover=1345):
    yaw = 1333 if direction == 'cw' else 1667
    if direction not in ('cw', 'ccw'): raise ValueError('Unknown training direction')
    return {'name': 'yaw_'+direction, 'kind': 'yaw', 'stages': [
        {'name': 'baseline', 'duration_s': 1., 'channels': [1500, 1500, hover, 1500]},
        {'name': 'pulse', 'duration_s': 1., 'channels': [1500, 1500, hover, yaw]},
        {'name': 'recovery', 'duration_s': 1.5, 'channels': [1500, 1500, hover, 1500]}]}


def local_candidate(incumbent, move, seen):
    """Three declared bounded refinements; deterministic fallback on duplicates."""
    wanted = dict(incumbent)
    if move == 0: wanted['p'] = round(wanted['p']*1.25)
    elif move == 1: wanted['p'] = round(wanted['p']*.75)
    elif move == 2: wanted['d'] += -10 if wanted['d'] >= 20 else 10
    else: raise ValueError('Unknown refinement move')
    alternatives = [wanted]
    for axis, delta in [('p', 5), ('p', -5), ('i', 5), ('i', -5), ('d', 5), ('d', -5)]:
        point = dict(incumbent); point[axis] += delta; alternatives.append(point)
    for point in alternatives:
        point = {axis: max(BOUNDS[axis][0], min(BOUNDS[axis][1], value))
                 for axis, value in point.items()}
        if key(point) not in seen: return point
    raise RuntimeError('No unused bounded refinement available')


def search_specification():
    return {'parameters': ['p_yaw', 'i_yaw', 'd_yaw'], 'bounds': BOUNDS,
        'feedforward_yaw': 0, 'feedforward_yaw_hold_gain': 0,
        'roll_pitch_gains_filters_limits_and_model_unchanged': True,
        'candidate_pairs_per_variant': PAIRS_PER_VARIANT,
        'fresh_world_eeprom_per_direction': True,
        'coarse': coarse_candidates(),
        'integral_stage': 'Top two feasible coarse points, filling missing seed slots by lowest penalized score, each with I=5,20,40',
        'local_stage': 'Three sequential incumbent refinements: P*1.25, P*.75, then D-10 if D>=20 else D+10; bounded and deduplicated',
        'duplicate_fallback': [['p', 5], ['p', -5], ['i', 5], ['i', -5], ['d', 5], ['d', -5]],
        'confirmation': 'Top three feasible explored candidates, filling missing slots by lowest penalized score, receive two additional fresh CW/CCW pairs; only fully feasible candidates may win using all three pairs',
        'failure_policy': 'Every failed trial is retained, penalized and counts within the same budget. No selective retry.',
        'direction_order': 'Alternates by pair number and variant; variant order counterbalanced at each pair index',
        'weights': DEFAULT_WEIGHTS, 'limits': SCORING_LIMITS,
        'training_cases': [training_case(direction) for direction in ('cw', 'ccw')],
        'evaluation_cases': cases(1345),
        'selection_excludes_evaluation_results': True,
        'claim_boundary': 'Finite simulation search, not a global optimum or physical flight PID recommendation'}


class ProfileTrial(Trial):
    def __init__(self, *args, gains, **kwargs):
        super().__init__(*args, **kwargs); self.gains = gains; self.profile_verified = False

    def recapture(self, first=False):
        if not self.profile_verified:
            try:
                self.record['yaw_profile_verification'] = verify_profile(
                    self.record, self.gains, directory=self.directory)
            except (ValueError, AssertionError, OSError) as error:
                raise IdentityError(str(error)) from error
            self.profile_verified = True
        return super().recapture(first)


class Campaign:
    def __init__(self, baseline, directory):
        self.directory = directory
        self.baseline_path = baseline
        self.baseline = json.loads((baseline/'comparison-summary.json').read_text())
        if not self.baseline['pass'] or self.baseline['protocol']['repetitions'] != 3:
            raise ValueError('A completed N3 baseline is required')
        if sum(len(run['cases']) for run in self.baseline['runs']) != 42:
            raise ValueError('Baseline must contain the complete 42-flight suite')
        for name, expected in self.baseline['protocol']['source_sha256'].items():
            actual = source_inputs().get(name)
            if actual is None or sha(actual) != expected:
                raise ValueError('Baseline execution inputs changed: '+name)
        self.frozen = freeze_sources(directory, inputs())
        self.specification = search_specification()
        self.specification.update(source_sha256=self.frozen,
            baseline_directory=str(baseline),
            baseline_summary_sha256=sha(baseline/'comparison-summary.json'))
        self.specification['sha256'] = fingerprint(self.specification)
        write_json(directory/'search-protocol.json', self.specification)
        self.report = {'pass': False, 'directory': str(directory), 'phases': {},
                       'search_protocol_sha256': self.specification['sha256']}
        self.candidates = {variant: [] for variant in VARIANTS}
        self.pair_count = {variant: 0 for variant in VARIANTS}
        self.control_reference = None
        self.binary_reference = {run['variant']: run['binary_sha256'] for run in self.baseline['runs']}
        self.baseline_shared = shared_msp_readback(self.baseline['runs'][0])['readback']
        self.plugin = None; self.plugin_sha = None
        self.checkpoint()

    def checkpoint(self): write_json(self.directory/'autotune-summary.json', self.report)

    def guard(self):
        verify_frozen_sources(self.directory, self.frozen, inputs())
        if self.plugin is not None and sha(self.plugin) != self.plugin_sha:
            raise AssertionError('Frozen plugin changed')

    def build(self):
        built = build_plugin(ARTIFACTS, 4); self.guard()
        self.plugin = self.directory/'plugin'/built.name; self.plugin.parent.mkdir()
        shutil.copy2(built, self.plugin); self.plugin_sha = sha(self.plugin)
        if self.plugin_sha != self.baseline['plugin_sha256']:
            raise AssertionError('Plugin binary differs from the supplied baseline')
        shutil.copy2(built.parent/'build-manifest.json', self.plugin.parent/'build-manifest.json')
        self.report['plugin_sha256'] = self.plugin_sha

    def overlay(self, gains):
        directory = self.directory/'profiles'; directory.mkdir(exist_ok=True)
        path = directory/('P%03d-I%03d-D%03d-F0.cli'%key(gains))
        data = overlay_text(gains).encode()
        if path.exists() and path.read_bytes() != data: raise AssertionError('Overlay mutated')
        if not path.exists(): path.write_bytes(data)
        return path

    def protocol(self, definitions, gains, repetitions, purpose):
        result = copy.deepcopy(self.baseline['protocol'])
        result.pop('sha256', None)
        result.update(cases=definitions, selected_cases=[case['name'] for case in definitions],
            repetitions=repetitions, hover_throttle_us=1345,
            variant_order=[variant for index in range(repetitions) for variant in
                (VARIANTS if index%2 == 0 else tuple(reversed(VARIANTS)))],
            source_sha256=self.frozen, purpose=purpose,
            source_snapshot_directory=str(self.directory/'source'),
            plugin_snapshot_directory=str(self.directory/'plugin'),
            search_protocol_sha256=self.specification['sha256'],
            intentionally_varied_yaw_gains=gains,
            feedforward_yaw=0, feedforward_yaw_hold_gain=0,
            common_msp_mask={'112': [[6, 9]], '94': [[36, 38]]})
        result['sha256'] = fingerprint(result)
        return result

    def one(self, path, variant, gains, definition, protocol):
        self.guard()
        single = copy.deepcopy(protocol); single['cases'] = [copy.deepcopy(definition)]
        trial = ProfileTrial(path, variant, self.plugin,
            self.directory/'source/gazebo/worlds/tricopter.sdf', single,
            gains=gains, extra_config=self.overlay(gains))
        trial.record.update(selected_case=definition['name'], fresh_case_reset=True,
                            requested_yaw_gains=gains)
        try:
            trial.execute(); record = analyze(path); verify_case(record, protocol)
            try:
                record['yaw_profile_verification'] = verify_profile(record, gains, directory=path)
            except (ValueError, AssertionError, OSError) as error:
                raise IdentityError(str(error)) from error
            shared = shared_msp_readback(record)
            if shared['readback'] != self.baseline_shared:
                raise IdentityError('Unexpected non-yaw setting change from baseline')
            record['shared_msp_readback'] = shared['readback']
            record['shared_msp_mask'] = shared['masked_byte_ranges']
            control = {name: record[name] for name in COMMON_KEYS if name != 'common_msp_readback'}
            control['shared_msp_readback'] = shared['readback']
            if self.control_reference is None: self.control_reference = control
            if control != self.control_reference: raise IdentityError('Non-yaw control/model settings changed')
            if record['binary_sha256'] != self.binary_reference[variant]:
                raise IdentityError('Firmware binary changed')
            write_json(path/'comparison-metrics.json', record)
            return record, None
        except IdentityError:
            raise
        except Exception as error:
            path.mkdir(parents=True, exist_ok=True)
            write_json(path/'evaluation-failure.json', {'error': repr(error),
                'variant': variant, 'gains': gains, 'case': definition['name']})
            return None, repr(error)
        finally: self.guard()

    def cohort(self, name, gains):
        directory = self.directory/name; directory.mkdir()
        protocol = self.protocol(cases(1345), gains, 3, name)
        write_json(directory/'protocol.json', protocol)
        summary = {'pass': False, 'directory': str(directory), 'protocol': protocol,
                   'fresh_case_reset': True, 'runs': []}
        self.report['phases'][name] = str(directory); self.checkpoint()
        try:
            for index, variant in enumerate(protocol['variant_order']):
                group_name = f'{index+1:02d}-{variant}'; group_path = directory/group_name; group_path.mkdir()
                group = {'variant': variant, 'pass': False, 'fresh_case_reset': True,
                    'group_directory': group_name, 'case_directory_base': 'group_directory',
                    'case_timestamps_are_case_local': True, 'case_directories': {},
                    'case_final_states': {}, 'protocol_sha256': protocol['sha256'], 'cases': []}
                summary['runs'].append(group)
                for definition in protocol['cases']:
                    case_name = definition['name']
                    record, error = self.one(group_path/case_name, variant, gains[variant], definition, protocol)
                    if error: raise RuntimeError(f'{name}/{group_name}/{case_name}: {error}')
                    for field in COMMON_KEYS+VARIANT_KEYS: group[field] = record[field]
                    group['shared_msp_readback'] = record['shared_msp_readback']
                    group['shared_msp_mask'] = record['shared_msp_mask']
                    group['requested_yaw_gains'] = gains[variant]
                    group['case_directories'][case_name] = case_name
                    group['case_final_states'][case_name] = {
                        'final_status': record['final_status'],
                        'all_owned_processes_stopped': record['all_owned_processes_stopped'],
                        'maximum_rc_send_gap_s': record['maximum_rc_send_gap_s']}
                    group['cases'].append(record['cases'][0])
                    write_json(directory/'comparison-summary.json', summary)
                group.update(pass_=True)
                del group['pass_']; group['pass'] = True
                group['final_status'] = {'armed': False, 'all_fresh_cases_disarmed': True}
                group['all_owned_processes_stopped'] = True
                group['maximum_rc_send_gap_s'] = max(value['maximum_rc_send_gap_s'] for value in group['case_final_states'].values())
            summary['pass'] = True
        except Exception as error:
            summary['error'] = repr(error); raise
        finally: write_json(directory/'comparison-summary.json', summary)
        print(f'COHORT COMPLETE {name}: 42 fresh flights', flush=True)
        return summary

    def ranked(self, variant, feasible_only=True):
        eligible = [item for item in self.candidates[variant]
                    if item['score']['feasible'] or not feasible_only]
        return sorted(eligible, key=lambda item: (item['score']['score'], key(item['gains'])))

    def seeds(self, variant, count):
        # A poor starting I gain must not prevent exploring a different I.
        # Failed seeds remain failed; a new gain tuple is a new candidate.
        good = self.ranked(variant)
        others = [item for item in self.ranked(variant, False) if item not in good]
        return (good+others)[:count]

    def candidate(self, variant, gains, stage, existing=None):
        index = self.pair_count[variant]; self.pair_count[variant] += 1
        directory = self.directory/'search'/variant/f'{index+1:02d}-{stage}'
        directory.mkdir(parents=True)
        if existing is None:
            existing = {'gains': gains, 'trials': [], 'pairs': [], 'introduced_stage': stage}
            self.candidates[variant].append(existing)
        definitions = [training_case(direction) for direction in
            (('cw', 'ccw') if (index+VARIANTS.index(variant))%2 == 0 else ('ccw', 'cw'))]
        protocol = self.protocol(definitions, {variant: gains}, 1, 'training-'+stage)
        write_json(directory/'protocol.json', protocol)
        pair = []
        for definition in definitions:
            direction = definition['name'][4:]
            path = directory/definition['name']
            record, error = self.one(path, variant, gains, definition, protocol)
            if error: scored = failed_trial(direction, error)
            else:
                with (path/'plant.csv').open() as stream:
                    rows = [{name: float(value) for name, value in row.items()} for row in csv.DictReader(stream)]
                scored = score_trial(rows, record['cases'][0],
                    health={'pass': record['pass'], 'all_owned_processes_stopped': record['all_owned_processes_stopped']},
                    limits=SCORING_LIMITS)
            scored.update(directory=str(path), gains=gains)
            pair.append(scored); existing['trials'].append(scored)
            write_json(directory/'pair-progress.json', pair)
        existing['pairs'].append(str(directory))
        existing['score'] = score_candidate(existing['trials'], expected_repetitions=len(existing['pairs']))
        write_json(self.directory/'search-progress.json', {
            'pairs_per_variant': self.pair_count, 'candidates': self.candidates})
        print(f'TUNE {variant} pair {index+1}/{PAIRS_PER_VARIANT} {stage} {gains} '
              f'score={existing["score"]["score"]:.6f} feasible={existing["score"]["feasible"]}', flush=True)

    def batch(self, candidates, stage):
        for index in range(len(candidates['stock'])):
            pair_index = self.pair_count['stock']
            for variant in (VARIANTS if pair_index%2 == 0 else tuple(reversed(VARIANTS))):
                self.candidate(variant, candidates[variant][index], stage)

    def search(self):
        self.batch({variant: coarse_candidates() for variant in VARIANTS}, 'coarse')
        integral = {}
        for variant in VARIANTS:
            seeds = self.seeds(variant, 2)
            if len(seeds) < 2: raise RuntimeError('Insufficient feasible coarse candidates for '+variant)
            integral[variant] = [dict(seed['gains'], i=i) for seed in seeds for i in (5, 20, 40)]
        self.batch(integral, 'integral')
        for move in range(3):
            pair_index = self.pair_count['stock']
            for variant in (VARIANTS if pair_index%2 == 0 else tuple(reversed(VARIANTS))):
                incumbent = self.seeds(variant, 1)[0]['gains']
                seen = {key(item['gains']) for item in self.candidates[variant]}
                self.candidate(variant, local_candidate(incumbent, move, seen), 'local')
        finalists = {variant: self.seeds(variant, 3) for variant in VARIANTS}
        if any(len(values) != 3 for values in finalists.values()):
            raise RuntimeError('Insufficient feasible candidates for confirmation')
        write_json(self.directory/'confirmation-selection.json', {
            variant: [item['gains'] for item in values] for variant, values in finalists.items()})
        for finalist in range(3):
            for repeat in range(2):
                pair_index = self.pair_count['stock']
                for variant in (VARIANTS if pair_index%2 == 0 else tuple(reversed(VARIANTS))):
                    item = finalists[variant][finalist]
                    self.candidate(variant, item['gains'], f'confirm-{repeat+2}', existing=item)
        if any(count != PAIRS_PER_VARIANT for count in self.pair_count.values()):
            raise AssertionError('Unequal search budget')
        selected = {}
        for variant, values in finalists.items():
            feasible = [item for item in values if item['score']['feasible']]
            if not feasible: raise RuntimeError('No finalist passed all confirmation flights')
            selected[variant] = min(feasible, key=lambda item: (item['score']['score'], key(item['gains'])))
        result = {'pass': True, 'selected': selected, 'pairs_per_variant': self.pair_count,
                  'all_candidates': self.candidates, 'selection_frozen_before_evaluation': True}
        write_json(self.directory/'selection.json', result)
        return {variant: result['selected'][variant]['gains'] for variant in VARIANTS}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--label', default='yaw-autotune')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    if args.dry_run:
        print(json.dumps(search_specification(), indent=2)); return
    PILOT['max_angle_deg'] = 12.
    directory = fresh_run_directory(ARTIFACTS, 'both', args.label)
    campaign = Campaign(args.baseline.resolve(), directory)
    print(f'Yaw autotune campaign: {directory}', flush=True)
    try:
        with SessionLock():
            campaign.build()
            campaign.cohort('f0-reference', {variant: dict(ORIGINAL) for variant in VARIANTS})
            selected = campaign.search(); campaign.report['selected_gains'] = selected; campaign.checkpoint()
            campaign.cohort('tuned-evaluation', selected)
            campaign.report['pass'] = True
    except Exception as error:
        campaign.report['error'] = repr(error); raise
    finally: campaign.checkpoint()
    print('Autotune and evaluation complete; all owned processes stopped.', flush=True)


if __name__ == '__main__': main()
