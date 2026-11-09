"""Search budget, confirmation and excitation separation without simulation."""
from pathlib import Path
import tempfile
import unittest

from autotune_yaw import (BOUNDS, Campaign, ORIGINAL, PAIRS_PER_VARIANT,
                          VARIANTS, coarse_candidates, key, local_candidate,
                          search_specification, training_case)
from check_comparison import cases


class SyntheticCampaign(Campaign):
    def __init__(self, directory):
        self.directory = directory
        self.candidates = {variant: [] for variant in VARIANTS}
        self.pair_count = {variant: 0 for variant in VARIANTS}
        self.attempts = []
        self.failed_confirmation_key = None

    def candidate(self, variant, gains, stage, existing=None):
        self.pair_count[variant] += 1
        self.attempts.append((variant, dict(gains), stage))
        if existing is None:
            existing = {'gains': gains, 'pairs': [], 'introduced_stage': stage}
            self.candidates[variant].append(existing)
        existing['pairs'].append(stage)
        target = (85, 20, 10) if variant == 'stock' else (45, 20, 10)
        score = sum(((a-b)/scale)**2 for a, b, scale in zip(key(gains), target, (100, 40, 50)))
        # A candidate that fails confirmation must not win by its initial score.
        if stage == 'confirm-2' and variant == 'modified' and self.failed_confirmation_key is None:
            self.failed_confirmation_key = key(gains)
        failed = (stage == 'confirm-2' and variant == 'modified' and
                  key(gains) == self.failed_confirmation_key)
        existing['score'] = {'feasible': existing.get('score', {}).get('feasible', True) and not failed,
                             'score': score+(1000 if failed else 0)}


class AutotuneSearchTest(unittest.TestCase):
    def test_rejected_coarse_start_does_not_skip_integral_exploration(self):
        class PoorInitialIntegral(SyntheticCampaign):
            def candidate(self, variant, gains, stage, existing=None):
                super().candidate(variant, gains, stage, existing)
                if stage == 'coarse':
                    item = self.candidates[variant][-1]
                    item['score']['feasible'] = False
                    item['score']['score'] += 1000
        with tempfile.TemporaryDirectory() as temp:
            campaign = PoorInitialIntegral(Path(temp))
            selected = campaign.search()
            self.assertEqual(campaign.pair_count, dict.fromkeys(VARIANTS, PAIRS_PER_VARIANT))
            for variant in VARIANTS:
                self.assertNotEqual(selected[variant]['i'], 10)
                self.assertTrue(all(not item['score']['feasible'] for item in campaign.candidates[variant]
                                    if item['introduced_stage'] == 'coarse'))

    def test_equal_budget_unique_exploration_and_repeated_finalists(self):
        with tempfile.TemporaryDirectory() as temp:
            campaign = SyntheticCampaign(Path(temp))
            chosen = campaign.search()
            self.assertEqual(campaign.pair_count, dict.fromkeys(VARIANTS, PAIRS_PER_VARIANT))
            for index in range(PAIRS_PER_VARIANT):
                actual = tuple(item[0] for item in campaign.attempts[2*index:2*index+2])
                self.assertEqual(actual, VARIANTS if index%2 == 0 else tuple(reversed(VARIANTS)))
            for variant in VARIANTS:
                points = campaign.candidates[variant]
                self.assertEqual(len(points), 18)
                self.assertEqual(len({key(item['gains']) for item in points}), 18)
                confirmed = [item for item in points if len(item['pairs']) == 3]
                self.assertEqual(len(confirmed), 3)
                self.assertTrue(any(item['gains'] == chosen[variant] and item['score']['feasible'] for item in confirmed))
                for item in points:
                    for axis, value in item['gains'].items():
                        self.assertLessEqual(BOUNDS[axis][0], value)
                        self.assertLessEqual(value, BOUNDS[axis][1])
            self.assertEqual(ORIGINAL, {'p': 50, 'i': 10, 'd': 20})

    def test_training_excitation_is_separate_from_evaluation(self):
        evaluation = {case['name']: case for case in cases(1345)}
        for direction in ('cw', 'ccw'):
            training = training_case(direction)
            self.assertEqual([stage['duration_s'] for stage in training['stages']], [1, 1, 1.5])
            self.assertNotEqual(training['stages'][1]['channels'][3],
                                evaluation['yaw_'+direction]['stages'][1]['channels'][3])
            self.assertTrue(all(stage['channels'][:3] == [1500, 1500, 1345] for stage in training['stages']))

    def test_duplicate_refinement_uses_unused_bounded_fallback(self):
        incumbent = {'p': 150, 'i': 40, 'd': 50}
        seen = {key(incumbent), (145, 40, 50)}
        proposal = local_candidate(incumbent, 0, seen)
        self.assertNotIn(key(proposal), seen)
        for axis, value in proposal.items():
            self.assertLessEqual(BOUNDS[axis][0], value)
            self.assertLessEqual(value, BOUNDS[axis][1])

    def test_protocol_declares_all_failures_count_and_no_holdout_selection(self):
        specification = search_specification()
        self.assertTrue(specification['selection_excludes_evaluation_results'])
        self.assertIn('No selective retry', specification['failure_policy'])
        self.assertEqual(len(coarse_candidates()), 9)
        self.assertEqual(coarse_candidates()[0], ORIGINAL)


if __name__ == '__main__': unittest.main()
