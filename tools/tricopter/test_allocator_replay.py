"""Actual-C allocation checked with cross products and independent convex bounds."""
import copy
from pathlib import Path
import tempfile
import unittest
import numpy as np

from allocator_cases import (load_protocol, physical_problem, virtual_problem, physical_requests, virtual_requests,
                             controls_from_effort, check_output)
from allocator_native import NativeAllocator
from allocator_reference import forward_geometry, geometric_matrix, sector_projection, linear_support_minimum
from allocator_replay import identification_demo


class AllocatorReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory(prefix='tricopter-allocator-')
        cls.native = NativeAllocator(Path(cls.temporary.name)/'build')
        cls.protocol = load_protocol()

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def test_direct_cross_product_forward_and_actual_held_angle(self):
        p = physical_problem(self.protocol)
        thrust, angle = np.asarray([[4.5, 4, 4.2]]), np.asarray([.1745329252])
        force, wrench = forward_geometry(p['positions'], thrust, angle, .01)
        out = self.native.run(self.protocol, thrust=thrust, angles=angle, reaction_yaw=[.01], use_reaction=[1], reaction_valid=[1])
        np.testing.assert_allclose(out['force_n'], force, atol=8e-7)
        np.testing.assert_allclose(out['achieved'], wrench, atol=8e-7)
        held = self.native.run(self.protocol, thrust=thrust, angles=[0], reaction_yaw=[.01], use_reaction=[1], reaction_valid=[1])
        self.assertGreater(np.linalg.norm(out['achieved']-held['achieved']), .1)
        self.assertEqual(held['force_n'][0, 1], 0)

    def test_physical_cases_have_primal_feasibility_and_independent_optimality_bound(self):
        target = physical_requests(self.protocol)
        reaction = np.asarray([case['reaction_yaw_nm'] for case in self.protocol['cases']])
        out = self.native.run(self.protocol, requests=target, reaction_yaw=reaction, use_reaction=np.ones(len(target)), reaction_valid=np.ones(len(target)))
        self.assertTrue(out['valid'].all()); self.assertTrue(out['converged'].all())
        p = physical_problem(self.protocol)
        for row, case in enumerate(self.protocol['cases']):
            with self.subTest(case=case['name']):
                cert = check_output(p, target[row], out['controls_n'][row], out['achieved'][row], [0, 0, 0, np.float32(reaction[row])])
                self.assertLessEqual(cert['global_objective_gap_upper_bound'], self.protocol['solver']['independent_gap_tolerance'])
                self.assertLess(max(cert['published_response_error_by_axis']), 2e-6)
        self.assertTrue(out['exact'][0]); self.assertFalse(out['exact'][-1])
        self.assertGreater(out['cost'][-1], 1)
        np.testing.assert_allclose(out['residual'], out['achieved']-out['requested'], atol=2e-6)
        np.testing.assert_allclose(out['omega_rad_s']**2*np.asarray(self.protocol['thrust_coefficients_n_per_rad_s_squared']), out['thrust_n'], rtol=3e-7, atol=1e-7)

    def test_reaction_yaw_applied_exactly_once_and_unknown_reaction_rejected(self):
        target = np.asarray([[12, 0, 0, .2]], np.float32)
        with_reaction = self.native.run(self.protocol, requests=target, reaction_yaw=[.03], use_reaction=[1], reaction_valid=[1])
        plain = self.native.run(self.protocol, requests=target-np.asarray([0, 0, 0, .03]))
        np.testing.assert_allclose(with_reaction['controls_n'], plain['controls_n'], atol=1e-6)
        invalid = self.native.run(self.protocol, requests=target, reaction_yaw=[.03], use_reaction=[1], reaction_valid=[0])
        self.assertFalse(invalid['valid'][0]); self.assertNotEqual(invalid['status'][0]&16, 0)

    def test_direct_map_requires_no_geometry_kt_or_physical_outputs(self):
        config = copy.deepcopy(self.protocol['direct_effectiveness'])
        target = virtual_requests(config)
        out = self.native.virtual_run(config, requests=target)
        self.assertTrue(out['valid'].all()); self.assertTrue(out['converged'].all())
        self.assertNotIn('force_n', out); self.assertNotIn('mechanical_rpm', out)
        p = virtual_problem(config)
        for row in range(len(target)):
            cert = check_output(p, target[row], out['controls'][row], out['achieved'][row])
            self.assertLessEqual(cert['global_objective_gap_upper_bound'], config['solver']['independent_gap_tolerance'])
        np.testing.assert_allclose(out['controls'][0], controls_from_effort(config['cases'][0]['effort'], .1), atol=1e-6)

    def test_response_units_and_uniform_effort_units_are_invariant(self):
        config = copy.deepcopy(self.protocol['direct_effectiveness'])
        target = virtual_requests(config)[[0, -1]]
        original = self.native.virtual_run(config, requests=target)
        factors = np.asarray([2, .5, 3, 4])
        changed = copy.deepcopy(config)
        changed['effectiveness_matrix'] = (factors[:, None]*np.asarray(config['effectiveness_matrix'])).tolist()
        changed['response_scales'] = (factors*np.asarray(config['response_scales'])).tolist()
        scaled = self.native.virtual_run(changed, requests=target*factors)
        np.testing.assert_allclose(scaled['controls'], original['controls'], atol=3e-6)
        changed = copy.deepcopy(config)
        changed['effectiveness_matrix'] = (np.asarray(config['effectiveness_matrix'])/2).tolist()
        changed['maximum_effort'] = (2*np.asarray(config['maximum_effort'])).tolist()
        changed['solver']['low_tail_effort'] *= 2
        changed['solver']['projected_gradient_tolerance'] /= 2
        scaled = self.native.virtual_run(changed, requests=target)
        np.testing.assert_allclose(scaled['controls']/2, original['controls'], atol=3e-6)
        np.testing.assert_allclose(scaled['achieved'], original['achieved'], atol=3e-6)

    def test_one_iteration_exposes_nonconvergence_and_degenerate_maps_are_invalid(self):
        config = copy.deepcopy(self.protocol['direct_effectiveness']); config['solver']['max_iterations'] = 1
        out = self.native.virtual_run(config, requests=[[1.5, 1, -.3, .6]])
        self.assertTrue(out['valid'][0]); self.assertFalse(out['converged'][0]); self.assertNotEqual(out['status'][0]&64, 0)
        config['effectiveness_matrix'] = np.zeros((4, 4)).tolist()
        out = self.native.virtual_run(config, requests=[[1, 0, 0, 0]])
        self.assertFalse(out['valid'][0]); self.assertNotEqual(out['status'][0]&4, 0)
        bad = copy.deepcopy(self.protocol); bad['geometry_m']['rotor_positions'] = [[0, 0, 0]]*3
        self.assertFalse(self.native.run(bad, requests=[[1, 0, 0, 0]])['valid'][0])

    def test_independent_support_bound_matches_dense_tail_search(self):
        gradient = np.asarray([.3, -.7, -.2, .9]); maximum = np.asarray([.8, 1, .9]); low, high = -.43, .61
        optimum = linear_support_minimum(gradient, maximum, low, high)
        theta = np.linspace(low, high, 20001)
        dense_tail = np.minimum(0, maximum[0]*(gradient[2]*np.cos(theta)+gradient[3]*np.sin(theta))).min()
        dense_value = min(0, gradient[0]*maximum[1])+min(0, gradient[1]*maximum[2])+dense_tail
        self.assertLessEqual(gradient@optimum, dense_value+1e-12)
        self.assertLess(dense_value-gradient@optimum, 1e-8)
        point = sector_projection([-.5, 1], .8, low, high)
        np.testing.assert_allclose(sector_projection(point, .8, low, high), point, atol=1e-14)

    def test_noiseless_synthetic_effectiveness_identification_reconstructs_heldout_requests(self):
        result = identification_demo(self.native, self.protocol['direct_effectiveness'])
        self.assertEqual(result['training_design_rank'], 4)
        self.assertLess(result['maximum_matrix_error'], 1e-12)
        self.assertLess(result['maximum_heldout_response_error'], 1e-5)
        self.assertTrue(result['synthetic_only'])

    def test_runtime_shape_finite_and_metadata_guards_before_ctypes(self):
        for requests in ([], [[1, 2]], [[float('nan'), 0, 0, 0]], [[float('inf'), 0, 0, 0]]):
            with self.assertRaises(ValueError): self.native.run(self.protocol, requests=requests)
            with self.assertRaises(ValueError): self.native.virtual_run(self.protocol['direct_effectiveness'], requests=requests)
        with self.assertRaises(ValueError): self.native.run(self.protocol, requests=[[1, 0, 0, 0]], use_reaction=[2])
        with self.assertRaises(ValueError): self.native.virtual_run(self.protocol['direct_effectiveness'], effort=[[.1]*3], directions=[])
        config = copy.deepcopy(self.protocol['direct_effectiveness']); config['effectiveness_matrix'] = [[1, 2]]
        with self.assertRaises(ValueError): self.native.virtual_run(config, requests=[[1, 0, 0, 0]])


if __name__ == '__main__': unittest.main()
