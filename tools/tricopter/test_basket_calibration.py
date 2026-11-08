"""Measured-angle export, paired regression and held-out reversal tests."""

import copy
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

import basket_calibration as calibration


def point(command, count, angle, index=0):
    return {"command_us": command, "corrected_counts": count, "basket_roll_deg": angle, "plateau_id": index}


def sample_branches():
    return {"increasing_pwm": [point(1700, 1000., 20.), point(1500, 1500., 0.), point(1100, 2000., -20.)],
            "decreasing_pwm": [point(1680, 1100., 21.), point(1480, 1400., 9.), point(1080, 2100., -19.)]}


def profile_from_branches(branches):
    return {"schema_version": 1, "kind": "tricopter_basket_measured_angle_mean_map",
            "source_position": "signed PB0-PC0 only", "angle_units": "radians", "operator_defined_reference": True,
            **calibration.mean_count_map(branches)}


NORMAL = np.array([.1, -.05, np.sqrt(.9875)])


def gravity(angle):
    n, x, y = calibration.projection_basis(NORMAL)
    angle = np.radians(angle)
    return (.05 * n + np.sqrt(1 - .05**2) * (np.cos(angle) * x - np.sin(angle) * y)).tolist()


def observation(index, command, approach, count, angle):
    return {"plateau_id": index, "target_us": command, "approach": approach,
            "fc": {"fresh_matching_rows": 20, "corrected_pb0_minus_pc0_counts": {"mean": count}},
            "adxl": {"paired_observation_qualified": True, "settled_mean_screen_pass": True,
                     "corrected_mean_g": gravity(angle)}}


def analysis_summary(points, raw_hash="2" * 64, fc_hash="3" * 64):
    return {"all_settled_windows_pass": True, "rotation_plane": {"identified": True, "normal_sensor_axes": NORMAL.tolist()},
            "plateaus": points,
            "provenance": {"servo_restored_exact": True, "input_sha256": {
                "adxl/capture.bin": raw_hash, "fc-samples.csv": fc_hash, "calibration-profile.json": "1" * 64}}}


def write_summary_inputs(folder):
    directions = [(1420, "initial")] + [(x, "decreasing") for x in (1400, 1300, 1200, 1100)]
    directions += [(x, "increasing") for x in (1200, 1300, 1400, 1500, 1600, 1700)]
    directions += [(x, "decreasing") for x in (1600, 1500, 1420)]
    points = []
    for i, (command, direction) in enumerate(directions):
        offset = .5 if direction == "increasing" else -.5
        count = 1500 - 1.05 * (command - 1510) + 5 * ((command - 1510) / 200)**2 + offset
        angle = .12 * (command - 1510) + .3 + offset
        points.append(observation(i, command, direction, count, angle))
    analysis = folder / "training-analysis.json"
    analysis.write_text(json.dumps(analysis_summary(points)))
    mounting = folder / "mounting.json"
    mounting.write_text(json.dumps({"motor_present": False, "airframe_roll_level_operator_report": True,
                                    "zero_definition": "operator-aligned flat basket"}))
    zero = folder / "zero.json"
    zero.write_text(json.dumps({"operator_alignment_reference": True, "configuration_restored_exact": True,
        "projection": {"normal_sensor_axes": NORMAL.tolist()},
        "branches": {name: {"interpolated_command_us": value} for name, value in
                     (("increasing_pwm", 1518.), ("decreasing_pwm", 1504.))},
        "provenance": {"hinge_analysis_sha256": calibration.sha256(analysis)}}))
    return analysis, mounting, zero


class MappingTests(unittest.TestCase):
    def test_union_knots_common_bounds_units_uncertainty_and_no_extrapolation(self):
        profile = profile_from_branches(sample_branches())
        self.assertEqual(profile["count_bounds"], [1100., 2000.])
        self.assertEqual([k["corrected_counts"] for k in profile["knots"]], [1100., 1400., 1500., 2000.])
        angle, separation = calibration.interpolate(profile, 1500.)
        self.assertAlmostEqual(np.degrees(angle), 2.5)
        self.assertAlmostEqual(np.degrees(separation), 2.5)
        self.assertTrue(np.all(np.diff([k["angle_rad"] for k in profile["knots"]]) < 0))
        for count in (1000., 2001., np.nan):
            with self.assertRaises(calibration.CalibrationError):
                calibration.interpolate(profile, count)
        with self.assertRaisesRegex(calibration.CalibrationError, "exceeds"):
            calibration.mean_count_map(sample_branches(), max_knots=3)

    def test_nonmonotonic_or_wrong_source_profiles_fail(self):
        good = profile_from_branches(sample_branches())
        for mutation in ("source", "negative_span", "nonmonotonic", "bounds"):
            profile = copy.deepcopy(good)
            if mutation == "source":
                profile["source_position"] = "raw PB0 fallback"
            elif mutation == "negative_span":
                profile["knots"][0]["half_branch_separation_rad"] = -.1
            elif mutation == "nonmonotonic":
                profile["knots"][1]["angle_rad"] = profile["knots"][0]["angle_rad"]
            else:
                profile["count_bounds"][0] -= 1
            with self.subTest(mutation=mutation), self.assertRaises(calibration.CalibrationError):
                calibration.validate_profile(profile)

    def test_projected_x_mechanical_sign_and_pitch_invariance(self):
        for angle in (-40., -5., 0., 17., 45.):
            self.assertAlmostEqual(float(calibration.basket_roll(gravity(angle), NORMAL)), angle, places=11)
        with self.assertRaisesRegex(calibration.CalibrationError, "weak projection"):
            calibration.projection_basis([1., 0, 0])
        with self.assertRaisesRegex(calibration.CalibrationError, "Weak gravity"):
            calibration.basket_roll(NORMAL, NORMAL)


class RegressionTests(unittest.TestCase):
    def test_equal_branch_pairing_not_sample_pooling_and_free_vs_forced_zero(self):
        middle, slope, intercept = 1511.75, .12, .3
        branches = {"increasing_pwm": [], "decreasing_pwm": []}
        commands = list(range(1120, 1681, 20))
        for i, command in enumerate(commands):
            mean = slope * (command - middle) + intercept
            branches["increasing_pwm"].append(point(command, 2000 - command, mean + 1, i))
            branches["decreasing_pwm"].append(point(command, 2001 - command, mean - 1, i + 100))
        branches["increasing_pwm"].append(point(1700, 300, 999., 999))
        branches["decreasing_pwm"].append(point(1100, 900, -999., 998))
        report = calibration.paired_command_regression(branches, middle)
        self.assertEqual(report["paired_command_points"], 29)
        self.assertEqual(report["branch_observations"], 58)
        self.assertEqual(report["unpaired_commands_excluded_us"], [1100, 1700])
        self.assertAlmostEqual(report["free_intercept"]["slope_deg_per_us"], slope, places=12)
        self.assertAlmostEqual(report["free_intercept"]["intercept_deg"], intercept, places=12)
        self.assertAlmostEqual(report["free_intercept"]["fitted_zero_crossing_us"], middle - intercept / slope)
        self.assertAlmostEqual(report["pearson_r"], 1.)
        self.assertAlmostEqual(report["maximum_full_branch_separation_deg"], 2.)
        self.assertEqual(report["forced_physical_zero"]["intercept_deg"], 0.)
        self.assertGreater(report["forced_physical_zero"]["rms"], .1)
        count_fit = calibration.paired_count_regression(report["paired_means"])
        self.assertEqual(count_fit["paired_command_points"], 29)
        self.assertAlmostEqual(count_fit["slope_deg_per_count"], -.12, places=12)
        self.assertAlmostEqual(count_fit["pearson_r"], -1.)
        self.assertAlmostEqual(count_fit["fitted_zero_count"], 2000.5 - (middle - intercept / slope))
        self.assertLess(count_fit["rms"], 1e-12)
        branches["decreasing_pwm"].append(copy.deepcopy(branches["decreasing_pwm"][0]))
        with self.assertRaisesRegex(calibration.CalibrationError, "duplicate"):
            calibration.paired_command_regression(branches, middle)


class SummaryWorkflowTests(unittest.TestCase):
    def test_summary_export_tracks_trust_boundary_and_rejects_duplicates(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            analysis, mounting, zero = write_summary_inputs(folder)
            with self.assertRaisesRegex(calibration.CalibrationError, "hash mismatch"):
                calibration.export_profile(analysis, mounting, zero)
            profile = calibration.export_profile(analysis, mounting, zero, summary_only=True)
            self.assertTrue(profile["operator_defined_reference"])
            self.assertFalse(profile["motor_present_during_characterization"])
            self.assertFalse(profile["reference"]["curve_warp_or_live_refit_applied"])
            self.assertIn("not reverified", profile["provenance"]["source_mode"])
            self.assertEqual(profile["regression"]["paired_command_points"], 5)
            altered = json.loads(analysis.read_text())
            altered["plateaus"].append(copy.deepcopy(altered["plateaus"][1]))
            with self.assertRaisesRegex(calibration.CalibrationError, "Duplicate"):
                calibration.extract_branches(altered, NORMAL)

    def test_heldout_minor_loops_stay_heldout_and_compare_bounded_models(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            analysis, mounting, zero = write_summary_inputs(folder)
            profile = calibration.export_profile(analysis, mounting, zero, summary_only=True)
            original_profile = json.dumps(profile, sort_keys=True)
            targets = [1400, 1450, 1430, 1470, 1440, 1450]
            errors = [.1, -.2, .3, -.4, .2, -.1]
            low, high = profile["count_bounds"]
            counts = np.linspace(low + 10, high - 10, len(targets))
            rows = []
            for i, (target, count, error) in enumerate(zip(targets, counts, errors)):
                prediction = np.degrees(calibration.interpolate(profile, count)[0])
                approach = "initial" if not i else "increasing" if target > targets[i - 1] else "decreasing"
                rows.append(observation(i, target, approach, float(count), prediction - error))
            heldout = folder / "heldout-analysis.json"
            heldout.write_text(json.dumps(analysis_summary(rows, "4" * 64, "5" * 64)))
            result = calibration.validate_reversals(profile, heldout, summary_only=True)
            self.assertFalse(result["fit_updated"])
            self.assertEqual(result["covered_points"], 6)
            self.assertAlmostEqual(result["mean_map_error_deg"]["rms"], np.sqrt(np.mean(np.square(errors))))
            self.assertAlmostEqual(result["mean_map_error_deg"]["max_abs"], .4)
            self.assertEqual(result["matched_model_comparison"]["points"], 6)
            self.assertFalse(result["matched_model_comparison"]["command_is_measured_position"])
            self.assertEqual(json.dumps(profile, sort_keys=True), original_profile)
            self.assertGreater(result["by_context"]["direction_reversal"]["count"], 0)
            failed = analysis_summary(copy.deepcopy(rows), "4" * 64, "5" * 64)
            failed["all_settled_windows_pass"] = False
            failed["plateaus"][3]["adxl"]["settled_mean_screen_pass"] = False
            heldout.write_text(json.dumps(failed))
            result = calibration.validate_reversals(profile, heldout, summary_only=True)
            self.assertEqual(result["input_points"], 6)
            self.assertEqual(result["covered_points"], 5)
            self.assertEqual(result["excluded_unqualified_plateau_ids"], [3])
            self.assertEqual(result["out_of_domain_points"], 0)
            self.assertFalse(result["qualified_mask"][3])
            self.assertTrue(result["points"][3]["errors_diagnostic_only"])
            self.assertEqual(result["mean_map_error_deg"]["count"], 5)
            self.assertEqual(result["matched_model_comparison"]["points"], 5)
            with self.assertRaisesRegex(calibration.CalibrationError, "unqualified"):
                calibration.source_analysis(heldout, summary_only=True)
            duplicate = analysis_summary(rows)  # New summary, reused training raw inputs.
            heldout.write_text(json.dumps(duplicate))
            with self.assertRaisesRegex(calibration.CalibrationError, "Training sweep"):
                calibration.validate_reversals(profile, heldout, summary_only=True)


if __name__ == "__main__":
    unittest.main()
