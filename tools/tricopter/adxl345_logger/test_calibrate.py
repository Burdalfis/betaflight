"""Synthetic off-axis recovery and raw-artifact qualification; no hardware."""

import hashlib
import json
from pathlib import Path
import shutil
import struct
import tempfile
import unittest
import zlib

import numpy as np

import calibrate
import capture


BIAS = np.array([-158.0, 136.0, -50.0])
SCALE = np.array([240.0, 251.0, 246.0])


def directions():
    # Intentionally tilted, not exact opposite axes or exactly orthogonal poses.
    vectors = np.array([[1, .08, -.04], [-1, .03, .07], [.02, 1, .05],
                        [-.05, -1, .04], [.03, -.02, 1], [.06, .01, -1.]])
    return vectors / np.linalg.norm(vectors, axis=1)[:, None]


def wire_frame(seq, stamp, kind, flags, xyz, aux):
    payload = capture.PAYLOAD.pack(b"ADXL", 1, kind, flags, seq, stamp,
        *xyz, aux, 0, 0, 0, 0, 0, 0, 0)
    return payload + struct.pack("<I", zlib.crc32(payload))


def make_capture(path, mean):
    mean = np.round(mean).astype(int)
    wire = wire_frame(0, 1000, capture.STATUS,
        capture.SENSOR_CONFIGURED | capture.USB_SESSION_START, (0xe5, 0x0d, 0x08), 0)
    wire += b"".join(wire_frame(i + 1, 2000 + 1250 * i, capture.SAMPLE,
        capture.SENSOR_CONFIGURED | capture.FIFO_COHERENT, tuple(mean), 0x0182)
        for i in range(1601))
    capture.record_stream([(wire, None, None)], path)
    return mean


def make_values_capture(path, values):
    wire = wire_frame(0, 1000, capture.STATUS,
        capture.SENSOR_CONFIGURED | capture.USB_SESSION_START, (0xe5, 0x0d, 0x08), 0)
    wire += b"".join(wire_frame(i + 1, 2000 + 1250 * i, capture.SAMPLE,
        capture.SENSOR_CONFIGURED | capture.FIFO_COHERENT,
        tuple(np.round(value).astype(int)), 0x0182) for i, value in enumerate(values))
    capture.record_stream([(wire, None, None)], path)


def synthetic_profile():
    profile = calibrate.fit_ellipsoid(BIAS + SCALE * directions())
    profile["sensor_register_tuple_devid_bw_format_fault"] = [0xe5, 0x0d, 0x08, 0]
    profile["training"] = {"pose_metrics": [{"input_sha256": {"capture.bin": "separate-synthetic-training-data"}}]}
    return profile


def make_session(path):
    names, pose_hashes = [], {}
    for i, mean in enumerate(BIAS + SCALE * directions()):
        folder = path / f"pose-{i}"
        observed = make_capture(folder, mean)
        pose = {"pose_id": folder.name, "accepted_for_calibration": True,
                "static_capture_gates": {"synthetic_stationary": True},
                "mean_raw_counts": observed.tolist(), "calibration_cohort": "synthetic",
                "file_sha256": {n: calibrate.sha256(folder / n) for n in ("capture.bin", "records.csv", "summary.json")}}
        (folder / "pose.json").write_text(json.dumps(pose))
        name = folder.name + "/pose.json"
        names.append(name)
        pose_hashes[name] = calibrate.sha256(folder / "pose.json")
    folder = path / "held-out-reference"
    mean = make_capture(folder, BIAS + SCALE * np.array([0., 0., 1.02]))
    pose = {"pose_id": folder.name, "accepted_for_calibration": False,
            "accepted_for_reference_repeatability": True, "calibration_cohort": "synthetic",
            "mean_raw_counts": mean.tolist(),
            "file_sha256": {n: calibrate.sha256(folder / n) for n in ("capture.bin", "records.csv", "summary.json")}}
    (folder / "pose.json").write_text(json.dumps(pose))
    name = folder.name + "/pose.json"
    session = {"pose_manifest": names, "pose_file_sha256": pose_hashes,
               "reference_repeat_pose_manifest": [name],
               "reference_repeat_pose_sha256": {name: calibrate.sha256(folder / "pose.json")},
               "current_calibration_cohort": "synthetic"}
    file = path / "session.json"
    file.write_text(json.dumps(session))
    return file


class EllipsoidTests(unittest.TestCase):
    def test_recovers_bias_and_scale_without_exact_axis_alignment(self):
        means = BIAS + SCALE * directions()
        profile = calibrate.fit_ellipsoid(means)
        np.testing.assert_allclose(profile["bias_counts"], BIAS, rtol=0, atol=1e-10)
        np.testing.assert_allclose(profile["counts_per_g"], SCALE, rtol=0, atol=1e-10)
        np.testing.assert_allclose(calibrate.apply_calibration(means, profile), directions(), atol=1e-12)
        self.assertEqual(profile["diagnostics"]["rank"], 6)
        with self.assertRaisesRegex(calibrate.CalibrationError, "conditioning"):
            calibrate.fit_ellipsoid(means, max_condition=1.01)

    def test_noisy_pose_means_recover_parameters_and_validate_unseen_directions(self):
        rng = np.random.default_rng(8401)
        observations = BIAS + SCALE * directions()[:, None, :] + rng.normal(0, 2, (6, 4000, 3))
        profile = calibrate.fit_ellipsoid(observations.mean(axis=1))
        np.testing.assert_allclose(profile["bias_counts"], BIAS, atol=.1, rtol=0)
        np.testing.assert_allclose(profile["counts_per_g"], SCALE, atol=.15, rtol=0)
        tilted = rng.normal(size=(100, 3))
        tilted /= np.linalg.norm(tilted, axis=1)[:, None]
        corrected = calibrate.apply_calibration(BIAS + SCALE * tilted, profile)
        self.assertLess(np.max(np.abs(np.linalg.norm(corrected, axis=1) - 1)), .001)

    def test_rejects_rank_deficient_incomplete_or_nonfinite_data(self):
        for vectors in (np.zeros((6, 3)), np.tile([1, 1, 1], (6, 1)),
                        np.array([[1, 1, 1], [1, 1, -1], [1, -1, 1],
                                  [-1, 1, 1], [-1, -1, 1], [-1, 1, -1]]),
                        np.full((6, 3), np.nan), np.zeros((5, 3))):
            with self.subTest(vectors=vectors):
                with self.assertRaises(calibrate.CalibrationError):
                    calibrate.fit_ellipsoid(vectors)

    def test_rejects_hyperboloid_and_invalid_profile_sensitivity(self):
        # These full-rank points satisfy x²+y²-z²=1, not a positive ellipsoid.
        v = np.array([[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0],
                      [np.sqrt(2), 0, 1], [-np.sqrt(2), 0, -1]])
        with self.assertRaisesRegex(calibrate.CalibrationError, "positive"):
            calibrate.fit_ellipsoid(v)
        for scale in ([0, 1, 1], [-1, 1, 1], [1, float("inf"), 1]):
            with self.assertRaises(calibrate.CalibrationError):
                calibrate.apply_calibration([[1, 2, 3]], {"bias_counts": [0, 0, 0], "counts_per_g": scale})


class ArtifactTests(unittest.TestCase):
    def test_hash_and_path_qualification(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            (path / "input").write_bytes(b"original")
            digest = hashlib.sha256(b"original").hexdigest()
            calibrate.verify_hashes(path, {"input": digest})
            (path / "input").write_bytes(b"changed")
            with self.assertRaisesRegex(calibrate.CalibrationError, "hash mismatch"):
                calibrate.verify_hashes(path, {"input": digest})
            with self.assertRaises(calibrate.CalibrationError):
                calibrate.verify_hashes(path, {"../outside": digest})

    def test_session_fit_keeps_repeat_independent_and_refuses_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            session = make_session(path)
            profile = calibrate.fit_session(session)
            np.testing.assert_allclose(profile["bias_counts"], BIAS, atol=1, rtol=0)
            np.testing.assert_allclose(profile["counts_per_g"], SCALE, atol=1, rtol=0)
            self.assertEqual(len(profile["training"]["pose_metrics"]), 6)
            heldout = profile["validation"]["reference_pose_metrics"]
            self.assertEqual(len(heldout), 1)
            self.assertGreater(heldout[0]["norm_residual_g"], .015)
            self.assertFalse(profile["device_writes"])
            self.assertFalse(profile["physical_angle_accuracy_qualified"])
            output = path / "profile.json"
            output.write_text(json.dumps(profile))
            self.assertEqual(calibrate.main(["fit", str(session), "--output", str(output)]), 1)
            validation = calibrate.validate_capture(profile, path / "held-out-reference")
            self.assertTrue(validation["norm_screen"]["pass"])
            validation = calibrate.validate_capture(profile, path / "held-out-reference", .005)
            self.assertFalse(validation["norm_screen"]["pass"])
            shutil.copytree(path / "pose-0", path / "training-alias")
            with self.assertRaisesRegex(calibrate.CalibrationError, "used for fitting"):
                calibrate.validate_capture(profile, path / "training-alias")
            wrong_rate = dict(profile, sensor_register_tuple_devid_bw_format_fault=[0xe5, 0x0a, 0x08, 0])
            with self.assertRaisesRegex(calibrate.CalibrationError, "configuration differs"):
                calibrate.validate_capture(wrong_rate, path / "held-out-reference")
            state = json.loads(session.read_text())
            name = state["pose_manifest"][0]
            pose_path = path / name
            pose = json.loads(pose_path.read_text())
            pose["accepted_for_calibration"] = False
            pose_path.write_text(json.dumps(pose))
            state["pose_file_sha256"][name] = calibrate.sha256(pose_path)
            session.write_text(json.dumps(state))
            with self.assertRaisesRegex(calibrate.CalibrationError, "not accepted"):
                calibrate.fit_session(session)
            state["historical_pose_manifest"] = [name]
            session.write_text(json.dumps(state))
            with self.assertRaisesRegex(calibrate.CalibrationError, "excluded"):
                calibrate.fit_session(session)

    def test_raw_capture_aliases_cannot_duplicate_fit_or_holdout_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            session_path = make_session(path)
            original = json.loads(session_path.read_text())

            state = json.loads(json.dumps(original))
            state["current_calibration_cohort"] = None
            session_path.write_text(json.dumps(state))
            with self.assertRaisesRegex(calibrate.CalibrationError, "nonempty"):
                calibrate.fit_session(session_path)

            def add_alias(source, name, fit_input):
                folder = path / name
                shutil.copytree(path / source, folder)
                pose_path = folder / "pose.json"
                pose = json.loads(pose_path.read_text())
                pose["pose_id"] = name
                pose["accepted_for_calibration"] = fit_input
                pose_path.write_text(json.dumps(pose))
                return name + "/pose.json", calibrate.sha256(pose_path)

            # Distinct metadata paths and pose IDs cannot count the same raw
            # capture twice, even when all individual hashes are correct.
            alias, digest = add_alias("pose-0", "fit-alias", True)
            state = json.loads(json.dumps(original))
            state["pose_manifest"][-1] = alias
            state["pose_file_sha256"][alias] = digest
            session_path.write_text(json.dumps(state))
            with self.assertRaisesRegex(calibrate.CalibrationError, "Duplicate raw capture used for training"):
                calibrate.fit_session(session_path)

            alias, digest = add_alias("pose-0", "reference-training-alias", False)
            state = json.loads(json.dumps(original))
            state["reference_repeat_pose_manifest"] = [alias]
            state["reference_repeat_pose_sha256"] = {alias: digest}
            session_path.write_text(json.dumps(state))
            with self.assertRaisesRegex(calibrate.CalibrationError, "holdout raw capture was used for fitting"):
                calibrate.fit_session(session_path)

            alias, digest = add_alias("held-out-reference", "reference-alias", False)
            state = json.loads(json.dumps(original))
            state["reference_repeat_pose_manifest"].append(alias)
            state["reference_repeat_pose_sha256"][alias] = digest
            session_path.write_text(json.dumps(state))
            with self.assertRaisesRegex(calibrate.CalibrationError, "Duplicate raw capture used for reference"):
                calibrate.fit_session(session_path)
            state = json.loads(json.dumps(original))
            reference = state["reference_repeat_pose_manifest"][0]
            reference_path = path / reference
            pose = json.loads(reference_path.read_text())
            pose["calibration_cohort"] = "other-hardware-session"
            reference_path.write_text(json.dumps(pose))
            state["reference_repeat_pose_sha256"][reference] = calibrate.sha256(reference_path)
            session_path.write_text(json.dumps(state))
            with self.assertRaisesRegex(calibrate.CalibrationError, "holdout cohort differs"):
                calibrate.fit_session(session_path)

    def test_raw_csv_disagreement_is_rejected_even_with_rehashed_csv(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            make_capture(path, [1, 2, 250])
            csv_file = path / "records.csv"
            csv_file.write_text(csv_file.read_text().replace(",1,2,250,", ",7,2,250,", 1))
            summary_path = path / "summary.json"
            summary = json.loads(summary_path.read_text())
            summary["files"]["records.csv"]["sha256"] = calibrate.sha256(csv_file)
            summary_path.write_text(json.dumps(summary))
            with self.assertRaisesRegex(calibrate.CalibrationError, "Raw/CSV mismatch"):
                calibrate.read_capture(path)

    def test_unqualified_bypass_and_failed_stationarity_are_not_fit_samples(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            wire = wire_frame(0, 1000, capture.STATUS, capture.SENSOR_CONFIGURED | capture.USB_SESSION_START,
                              (0xe5, 0x0d, 0x08), 0)
            wire += b"".join(wire_frame(i + 1, 2000 + 1250 * i, capture.SAMPLE,
                capture.SENSOR_CONFIGURED, (1, 2, 250), 0x82) for i in range(1601))
            capture.record_stream([(wire, None, None)], path)
            with self.assertRaisesRegex(calibrate.CalibrationError, "incoherent"):
                calibrate.read_capture(path)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            wire = wire_frame(0, 1000, capture.STATUS, capture.SENSOR_CONFIGURED | capture.USB_SESSION_START,
                              (0xe5, 0x0d, 0x08), 0)
            wire += b"".join(wire_frame(i + 1, 2000 + 1250 * i, capture.SAMPLE,
                capture.SENSOR_CONFIGURED | capture.FIFO_COHERENT, (10 if i % 2 else -10, 2, 250),
                0x0182) for i in range(1601))
            capture.record_stream([(wire, None, None)], path)
            with self.assertRaisesRegex(calibrate.CalibrationError, "stationary-session gates"):
                calibrate.read_capture(path)

    def test_opt_in_static_mean_policy_preserves_original_noise_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            rng = np.random.default_rng(77441)
            values = BIAS + SCALE * np.array([0., 0., 1.]) + rng.normal(0, 8, (3201, 3))
            make_values_capture(path, values)
            profile = synthetic_profile()
            original_profile = json.dumps(profile, sort_keys=True)
            with self.assertRaisesRegex(calibrate.CalibrationError, "stationary-session gates"):
                calibrate.validate_capture(profile, path)
            result = calibrate.validate_capture(profile, path, stationarity_policy="static-mean-v2")
            self.assertFalse(result["original_static_gate_pass"])
            self.assertFalse(result["metrics"]["stationarity"]["original_raw_std_below_5_counts"])
            self.assertTrue(result["metrics"]["stationarity"]["selected_static_mean_gate_pass"])
            self.assertTrue(result["norm_screen"]["every_one_second_mean_pass"])
            self.assertTrue(result["norm_screen"]["pass"])
            self.assertTrue(all(s > 5 for s in result["metrics"]["raw_std_counts"]))
            self.assertEqual(json.dumps(profile, sort_keys=True), original_profile)
            wrong_rate = dict(profile, sensor_register_tuple_devid_bw_format_fault=[0xe5, 0x0a, 0x08, 0])
            with self.assertRaisesRegex(calibrate.CalibrationError, "configuration differs"):
                calibrate.validate_capture(wrong_rate, path, stationarity_policy="static-mean-v2")

    def test_static_mean_policy_still_rejects_drift_and_single_block_norm_outlier(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            values = np.tile(BIAS + SCALE * np.array([0., 0., 1.]), (3201, 1))
            values[1600:, 0] += 5
            make_values_capture(path / "drifting", values)
            profile = synthetic_profile()
            with self.assertRaisesRegex(calibrate.CalibrationError, "block-mean span"):
                calibrate.validate_capture(profile, path / "drifting", stationarity_policy="static-mean-v2")

            # All means span just one Z count: stable by the raw block rule.
            # Overall norm stays below1.03, but a single full1s block exceeds it.
            values = np.tile([-158, 136, 203], (3201, 1))
            values[800:1600, 2] = 204
            make_values_capture(path / "radial-outlier", values)
            strict = calibrate.validate_capture(profile, path / "radial-outlier")
            self.assertTrue(strict["norm_screen"]["overall_mean_pass"])
            self.assertTrue(strict["norm_screen"]["pass"])
            result = calibrate.validate_capture(profile, path / "radial-outlier", stationarity_policy="static-mean-v2")
            self.assertTrue(result["original_static_gate_pass"])
            self.assertTrue(result["norm_screen"]["overall_mean_pass"])
            self.assertFalse(result["norm_screen"]["every_one_second_mean_pass"])
            self.assertFalse(result["norm_screen"]["pass"])


if __name__ == "__main__":
    unittest.main()
