"""Synthetic settled basket geometry, stale-data and clock-association tests."""

import csv
from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import struct
import tempfile
import unittest
import zlib

import numpy as np

import basket_analyze as analyze
import basket_capture as basket
import calibrate
import capture


BIAS = np.array([-158., 134., -50.])
SCALE = np.array([240., 240., 246.])
ANGLES = np.array([10., 12., 10.4, 8., 9.4])
TARGETS = [1420, 1400, 1420, 1440, 1420]


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2) + "\n")


def frame(seq, stamp, kind, flags, xyz, aux, pwm=0):
    payload = capture.PAYLOAD.pack(b"ADXL", 1, kind, flags, seq, stamp,
        *xyz, aux, 0, pwm, seq if pwm else 0, stamp - 5000 if pwm else 0, 0, 0, 0)
    return payload + struct.pack("<I", zlib.crc32(payload))


def fixture(folder, mcu_origin_us=123456789, stale=False, ambiguous_batch=False, pwm_offset_us=-2):
    folder.mkdir()
    host_origin = 9000000000000
    identity = {"api_hex": "000130", "variant": "BTFL", "uid_hex": "12" * 12}
    profile = {"schema_version": 1, "device_writes": False,
               "bias_counts": BIAS.tolist(), "counts_per_g": SCALE.tolist(),
               "sensor_register_tuple_devid_bw_format_fault": [0xe5, 0x0d, 0x08, 0],
               "training": {"pose_metrics": []}}
    write_json(folder / "calibration-profile.json", profile)
    profile_hash = calibrate.sha256(folder / "calibration-profile.json")
    validation = {"profile_sha256": profile_hash, "independent_static_validation_pass": True,
        "required_tilted_poses": 4, "policy_version": 2, "stationarity_policy": "static-mean-v2",
        "screening_criteria": {"synthetic": True},
        "captures": [{"pose_id": str(i), "capture_sha256": str(i + 1) * 64, "validation_pass": True} for i in range(4)]}
    write_json(folder / "static-validation-report.json", validation)
    original = basket.SERVO_RECORD.pack(1100, 1700, 1420, 100, 255, 0) * 8
    (folder / "original-servo-table.bin").write_bytes(original)
    advanced = bytearray(20)
    advanced[18:20] = bytes((103, 104))
    guard = {"advanced_config_hex": bytes(advanced).hex(), "status_hex": bytes(16).hex(),
             "mixer_config_hex": "0100", "motor_payload_hex": struct.pack("<8H", 1000, 1000, 1000, 0, 0, 0, 0, 0).hex()}
    backup = {"identity": identity, "servo_index": 5, "servo_table_hex": original.hex(), "boxids_hex": "0001", **guard}
    write_json(folder / "fc-backup.json", backup)
    plan = {"kind": "center-check", "servo_index": 5, "settle_s": 1., "measure_s": 2., "sample_hz": 20.,
            "plateaus": [{"plateau_id": i, "target_us": value} for i, value in enumerate(TARGETS)]}
    write_json(folder / "plan.json", plan)
    events = []
    for i, target in enumerate(TARGETS):
        start = host_origin + int((1 + i * 3.1) * 1e9)
        events.append({"plateau_id": i, "target_us": target,
            "command_request_host_monotonic_ns": start,
            "command_ack_host_monotonic_ns": start + 10000000,
            "settle_end_host_monotonic_ns": start + 1010000000,
            "planned_end_host_monotonic_ns": start + 3010000000,
            "plateau_end_host_monotonic_ns": start + 3020000000})
    write_json(folder / "fc-events.json", events)
    write_json(folder / "fc-run.json", {"sequence_complete": True, "servo_restored_exact": True,
        "identity": identity, "events": events, "final_servo_table_hex": original.hex(),
        "pre_restore_guard": guard, "post_restore_guard": guard})
    write_json(folder / "run.json", {"mode": "execute", "sequence_complete": True,
        "physical_setup_confirmed_by_caller": True, "profile_sha256": profile_hash,
        "validation_sha256": calibrate.sha256(folder / "static-validation-report.json"), "tool_sha256": "test-tool"})
    columns = ["request_host_monotonic_ns", "response_host_monotonic_ns", "plateau_id", "phase", "target_us",
               "position_fresh", "command_matches", "settled_candidate", "raw_pb0", "final_pwm_us", "raw_pc0",
               "corrected_pb0_minus_pc0", "pair_age_100us", "flags", "rpm_age_100us", "rpm_invalid_stat"]
    truths = []
    with (folder / "fc-samples.csv").open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(columns)
        for i, event in enumerate(events):
            angle = ANGLES[i]
            # Deliberately nonlinear position conversion, independent of PWM.
            corrected = round(1700 - 20 * angle + .1 * angle**2)
            truths.append(corrected)
            for j in range(60):
                before = event["command_ack_host_monotonic_ns"] + j * 50000000
                after = before + 2000000
                measure = after >= event["settle_end_host_monotonic_ns"]
                fresh = not (stale and j == 28)
                observed = corrected if fresh else 700
                writer.writerow([before, after, i, "measure" if measure else "settling", TARGETS[i],
                    int(fresh), 1, int(measure and fresh), observed + 5, TARGETS[i], 5, observed,
                    2 if fresh else 15, 15 if fresh else 7, 1, 0])
    chunks = [(frame(0, mcu_origin_us, capture.STATUS, 65, (0xe5, 0x0d, 0x08), 0), host_origin, 0)]
    combined = []
    for i in range(14001):
        stamp = mcu_origin_us + i * 1250 + 10000
        host = host_origin + i * 1250000 + 10000000
        active = max([j for j, e in enumerate(events) if host >= e["command_ack_host_monotonic_ns"]] or [0])
        angle = np.radians(ANGLES[active])
        g = np.array([np.cos(angle), np.sin(angle), 0.])
        raw = np.round(BIAS + SCALE * g).astype(int)
        wire = frame(i + 1, stamp, capture.SAMPLE, 0x85, tuple(raw), 0x0182, TARGETS[active] + pwm_offset_us)
        if ambiguous_batch and i < 320:
            combined.append(wire)
            if i == 319:
                chunks.append((b"".join(combined), host, 0))
        else:
            chunks.append((wire, host, 0))
    capture.record_stream(chunks, folder / "adxl")
    return truths


class PlaneTests(unittest.TestCase):
    def test_rotated_circle_plane_recovers_signed_angles_without_pwm_model(self):
        normal = np.array([.3, .4, np.sqrt(.75)])
        x = np.array([.8, -.6, 0.])
        y = np.cross(normal, x)
        angles = np.array([-60., -30., 0., 30., 60., 30., 0., -30., -60.])
        g = .2 * normal + np.sqrt(.96) * (np.cos(np.radians(angles))[:, None] * x + np.sin(np.radians(angles))[:, None] * y)
        result = analyze.fit_rotation_plane(g, reference_index=2)
        self.assertTrue(result["identified"])
        np.testing.assert_allclose(result["normal_sensor_axes"], normal, atol=1e-12)
        np.testing.assert_allclose(result["relative_angles_deg"], angles, atol=1e-12)
        self.assertFalse(result["absolute_vertical_zero_qualified"])

    def test_small_arc_and_weak_projection_do_not_identify_plane(self):
        for radius, angles in [(1., [-2, -1, 0, 1, 2]), (.1, [-90, -45, 0, 45, 90])]:
            angles = np.radians(angles)
            g = np.column_stack((radius * np.cos(angles), radius * np.sin(angles), np.full(5, np.sqrt(1 - radius**2))))
            self.assertFalse(analyze.fit_rotation_plane(g)["identified"])
        result = analyze.fit_rotation_plane(np.tile([1., 0, 0], (5, 1)))
        json.dumps(result, allow_nan=False)
        self.assertFalse(result["identified"])


class CaptureTests(unittest.TestCase):
    def test_nonlinear_adc_hysteresis_stale_exclusion_and_unrelated_mcu_origin(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory) / "run"
            truth = fixture(folder, mcu_origin_us=9000123456789, stale=True)
            result = analyze.analyze(folder)
            self.assertTrue(result["all_settled_windows_pass"])
            self.assertFalse(result["rotation_plane"]["identified"])
            quantized = np.asarray([np.round(BIAS + SCALE * np.array([np.cos(np.radians(a)), np.sin(np.radians(a)), 0.])) for a in ANGLES])
            gravity = (quantized - BIAS) / SCALE
            observed_angles = np.degrees(np.arctan2(gravity[:, 1], gravity[:, 0]))
            for i, plateau in enumerate(result["plateaus"]):
                self.assertEqual(plateau["fc"]["corrected_pb0_minus_pc0_counts"]["mean"], truth[i])
                self.assertEqual(plateau["fc"]["excluded_stale_or_mismatched_rows"], 1)
                self.assertAlmostEqual(plateau["xy_diagnostic_relative_deg"], observed_angles[i] - observed_angles[0], places=10)
                self.assertAlmostEqual(plateau["xy_diagnostic_relative_deg"], ANGLES[i] - ANGLES[0], delta=.25)
                self.assertIsNone(plateau["rotation_plane_relative_deg"])
            opposite = [p for p in result["same_target_comparisons"] if p["opposite_approaches"]]
            self.assertEqual(len(opposite), 1)
            self.assertAlmostEqual(opposite[0]["delta_xy_diagnostic_deg_b_minus_a"], observed_angles[4] - observed_angles[2], places=10)
            self.assertAlmostEqual(opposite[0]["delta_xy_diagnostic_deg_b_minus_a"], -1., delta=.25)
            self.assertEqual(opposite[0]["delta_corrected_adc_counts_b_minus_a"], truth[4] - truth[2])
            self.assertFalse(result["exact_clock_synchronization"])

    def test_ambiguous_usb_batch_and_unrestored_configuration_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory) / "run"
            fixture(folder, ambiguous_batch=True)
            with self.assertRaisesRegex(analyze.AnalysisError, "USB batching"):
                analyze.analyze(folder)
            report_path = folder / "fc-run.json"
            report = json.loads(report_path.read_text())
            report["final_servo_table_hex"] = "00" * 96
            write_json(report_path, report)
            with self.assertRaisesRegex(analyze.AnalysisError, "servo tables differ"):
                analyze.analyze(folder)

    def test_wrong_observed_pwm_is_diagnostic_only_and_cli_returns_failed_screen(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory) / "run"
            fixture(folder, pwm_offset_us=30)
            output = Path(directory) / "analysis.json"
            with redirect_stdout(io.StringIO()):
                code = analyze.main([str(folder), "--output", str(output)])
            self.assertEqual(code, 2)
            result = json.loads(output.read_text())
            self.assertFalse(result["all_settled_windows_pass"])
            for plateau in result["plateaus"]:
                self.assertGreater(plateau["adxl"]["fresh_pwm_rows"], 100)
                self.assertEqual(plateau["adxl"]["fresh_matching_pwm_rows"], 0)
                self.assertEqual(plateau["adxl"]["association"], "host_window_only_diagnostic")
                self.assertFalse(plateau["adxl"]["paired_observation_qualified"])


if __name__ == "__main__":
    unittest.main()
