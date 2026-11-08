"""Hardware-free basket-capture guards, failure recovery and stream checks."""

import csv
import contextlib
import hashlib
import io
import json
import math
from pathlib import Path
import signal
import struct
import subprocess
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import zlib

import basket_capture as basket
import capture


class Clock:
    def __init__(self):
        self.now = 10.0

    def monotonic(self):
        return self.now

    def monotonic_ns(self):
        return round(self.now * 1e9)

    def sleep(self, duration):
        self.now += max(duration, .000001)


class FakeMSP:
    def __init__(self):
        self.uid = bytes(range(12))
        self.api, self.variant = basket.EXPECTED_API, b"BTFL"
        self.mode, self.armed, self.motors = 103, False, (1000,) * 3 + (0,) * 5
        self.mixer = 1
        self.table = bytearray(basket.SERVO_RECORD.pack(1000, 2000, 1500, -70, 255, 0x12345678) * 8)
        self.table[60:72] = basket.SERVO_RECORD.pack(1100, 1700, 1420, 100, 255, 0x87654321)
        self.original = bytes(self.table)
        self.pwm, self.debug_calls = 1420, 0
        self.trace, self.writes = [], []
        self.debug_hook, self.after_write = None, None
        self.lose_ack_once = False
        self.closed = False

    def close(self):
        self.closed = True

    def request(self, command, payload=b"", **_):
        self.trace.append((command, payload))
        if command == basket.API_VERSION:
            return self.api
        if command == basket.FC_VARIANT:
            return self.variant
        if command == basket.UID:
            return self.uid
        if command == basket.BOXIDS:
            return bytes((0, 1, 2, 36))
        if command == basket.ADVANCED_CONFIG:
            # Source MSP layout: gyro_check_overflow[17], debug_mode[18], DEBUG_COUNT[19].
            return bytes(17) + bytes((0, self.mode, 104))
        if command == basket.MIXER_CONFIG:
            return bytes((self.mixer, 0))
        if command == basket.STATUS_EX:
            status = bytearray(16)
            struct.pack_into("<I", status, 6, int(self.armed))
            return bytes(status)
        if command == basket.MOTOR:
            return struct.pack("<8H", *self.motors)
        if command == basket.SERVO_CONFIGURATIONS:
            return bytes(self.table)
        if command == basket.DEBUG:
            self.debug_calls += 1
            values = (1600, self.pwm, 100, 1500, 2, 15, -1, -1)
            if self.debug_hook:
                values = self.debug_hook(self, values)
            return struct.pack("<8h", *values)
        if command == basket.SET_SERVO_CONFIGURATION:
            self.writes.append(payload)
            start = payload[0] * 12
            self.table[start:start + 12] = payload[1:]
            fields = basket.SERVO_RECORD.unpack(payload[1:])
            self.pwm = fields[2]
            if self.after_write:
                self.after_write(self)
            if self.lose_ack_once:
                self.lose_ack_once = False
                raise basket.BasketError("Injected ACK timeout after applying write")
            return b""
        raise AssertionError(f"Unexpected command {command}")


def adxl_frame(seq, kind=capture.SAMPLE, flags=0x81, xyz=(-158, 135, 200), aux=0x0182,
               overrun=0, gap=0, drops=0):
    payload = capture.PAYLOAD.pack(b"ADXL", 1, kind, flags, seq, 1000 + seq * 1300,
                                   *xyz, aux, 0, 0, 0, 0, overrun, gap, drops)
    return payload + struct.pack("<I", zlib.crc32(payload))


def static_stream(samples=150):
    return adxl_frame(0, kind=capture.STATUS, flags=0x41, xyz=(0xe5, 0x0d, 8), aux=0) + b"".join(
        adxl_frame(i) for i in range(1, samples + 1))


class GuardTests(unittest.TestCase):
    def test_aliases_and_wrong_usb_serial_are_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            fc, teensy = root / "fc", root / "teensy"
            fc.touch()
            teensy.touch()
            alias = root / "alias"
            alias.symlink_to(teensy)
            with self.assertRaises(basket.BasketError):
                basket.resolved_ports(alias, teensy)
            ports = [SimpleNamespace(device=str(teensy), vid=0x16c0, pid=0x483, serial_number="expected")]
            self.assertEqual(basket.verify_usb_identity(fc, alias, "expected", ports)["teensy_serial"], "expected")
            with self.assertRaises(basket.BasketError):
                basket.verify_usb_identity(fc, teensy, "wrong", ports)
            with self.assertRaises(basket.BasketError):
                basket.verify_usb_identity(teensy, fc, "expected", ports)

    def test_fc_wrong_variant_api_uid_armed_and_motor_guards(self):
        for field, value in (("variant", b"INAV"), ("api", bytes((0, 1, 47))),
                             ("armed", True), ("motors", (1000, 1100, 1000) + (0,) * 5),
                             ("mode", 102), ("mixer", 3)):
            fake = FakeMSP()
            setattr(fake, field, value)
            with self.subTest(field=field), self.assertRaises(basket.BasketError):
                basket.Controller(fake)
            self.assertFalse(fake.writes)
        with self.assertRaises(basket.BasketError):
            basket.Controller(FakeMSP(), expected_uid="ff" * 12)

    def test_corrected_negative_one_and_ground_zero_remain_valid(self):
        for wiper, ground, corrected in ((1, 2, -1), (1600, 0, 1600)):
            raw = struct.pack("<8h", wiper, 1420, ground, corrected, 2, 15, -1, -1)
            values, fresh = basket.ground_sample(raw)
            self.assertEqual(values[3], corrected)
            self.assertTrue(fresh)
        for values in ((1600, 1420, 100, 1600, 2, 15, -1, -1),
                       (1600, 1420, 100, 1500, 2, 3, -1, -1),
                       (1600, 1420, 100, 1500, 2, 0x80f, -1, -1),
                       (4095, 1420, 0, 4095, 2, 15, -1, -1)):
            with self.assertRaises(basket.BasketError):
                basket.ground_sample(struct.pack("<8h", *values))

    def test_only_small_center_plan_can_execute(self):
        self.assertEqual(basket.validate_plan(basket.make_plan()), list(basket.CENTER_TARGETS))
        sweep = basket.make_plan("sweep-preview")
        self.assertIn(1100, basket.validate_plan(sweep, execute=False))
        self.assertIn(1700, basket.validate_plan(sweep, execute=False))
        with self.assertRaises(basket.BasketError):
            basket.validate_plan(sweep, execute=True)
        bad = basket.make_plan()
        bad["plateaus"][1]["target_us"] = 1700
        with self.assertRaises(basket.BasketError):
            basket.validate_plan(bad)
        fake = FakeMSP()
        controller = basket.Controller(fake)
        with self.assertRaises(basket.BasketError):
            controller.set_target(1100)
        self.assertFalse(fake.writes)


class RecoveryTests(unittest.TestCase):
    def run_case(self, configure=None, health=None):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        folder = Path(temp.name)
        fake, clock = FakeMSP(), Clock()
        if configure:
            configure(fake)
        controller = basket.Controller(fake, clock=clock)
        plan = basket.make_plan(settle_s=.25, measure_s=1, sample_hz=50)
        return folder, fake, clock, controller, plan, (lambda: health(fake, clock)) if health else (lambda: None)

    def test_complete_run_and_isolated_stale_rows_restore_raw_bytes(self):
        def configure(fake):
            def debug_hook(state, values):
                return (*values[:4], 12, 7, *values[6:]) if state.debug_calls % 19 == 0 else values
            fake.debug_hook = debug_hook
        folder, fake, _, controller, plan, check = self.run_case(configure)
        report = controller.record(plan, folder, check)
        self.assertTrue(report["sequence_complete"])
        self.assertTrue(report["servo_restored_exact"])
        self.assertEqual(bytes(fake.table), fake.original)
        self.assertEqual(fake.writes[-1], bytes((5,)) + fake.original[60:72])
        self.assertTrue({command for command, _ in fake.trace} <= {1, 2, 42, 90, 104, 119, 120, 150, 160, 212, 254})
        self.assertEqual({command for command, _ in fake.trace if command >= 200 and command != 254}, {212})
        with (folder / "fc-samples.csv").open() as handle:
            rows = list(csv.DictReader(handle))
        stale = [row for row in rows if row["position_fresh"] == "0"]
        self.assertTrue(stale)
        self.assertTrue(all(row["settled_candidate"] == "0" for row in stale))
        self.assertTrue(all(int(row["corrected_pb0_minus_pc0"]) == int(row["raw_pb0"]) - int(row["raw_pc0"]) for row in rows))
        self.assertEqual((folder / "original-servo-table.bin").read_bytes(), fake.original)

    def test_keyboard_interrupt_after_motion_restores_exactly(self):
        def interrupt(fake, clock):
            if fake.writes and clock.now > 10.05:
                raise KeyboardInterrupt()
        folder, fake, _, controller, plan, health = self.run_case(health=interrupt)
        with self.assertRaises(KeyboardInterrupt):
            controller.record(plan, folder, health)
        report = json.loads((folder / "fc-run.json").read_text())
        self.assertFalse(report["sequence_complete"])
        self.assertTrue(report["servo_restored_exact"])
        self.assertEqual(bytes(fake.table), fake.original)

    def test_write_applied_but_ack_lost_still_restores(self):
        folder, fake, _, controller, plan, health = self.run_case()
        fake.lose_ack_once = True
        with self.assertRaises(basket.BasketError):
            controller.record(plan, folder, health)
        self.assertEqual(bytes(fake.table), fake.original)
        self.assertTrue(json.loads((folder / "fc-run.json").read_text())["servo_restored_exact"])

    def test_persistent_stale_or_too_few_fresh_measure_rows_stop_and_restore(self):
        for alternate in (False, True):
            def configure(fake):
                def hook(state, values):
                    stale = bool(state.writes) and (not alternate or state.debug_calls % 2 == 0)
                    return (*values[:4], 12, 7, *values[6:]) if stale else values
                fake.debug_hook = hook
            folder, fake, _, controller, plan, health = self.run_case(configure)
            with self.subTest(alternate=alternate), self.assertRaises(basket.BasketError):
                controller.record(plan, folder, health)
            self.assertEqual(bytes(fake.table), fake.original)
            self.assertTrue(json.loads((folder / "fc-run.json").read_text())["servo_restored_exact"])

    def test_invalid_adc_schema_stops_without_raw_fallback_and_restores(self):
        folder, fake, _, controller, plan, health = self.run_case()
        fake.debug_hook = lambda state, values: (*values[:3], 1600, *values[4:]) if state.writes else values
        with self.assertRaises(basket.BasketError):
            controller.record(plan, folder, health)
        self.assertEqual(bytes(fake.table), fake.original)
        self.assertEqual(len(fake.writes), 2)

    def test_changed_identity_or_armed_state_blocks_restore_writes(self):
        for unsafe in ("identity", "armed", "motors"):
            folder, fake, _, controller, plan, health = self.run_case()
            def change(state):
                if unsafe == "identity":
                    state.uid = b"x" * 12
                elif unsafe == "armed":
                    state.armed = True
                else:
                    state.motors = (1100,) + state.motors[1:]
            fake.after_write = change
            with self.subTest(unsafe=unsafe), self.assertRaises(basket.BasketError):
                controller.record(plan, folder, health)
            report = json.loads((folder / "fc-run.json").read_text())
            self.assertFalse(report["servo_restored_exact"])
            self.assertIn("critical_restore_failure", report)
            self.assertEqual(len(fake.writes), 1)

    def test_external_servo_change_is_not_overwritten_during_restore(self):
        folder, fake, _, controller, plan, health = self.run_case()
        fake.after_write = lambda state: state.table.__setitem__(0, state.table[0] ^ 1)
        with self.assertRaises(basket.BasketError):
            controller.record(plan, folder, health)
        report = json.loads((folder / "fc-run.json").read_text())
        self.assertIn("critical_restore_failure", report)
        self.assertEqual(len(fake.writes), 1)


class ProfileAndPreviewTests(unittest.TestCase):
    def profile_files(self, root):
        profile, report = root / "profile.json", root / "validation.json"
        profile.write_text(json.dumps({"schema_version": 1, "device_writes": False,
                                       "sensor_register_tuple_devid_bw_format_fault": [0xe5, 0x0d, 0x08, 0],
                                       "bias_counts": [-158, 134, -50], "counts_per_g": [240, 240, 247]}))
        validation = {"profile_sha256": basket.sha256(profile), "independent_static_validation_pass": True,
                      "required_tilted_poses": 6,
                      "captures": [{"pose_id": f"tilt-{i}", "capture_sha256": hashlib.sha256(f"tilt{i}".encode()).hexdigest(), "validation_pass": True} for i in range(6)]}
        report.write_text(json.dumps(validation))
        return profile, report

    def test_validation_must_match_profile_and_all_unique_tilted_poses(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            profile, report = self.profile_files(root)
            self.assertEqual(basket.load_validated_profile(profile, report)["profile"]["counts_per_g"], [240, 240, 247])
            for mutate in (lambda x: x.update(profile_sha256="f" * 64),
                           lambda x: x.update(independent_static_validation_pass=False),
                           lambda x: x.update(required_tilted_poses=6.0),
                           lambda x: x["captures"].pop(),
                           lambda x: x["captures"][0].update(validation_pass=False),
                           lambda x: x["captures"][0].update(pose_id="tilt-1"),
                           lambda x: x["captures"][0].update(capture_sha256=x["captures"][1]["capture_sha256"])):
                _, report = self.profile_files(root)
                value = json.loads(report.read_text())
                mutate(value)
                report.write_text(json.dumps(value))
                with self.assertRaises(basket.BasketError):
                    basket.load_validated_profile(profile, report)

    def test_nonpositive_or_nonfinite_scale_is_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            for scale in (0, -1, float("nan")):
                profile, report = self.profile_files(root)
                value = json.loads(profile.read_text())
                value["counts_per_g"][0] = scale
                profile.write_text(json.dumps(value))
                with self.assertRaises(basket.BasketError):
                    basket.load_validated_profile(profile, report)

    def test_four_pose_static_mean_policy_is_explicit_and_training_data_are_excluded(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            profile, report = self.profile_files(root)
            validation = json.loads(report.read_text())
            validation.update(policy_version=2, stationarity_policy="static-mean-v2", required_tilted_poses=4,
                              screening_criteria={"mean_norm_error_g": .03, "omitted_xz": True})
            validation["captures"] = validation["captures"][:4]
            report.write_text(json.dumps(validation))
            basket.load_validated_profile(profile, report)
            value = json.loads(profile.read_text())
            value["training"] = {"pose_metrics": [{"input_sha256": {"capture.bin": validation["captures"][0]["capture_sha256"]}}]}
            profile.write_text(json.dumps(value))
            validation["profile_sha256"] = basket.sha256(profile)
            report.write_text(json.dumps(validation))
            with self.assertRaises(basket.BasketError):
                basket.load_validated_profile(profile, report)
            value.pop("training")
            profile.write_text(json.dumps(value))
            validation["profile_sha256"] = basket.sha256(profile)
            validation["policy_version"] = 1
            report.write_text(json.dumps(validation))
            with self.assertRaises(basket.BasketError):
                basket.load_validated_profile(profile, report)

    def test_sensor_config_mismatch_is_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            profile, report = self.profile_files(root)
            value = json.loads(profile.read_text())
            value["sensor_register_tuple_devid_bw_format_fault"][1] = 0x0a
            profile.write_text(json.dumps(value))
            validation = json.loads(report.read_text())
            validation["profile_sha256"] = basket.sha256(profile)
            report.write_text(json.dumps(validation))
            with self.assertRaises(basket.BasketError):
                basket.load_validated_profile(profile, report)

    def test_default_preview_opens_no_ports_and_existing_folder_is_untouched(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            profile, report = self.profile_files(root)
            args = ["--fc-port", str(root / "fc"), "--teensy-port", str(root / "teensy"),
                    "--teensy-serial", "123", "--profile", str(profile), "--validation-report", str(report),
                    "--output", str(root / "output")]
            with contextlib.redirect_stdout(io.StringIO()), \
                    patch.object(basket, "MSP", side_effect=AssertionError("Must not open FC")), \
                    patch.object(basket, "verify_usb_identity", side_effect=AssertionError("Must not enumerate ports")):
                self.assertEqual(basket.main(args), 0)
            saved = (root / "output/plan.json").read_bytes()
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(basket.main(args), 1)
            self.assertEqual((root / "output/plan.json").read_bytes(), saved)
            occupied = root / "occupied"
            occupied.mkdir()
            (occupied / "keep.txt").write_text("keep")
            args[-1] = str(occupied)
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(basket.main(args), 1)
            self.assertEqual([p.name for p in occupied.iterdir()], ["keep.txt"])

    def test_execution_requires_external_setup_attestation_and_explicit_uid(self):
        args = ["--fc-port", "fc", "--teensy-port", "teensy", "--teensy-serial", "123",
                "--profile", "profile", "--validation-report", "validation", "--output", "out", "--execute"]
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            basket.arguments(args)
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            basket.arguments(args + ["--setup-confirmed"])

    def test_inspection_reads_identity_and_original_config_without_writes(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            profile, report = self.profile_files(root)
            fake = FakeMSP()
            args = ["--fc-port", "explicit-fc", "--teensy-port", "explicit-teensy", "--teensy-serial", "123",
                    "--profile", str(profile), "--validation-report", str(report), "--output", str(root / "inspect"), "--inspect"]
            with contextlib.redirect_stdout(io.StringIO()), patch.object(basket, "MSP", return_value=fake), \
                    patch.object(basket, "verify_usb_identity", return_value={"teensy_serial": "123"}):
                self.assertEqual(basket.main(args), 0)
            self.assertFalse(fake.writes)
            self.assertTrue(fake.closed)
            preflight = json.loads((root / "inspect/fc-readonly-preflight.json").read_text())
            self.assertEqual(preflight["identity"]["uid_hex"], fake.uid.hex())
            self.assertEqual(preflight["servo_table_hex"], fake.original.hex())


class StreamTests(unittest.TestCase):
    def test_monitor_requires_live_clean_configured_sensor_before_motion(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (root / "capture.bin").write_bytes(static_stream())
            process = SimpleNamespace(poll=lambda: None)
            monitor = basket.AdxlMonitor(root, process, clock=Clock())
            monitor.wait_ready()
            self.assertTrue(monitor.status_seen)
            self.assertEqual(monitor.samples, 150)
            with (root / "capture.bin").open("ab") as handle:
                handle.write(adxl_frame(151, overrun=1))
            with self.assertRaises(basket.BasketError):
                monitor.poll()

    def test_monitor_rejects_foreign_sensor_status_and_corrupt_bytes(self):
        for wire in (adxl_frame(0, kind=capture.STATUS, flags=0x41, xyz=(0, 0x0d, 8), aux=0),
                     b"noise" + static_stream()):
            with tempfile.TemporaryDirectory() as temp:
                root = Path(temp)
                (root / "capture.bin").write_bytes(wire)
                with self.assertRaises(basket.BasketError):
                    basket.AdxlMonitor(root, SimpleNamespace(poll=lambda: None), clock=Clock()).poll()

    def test_completed_child_summary_integrity_and_counter_guard(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            report = capture.record_stream([(static_stream() + b"A", 123, 456)], root)
            self.assertEqual(basket.validate_adxl_summary(root)["quality"]["sample_rows"], 150)
            report["quality"]["epochs"][0]["last_counters"]["queue_dropped_frames"] = 1
            (root / "summary.json").write_text(json.dumps(report))
            with self.assertRaises(basket.BasketError):
                basket.validate_adxl_summary(root)

    def test_child_stop_uses_interrupt_and_preserves_timeout_failure(self):
        class Child:
            def __init__(self, timeout=False):
                self.signals, self.killed, self.timeout = [], False, timeout
            def poll(self):
                return None
            def send_signal(self, sig):
                self.signals.append(sig)
            def wait(self, timeout):
                if self.timeout and not self.killed:
                    raise subprocess.TimeoutExpired("logger", timeout)
                return 0
            def kill(self):
                self.killed = True
        child = Child()
        self.assertEqual(basket.stop_capture(child), 0)
        self.assertEqual(child.signals, [signal.SIGINT])
        child = Child(timeout=True)
        with self.assertRaises(basket.BasketError):
            basket.stop_capture(child)
        self.assertTrue(child.killed)

    def test_msp_does_not_accept_echo_as_response(self):
        class Serial:
            def __init__(self):
                self.data = bytearray()
            def reset_input_buffer(self):
                self.data.clear()
            def write(self, packet):
                command = packet[4]
                body = bytes((4, command)) + b"BTFL"
                checksum = 0
                for byte in body:
                    checksum ^= byte
                self.data.extend(packet + b"$M>" + body + bytes((checksum,)))
            def flush(self):
                pass
            @property
            def in_waiting(self):
                return len(self.data)
            def read(self, count):
                data = bytes(self.data[:count])
                del self.data[:count]
                return data
        self.assertEqual(basket.MSP("explicit", serial_connection=Serial()).request(2), b"BTFL")


class RollZeroTests(unittest.TestCase):
    def test_fixed_zero_targets_only_and_cli_needs_no_full_range_binding(self):
        expected = [1420, 1440, 1460, 1480, 1500, 1510, 1519, 1525, 1545, 1560,
                    1540, 1520, 1510, 1504, 1495, 1480, 1460, 1440, 1420]
        plan = basket.make_plan("roll-zero-check")
        self.assertEqual(basket.validate_plan(plan), expected)
        plan["plateaus"][6]["target_us"] = 1518  # Bounded, but not the cleared fixed sequence.
        with self.assertRaises(basket.BasketError):
            basket.validate_plan(plan)
        for invalid in (1701, 1540):
            plan = basket.make_plan("roll-zero-check")
            plan["plateaus"][1]["target_us"] = invalid
            with self.assertRaises(basket.BasketError):
                basket.validate_plan(plan)
        args = basket.arguments(["--fc-port", "fc", "--teensy-port", "teensy", "--teensy-serial", "123",
                                 "--fc-uid", "00" * 12, "--profile", "profile", "--validation-report", "report",
                                 "--output", "out", "--execute", "--setup-confirmed", "--plan-kind", "roll-zero-check"])
        self.assertIsNone(args.range_authorization)

    def test_zero_check_requires_external_observer_before_writes(self):
        fake = FakeMSP()
        with tempfile.TemporaryDirectory() as temp, self.assertRaises(basket.BasketError):
            basket.Controller(fake, clock=Clock()).record(basket.make_plan("roll-zero-check"), temp)
        self.assertFalse(fake.writes)

    def test_zero_check_observes_every_plateau_and_restores_exact_config(self):
        fake = FakeMSP()
        observed = []
        def observe(event):
            observed.append(event["target_us"])
            return {"samples": 1000, "corrected_mean_g": [1, 0, 0], "corrected_mean_norm_g": 1}
        with tempfile.TemporaryDirectory() as temp:
            report = basket.Controller(fake, clock=Clock()).record(
                basket.make_plan("roll-zero-check", settle_s=.25, measure_s=1), temp, plateau_observer=observe)
        self.assertEqual(tuple(observed), basket.ROLL_ZERO_TARGETS)
        self.assertTrue(report["sequence_complete"])
        self.assertTrue(report["servo_restored_exact"])
        self.assertEqual(bytes(fake.table), fake.original)
        self.assertTrue(all("external_observation" in q for q in report["plateau_quality"]))
        self.assertTrue({command for command, _ in fake.trace} <= {1, 2, 42, 90, 104, 119, 120, 150, 160, 212, 254})

    def test_zero_check_external_failure_or_bad_norm_stops_and_restores(self):
        for failure in ("missing", "gross", "nonfinite"):
            fake = FakeMSP()
            def observe(event):
                if event["target_us"] == 1500:
                    if failure == "missing":
                        raise basket.BasketError("Injected external observation failure")
                    return {"corrected_mean_norm_g": 1.07 if failure == "gross" else float("nan")}
                return {"corrected_mean_norm_g": 1}
            with tempfile.TemporaryDirectory() as temp:
                with self.subTest(failure=failure), self.assertRaises(basket.BasketError):
                    basket.Controller(fake, clock=Clock()).record(
                        basket.make_plan("roll-zero-check", settle_s=.25, measure_s=1), temp, plateau_observer=observe)
                report = json.loads((Path(temp) / "fc-run.json").read_text())
                self.assertTrue(report["servo_restored_exact"])
                self.assertEqual(len(report["events"]), 5)
                self.assertEqual(bytes(fake.table), fake.original)


class ReversalTests(unittest.TestCase):
    def test_high_retry_fixed_sequence_bounds_and_longer_watchdog(self):
        expected = [1420, 1440, 1460, 1480, 1500, 1520, 1540, 1560, 1580, 1600, 1620, 1640,
                    1650, 1640, 1650, 1660, 1650, 1630, 1610, 1590, 1570, 1550, 1530,
                    1510, 1490, 1470, 1450, 1430, 1420]
        plan = basket.make_plan("reversal-high-check", settle_s=2, measure_s=4)
        self.assertEqual(basket.validate_plan(plan), expected)
        self.assertEqual(len(expected), 29)
        self.assertTrue(all(1100 <= t <= 1700 for t in expected))
        self.assertLessEqual(max(abs(b - a) for a, b in zip(expected, expected[1:])), 20)
        self.assertEqual(basket.capture_budget_seconds(plan), 233)
        self.assertFalse(plan["calibration_refit_allowed"])
        self.assertEqual(plan["dataset_role"], "held-out-reversal-validation")
        plan["plateaus"][15]["target_us"] = 1659
        with self.assertRaises(basket.BasketError):
            basket.validate_plan(plan)

    def test_exact_frozen_sequence_bounds_metadata_and_cli(self):
        expected = [1420, 1440, 1460, 1480, 1500, 1512, 1517, 1512, 1507, 1512, 1522, 1512,
                    1502, 1512, 1532, 1512, 1492, 1512, 1532, 1552, 1532, 1512, 1492, 1472,
                    1492, 1512, 1492, 1472, 1452, 1432, 1412, 1392, 1372, 1352, 1332, 1312,
                    1300, 1310, 1300, 1290, 1300, 1320, 1340, 1360, 1380, 1400, 1420, 1440,
                    1460, 1480, 1500, 1520, 1540, 1560, 1580, 1600, 1620, 1640, 1650, 1640,
                    1650, 1660, 1650, 1630, 1610, 1590, 1570, 1550, 1530, 1510, 1490, 1470,
                    1450, 1430, 1420]
        plan = basket.make_plan("reversal-check")
        self.assertEqual(basket.validate_plan(plan), expected)
        self.assertEqual(len(expected), 75)
        self.assertLessEqual(max(abs(b - a) for a, b in zip(expected, expected[1:])), 20)
        self.assertTrue(all(1100 <= target <= 1700 for target in expected))
        self.assertFalse(plan["calibration_refit_allowed"])
        self.assertEqual(plan["dataset_role"], "held-out-reversal-validation")
        self.assertEqual({p["signed_excursion_us"] for p in plan["plateaus"]}, {None, 5, -5, 10, -10, 20, -20, 40, -40})
        self.assertEqual(basket.capture_budget_seconds(plan), 330)
        args = basket.arguments(["--fc-port", "fc", "--teensy-port", "teensy", "--teensy-serial", "123",
                                 "--fc-uid", "00" * 12, "--profile", "profile", "--validation-report", "report",
                                 "--output", "out", "--execute", "--setup-confirmed", "--plan-kind", "reversal-check"])
        self.assertIsNone(args.range_authorization)
        for replacement in (1516, 1701):
            changed = basket.make_plan("reversal-check")
            changed["plateaus"][6]["target_us"] = replacement
            with self.assertRaises(basket.BasketError):
                basket.validate_plan(changed)
        changed = basket.make_plan("reversal-check")
        changed["calibration_refit_allowed"] = True
        with self.assertRaises(basket.BasketError):
            basket.validate_plan(changed)

    def test_reversal_requires_paired_observer_before_any_write(self):
        for kind in ("reversal-check", "reversal-high-check"):
            fake = FakeMSP()
            with self.subTest(kind=kind), tempfile.TemporaryDirectory() as temp, self.assertRaises(basket.BasketError):
                basket.Controller(fake, clock=Clock()).record(basket.make_plan(kind), temp)
            self.assertFalse(fake.writes)

    def test_all_reversal_plateaus_preserve_history_and_restore_exactly(self):
        for kind, targets in (("reversal-check", basket.REVERSAL_TARGETS), ("reversal-high-check", basket.REVERSAL_HIGH_TARGETS)):
            fake, observed = FakeMSP(), []
            def observe(event):
                observed.append((event["target_us"], event["block"], event["measurement_role"]))
                return {"samples": 1000, "corrected_mean_g": [1, 0, 0], "corrected_mean_norm_g": 1}
            with self.subTest(kind=kind), tempfile.TemporaryDirectory() as temp:
                report = basket.Controller(fake, clock=Clock()).record(
                    basket.make_plan(kind, settle_s=.25, measure_s=1), temp, plateau_observer=observe)
            self.assertEqual(tuple(row[0] for row in observed), targets)
            self.assertEqual(len(report["events"]), len(targets))
            self.assertTrue(report["sequence_complete"])
            self.assertTrue(report["servo_restored_exact"])
            self.assertFalse(report["calibration_refit_allowed"])
            self.assertEqual(report["dataset_role"], "held-out-reversal-validation")
            self.assertEqual(bytes(fake.table), fake.original)
            self.assertTrue({command for command, _ in fake.trace} <= {1, 2, 42, 90, 104, 119, 120, 150, 160, 212, 254})

    def test_external_failures_stop_reversal_and_restore_without_refitting(self):
        for kind, fail_target, event_count in (("reversal-check", 1517, 7), ("reversal-high-check", 1660, 16)):
            for failure in ("missing", "gross", "nonfinite"):
                fake = FakeMSP()
                def observe(event):
                    if event["target_us"] == fail_target:
                        if failure == "missing":
                            raise basket.BasketError("Injected paired observation failure")
                        return {"corrected_mean_norm_g": 1.07 if failure == "gross" else float("nan")}
                    return {"corrected_mean_norm_g": 1}
                with tempfile.TemporaryDirectory() as temp:
                    with self.subTest(kind=kind, failure=failure), self.assertRaises(basket.BasketError):
                        basket.Controller(fake, clock=Clock()).record(
                            basket.make_plan(kind, settle_s=.25, measure_s=1), temp, plateau_observer=observe)
                    report = json.loads((Path(temp) / "fc-run.json").read_text())
                    self.assertTrue(report["servo_restored_exact"])
                    self.assertEqual(len(report["events"]), event_count)
                    self.assertFalse(report["calibration_refit_allowed"])
                    self.assertEqual(bytes(fake.table), fake.original)


class FullSweepTests(unittest.TestCase):
    def evidence(self, root):
        center = root / "center"
        center.mkdir()
        fake = FakeMSP()
        basket.Controller(fake, clock=Clock()).record(basket.make_plan(settle_s=.25, measure_s=1), center)
        capture.record_stream([(static_stream() + b"A", 123, 456)], center / "adxl")
        binding = {"profile_sha256": "a" * 64, "validation_sha256": "b" * 64}
        (center / "run.json").write_text(json.dumps({"mode": "execute", "sequence_complete": True, **binding}))
        plan = basket.make_plan("full-sweep", settle_s=.25, measure_s=1)
        plan["servo_index"] = 5
        auth = {"schema_version": 1, "action": "basket-full-sweep", "user_authorized_range_us": [1100, 1700],
                "fc_uid_hex": fake.uid.hex(), "servo_index": 5, **binding, "plan_sha256": basket.plan_fingerprint(plan),
                "center_session": str(center), "center_success_verified": True,
                "center_files_sha256": {name: basket.sha256(center / name) for name in
                                        ("run.json", "fc-run.json", "adxl/summary.json", "original-servo-table.bin")},
                "response_guards": {"corrected_slope_sign": -1, "stage_span_us": 100,
                                    "minimum_stage_delta_counts": 20, "opposite_tolerance_counts": 5,
                                    "minimum_stage_gravity_change_g": .03, "maximum_mean_norm_error_g": .06}}
        path = root / "authorization.json"
        path.write_text(json.dumps(auth))
        proof = basket.load_range_authorization(path, basket.sha256(path), plan, fake.uid.hex(), 5, binding)
        return plan, proof, fake, path, binding

    @staticmethod
    def observer(event):
        angle = (1420 - event["target_us"]) * .0017
        return {"samples": 1000, "corrected_mean_g": [math.cos(angle), math.sin(angle), 0],
                "corrected_mean_norm_g": 1.0}

    def test_full_sweep_requires_exact_authorization_and_center_evidence(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            plan, proof, fake, path, binding = self.evidence(root)
            self.assertEqual(len(basket.validate_plan(plan, full_range_authorized=True)), 61)
            with self.assertRaises(basket.BasketError):
                basket.validate_plan(plan)
            with self.assertRaises(basket.BasketError):
                basket.load_range_authorization(path, "0" * 64, plan, fake.uid.hex(), 5, binding)
            changed = json.loads(path.read_text())
            changed["center_success_verified"] = False
            path.write_text(json.dumps(changed))
            with self.assertRaises(basket.BasketError):
                basket.load_range_authorization(path, basket.sha256(path), plan, fake.uid.hex(), 5, binding)
            changed["center_success_verified"] = True
            path.write_text(json.dumps(changed))
            (root / "center/fc-run.json").write_text("{}")
            with self.assertRaises(basket.BasketError):
                basket.load_range_authorization(path, basket.sha256(path), plan, fake.uid.hex(), 5, binding)

    def test_plan_binding_normalizes_number_formatting_but_rejects_changed_steps(self):
        a = basket.make_plan("full-sweep", settle_s=1, measure_s=2)
        b = basket.make_plan("full-sweep", settle_s=1.0, measure_s=2.0)
        self.assertEqual(basket.plan_fingerprint(a), basket.plan_fingerprint(b))
        b["measure_s"] = 3
        self.assertNotEqual(basket.plan_fingerprint(a), basket.plan_fingerprint(b))

    def test_authorized_bidirectional_run_emits_progress_and_restores_exactly(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            plan, proof, fake, _, _ = self.evidence(root)
            fake.debug_hook = lambda state, values: (3100 - state.pwm, state.pwm, 100, 3000 - state.pwm, *values[4:])
            controller = basket.Controller(fake, clock=Clock())
            out = root / "full"
            out.mkdir()
            seen = []
            report = controller.record(plan, out, range_authorization=proof, plateau_observer=self.observer,
                                       progress_callback=seen.append)
            self.assertTrue(report["sequence_complete"])
            self.assertTrue(report["servo_restored_exact"])
            self.assertEqual(bytes(fake.table), fake.original)
            complete = [row for row in seen if row["event"] == "plateau_complete"]
            self.assertEqual(len(complete), 61)
            self.assertTrue(any(row.get("stage_checkpoint") for row in complete))
            self.assertEqual(len((out / "progress.jsonl").read_text().splitlines()), len(seen))
            self.assertEqual({row["event"] for row in seen}, {"command_start", "plateau_complete", "configuration_restored"})

    def test_stall_and_opposite_response_stop_before_expanding_and_restore(self):
        for response in ("stall", "opposite", "external-stall"):
            with tempfile.TemporaryDirectory() as temp:
                root = Path(temp)
                plan, proof, fake, _, _ = self.evidence(root)
                if response == "opposite":
                    fake.debug_hook = lambda state, values: (state.pwm + 100, state.pwm, 100, state.pwm, *values[4:])
                elif response == "external-stall":
                    fake.debug_hook = lambda state, values: (3100 - state.pwm, state.pwm, 100, 3000 - state.pwm, *values[4:])
                controller = basket.Controller(fake, clock=Clock())
                out = root / "failed"
                out.mkdir()
                observe = (lambda event: {"samples": 1000, "corrected_mean_g": [1, 0, 0], "corrected_mean_norm_g": 1}) if response == "external-stall" else self.observer
                with self.subTest(response=response), self.assertRaises(basket.BasketError):
                    controller.record(plan, out, range_authorization=proof, plateau_observer=observe)
                report = json.loads((out / "fc-run.json").read_text())
                self.assertTrue(report["servo_restored_exact"])
                self.assertLessEqual(len(report["events"]), 6)
                self.assertEqual(bytes(fake.table), fake.original)

    def test_full_range_no_proof_never_writes_and_sigterm_raises_interrupt(self):
        fake = FakeMSP()
        with tempfile.TemporaryDirectory() as temp, self.assertRaises(basket.BasketError):
            basket.Controller(fake, clock=Clock()).record(basket.make_plan("full-sweep"), temp)
        self.assertFalse(fake.writes)
        with self.assertRaises(KeyboardInterrupt):
            basket.interrupted(signal.SIGTERM, None)

    def test_child_watchdog_budget_covers_per_plateau_msp_configuration_latency(self):
        self.assertEqual(basket.capture_budget_seconds(basket.make_plan("full-sweep")), 274)
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            plan, proof, fake, _, _ = self.evidence(root)
            clock = Clock()
            original_request = fake.request
            def delayed_request(command, payload=b"", **kwargs):
                # Three configuration transactions before each hold add225ms;
                # their real elapsed time is outside the planned plateau timer.
                if command in (basket.SERVO_CONFIGURATIONS, basket.SET_SERVO_CONFIGURATION):
                    clock.sleep(.075)
                return original_request(command, payload, **kwargs)
            fake.request = delayed_request
            fake.debug_hook = lambda state, values: (3100 - state.pwm, state.pwm, 100, 3000 - state.pwm, *values[4:])
            controller = basket.Controller(fake, clock=clock)
            start = clock.monotonic()
            budget = basket.capture_budget_seconds(plan)
            def watchdog():
                if clock.monotonic() - start >= budget:
                    raise basket.BasketError("Child capture deadline expired")
            out = root / "latency"
            out.mkdir()
            report = controller.record(plan, out, health_check=watchdog,
                                       range_authorization=proof, plateau_observer=self.observer)
            elapsed = clock.monotonic() - start
            old_budget = len(plan["plateaus"]) * (plan["settle_s"] + plan["measure_s"]) + 10
            self.assertGreater(elapsed, old_budget)
            self.assertLess(elapsed, budget)
            self.assertTrue(report["sequence_complete"])
            self.assertTrue(report["servo_restored_exact"])
            self.assertEqual(bytes(fake.table), fake.original)

    def test_msp_rejects_bad_checksum_and_foreign_command(self):
        class Serial:
            def __init__(self):
                self.data = bytearray()
            def reset_input_buffer(self):
                self.data.clear()
            @staticmethod
            def reply(command, content, corrupt=False):
                body = bytes((len(content), command)) + content
                checksum = 0
                for byte in body:
                    checksum ^= byte
                return b"$M>" + body + bytes((checksum ^ int(corrupt),))
            def write(self, _):
                self.data.extend(self.reply(2, b"WRNG", True) + self.reply(3, b"OTHR") + self.reply(2, b"BTFL"))
            def flush(self):
                pass
            @property
            def in_waiting(self):
                return len(self.data)
            def read(self, count):
                data = bytes(self.data[:count])
                del self.data[:count]
                return data
        self.assertEqual(basket.MSP("explicit", serial_connection=Serial()).request(2), b"BTFL")


if __name__ == "__main__":
    unittest.main()
