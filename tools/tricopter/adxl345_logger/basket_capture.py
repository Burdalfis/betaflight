#!/usr/bin/env python3
"""Settled basket calibration capture, with a preview-only default.

Only MSP_SET_SERVO_CONFIGURATION can write to the FC. No arming, motor-test,
EEPROM, debug-mode or Blackbox writes are implemented. A full sweep additionally
requires a hashed range authorization bound to a successful center check.
Raw PB0/PC0 are diagnostic; measured position is always signed PB0-PC0.
Host monotonic times associate long settled windows, not synchronized samples.
"""

import argparse
from collections import deque
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import statistics
import struct
import subprocess
import sys
import time

import capture


API_VERSION = 1
FC_VARIANT = 2
MOTOR = 104
BOXIDS = 119
SERVO_CONFIGURATIONS = 120
STATUS_EX = 150
UID = 160
ADVANCED_CONFIG = 90
MIXER_CONFIG = 42
SET_SERVO_CONFIGURATION = 212
DEBUG = 254
EXPECTED_API = bytes((0, 1, 48))
TRI_GROUND_MODE = 103
SERVO_RECORD = struct.Struct("<HHHbBI")
MINIMUM_US, MAXIMUM_US, CENTER_US = 1100, 1700, 1420
CENTER_TARGETS = (1420, 1400, 1420, 1440, 1420)
ROLL_ZERO_TARGETS = (1420, 1440, 1460, 1480, 1500, 1510, 1519, 1525, 1545, 1560,
                     1540, 1520, 1510, 1504, 1495, 1480, 1460, 1440, 1420)
REVERSAL_BLOCKS = (
    ("travel-to-1512", 1512, None, (1420, 1440, 1460, 1480, 1500, 1512)),
    ("center-plus-5", 1512, 5, (1517, 1512)),
    ("center-minus-5", 1512, -5, (1507, 1512)),
    ("center-plus-10", 1512, 10, (1522, 1512)),
    ("center-minus-10", 1512, -10, (1502, 1512)),
    ("center-plus-20", 1512, 20, (1532, 1512)),
    ("center-minus-20", 1512, -20, (1492, 1512)),
    ("center-plus-40", 1512, 40, (1532, 1552, 1532, 1512)),
    ("center-minus-40", 1512, -40, (1492, 1472, 1492, 1512)),
    ("travel-to-1300", 1300, None, (1492, 1472, 1452, 1432, 1412, 1392, 1372, 1352, 1332, 1312, 1300)),
    ("low-plus-10", 1300, 10, (1310, 1300)),
    ("low-minus-10", 1300, -10, (1290, 1300)),
    ("travel-to-1650", 1650, None, tuple(range(1320, 1641, 20)) + (1650,)),
    ("high-minus-10", 1650, -10, (1640, 1650)),
    ("high-plus-10", 1650, 10, (1660, 1650)),
    ("return-to-1420", 1420, None, tuple(range(1630, 1429, -20)) + (1420,)),
)
REVERSAL_TARGETS = tuple(target for _, _, _, targets in REVERSAL_BLOCKS for target in targets)
REVERSAL_HIGH_BLOCKS = (
    ("travel-to-1650", 1650, None, (1420, 1440, 1460, 1480, 1500, 1520, 1540, 1560, 1580, 1600, 1620, 1640, 1650)),
    ("high-minus-10", 1650, -10, (1640, 1650)),
    ("high-plus-10", 1650, 10, (1660, 1650)),
    ("return-to-1420", 1420, None, (1630, 1610, 1590, 1570, 1550, 1530, 1510, 1490, 1470, 1450, 1430, 1420)),
)
REVERSAL_HIGH_TARGETS = tuple(target for _, _, _, targets in REVERSAL_HIGH_BLOCKS for target in targets)
FULL_TARGETS = tuple([CENTER_US] + list(range(1400, 1099, -20)) + list(range(1120, 1701, 20)) + list(range(1680, 1419, -20)))


class BasketError(RuntimeError):
    pass


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    with Path(path).open("x") as handle:
        json.dump(value, handle, indent=2, allow_nan=False)
        handle.write("\n")


def resolved_ports(fc_port, teensy_port, require_exists=False):
    fc = Path(fc_port).expanduser().resolve(strict=require_exists)
    teensy = Path(teensy_port).expanduser().resolve(strict=require_exists)
    if fc == teensy or (fc.exists() and teensy.exists() and fc.samefile(teensy)):
        raise BasketError("FC and Teensy ports must resolve to different devices")
    return str(fc), str(teensy)


def verify_usb_identity(fc_port, teensy_port, expected_serial, ports=None):
    fc, teensy = resolved_ports(fc_port, teensy_port, require_exists=True)
    if ports is None:
        from serial.tools import list_ports
        ports = list_ports.comports()
    matches = [p for p in ports if str(Path(p.device).resolve()) == teensy]
    if (len(matches) != 1 or matches[0].vid != 0x16c0
            or str(matches[0].serial_number) != expected_serial):
        raise BasketError("Explicit Teensy port does not match the expected USB serial identity")
    if any(str(Path(p.device).resolve()) == fc and p.vid == 0x16c0 for p in ports):
        raise BasketError("The FC port identifies as a Teensy")
    return {"fc_resolved": fc, "teensy_resolved": teensy,
            "teensy_serial": expected_serial, "teensy_vid": matches[0].vid,
            "teensy_pid": matches[0].pid}


def load_validated_profile(profile_path, report_path):
    """Check bindings in the caller's independent-validation aggregate.

    The aggregate is an attestation produced by the offline validation workflow;
    this function does not replay its tilted captures or certify angular accuracy.
    """
    profile_path, report_path = Path(profile_path), Path(report_path)
    profile = json.loads(profile_path.read_text())
    report = json.loads(report_path.read_text())
    for key in ("bias_counts", "counts_per_g"):
        values = profile.get(key)
        if (not isinstance(values, list) or len(values) != 3
                or any(isinstance(v, bool) or not isinstance(v, (float, int))
                       or not math.isfinite(v) for v in values)):
            raise BasketError(f"Invalid calibration {key}")
    if any(v <= 0 for v in profile["counts_per_g"]):
        raise BasketError("Calibration sensitivities must be positive")
    if profile.get("schema_version") != 1 or profile.get("device_writes") is not False:
        raise BasketError("Expected an offline version-1 calibration profile")
    if profile.get("sensor_register_tuple_devid_bw_format_fault") != [0xe5, 0x0d, 0x08, 0]:
        raise BasketError("Calibration must match the logger's E5/800Hz/full-resolution configuration")
    required = report.get("required_tilted_poses")
    records = report.get("captures")
    if (report.get("profile_sha256") != sha256(profile_path)
            or report.get("independent_static_validation_pass") is not True
            or isinstance(required, bool) or not isinstance(required, int) or required not in (4, 6)
            or not isinstance(records, list) or len(records) < required):
        raise BasketError("Profile lacks matching independent static validation")
    if required == 4 and (report.get("policy_version") != 2
                         or report.get("stationarity_policy") != "static-mean-v2"
                         or not isinstance(report.get("screening_criteria"), dict)
                         or not report["screening_criteria"]):
        raise BasketError("Four-pose validation requires the explicit static-mean-v2 policy and criteria")
    ids, digests = [], []
    training_hashes = {m.get("input_sha256", {}).get("capture.bin")
                       for m in profile.get("training", {}).get("pose_metrics", [])}
    for record in records:
        name, digest = record.get("pose_id"), record.get("capture_sha256")
        if (not isinstance(name, str) or not name or record.get("validation_pass") is not True
                or not isinstance(digest, str) or len(digest) != 64
                or any(c not in "0123456789abcdef" for c in digest)):
            raise BasketError("Invalid tilted-validation capture entry")
        ids.append(name)
        digests.append(digest)
        if digest in training_hashes:
            raise BasketError("A training capture cannot serve as independent tilted validation")
    if len(ids) != len(set(ids)) or len(digests) != len(set(digests)):
        raise BasketError("Tilted-validation IDs and raw capture hashes must be unique")
    return {"profile": profile, "validation": report,
            "profile_sha256": sha256(profile_path), "validation_sha256": sha256(report_path)}


def make_plan(kind="center-check", settle_s=1.0, measure_s=2.0, sample_hz=50.0):
    for value, lower, upper in ((settle_s, .25, 10), (measure_s, 1, 30), (sample_hz, 10, 200)):
        if not math.isfinite(value) or not lower <= value <= upper:
            raise BasketError("Invalid settling, measurement duration or sample rate")
    if kind == "center-check":
        targets = list(CENTER_TARGETS)
    elif kind == "roll-zero-check":
        targets = list(ROLL_ZERO_TARGETS)
    elif kind == "reversal-check":
        targets = list(REVERSAL_TARGETS)
    elif kind == "reversal-high-check":
        targets = list(REVERSAL_HIGH_TARGETS)
    elif kind in ("sweep-preview", "full-sweep"):
        targets = list(FULL_TARGETS)
    else:
        raise BasketError("Unknown plan kind")
    plan = {"kind": kind, "execution_supported": kind != "sweep-preview",
            "settle_s": settle_s, "measure_s": measure_s, "sample_hz": sample_hz,
            "plateaus": [{"plateau_id": i, "target_us": target} for i, target in enumerate(targets)],
            "position_source": "signed TRI_GROUND debug[3] = PB0-PC0 only",
            "timing": "Host monotonic association of settled plateaus; no exact FC/Teensy clock synchronization or dynamic timing claim",
            "temporary_configuration": "Selected servo middle=target, rate=0; preserve all other raw fields and restore the exact original record in finally",
            "physical_setup": "Actual execution requires physical setup confirmation outside this tool"}
    if kind in ("reversal-check", "reversal-high-check"):
        plan.update(dataset_role="held-out-reversal-validation", calibration_refit_allowed=False,
                    approach_history="Retain all preceding travel and loop commands; do not reset state at reversals")
        plateaus = []
        blocks = REVERSAL_BLOCKS if kind == "reversal-check" else REVERSAL_HIGH_BLOCKS
        for block, center, excursion, block_targets in blocks:
            for target in block_targets:
                role = ("center-arrival" if target == center else "travel") if excursion is None else (
                    "center-return" if target == center else "loop-extreme" if target == center + excursion else "loop-intermediate")
                plateaus.append({"plateau_id": len(plateaus), "target_us": target, "block": block,
                                 "nominal_center_us": center, "signed_excursion_us": excursion, "measurement_role": role})
        plan["plateaus"] = plateaus
    validate_plan(plan, execute=False)
    return plan


def validate_plan(plan, execute=True, full_range_authorized=False):
    if plan["kind"] in ("reversal-check", "reversal-high-check") and (plan.get("dataset_role") != "held-out-reversal-validation"
                                             or plan.get("calibration_refit_allowed") is not False):
        raise BasketError("Reversal records must remain held-out validation, not calibration refitting")
    for key, lower, upper in (("settle_s", .25, 10), ("measure_s", 1, 30), ("sample_hz", 10, 200)):
        value = plan[key]
        if not math.isfinite(value) or not lower <= value <= upper:
            raise BasketError("Invalid settling, measurement duration or sample rate")
    targets = [p["target_us"] for p in plan["plateaus"]]
    if (not targets or any(type(t) is not int or not MINIMUM_US <= t <= MAXIMUM_US for t in targets)
            or targets[0] != CENTER_US or targets[-1] != CENTER_US
            or any(abs(b - a) > 20 for a, b in zip(targets, targets[1:]))):
        raise BasketError("Plan exceeds bounded targets or 20-us step limit")
    if execute and not ((plan["kind"] == "center-check" and tuple(targets) == CENTER_TARGETS)
                        or (plan["kind"] == "roll-zero-check" and tuple(targets) == ROLL_ZERO_TARGETS)
                        or (plan["kind"] == "reversal-check" and tuple(targets) == REVERSAL_TARGETS)
                        or (plan["kind"] == "reversal-high-check" and tuple(targets) == REVERSAL_HIGH_TARGETS)
                        or (plan["kind"] == "full-sweep" and tuple(targets) == FULL_TARGETS and full_range_authorized)):
        raise BasketError("Execution requires a fixed center/roll-zero/reversal plan or authorized full sweep; sweep-preview never executes")
    return targets


def plan_fingerprint(plan):
    """Canonical executable-plan hash; independent of JSON whitespace/int-vs-float formatting."""
    fields = {"kind": plan["kind"], "servo_index": plan.get("servo_index", 5),
              **{key: float(plan[key]) for key in ("settle_s", "measure_s", "sample_hz")},
              "targets_us": [entry["target_us"] for entry in plan["plateaus"]]}
    return hashlib.sha256(json.dumps(fields, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def capture_budget_seconds(plan):
    """Child watchdog only; the parent interrupts capture when the sequence ends.

    Plateau timers exclude pre-command MSP/configuration overhead. Reserve one
    additional second per plateau plus 30 seconds for startup/restoration/USB.
    """
    return len(plan["plateaus"]) * (plan["settle_s"] + plan["measure_s"] + 1.0) + 30


def load_range_authorization(path, expected_sha256, plan, fc_uid_hex, servo_index, profile_binding):
    """Verify root's explicit range/response authorization against immutable center evidence.

    A successful center capture and its old source fingerprint remain historical
    evidence; this deliberately does not compare that old tool hash to this file.
    """
    if sha256(path) != expected_sha256:
        raise BasketError("Range authorization file does not match the explicit SHA-256")
    auth = json.loads(Path(path).read_text())
    if (auth.get("schema_version") != 1 or auth.get("action") != "basket-full-sweep"
            or auth.get("user_authorized_range_us") != [MINIMUM_US, MAXIMUM_US]
            or auth.get("center_success_verified") is not True
            or auth.get("fc_uid_hex") != fc_uid_hex.lower() or auth.get("servo_index") != servo_index
            or auth.get("profile_sha256") != profile_binding["profile_sha256"]
            or auth.get("validation_sha256") != profile_binding["validation_sha256"]
            or auth.get("plan_sha256") != plan_fingerprint(plan)):
        raise BasketError("Full-range authorization is missing or bound to another plan/controller/calibration")
    center = Path(auth["center_session"])
    required = {"run.json", "fc-run.json", "adxl/summary.json", "original-servo-table.bin"}
    fingerprints = auth.get("center_files_sha256", {})
    if not center.is_absolute() or not required.issubset(fingerprints):
        raise BasketError("Authorization must bind the absolute center session and required evidence files")
    for name in required:
        if sha256(center / name) != fingerprints[name]:
            raise BasketError(f"Center evidence changed: {name}")
    master = json.loads((center / "run.json").read_text())
    fc = json.loads((center / "fc-run.json").read_text())
    original = (center / "original-servo-table.bin").read_bytes()
    if (master.get("mode") != "execute" or master.get("sequence_complete") is not True or master.get("error")
            or master.get("profile_sha256") != auth["profile_sha256"]
            or master.get("validation_sha256") != auth["validation_sha256"]
            or fc.get("sequence_complete") is not True or fc.get("servo_restored_exact") is not True
            or fc.get("error") or fc.get("critical_restore_failure")
            or fc.get("identity", {}).get("uid_hex") != auth["fc_uid_hex"]
            or fc.get("final_servo_table_hex") != original.hex()
            or tuple(event["target_us"] for event in fc.get("events", [])) != CENTER_TARGETS):
        raise BasketError("Bound center session is not a successful exactly restored center check")
    validate_adxl_summary(center / "adxl")
    guards = dict(auth.get("response_guards", {}))
    if (type(guards.get("corrected_slope_sign")) is not int or guards["corrected_slope_sign"] not in (-1, 1)
            or guards.get("stage_span_us") != 100):
        raise BasketError("Range response guards need the measured polarity and 100-us stages")
    guards.setdefault("maximum_mean_norm_error_g", .06)
    guards.setdefault("minimum_stage_gravity_change_g", .01)
    for key, minimum, maximum in (("minimum_stage_delta_counts", 0, 4095), ("opposite_tolerance_counts", -1, 4095),
                                   ("maximum_mean_norm_error_g", 0, .1), ("minimum_stage_gravity_change_g", 0, .2)):
        value = guards.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not minimum < value <= maximum:
            raise BasketError(f"Invalid frozen range response guard: {key}")
    return {"authorization": auth, "authorization_sha256": expected_sha256,
            "validated_for_plan_sha256": auth["plan_sha256"], "fc_uid_hex": auth["fc_uid_hex"],
            "center_original_servo_table_hex": original.hex(), "response_guards": guards}


def crc8(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0xd5) & 255 if crc & 128 else (crc << 1) & 255
    return crc


class MSP:
    """Exclusive single-request MSP transport; no implicit port or CLI mode."""
    def __init__(self, port, serial_connection=None):
        if serial_connection is None:
            import serial
            serial_connection = serial.Serial(port, 115200, timeout=.005, exclusive=True)
        self.serial = serial_connection

    def close(self):
        self.serial.close()

    def request(self, command, payload=b"", timeout=.5):
        self.serial.reset_input_buffer()
        if command <= 255 and len(payload) <= 255:
            body = bytes((len(payload), command)) + payload
            checksum = 0
            for byte in body:
                checksum ^= byte
            packet = b"$M<" + body + bytes((checksum,))
        else:
            body = struct.pack("<BHH", 0, command, len(payload)) + payload
            packet = b"$X<" + body + bytes((crc8(body),))
        self.serial.write(packet)
        self.serial.flush()
        buf = bytearray()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            buf.extend(self.serial.read(max(1, self.serial.in_waiting)))
            while len(buf) >= 6:
                if buf[:2] not in (b"$M", b"$X"):
                    del buf[0]
                    continue
                if buf[1] == ord("M"):
                    size, returned = buf[3], buf[4]
                    total, start = size + 6, 5
                    if len(buf) < total:
                        break
                    checksum = 0
                    for byte in buf[3:total]:
                        checksum ^= byte
                    valid = checksum == 0
                else:
                    if len(buf) < 9:
                        break
                    _, returned, size = struct.unpack_from("<BHH", buf, 3)
                    if size > 4096:
                        del buf[0]
                        continue
                    total, start = size + 9, 8
                    if len(buf) < total:
                        break
                    valid = crc8(buf[3:total - 1]) == buf[total - 1]
                reply = bytes(buf[:total])
                del buf[:total]
                if not valid or returned != command or reply[2] not in (ord(">"), ord("!")):
                    continue
                if reply[2] == ord("!"):
                    raise BasketError(f"FC rejected MSP command {command}")
                return reply[start:-1]
        raise BasketError(f"MSP command {command} timed out")


def ground_sample(payload):
    if len(payload) != 16:
        raise BasketError("TRI_GROUND requires exactly eight signed 16-bit debug fields")
    values = struct.unpack("<8h", payload)
    wiper, pwm, ground, corrected, age, flags, _, _ = values
    if (flags < 0 or flags & ~0x7ff or flags & 7 != 7 or not 0 < wiper < 4095 or not 0 <= ground < 4095
            or corrected != wiper - ground or age < 0
            or flags & ((1 << 8) | (1 << 10)) or not MINIMUM_US <= pwm <= MAXIMUM_US):
        raise BasketError("Unsupported/unready, railed or invalid PB0-PC0 schema")
    if flags & 8 and age > 10:
        raise BasketError("Inconsistent TRI_GROUND freshness and encoded age")
    return values, bool(flags & 8)


class Controller:
    def __init__(self, msp, expected_uid=None, servo_index=5, clock=time):
        if type(servo_index) is not int or not 0 <= servo_index <= 7:
            raise BasketError("Servo index must be 0..7")
        self.msp, self.clock, self.servo_index = msp, clock, servo_index
        self.identity = self.read_identity()
        if expected_uid is not None and self.identity["uid_hex"] != expected_uid.lower():
            raise BasketError("FC UID does not match the explicitly expected controller")
        boxes = msp.request(BOXIDS)
        if 0 not in boxes or boxes.index(0) >= 32:
            raise BasketError("Cannot locate ARM within the supported status bitmask")
        self.arm_index = boxes.index(0)
        self.original = msp.request(SERVO_CONFIGURATIONS)
        start = servo_index * SERVO_RECORD.size
        if len(self.original) % SERVO_RECORD.size or len(self.original) < start + SERVO_RECORD.size:
            raise BasketError("Invalid servo-configuration payload")
        self.original_record = self.original[start:start + SERVO_RECORD.size]
        self.expected_table = self.original
        self.allowed_restore_tables = {self.original}
        self.motion_targets = set(CENTER_TARGETS)
        self.commanded_target = CENTER_US
        self.fields = SERVO_RECORD.unpack(self.original_record)
        if self.fields[:3] != (MINIMUM_US, MAXIMUM_US, CENTER_US) or self.fields[4] != 255:
            raise BasketError("Expected existing 1100/1700/1420 limits and no channel forwarding")
        state = self.guard()
        self.backup = {"identity": self.identity, "servo_index": servo_index,
                       "servo_table_hex": self.original.hex(), "servo_record_hex": self.original_record.hex(),
                       "servo_fields": list(self.fields), "boxids_hex": boxes.hex(), **state}

    def read_identity(self):
        api, variant, uid = self.msp.request(API_VERSION), self.msp.request(FC_VARIANT), self.msp.request(UID)
        if api != EXPECTED_API or variant != b"BTFL" or len(uid) != 12:
            raise BasketError("Expected Betaflight MSP API 1.48 and a complete FC UID")
        return {"api_hex": api.hex(), "variant": "BTFL", "uid_hex": uid.hex()}

    def guard(self, require_ground=True):
        if self.read_identity() != self.identity:
            raise BasketError("FC identity changed; no further writes are safe")
        advanced = self.msp.request(ADVANCED_CONFIG)
        if len(advanced) < 20 or advanced[18] != TRI_GROUND_MODE or advanced[19] <= TRI_GROUND_MODE:
            raise BasketError("TRI_GROUND must already be configured; tool will not change debug mode")
        mixer = self.msp.request(MIXER_CONFIG)
        if len(mixer) != 2 or mixer[0] != 1:
            raise BasketError("The standard TRI mixer must already be configured")
        status = self.msp.request(STATUS_EX)
        if len(status) < 16 or struct.unpack_from("<I", status, 6)[0] & (1 << self.arm_index):
            raise BasketError("FC is armed or its status is unavailable")
        motor_payload = self.msp.request(MOTOR)
        if len(motor_payload) < 6 or len(motor_payload) % 2:
            raise BasketError("Invalid motor-output payload")
        motors = struct.unpack("<" + "H" * (len(motor_payload) // 2), motor_payload)
        if motors[:3] != (1000, 1000, 1000):
            raise BasketError("All three motor outputs must remain 1000")
        result = {"advanced_config_hex": advanced.hex(), "status_hex": status.hex(),
                  "mixer_config_hex": mixer.hex(),
                  "motor_payload_hex": motor_payload.hex(), "motor_outputs": list(motors)}
        if require_ground:
            raw = self.msp.request(DEBUG)
            values, fresh = ground_sample(raw)
            result.update(debug_hex=raw.hex(), debug_values=list(values), position_fresh=fresh)
        return result

    def set_target(self, target):
        if target not in self.motion_targets or abs(target - self.commanded_target) > 20:
            raise BasketError("Target is outside the authorized plan or 20-us step limit")
        minimum, maximum, _, _, forward, reversed_sources = self.fields
        record = SERVO_RECORD.pack(minimum, maximum, target, 0, forward, reversed_sources)
        if self.msp.request(SERVO_CONFIGURATIONS) != self.expected_table:
            raise BasketError("Servo configuration changed outside this session")
        expected = bytearray(self.original)
        start = self.servo_index * SERVO_RECORD.size
        expected[start:start + SERVO_RECORD.size] = record
        self.allowed_restore_tables.add(bytes(expected))
        self.msp.request(SET_SERVO_CONFIGURATION, bytes((self.servo_index,)) + record)
        if self.msp.request(SERVO_CONFIGURATIONS) != bytes(expected):
            raise BasketError("Temporary servo configuration did not read back exactly")
        self.expected_table = bytes(expected)
        self.allowed_restore_tables = {self.expected_table}
        self.commanded_target = target

    def record(self, plan, folder, health_check=lambda: None, range_authorization=None,
               plateau_observer=None, progress_callback=None):
        full = plan["kind"] == "full-sweep"
        paired_check = plan["kind"] in ("roll-zero-check", "reversal-check", "reversal-high-check")
        if paired_check and plateau_observer is None:
            raise BasketError("Fixed settled check requires live paired external observations")
        if full and (not range_authorization or plateau_observer is None
                     or range_authorization["validated_for_plan_sha256"] != plan_fingerprint(plan)
                     or range_authorization["fc_uid_hex"] != self.identity["uid_hex"]
                     or range_authorization["authorization"]["servo_index"] != self.servo_index
                     or range_authorization["center_original_servo_table_hex"] != self.original.hex()):
            raise BasketError("Full sweep needs matching center evidence and live external observations")
        self.motion_targets = set(validate_plan(plan, execute=True, full_range_authorized=full and range_authorization is not None))
        folder = Path(folder)
        write_json(folder / "fc-backup.json", self.backup)
        with (folder / "original-servo-table.bin").open("xb") as handle:
            handle.write(self.original)
        report = {"sequence_complete": False, "restore_required": False, "servo_restored_exact": None,
                  "pid": os.getpid(),
                  "position_source": "signed PB0-PC0 only", "events": [], "plateau_quality": [],
                  "timing": plan["timing"], "identity": self.identity}
        if full:
            report["range_authorization_sha256"] = range_authorization["authorization_sha256"]
            report["response_guards"] = range_authorization["response_guards"]
        elif paired_check:
            report["response_guards"] = {"maximum_mean_norm_error_g": .06}
        if plan["kind"] in ("reversal-check", "reversal-high-check"):
            report.update(dataset_role=plan["dataset_role"], calibration_refit_allowed=False)
        progress = (folder / "progress.jsonl").open("x")
        def emit(event):
            event = {"pid": os.getpid(), **event}
            progress.write(json.dumps(event, allow_nan=False) + "\n")
            progress.flush()
            if progress_callback is not None:
                progress_callback(event)
        previous, anchor, direction = None, None, 0
        error, last_fresh = None, None
        try:
            health_check()
            initial = self.guard()
            if initial["debug_values"][1] != CENTER_US:
                raise BasketError("Initial servo output must already be 1420 before the small center check")
            startup_deadline = self.clock.monotonic() + .1
            while not initial["position_fresh"]:
                if self.clock.monotonic() >= startup_deadline:
                    raise BasketError("No fresh PB0-PC0 sample within 100 ms before motion")
                health_check()
                self.clock.sleep(.005)
                values, fresh = ground_sample(self.msp.request(DEBUG))
                initial.update(debug_values=list(values), position_fresh=fresh)
            last_fresh = self.clock.monotonic()
            with (folder / "fc-samples.csv").open("x", newline="") as handle:
                columns = ["request_host_monotonic_ns", "response_host_monotonic_ns", "plateau_id",
                           "phase", "target_us", "position_fresh", "command_matches", "settled_candidate",
                           "raw_pb0", "final_pwm_us", "raw_pc0", "corrected_pb0_minus_pc0", "pair_age_100us", "flags", "rpm_age_100us", "rpm_invalid_stat"]
                writer = csv.writer(handle)
                writer.writerow(columns)
                for entry in plan["plateaus"]:
                    health_check()
                    state = self.guard()
                    if state["position_fresh"]:
                        last_fresh = self.clock.monotonic()
                    elif self.clock.monotonic() - last_fresh >= .1:
                        raise BasketError("No fresh PB0-PC0 observation for 100 ms before command")
                    target = entry["target_us"]
                    event = dict(entry, command_request_host_monotonic_ns=self.clock.monotonic_ns())
                    report["events"].append(event)
                    emit({"event": "command_start", **event})
                    # Set before the request: an ACK timeout can still mean the write executed.
                    report["restore_required"] = True
                    self.set_target(target)
                    event["command_ack_host_monotonic_ns"] = self.clock.monotonic_ns()
                    start = self.clock.monotonic()
                    settle_end = start + plan["settle_s"]
                    end = settle_end + plan["measure_s"]
                    event.update(settle_end_host_monotonic_ns=int(settle_end * 1e9),
                                 planned_end_host_monotonic_ns=int(end * 1e9))
                    next_guard = start
                    quality = {"plateau_id": entry["plateau_id"], "target_us": target,
                               "measure_rows": 0, "fresh_matching_measure_rows": 0, "stale_rows": 0}
                    report["plateau_quality"].append(quality)
                    position_values = []
                    while self.clock.monotonic() < end:
                        iteration_start = self.clock.monotonic()
                        health_check()
                        if iteration_start >= next_guard:
                            self.guard()
                            next_guard = self.clock.monotonic() + .1
                        before = self.clock.monotonic_ns()
                        values, fresh = ground_sample(self.msp.request(DEBUG))
                        after = self.clock.monotonic_ns()
                        now = self.clock.monotonic()
                        if fresh:
                            last_fresh = now
                        elif now - last_fresh >= .1:
                            raise BasketError("No fresh PB0-PC0 observation for 100 ms")
                        matches = values[1] == target
                        if not matches and now - start > .25:
                            raise BasketError("Final servo PWM did not track the bounded target")
                        measure = now >= settle_end
                        if measure:
                            quality["measure_rows"] += 1
                            quality["fresh_matching_measure_rows"] += int(fresh and matches)
                            if fresh and matches:
                                position_values.append(values[3])
                        quality["stale_rows"] += int(not fresh)
                        writer.writerow([before, after, entry["plateau_id"], "measure" if measure else "settling",
                                         target, int(fresh), int(matches), int(measure and fresh and matches), *values])
                        self.clock.sleep(max(0, 1 / plan["sample_hz"] - (self.clock.monotonic() - iteration_start)))
                    event["plateau_end_host_monotonic_ns"] = self.clock.monotonic_ns()
                    count, accepted = quality["measure_rows"], quality["fresh_matching_measure_rows"]
                    minimum = max(5, math.ceil(plan["measure_s"] * plan["sample_hz"] * .5))
                    quality.update(minimum_fresh_measure_rows=minimum,
                                   fresh_matching_fraction=accepted / count if count else 0)
                    if accepted < minimum or not count or accepted / count < .9:
                        raise BasketError("Too few fresh, command-matching settled-window samples")
                    quality["corrected_count_median"] = statistics.median(position_values)
                    if plateau_observer is not None:
                        health_check()
                        external_observation = plateau_observer(event)
                        if paired_check:
                            norm = external_observation.get("corrected_mean_norm_g")
                            if not isinstance(norm, (int, float)) or not math.isfinite(norm) or abs(norm - 1) > .06:
                                quality["rejected_external_mean_norm"] = repr(norm)
                                raise BasketError("Settled-check external gravity mean exceeded the gross norm guard")
                        quality["external_observation"] = external_observation
                    handle.flush()
                    if full:
                        guards = report["response_guards"]
                        external = quality["external_observation"]
                        if abs(external["corrected_mean_norm_g"] - 1) > guards["maximum_mean_norm_error_g"]:
                            raise BasketError("External gravity mean exceeded the frozen gross-motion norm guard")
                        if previous is not None:
                            step_direction = 1 if target > previous["target_us"] else -1
                            reversal_step = direction != 0 and step_direction != direction
                            if step_direction != direction:
                                anchor, direction = previous, step_direction
                            directed = (quality["corrected_count_median"] - previous["corrected_count_median"]) * guards["corrected_slope_sign"] * direction
                            quality["reversal_hysteresis_step"] = reversal_step
                            if not reversal_step and directed < -guards["opposite_tolerance_counts"]:
                                raise BasketError("Canonical position moved opposite the frozen command polarity")
                            span = abs(target - anchor["target_us"])
                            if span >= guards["stage_span_us"]:
                                moved = (quality["corrected_count_median"] - anchor["corrected_count_median"]) * guards["corrected_slope_sign"] * direction
                                change = math.sqrt(sum((a - b) ** 2 for a, b in zip(external["corrected_mean_g"], anchor["external_observation"]["corrected_mean_g"])))
                                quality["stage_checkpoint"] = {"span_us": span, "directed_counts": moved, "gravity_vector_change_g": change}
                                if moved < guards["minimum_stage_delta_counts"] or change < guards["minimum_stage_gravity_change_g"]:
                                    raise BasketError("Insufficient canonical/external response across a 100-us stage")
                                anchor = quality
                        previous = quality
                    emit({"event": "plateau_complete", **quality})
            self.guard()
            report["sequence_complete"] = True
        except BaseException as caught:
            error = caught
            report["error"] = f"{type(caught).__name__}: {caught}"
            emit({"event": "sequence_stopped", "error": report["error"]})
        finally:
            if report["restore_required"]:
                try:
                    # Restoration uses only pinned identity, existing debug-mode and actuator guards;
                    # a stale/bad ADC observation must not prevent restoring a known raw config.
                    report["pre_restore_guard"] = self.guard(require_ground=False)
                    if self.msp.request(SERVO_CONFIGURATIONS) not in self.allowed_restore_tables:
                        raise BasketError("Unexpected servo configuration; automatic restoration blocked")
                    report["restore_request_host_monotonic_ns"] = self.clock.monotonic_ns()
                    self.msp.request(SET_SERVO_CONFIGURATION, bytes((self.servo_index,)) + self.original_record)
                    final = self.msp.request(SERVO_CONFIGURATIONS)
                    report["final_servo_table_hex"] = final.hex()
                    report["servo_restored_exact"] = final == self.original
                    report["post_restore_guard"] = self.guard(require_ground=False)
                    if not report["servo_restored_exact"]:
                        raise BasketError("Full servo table did not restore exactly")
                except BaseException as restore_error:
                    report["critical_restore_failure"] = f"{type(restore_error).__name__}: {restore_error}"
                    report["servo_restored_exact"] = False
                    if error is None:
                        error = BasketError(report["critical_restore_failure"])
            write_json(folder / "fc-events.json", report["events"])
            write_json(folder / "fc-run.json", report)
            emit({"event": "configuration_restored", "servo_restored_exact": report["servo_restored_exact"],
                  "critical_restore_failure": report.get("critical_restore_failure")})
            progress.close()
        if error is not None:
            raise error
        return report


class AdxlMonitor:
    """Check the child's flushed raw stream before/during actuator commands."""
    def __init__(self, folder, process, clock=time):
        self.folder, self.process, self.clock = Path(folder), process, clock
        self.decoder, self.tracker = capture.Decoder(), capture.Tracker()
        self.offset, self.samples, self.status_seen = 0, 0, False
        self.counters = None
        self.last_data = clock.monotonic()
        self.observations = deque(maxlen=20000)

    def poll(self, require_running=True):
        if require_running and self.process.poll() is not None:
            raise BasketError("ADXL capture process ended before the FC sequence")
        raw = self.folder / "capture.bin"
        if raw.exists():
            with raw.open("rb") as handle:
                handle.seek(self.offset)
                data = handle.read(65536)
                self.offset += len(data)
            if data:
                self.last_data = self.clock.monotonic()
            for record in self.decoder.feed(data):
                row = self.tracker.row(record)
                counters = tuple(record[k] for k in capture.COUNTER_FIELDS)
                if self.counters is not None and counters != self.counters:
                    raise BasketError("ADXL acquisition/drop counter changed")
                self.counters = counters
                if row["seq_gap_before"] or row["seq_discontinuity"]:
                    raise BasketError("ADXL record sequence is discontinuous")
                if record["flags"] & ~0xff or (record["flags"] & capture.USB_SESSION_START and self.tracker.counts["usb_session_start_rows"] > 1):
                    raise BasketError("ADXL stream has unknown flags or an extra USB session")
                if record["record_type"] == capture.STATUS:
                    if (not record["flags"] & capture.SENSOR_CONFIGURED
                            or (record["wire_x"], record["wire_y"], record["wire_z"], record["aux"]) != (0xe5, 0x0d, 0x08, 0)):
                        raise BasketError("ADXL configuration or acquisition fault")
                    self.status_seen = True
                elif record["record_type"] == capture.SAMPLE:
                    if (row["sensor_configured"] != 1
                            or not (row["fifo_coherent"] == 1 or row["drdy_timestamp_qualified"] == 1)
                            or any(row[k] != 0 for k in ("sensor_overrun", "service_gap", "poll_fallback"))):
                        raise BasketError("ADXL sample is not configured, coherent and clean")
                    self.samples += 1
                    self.observations.append((self.clock.monotonic_ns(), record["wire_x"], record["wire_y"], record["wire_z"]))
            if any(self.decoder.stats[k] for k in ("crc_errors", "unsupported_versions", "unsupported_types", "discarded_bytes")):
                raise BasketError("ADXL framing/integrity failure")
        if require_running and self.clock.monotonic() - self.last_data > .5:
            raise BasketError("ADXL capture has not delivered data for 500 ms")

    def wait_ready(self, timeout=3):
        deadline = self.clock.monotonic() + timeout
        while self.clock.monotonic() < deadline:
            self.poll()
            if self.status_seen and self.samples >= 50:
                return
            self.clock.sleep(.02)
        raise BasketError("No verified ADXL status and coherent sample stream before motion")

    def settled_mean(self, event, profile):
        """Long-window diagnostic using file-observation host times, not clock-synchronized samples."""
        begin, end = event["settle_end_host_monotonic_ns"], event["plateau_end_host_monotonic_ns"]
        rows = [row[1:] for row in self.observations if begin <= row[0] <= end]
        if len(rows) < 100:
            raise BasketError("Too few coherent external samples in the settled host window")
        raw = [sum(row[i] for row in rows) / len(rows) for i in range(3)]
        corrected = [(value - bias) / scale for value, bias, scale in zip(raw, profile["bias_counts"], profile["counts_per_g"])]
        norm = math.sqrt(sum(value * value for value in corrected))
        return {"samples": len(rows), "raw_mean_counts": raw, "corrected_mean_g": corrected,
                "corrected_mean_norm_g": norm,
                "xy_projection_deg": math.degrees(math.atan2(corrected[1], corrected[0])),
                "within_formal_3percent_norm_screen": abs(norm - 1) <= .03,
                "timing": "Host file-observation association only; no exact synchronization or hinge-angle claim"}


def validate_adxl_summary(folder):
    folder = Path(folder)
    summary = json.loads((folder / "summary.json").read_text())
    parser, quality = summary["parser"], summary["quality"]
    if (summary["end_reason"] not in ("input_exhausted", "keyboard_interrupt")
            or not summary["source_files_unchanged_during_capture"]
            or any(parser[k] for k in ("crc_errors", "unsupported_versions", "unsupported_types", "discarded_bytes"))
            or quality["sample_rows"] < 100 or quality["configured_sample_rows"] != quality["sample_rows"]
            or quality["status_rows"] < 1 or set(quality["status_fault_code_counts"]) != {"0"}
            or quality["fifo_coherent_sample_rows"] + quality["qualified_sample_timestamps"] != quality["sample_rows"]
            or any(quality[k] for k in ("missing_record_sequences", "sequence_discontinuities", "rows_with_unknown_flag_bits",
                                        "sensor_overrun_sample_rows", "service_gap_sample_rows", "poll_fallback_sample_rows",
                                        "fifo_invalid_depth_rows", "fifo_timestamp_conflict_rows"))
            or len(quality["epochs"]) != 1 or quality["usb_session_start_rows"] != 1):
        raise BasketError("ADXL capture summary failed integrity/coherence checks")
    epoch = quality["epochs"][0]
    if epoch["first_counters"] != epoch["last_counters"]:
        raise BasketError("ADXL capture counters changed")
    tail_count = parser["truncated_tail_bytes"]
    wire = (folder / "capture.bin").read_bytes()
    tail = wire[-tail_count:] if tail_count else b""
    if tail_count >= 64 or (tail and not (b"ADXL".startswith(tail) if len(tail) < 4 else tail.startswith(b"ADXL"))):
        raise BasketError("Unexpected ADXL partial-frame tail")
    for name, record in summary["files"].items():
        if sha256(folder / name) != record["sha256"]:
            raise BasketError("ADXL artifact hash mismatch")
    return summary


def stop_capture(process):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
    try:
        return process.wait(timeout=3)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=2)
        raise BasketError("ADXL logger did not stop cleanly; partial artifacts retained")


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fc-port", required=True)
    parser.add_argument("--teensy-port", required=True)
    parser.add_argument("--teensy-serial", required=True)
    parser.add_argument("--fc-uid", help="Expected 24-hex-digit raw MSP UID, obtained by a prior --inspect")
    parser.add_argument("--profile", required=True, type=Path)
    parser.add_argument("--validation-report", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--inspect", action="store_true", help="Read-only FC/USB preflight; no actuator writes")
    mode.add_argument("--execute", action="store_true", help="Execute fixed center/roll-zero/reversal check or separately authorized full sweep")
    parser.add_argument("--setup-confirmed", action="store_true", help="Caller attests physical setup was confirmed outside this tool")
    parser.add_argument("--plan-kind", choices=("center-check", "roll-zero-check", "reversal-check", "reversal-high-check", "sweep-preview", "full-sweep"), default="center-check")
    parser.add_argument("--range-authorization", type=Path)
    parser.add_argument("--authorization-sha256")
    parser.add_argument("--servo-index", type=int, default=5)
    parser.add_argument("--settle-seconds", type=float, default=1)
    parser.add_argument("--measure-seconds", type=float, default=2)
    parser.add_argument("--sample-hz", type=float, default=50)
    args = parser.parse_args(argv)
    if args.execute and (not args.setup_confirmed or not args.fc_uid):
        parser.error("--execute requires --setup-confirmed and the --fc-uid from read-only inspection")
    if args.execute and args.plan_kind == "full-sweep" and (not args.range_authorization or not args.authorization_sha256):
        parser.error("Full sweep requires --range-authorization and --authorization-sha256")
    if args.fc_uid is not None and (len(args.fc_uid) != 24 or any(c not in "0123456789abcdefABCDEF" for c in args.fc_uid)):
        parser.error("--fc-uid must be exactly 24 hexadecimal digits")
    if not 0 <= args.servo_index <= 7:
        parser.error("--servo-index must be 0..7")
    return args


def main(argv=None):
    args = arguments(argv)
    previous_termination_handler = signal.signal(signal.SIGTERM, interrupted) if args.execute else None
    process, connection, log = None, None, None
    output_owned = False
    result = {"started_utc": datetime.now(timezone.utc).isoformat(), "mode": "execute" if args.execute else "inspect" if args.inspect else "plan",
              "pid": os.getpid(),
              "sequence_complete": False, "physical_setup_confirmed_by_caller": args.setup_confirmed,
              "exact_clock_synchronization": False, "tool_sha256": sha256(__file__)}
    try:
        plan = make_plan(args.plan_kind, args.settle_seconds, args.measure_seconds, args.sample_hz)
        plan["servo_index"] = args.servo_index
        validate_plan(plan, execute=args.execute, full_range_authorized=args.execute and args.plan_kind == "full-sweep")
        fc, teensy = resolved_ports(args.fc_port, args.teensy_port)
        profile = load_validated_profile(args.profile, args.validation_report)
        range_auth = load_range_authorization(args.range_authorization, args.authorization_sha256, plan, args.fc_uid,
                                              args.servo_index, profile) if args.execute and args.plan_kind == "full-sweep" else None
        if args.output.exists() and any(args.output.iterdir()):
            raise BasketError("Output directory must be empty; existing artifacts are never overwritten")
        args.output.mkdir(parents=True, exist_ok=True)
        output_owned = True
        result.update(fc_port=str(args.fc_port), teensy_port=str(args.teensy_port),
                      fc_resolved=fc, teensy_resolved=teensy, profile_sha256=profile["profile_sha256"],
                      validation_sha256=profile["validation_sha256"], plan_sha256=plan_fingerprint(plan))
        write_json(args.output / "plan.json", plan)
        write_json(args.output / "calibration-profile.json", profile["profile"])
        write_json(args.output / "static-validation-report.json", profile["validation"])
        if range_auth:
            write_json(args.output / "range-authorization.json", range_auth["authorization"])
            result["range_authorization_sha256"] = range_auth["authorization_sha256"]
        if not args.inspect and not args.execute:
            result["status"] = "Plan only; no serial ports opened or device commands sent"
        else:
            result["usb_identity"] = verify_usb_identity(args.fc_port, args.teensy_port, args.teensy_serial)
            connection = MSP(args.fc_port)
            controller = Controller(connection, expected_uid=args.fc_uid, servo_index=args.servo_index)
            if args.inspect:
                write_json(args.output / "fc-readonly-preflight.json", controller.backup)
                result.update(status="Read-only preflight complete; no device writes", identity=controller.identity)
            else:
                duration = capture_budget_seconds(plan)
                log = (args.output / "adxl-process.log").open("xb")
                command = [sys.executable, str(Path(__file__).with_name("capture.py")), "--port", args.teensy_port,
                           "--output", str(args.output / "adxl"), "--seconds", str(duration)]
                result["adxl_command"] = command
                process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
                monitor = AdxlMonitor(args.output / "adxl", process)
                monitor.wait_ready()
                controller.record(plan, args.output, health_check=monitor.poll, range_authorization=range_auth,
                                  plateau_observer=(lambda event: monitor.settled_mean(event, profile["profile"]))
                                  if range_auth or plan["kind"] in ("roll-zero-check", "reversal-check", "reversal-high-check") else None,
                                  progress_callback=lambda event: print(json.dumps(event, allow_nan=False), flush=True))
                result["adxl_exit_code"] = stop_capture(process)
                process = None
                if result["adxl_exit_code"] != 0:
                    raise BasketError("ADXL capture process exited unsuccessfully")
                while True:
                    before = monitor.offset
                    monitor.poll(require_running=False)
                    if monitor.offset == before:
                        break
                summary = validate_adxl_summary(args.output / "adxl")
                result.update(sequence_complete=True, status=plan["kind"] + " recording complete; inspect settled-window stability before fitting",
                              adxl_samples=summary["quality"]["sample_rows"])
        return_code = 0
    except (Exception, KeyboardInterrupt) as error:
        result["error"] = f"{type(error).__name__}: {error}"
        return_code = 130 if isinstance(error, KeyboardInterrupt) else 1
    finally:
        if process is not None:
            try:
                result["adxl_exit_code"] = stop_capture(process)
            except Exception as error:
                result["adxl_stop_error"] = str(error)
        if log is not None:
            log.close()
        if connection is not None:
            connection.close()
        result["finished_utc"] = datetime.now(timezone.utc).isoformat()
        if output_owned and not (args.output / "run.json").exists():
            write_json(args.output / "run.json", result)
        if previous_termination_handler is not None:
            signal.signal(signal.SIGTERM, previous_termination_handler)
    print(json.dumps(result, indent=2))
    return return_code


def interrupted(*_):
    raise KeyboardInterrupt()


if __name__ == "__main__":
    raise SystemExit(main())
