#!/usr/bin/env python3
"""Analyze settled basket plateaus offline; never command hardware.

FC and Teensy clocks are not synchronized. Broad, inset host-arrival windows
and observed input PWM associate settled observations only. A fitted gravity
rotation plane needs sufficient observed curvature and angular coverage;
otherwise XY angles are explicitly diagnostic, not a measured pivot angle.
"""

import argparse
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import struct
import sys

import numpy as np

import basket_capture
import calibrate
import capture


class AnalysisError(ValueError):
    pass


def describe(values):
    a = np.asarray(values, dtype=float)
    if not len(a) or not np.isfinite(a).all():
        raise AnalysisError("Missing or nonfinite observations")
    return {"count": len(a), "mean": float(a.mean()), "std_population": float(a.std()),
            "min": float(a.min()), "max": float(a.max())}


def wrapped_difference(a, b):
    return float((a - b + 180) % 360 - 180)


def fit_rotation_plane(vectors, reference_index=0):
    """Estimate a small-circle plane only when actual gravity vectors support it.

    Thresholds are conservative numerical/session screens, not angle-accuracy
    specifications. No PWM values or assumed command-to-angle law enter here.
    """
    g = np.asarray(vectors, dtype=float)
    if g.ndim != 2 or g.shape[1] != 3 or len(g) < 3 or not np.isfinite(g).all():
        raise AnalysisError("Need at least three finite gravity vectors")
    norms = np.linalg.norm(g, axis=1)
    if np.any(norms < .25):
        return {"identified": False, "status": "plane_not_identified", "reasons": ["Weak gravity magnitude"]}
    directions = g / norms[:, None]
    span = float(np.degrees(np.arccos(np.clip((directions @ directions.T).min(), -1, 1))))
    _, singular, vt = np.linalg.svd(g - g.mean(axis=0), full_matrices=False)
    ratio = float(singular[1] / singular[0]) if singular[0] else 0.
    residual_ratio = float(singular[2] / singular[1]) if singular[1] else float("inf")
    curvature = float(singular[1] / math.sqrt(len(g)))
    reasons = []
    if len(g) < 5:
        reasons.append("Fewer than five plateau observations")
    if span < 30:
        reasons.append("Observed gravity-direction span below30degrees")
    if ratio < .08 or curvature < .01:
        reasons.append("Observed arc curvature is too small to identify a plane")
    if singular[1] and residual_ratio > .25:
        reasons.append("Out-of-plane residual is large relative to observed curvature")
    result = {"identified": False, "status": "plane_not_identified", "reasons": reasons,
              "observed_gravity_direction_span_deg": span, "centered_singular_values_g": singular.tolist(),
              "second_to_first_singular_ratio": ratio,
              "third_to_second_singular_ratio": residual_ratio if math.isfinite(residual_ratio) else None,
              "curvature_rms_g": curvature,
              "screening_thresholds": {"minimum_plateaus": 5, "minimum_gravity_direction_span_deg": 30,
                  "minimum_second_to_first_ratio": .08, "minimum_curvature_rms_g": .01,
                  "maximum_third_to_second_ratio": .25, "minimum_axis_perpendicular_gravity_g": .25}}
    if reasons:
        return result
    normal = vt[-1]
    if normal[np.argmax(np.abs(normal))] < 0:
        normal = -normal
    projected = g - (g @ normal)[:, None] * normal
    radius = np.linalg.norm(projected, axis=1)
    if np.any(radius < .25):
        result["reasons"].append("Weak gravity projection perpendicular to the estimated axis")
        return result
    basis_x = projected[reference_index] / radius[reference_index]
    basis_y = np.cross(normal, basis_x)
    angles = np.degrees(np.arctan2(projected @ basis_y, projected @ basis_x))
    result.update(identified=True, status="identified_conditionally", reasons=[],
                  normal_sensor_axes=normal.tolist(), mean_plane_offset_g=float(g.mean(axis=0) @ normal),
                  perpendicular_gravity_g=radius.tolist(), relative_angles_deg=angles.tolist(),
                  angle_measurand="Rotation of the observed gravity vector in sensor coordinates, relative to the recorded1420reference",
                  sign_convention="Right-hand gravity-vector rotation about the fitted normal, whose largest sensor-axis component is positive. Mechanical basket orientation about the corresponding fixed hinge changes with opposite sign; physical airframe tilt sign is not assigned.",
                  absolute_vertical_zero_qualified=False,
                  interpretation="Conditional relative angles from actual settled gravity vectors; cross-axis calibration and physical pivot-axis/vertical alignment are not independently proven.")
    return result


def load_adxl(folder):
    summary = basket_capture.validate_adxl_summary(folder)
    wire = (folder / "capture.bin").read_bytes()
    decoder, tracker = capture.Decoder(), capture.Tracker()
    decoded = [tracker.row(r) for r in decoder.feed(wire)]
    parser = decoder.finish()
    with (folder / "records.csv").open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    if len(rows) != len(decoded):
        raise AnalysisError("ADXL raw/CSV row count differs")
    for row, raw in zip(rows, decoded):
        for key, value in row.items():
            if key not in ("host_rx_monotonic_ns", "host_rx_unix_ns") and value != str(raw.get(key, "")):
                raise AnalysisError(f"ADXL raw/CSV mismatch in {key}")
    quality = tracker.summary()
    if any(parser[k] for k in ("crc_errors", "unsupported_versions", "unsupported_types", "discarded_bytes")):
        raise AnalysisError("ADXL binary integrity failed")
    if any(quality[k] for k in ("missing_record_sequences", "sequence_discontinuities", "rows_with_unknown_flag_bits")):
        raise AnalysisError("ADXL stream discontinuity")
    for key in capture.COUNTER_FIELDS:
        if len({int(row[key]) for row in rows}) != 1:
            raise AnalysisError("ADXL counter changed within capture")
    samples = [r for r in rows if r["type"] == "sample"]
    if not samples:
        raise AnalysisError("No ADXL samples")
    for row in samples:
        if (row["sensor_configured"] != "1" or not (row.get("fifo_coherent") == "1" or row["drdy_timestamp_qualified"] == "1")
                or any(row[k] != "0" for k in ("sensor_overrun", "service_gap", "poll_fallback"))):
            raise AnalysisError("Unconfigured, incoherent or faulted ADXL sample")
    return samples, summary


def verify_guard(state, arm_index):
    status = bytes.fromhex(state["status_hex"])
    advanced = bytes.fromhex(state["advanced_config_hex"])
    motor = bytes.fromhex(state["motor_payload_hex"])
    if (len(status) < 16 or struct.unpack_from("<I", status, 6)[0] & (1 << arm_index)
            or len(advanced) < 20 or advanced[18] != 103 or advanced[19] <= 103
            or state["mixer_config_hex"] != "0100" or len(motor) < 6 or len(motor) % 2
            or struct.unpack_from("<3H", motor) != (1000, 1000, 1000)):
        raise AnalysisError("FC guard does not establish disarmed TRI, stopped motors and TRI_GROUND")


def load_run(folder):
    names = ("run.json", "plan.json", "fc-backup.json", "fc-run.json", "fc-events.json",
             "original-servo-table.bin", "fc-samples.csv", "calibration-profile.json", "static-validation-report.json")
    if any(not (folder / n).is_file() for n in names):
        raise AnalysisError("Incomplete basket capture artifacts")
    run, plan, backup, fc_run, events = [json.loads((folder / n).read_text()) for n in names[:5]]
    if (run.get("mode") != "execute" or run.get("sequence_complete") is not True
            or run.get("physical_setup_confirmed_by_caller") is not True
            or fc_run.get("sequence_complete") is not True or fc_run.get("servo_restored_exact") is not True
            or "error" in run or "error" in fc_run or "critical_restore_failure" in fc_run):
        raise AnalysisError("Basket sequence incomplete or restoration not verified")
    loaded = basket_capture.load_validated_profile(folder / "calibration-profile.json", folder / "static-validation-report.json")
    if run["profile_sha256"] != loaded["profile_sha256"] or run["validation_sha256"] != loaded["validation_sha256"]:
        raise AnalysisError("Run/profile/validation hashes disagree")
    if (backup["identity"] != fc_run["identity"] or backup["identity"]["variant"] != "BTFL"
            or backup["identity"]["api_hex"] != "000130" or len(backup["identity"]["uid_hex"]) != 24):
        raise AnalysisError("FC identity binding is invalid")
    original = (folder / "original-servo-table.bin").read_bytes()
    if original.hex() != backup["servo_table_hex"] or original.hex() != fc_run["final_servo_table_hex"]:
        raise AnalysisError("Original and restored full servo tables differ")
    index = backup["servo_index"]
    if index != plan["servo_index"] or len(original) % basket_capture.SERVO_RECORD.size:
        raise AnalysisError("Servo index/table schema mismatch")
    record = basket_capture.SERVO_RECORD.unpack_from(original, index * basket_capture.SERVO_RECORD.size)
    if record[:3] != (1100, 1700, 1420) or record[4] != 255:
        raise AnalysisError("Original servo limits/center/forwarding differ")
    boxes = bytes.fromhex(backup["boxids_hex"])
    if 0 not in boxes or boxes.index(0) >= 32:
        raise AnalysisError("Cannot locate ARM status bit")
    for guard in (backup, fc_run["pre_restore_guard"], fc_run["post_restore_guard"]):
        verify_guard(guard, boxes.index(0))
    if events != fc_run["events"] or len(events) != len(plan["plateaus"]):
        raise AnalysisError("FC event reports disagree")
    previous_end = None
    for event, planned in zip(events, plan["plateaus"]):
        if event["plateau_id"] != planned["plateau_id"] or event["target_us"] != planned["target_us"]:
            raise AnalysisError("Event target/index differs from plan")
        stamps = [event[k] for k in ("command_request_host_monotonic_ns", "command_ack_host_monotonic_ns",
                  "settle_end_host_monotonic_ns", "planned_end_host_monotonic_ns", "plateau_end_host_monotonic_ns")]
        if not all(type(v) is int for v in stamps) or not all(b > a for a, b in zip(stamps, stamps[1:])):
            raise AnalysisError("Ambiguous or unordered FC event timestamps")
        if previous_end is not None and stamps[0] < previous_end:
            raise AnalysisError("Overlapping plateau command intervals")
        previous_end = stamps[-1]
    return run, plan, backup, fc_run, events, loaded, {n: calibrate.sha256(folder / n) for n in names}


def validate_host_association(samples, events, inset_s):
    try:
        host = np.asarray([int(r["host_rx_monotonic_ns"]) for r in samples], dtype=np.int64)
        mcu = np.asarray([int(r["time_us"]) for r in samples], dtype=np.int64)
    except ValueError as error:
        raise AnalysisError("ADXL host arrival timestamps are absent") from error
    if np.any(host <= 0) or np.any(np.diff(host) < 0) or np.any(np.diff(mcu) <= 0):
        raise AnalysisError("Ambiguous ADXL host/sample timestamps")
    if host[0] > events[0]["command_request_host_monotonic_ns"] or host[-1] < events[-1]["plateau_end_host_monotonic_ns"]:
        raise AnalysisError("ADXL host stream does not bracket the FC sequence")
    x, y = (mcu - mcu[0]) / 1e6, (host - host[0]) / 1e9
    slope, intercept = np.polyfit(x, y, 1)
    residual_span = float(np.ptp(y - (slope * x + intercept)))
    boundaries = np.r_[0, np.flatnonzero(np.diff(host)) + 1, len(host)]
    max_batch_span = max(float((mcu[b - 1] - mcu[a]) / 1e6) for a, b in zip(boundaries, boundaries[1:]))
    if not .98 <= slope <= 1.02 or residual_span > inset_s / 2 or max_batch_span > inset_s / 2:
        raise AnalysisError("USB batching/arrival variation is too large for the requested interior window")
    return host, {"arrival_vs_pop_clock_slope_diagnostic": float(slope), "residual_span_s": residual_span,
                  "maximum_same_arrival_batch_pop_span_s": max_batch_span,
                  "exact_clock_alignment": False,
                  "limitation": "Host receipt is not sensor capture time. Interior windows, clean continuous transport and observed PWM support settled association; a constant unmeasured USB latency is not independently calibrated."}


def analyze(folder, inset_s=.2, pwm_tolerance_us=8):
    folder = Path(folder).resolve()
    if not math.isfinite(inset_s) or inset_s <= 0 or not 0 < pwm_tolerance_us < 10:
        raise AnalysisError("Require positive finite inset and PWM tolerance below10us")
    run, plan, backup, fc_run, events, loaded, hashes = load_run(folder)
    profile = loaded["profile"]
    samples, summary = load_adxl(folder / "adxl")
    host, association = validate_host_association(samples, events, inset_s)
    with (folder / "fc-samples.csv").open(newline="") as handle:
        fc_rows = list(csv.DictReader(handle))
    if not fc_rows:
        raise AnalysisError("No FC observations")
    known_ids = {e["plateau_id"]: e for e in events}
    for row in fc_rows:
        try:
            values = [int(row[k]) for k in ("raw_pb0", "final_pwm_us", "raw_pc0", "corrected_pb0_minus_pc0", "pair_age_100us", "flags", "rpm_age_100us", "rpm_invalid_stat")]
            _, fresh = basket_capture.ground_sample(struct.pack("<8h", *values))
            request, response, index, target = [int(row[k]) for k in ("request_host_monotonic_ns", "response_host_monotonic_ns", "plateau_id", "target_us")]
        except (ValueError, struct.error, basket_capture.BasketError) as error:
            raise AnalysisError("Invalid canonical PB0-PC0 observation") from error
        if index not in known_ids or target != known_ids[index]["target_us"] or response < request:
            raise AnalysisError("FC row target/index/time does not match event")
        matches = values[1] == target
        if (int(row["position_fresh"]) != fresh or int(row["command_matches"]) != matches
                or int(row["settled_candidate"]) != (fresh and matches and row["phase"] == "measure")):
            raise AnalysisError("FC freshness/matching aliases disagree with canonical data")
    plateaus = []
    previous_target = None
    for event in events:
        start = event["settle_end_host_monotonic_ns"] + int(inset_s * 1e9)
        end = event["planned_end_host_monotonic_ns"] - int(inset_s * 1e9)
        if end - start < 500000000:
            raise AnalysisError("Interior settled window is shorter than500ms")
        target, index = event["target_us"], event["plateau_id"]
        fc = [r for r in fc_rows if int(r["plateau_id"]) == index and r["phase"] == "measure"
              and start <= int(r["request_host_monotonic_ns"]) <= int(r["response_host_monotonic_ns"]) <= end]
        good_fc = [r for r in fc if r["position_fresh"] == "1" and r["command_matches"] == "1"]
        if len(good_fc) < 5 or len(good_fc) / max(1, len(fc)) < .9:
            raise AnalysisError("Too few fresh command-matching interior PB0-PC0 observations")
        in_window = [r for r, stamp in zip(samples, host) if start <= stamp <= end]
        fresh_pwm = [r for r in in_window if r["pwm_fresh"] == "1"]
        paired = [r for r in fresh_pwm if abs(int(r["pwm_width_us"]) - target) <= pwm_tolerance_us]
        primary = len(paired) >= 100
        selected = paired if primary else in_window
        if len(selected) < 100:
            raise AnalysisError("Too few ADXL observations in the interior settled window")
        raw = np.asarray([[int(r["raw_" + k]) for k in "xyz"] for r in selected], dtype=float)
        stamps = np.asarray([int(r["host_rx_monotonic_ns"]) for r in selected], dtype=np.int64)
        if stamps[0] > start + int(.15e9) or stamps[-1] < end - int(.15e9) or np.max(np.diff(stamps)) > int(.15e9):
            raise AnalysisError("ADXL selected observations do not cover the settled window continuously")
        blocks = []
        for a, b in zip(np.linspace(start, end, 5)[:-1], np.linspace(start, end, 5)[1:]):
            data = raw[(stamps >= a) & (stamps <= b)]
            if len(data) < 20:
                raise AnalysisError("Insufficient ADXL data in an interior quarter-window")
            blocks.append(data.mean(axis=0))
        blocks = np.asarray(blocks)
        gravity = calibrate.apply_calibration(raw.mean(axis=0), profile)
        block_g = calibrate.apply_calibration(blocks, profile)
        norms = np.linalg.norm(block_g, axis=1)
        stable = bool((np.ptp(blocks, axis=0) < 3).all() and (np.abs(norms - 1) <= .03).all()
                      and abs(np.linalg.norm(gravity) - 1) <= .03)
        xy_projection = float(np.linalg.norm(gravity[:2]))
        angle = float(np.degrees(np.arctan2(gravity[1], gravity[0]))) if xy_projection >= .25 else None
        approach = "initial" if previous_target is None else "increasing" if target > previous_target else "decreasing" if target < previous_target else "unchanged"
        plateaus.append({"plateau_id": index, "target_us": target, "approach": approach,
            "window_start_host_ns": start, "window_end_host_ns": end,
            "fc": {"interior_rows": len(fc), "fresh_matching_rows": len(good_fc),
                   "excluded_stale_or_mismatched_rows": len(fc) - len(good_fc),
                   "corrected_pb0_minus_pc0_counts": describe([int(r["corrected_pb0_minus_pc0"]) for r in good_fc]),
                   "raw_pb0_diagnostic_counts": describe([int(r["raw_pb0"]) for r in good_fc]),
                   "raw_pc0_diagnostic_counts": describe([int(r["raw_pc0"]) for r in good_fc])},
            "adxl": {"host_window_rows": len(in_window), "fresh_pwm_rows": len(fresh_pwm),
                     "fresh_matching_pwm_rows": len(paired), "selected_rows": len(selected),
                     "association": "inset_host_window_and_fresh_matching_input_pwm" if primary else "host_window_only_diagnostic",
                     "paired_observation_qualified": primary,
                     "observed_fresh_pwm_us": describe([int(r["pwm_width_us"]) for r in fresh_pwm]) if fresh_pwm else None,
                     "mean_raw_counts": raw.mean(axis=0).tolist(), "raw_std_counts": raw.std(axis=0).tolist(),
                     "corrected_mean_g": gravity.tolist(), "corrected_mean_norm_g": float(np.linalg.norm(gravity)),
                     "interior_quarter_mean_span_counts": np.ptp(blocks, axis=0).tolist(),
                     "interior_quarter_norms_g": norms.tolist(), "settled_mean_screen_pass": stable,
                     "xy_diagnostic_angle_deg": angle, "xy_gravity_projection_g": xy_projection}})
        previous_target = target
    references = [i for i, p in enumerate(plateaus) if p["target_us"] == 1420]
    if not references:
        raise AnalysisError("No recorded1420reference plateau")
    reference = references[0]
    plane = fit_rotation_plane([p["adxl"]["corrected_mean_g"] for p in plateaus], reference)
    xy_reference = plateaus[reference]["adxl"]["xy_diagnostic_angle_deg"]
    for i, plateau in enumerate(plateaus):
        xy = plateau["adxl"]["xy_diagnostic_angle_deg"]
        plateau["xy_diagnostic_relative_deg"] = wrapped_difference(xy, xy_reference) if xy is not None and xy_reference is not None else None
        plateau["rotation_plane_relative_deg"] = plane["relative_angles_deg"][i] if plane["identified"] else None
    comparisons = []
    for i, a in enumerate(plateaus):
        for b in plateaus[i + 1:]:
            if a["target_us"] != b["target_us"]:
                continue
            comparisons.append({"target_us": a["target_us"], "plateau_a": a["plateau_id"], "plateau_b": b["plateau_id"],
                "approach_a": a["approach"], "approach_b": b["approach"],
                "opposite_approaches": {a["approach"], b["approach"]} == {"increasing", "decreasing"},
                "delta_corrected_adc_counts_b_minus_a": b["fc"]["corrected_pb0_minus_pc0_counts"]["mean"] - a["fc"]["corrected_pb0_minus_pc0_counts"]["mean"],
                "delta_xy_diagnostic_deg_b_minus_a": wrapped_difference(b["xy_diagnostic_relative_deg"], a["xy_diagnostic_relative_deg"]) if a["xy_diagnostic_relative_deg"] is not None and b["xy_diagnostic_relative_deg"] is not None else None,
                "delta_rotation_plane_deg_b_minus_a": wrapped_difference(b["rotation_plane_relative_deg"], a["rotation_plane_relative_deg"]) if plane["identified"] else None})
    hashes.update({"adxl/" + n: calibrate.sha256(folder / "adxl" / n) for n in ("capture.bin", "records.csv", "summary.json")})
    return {"schema_version": 1, "created_utc": datetime.now(timezone.utc).isoformat(),
        "position_source": "fresh signed PB0-PC0 only; raw channels are diagnostics",
        "exact_clock_synchronization": False, "physical_vertical_zero_qualified": False,
        "all_settled_windows_pass": all(p["adxl"]["settled_mean_screen_pass"] and p["adxl"]["paired_observation_qualified"] for p in plateaus),
        "reference_plateau_id": plateaus[reference]["plateau_id"], "reference_target_us": 1420,
        "inset_seconds": inset_s, "fresh_pwm_tolerance_us": pwm_tolerance_us,
        "host_association": association, "rotation_plane": plane,
        "plateaus": plateaus, "same_target_comparisons": comparisons,
        "provenance": {"folder": str(folder), "input_sha256": hashes,
                       "analysis_tool_sha256": calibrate.sha256(__file__), "capture_tool_sha256_recorded": run["tool_sha256"],
                       "fc_identity": backup["identity"], "servo_restored_exact": True,
                       "limitation": "FC observations are decoded CSV plus guarded run reports; raw MSP wire frames were not preserved. Hashes establish artifact identity, not independent physical truth."},
        "interpretation": "Settled relative gravity-vector observations only; gravity rotation in sensor coordinates has the opposite sign to sensor orientation about a corresponding fixed hinge. A small center arc cannot identify that plane reliably; XY results remain diagnostic when plane_not_identified. No angle is inferred from PWM, no dynamic timing or absolute vertical zero is claimed, and no device output is written."}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--inset-seconds", type=float, default=.2)
    parser.add_argument("--pwm-tolerance-us", type=float, default=8)
    args = parser.parse_args(argv)
    try:
        if args.output.exists():
            raise AnalysisError("Output already exists")
        result = analyze(args.folder, args.inset_seconds, args.pwm_tolerance_us)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x") as handle:
            json.dump(result, handle, indent=2, allow_nan=False)
            handle.write("\n")
        print(json.dumps({"all_settled_windows_pass": result["all_settled_windows_pass"],
                          "rotation_plane": result["rotation_plane"], "same_target_comparisons": result["same_target_comparisons"]}, indent=2))
        return 0 if result["all_settled_windows_pass"] else 2
    except (AnalysisError, basket_capture.BasketError, ValueError, OSError, KeyError, struct.error, np.linalg.LinAlgError) as error:
        print(f"Basket analysis failed: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
