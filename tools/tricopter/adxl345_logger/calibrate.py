#!/usr/bin/env python3
"""Offline ADXL345 bias/diagonal-scale fitting and independent static validation.

No sensor commands or offset-register writes.  Pose directions may be approximate:
the fit constrains the corrected *magnitude* to 1 g, not individual axis values.
Six means determine six parameters, so a small training residual is not validation.
"""

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

import capture


class CalibrationError(ValueError):
    pass


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def fit_ellipsoid(means, max_condition=1e6):
    """Solve a normalized axis-aligned ellipsoid; return positive counts/g.

    For normalized u, solve sum(a_i*u_i**2 + d_i*u_i) = 1. Completing
    squares gives center -d/(2a) and radii sqrt((1+sum(a*center**2))/a).
    Equal pose weighting prevents one long recording dominating the fit.
    """
    if not np.isfinite(max_condition) or max_condition < 1:
        raise CalibrationError("Maximum condition must be finite and at least one")
    x = np.asarray(means, dtype=float)
    if x.ndim != 2 or x.shape[1] != 3 or len(x) < 6 or not np.isfinite(x).all():
        raise CalibrationError("Need at least six finite 3D pose means")
    center = x.mean(axis=0)
    normalization = x.std(axis=0)
    if np.any(normalization <= np.finfo(float).eps):
        raise CalibrationError("Pose means do not span all three axes")
    u = (x - center) / normalization
    design = np.column_stack((u * u, u))
    beta, _, rank, singular = np.linalg.lstsq(design, np.ones(len(x)), rcond=None)
    condition = float(singular[0] / singular[-1]) if singular[-1] else float("inf")
    if rank != 6 or not np.isfinite(condition) or condition > max_condition:
        raise CalibrationError(f"Rank/conditioning failure: rank={rank}, condition={condition:g}")
    quadratic, linear = beta[:3], beta[3:]
    if np.any(quadratic <= 0):
        raise CalibrationError("Fit is not a positive axis-aligned ellipsoid")
    normalized_bias = -linear / (2 * quadratic)
    radius_squared = 1 + np.sum(quadratic * normalized_bias**2)
    bias = center + normalization * normalized_bias
    scale = normalization * np.sqrt(radius_squared / quadratic)
    if not np.isfinite(bias).all() or not np.isfinite(scale).all() or np.any(scale <= 0):
        raise CalibrationError("Nonfinite or nonpositive fitted sensitivity")
    if not ((x.min(axis=0) < bias) & (bias < x.max(axis=0))).all():
        raise CalibrationError("Calibration poses must straddle the fitted center on every axis")
    corrected = (x - bias) / scale
    return {
        "bias_counts": bias.tolist(),
        "counts_per_g": scale.tolist(),
        "diagnostics": {
            "rank": int(rank), "normalized_design_condition": condition,
            "maximum_allowed_condition": max_condition,
            "normalization_center_counts": center.tolist(),
            "normalization_scale_counts": normalization.tolist(),
            "normalized_quadratic_coefficients": quadratic.tolist(),
            "normalized_linear_coefficients": linear.tolist(),
            "training_norm_residual_g": (np.linalg.norm(corrected, axis=1) - 1).tolist(),
            "equal_weight_per_pose": True,
        },
    }


def apply_calibration(raw, profile):
    raw = np.asarray(raw, dtype=float)
    bias = np.asarray(profile["bias_counts"], dtype=float)
    scale = np.asarray(profile["counts_per_g"], dtype=float)
    if raw.shape[-1:] != (3,) or bias.shape != (3,) or scale.shape != (3,):
        raise CalibrationError("Raw vectors, bias and sensitivity must have three axes")
    if not all(np.isfinite(a).all() for a in (raw, bias, scale)) or np.any(scale <= 0):
        raise CalibrationError("Nonfinite inputs or nonpositive sensitivity")
    return (raw - bias) / scale


def verify_hashes(folder, expected):
    folder = Path(folder).resolve()
    for name, digest in expected.items():
        path = (folder / name).resolve()
        if not path.is_relative_to(folder) or not path.is_file() or sha256(path) != digest:
            raise CalibrationError(f"Missing file or hash mismatch: {name}")


def read_capture(folder, expected_hashes=None, stationarity_policy="strict"):
    """Verify hashes, replay binary into CSV aliases, then select static vectors.

    Hashes tie artifacts together, not to an independently known flashed image.
    Decoder replay checks raw/CSV agreement using the repository decoder. Firmware
    provenance and independent physical accuracy remain separate evidence.
    Training always uses strict stationarity. Explicit static-mean-v2 is for
    validation of averaged vectors, preserving rather than passing the old
    raw-noise gate, and must additionally pass corrected per-block norm checks.
    """
    if stationarity_policy not in ("strict", "static-mean-v2"):
        raise CalibrationError("Unknown stationarity policy")
    folder = Path(folder)
    if expected_hashes:
        verify_hashes(folder, expected_hashes)
    summary = json.loads((folder / "summary.json").read_text())
    verify_hashes(folder, {name: rec["sha256"] for name, rec in summary["files"].items()})
    if not {"capture.bin", "records.csv"}.issubset(summary["files"]):
        raise CalibrationError("Summary does not hash both binary and CSV")
    if not summary.get("source_files_unchanged_during_capture"):
        raise CalibrationError("Capture source fingerprints changed")
    if summary["end_reason"] not in ("input_exhausted", "keyboard_interrupt"):
        raise CalibrationError("Capture ended with an input error")
    wire = (folder / "capture.bin").read_bytes()
    decoder, tracker = capture.Decoder(), capture.Tracker()
    replayed = [tracker.row(record) for record in decoder.feed(wire)]
    parser = decoder.finish()
    if any(parser[k] for k in ("crc_errors", "unsupported_versions", "unsupported_types", "discarded_bytes")):
        raise CalibrationError("Binary framing/integrity failure")
    tail = wire[-parser["truncated_tail_bytes"]:] if parser["truncated_tail_bytes"] else b""
    if len(tail) >= 64 or (tail and not (b"ADXL".startswith(tail) if len(tail) < 4 else tail.startswith(b"ADXL"))):
        raise CalibrationError("Unexpected trailing data")
    with (folder / "records.csv").open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    if len(rows) != len(replayed):
        raise CalibrationError("Raw/CSV row count mismatch")
    for row, replay in zip(rows, replayed):
        for key, value in row.items():
            if key not in ("host_rx_monotonic_ns", "host_rx_unix_ns") and str(replay.get(key, "")) != value:
                raise CalibrationError(f"Raw/CSV mismatch: record {row.get('record_seq')} field {key}")
    quality = tracker.summary()
    if (quality["missing_record_sequences"] or quality["sequence_discontinuities"]
            or quality["rows_with_unknown_flag_bits"] or len(quality["epochs"]) != 1
            or quality["usb_session_start_rows"] != 1):
        raise CalibrationError("Capture has a sequence, epoch or flag ambiguity")
    for key in capture.COUNTER_FIELDS:
        if len({int(row[key]) for row in rows}) != 1:
            raise CalibrationError(f"Capture counter changed: {key}")
    statuses = [row for row in rows if row["type"] == "status"]
    registers = sorted({(int(r["devid"]), int(r["bw_rate"]), int(r["data_format"]), int(r["fault_code"])) for r in statuses})
    if not statuses or any(r["sensor_configured"] != "1" for r in statuses):
        raise CalibrationError("No healthy configured status records")
    if len(registers) != 1 or registers[0][0] != 0xe5 or registers[0][2:] != (0x08, 0):
        raise CalibrationError("Unexpected or changing sensor configuration")
    samples = [row for row in rows if row["type"] == "sample"]
    if len(samples) < 2:
        raise CalibrationError("Insufficient sensor samples")
    for row in samples:
        if (row["sensor_configured"] != "1" or not (row.get("fifo_coherent") == "1" or row["drdy_timestamp_qualified"] == "1")
                or any(row[k] != "0" for k in ("sensor_overrun", "service_gap", "poll_fallback"))):
            raise CalibrationError("Unconfigured, incoherent or faulted sample")
    axes = np.asarray([[int(r["raw_" + k]) for k in "xyz"] for r in samples], dtype=float)
    times = np.asarray([int(r["time_us"]) for r in samples], dtype=np.int64)
    if not (np.diff(times) > 0).all():
        raise CalibrationError("Sample read/pop times are not strictly increasing")
    seconds = (times - times[0]) / 1e6
    blocks = np.asarray([axes[(seconds >= i) & (seconds < i + 1)].mean(axis=0)
        for i in range(int(seconds[-1])) if np.count_nonzero((seconds >= i) & (seconds < i + 1)) >= 100])
    enough_blocks = len(blocks) >= 2
    raw_std = axes.std(axis=0)
    block_span = np.ptp(blocks, axis=0) if enough_blocks else np.full(3, np.inf)
    original_raw_std_pass = bool((raw_std < 5).all())
    stable_block_means = bool((block_span < 3).all())
    original_static_gate_pass = enough_blocks and original_raw_std_pass and stable_block_means
    selected_gate_pass = enough_blocks and stable_block_means and (stationarity_policy == "static-mean-v2" or original_raw_std_pass)
    if not selected_gate_pass:
        if stationarity_policy == "strict":
            raise CalibrationError("Original stationary-session gates failed (<5 count std, <3 count block-mean span)")
        raise CalibrationError("Static-mean-v2 stationary-session gates failed (need >=2 complete 1s blocks and <3 count axis block-mean span)")
    return {
        "folder": str(folder.resolve()), "sample_count": len(samples),
        "mean_counts": axes.mean(axis=0).tolist(), "std_counts": axes.std(axis=0).tolist(),
        "one_second_means_counts": blocks.tolist(), "register_tuple": list(registers[0]),
        "stationarity": {
            "policy": stationarity_policy,
            "original_raw_std_below_5_counts": original_raw_std_pass,
            "original_static_gate_pass": original_static_gate_pass,
            "complete_one_second_blocks": len(blocks),
            "one_second_axis_mean_span_counts": block_span.tolist(),
            "one_second_axis_mean_span_below_3_counts": stable_block_means,
            "selected_static_mean_gate_pass": selected_gate_pass,
            "interpretation": "Original raw-noise failure remains a failure. Explicit static-mean-v2 assesses averaged steady vectors using unchanged block-mean stability; raw noise and slow wander remain reported, without a white-noise or angular-accuracy claim.",
        },
        "input_sha256": {name: sha256(folder / name) for name in ("capture.bin", "records.csv", "summary.json")},
        "measurement_time": "FIFO pop/SPI read times have unmeasured sample age; precise movement timing is not qualified by static coherence.",
    }


def pose_metric(record, profile):
    vector = apply_calibration(record["mean_counts"], profile)
    blocks = apply_calibration(record["one_second_means_counts"], profile)
    return {
        "pose": record.get("pose_id", Path(record["folder"]).name),
        "sample_count": record["sample_count"], "mean_counts": record["mean_counts"],
        "corrected_mean_g": vector.tolist(), "corrected_mean_norm_g": float(np.linalg.norm(vector)),
        "norm_residual_g": float(np.linalg.norm(vector) - 1),
        "one_second_norm_residual_range_g": [float(v) for v in (np.linalg.norm(blocks, axis=1).min() - 1, np.linalg.norm(blocks, axis=1).max() - 1)],
        "input_sha256": record["input_sha256"],
        "raw_std_counts": record["std_counts"],
        "stationarity": record["stationarity"],
    }


def load_pose(session_folder, name, expected_pose_hash, fit_input):
    path = (session_folder / name).resolve()
    if not path.is_relative_to(session_folder.resolve()) or not expected_pose_hash or sha256(path) != expected_pose_hash:
        raise CalibrationError(f"Pose metadata hash mismatch: {name}")
    pose = json.loads(path.read_text())
    if fit_input and (pose.get("accepted_for_calibration") is not True or not all(pose.get("static_capture_gates", {}).values())):
        raise CalibrationError(f"Pose is not accepted for fitting: {name}")
    record = read_capture(path.parent, pose["file_sha256"])
    if not np.allclose(record["mean_counts"], pose["mean_raw_counts"], rtol=0, atol=1e-9):
        raise CalibrationError(f"Pose mean does not match samples: {name}")
    record.update(pose_id=pose["pose_id"], pose_sha256=expected_pose_hash,
                  calibration_cohort=pose.get("calibration_cohort"))
    return record


def fit_session(session_path):
    session_path = Path(session_path).resolve()
    session = json.loads(session_path.read_text())
    cohort = session.get("current_calibration_cohort")
    if not isinstance(cohort, str) or not cohort.strip():
        raise CalibrationError("A nonempty current calibration cohort is required")
    names = session["pose_manifest"]
    forbidden = set(session.get("reference_repeat_pose_manifest", [])) | set(session.get("rejected_pose_attempts", [])) | set(session.get("provisional_pose_attempts", [])) | set(session.get("historical_pose_manifest", []))
    if len(names) != len(set(names)) or set(names) & forbidden:
        raise CalibrationError("Duplicate or excluded pose included in fitting")
    records = [load_pose(session_path.parent, n, session["pose_file_sha256"].get(n), True) for n in names]
    if any(r["calibration_cohort"] != cohort for r in records):
        raise CalibrationError("Fit input cohorts do not match current session")
    if len({tuple(r["register_tuple"]) for r in records}) != 1:
        raise CalibrationError("Do not mix sensor configurations")
    training_hashes = [r["input_sha256"]["capture.bin"] for r in records]
    if len(training_hashes) != len(set(training_hashes)):
        raise CalibrationError("Duplicate raw capture used for training, including aliases")
    profile = fit_ellipsoid([r["mean_counts"] for r in records])
    validations = [load_pose(session_path.parent, n, session.get("reference_repeat_pose_sha256", {}).get(n), False)
                   for n in session.get("reference_repeat_pose_manifest", [])]
    if any(r["calibration_cohort"] != cohort for r in validations):
        raise CalibrationError("Reference holdout cohort differs from fit inputs")
    if any(r["register_tuple"] != records[0]["register_tuple"] for r in validations):
        raise CalibrationError("Reference validation sensor configuration differs from fit inputs")
    reference_hashes = [r["input_sha256"]["capture.bin"] for r in validations]
    if set(training_hashes) & set(reference_hashes):
        raise CalibrationError("Reference holdout raw capture was used for fitting")
    if len(reference_hashes) != len(set(reference_hashes)):
        raise CalibrationError("Duplicate raw capture used for reference holdouts, including aliases")
    profile.update({
        "schema_version": 1, "created_utc": datetime.now(timezone.utc).isoformat(),
        "calibration_cohort": cohort,
        "status": "provisional; independent tilted validation pending",
        "correction": "a_g=(raw_counts-bias_counts)/counts_per_g",
        "model": "Constant 3-axis bias plus positive diagonal counts/g; no cross-axis or mounting rotation fit",
        "sensor_register_tuple_devid_bw_format_fault": records[0]["register_tuple"],
        "device_writes": False, "physical_angle_accuracy_qualified": False,
        "training": {"pose_metrics": [pose_metric(r, profile) for r in records],
                     "warning": "Six pose means determine six parameters; near-zero training residual is not independent validation."},
        "validation": {"reference_pose_metrics": [pose_metric(r, profile) for r in validations],
                       "warning": "Reference repeats remain independent of fitting; between-run drift is not removed or pooled away. Gravity-norm agreement alone does not establish absolute angular accuracy."},
        "provenance": {"session": str(session_path), "session_sha256": sha256(session_path),
                       "tool_sha256": sha256(__file__), "decoder_sha256": sha256(Path(capture.__file__)),
                       "numpy_version": np.__version__,
                       "fit_pose_sha256": {n: r["pose_sha256"] for n, r in zip(names, records)},
                       "trust_boundary": "Replayed raw binary agrees with CSV; artifact hashes and host source fingerprints do not independently prove flashed firmware or physical calibration accuracy."},
    })
    # Substitution explores within-pose window variation, not a confidence interval.
    means = np.asarray([r["mean_counts"] for r in records])
    alternatives = []
    for i, record in enumerate(records):
        for block in record["one_second_means_counts"]:
            perturbed = means.copy()
            perturbed[i] = block
            alternatives.append(fit_ellipsoid(perturbed))
    profile["window_substitution_sensitivity"] = {
        "method": "Replace one pose mean at a time with each complete 1s window mean, refit with all other pose means fixed. Ranges are sensitivity diagnostics, not confidence intervals or bounds on handling/time drift.",
        "refits": len(alternatives),
    }
    for key in ("bias_counts", "counts_per_g"):
        values = np.asarray([a[key] for a in alternatives])
        profile["window_substitution_sensitivity"][key + "_min"] = values.min(axis=0).tolist()
        profile["window_substitution_sensitivity"][key + "_max"] = values.max(axis=0).tolist()
    return profile


def validate_capture(profile, capture_folder, max_norm_error_g=0.03, stationarity_policy="strict"):
    if not np.isfinite(max_norm_error_g) or max_norm_error_g <= 0:
        raise CalibrationError("Norm threshold must be finite and positive")
    record = read_capture(capture_folder, stationarity_policy=stationarity_policy)
    training_hashes = {r["input_sha256"]["capture.bin"] for r in profile["training"]["pose_metrics"]}
    if record["input_sha256"]["capture.bin"] in training_hashes:
        raise CalibrationError("Validation raw capture was used for fitting, including aliases")
    expected = profile.get("sensor_register_tuple_devid_bw_format_fault")
    if not expected or record["register_tuple"] != expected:
        raise CalibrationError("Validation sensor configuration differs from calibration profile")
    metrics = pose_metric(record, profile)
    block_norms = np.linalg.norm(apply_calibration(record["one_second_means_counts"], profile), axis=1)
    overall_pass = abs(metrics["norm_residual_g"]) <= max_norm_error_g
    blocks_pass = bool((np.abs(block_norms - 1) <= max_norm_error_g).all())
    return {
        "schema_version": 1,
        "status": "independent static validation observation; no absolute angle qualification",
        "stationarity_policy": stationarity_policy,
        "original_static_gate_pass": record["stationarity"]["original_static_gate_pass"],
        "metrics": metrics,
        "norm_screen": {
            "maximum_absolute_mean_norm_residual_g": max_norm_error_g,
            "overall_mean_pass": overall_pass,
            "complete_one_second_mean_norms_g": block_norms.tolist(),
            "every_one_second_mean_pass": blocks_pass,
            "every_one_second_mean_required": stationarity_policy == "static-mean-v2",
            "pass": overall_pass and (blocks_pass or stationarity_policy == "strict"),
            "basis": "Practical session screen, not a sensor accuracy specification or proof of angular accuracy. Reference-repeat drift must remain visible separately.",
        },
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    fit = sub.add_parser("fit")
    fit.add_argument("session", type=Path)
    fit.add_argument("--output", required=True, type=Path)
    validate = sub.add_parser("validate")
    validate.add_argument("profile", type=Path)
    validate.add_argument("capture_folder", type=Path)
    validate.add_argument("--output", required=True, type=Path)
    validate.add_argument("--max-norm-error-g", type=float, default=0.03)
    validate.add_argument("--stationarity-policy", choices=("strict", "static-mean-v2"), default="strict",
                          help="Explicit static-mean-v2 uses stable 1s axis means plus every 1s corrected norm; preserves original raw-noise failure")
    args = parser.parse_args(argv)
    try:
        if args.output.exists():
            raise CalibrationError("Output already exists; choose a new filename")
        if args.command == "fit":
            result = fit_session(args.session)
        else:
            profile = json.loads(args.profile.read_text())
            result = validate_capture(profile, args.capture_folder, args.max_norm_error_g, args.stationarity_policy)
            result.update(profile_sha256=sha256(args.profile), tool_sha256=sha256(__file__))
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x") as handle:
            json.dump(result, handle, indent=2, allow_nan=False)
            handle.write("\n")
        print(json.dumps({k: result[k] for k in ("status", "bias_counts", "counts_per_g", "metrics") if k in result}, indent=2))
        return 0 if result.get("norm_screen", {}).get("pass", True) else 2
    except (CalibrationError, OSError, KeyError, TypeError, json.JSONDecodeError, np.linalg.LinAlgError) as error:
        print(f"Calibration failed: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
