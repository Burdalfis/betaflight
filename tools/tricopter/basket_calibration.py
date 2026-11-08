#!/usr/bin/env python3
"""Export a frozen offline measured basket-angle map and test new settled holds.

The input is fresh signed PB0-PC0, never a command-derived position. Major
approach branches are averaged at matched ADC counts; approach is not switched
instantaneously on a PWM reversal. The half-branch separation is descriptive,
not a confidence interval or an absolute/loaded-angle accuracy claim.
"""

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys

import numpy as np


class CalibrationError(ValueError):
    pass


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text())


def projection_basis(normal):
    n = np.asarray(normal, dtype=float)
    if n.shape != (3,) or not np.isfinite(n).all() or np.linalg.norm(n) < .5:
        raise CalibrationError("Invalid fitted hinge normal")
    n /= np.linalg.norm(n)
    axis_x = np.array([1., 0., 0.])
    u = axis_x - np.dot(axis_x, n) * n
    if np.linalg.norm(u) < .25:
        raise CalibrationError("Sensor +X has weak projection into the hinge plane")
    u /= np.linalg.norm(u)
    return n, u, np.cross(n, u)


def basket_roll(vectors, normal):
    n, u, v = projection_basis(normal)
    g = np.asarray(vectors, dtype=float)
    if g.shape[-1:] != (3,) or not np.isfinite(g).all():
        raise CalibrationError("Invalid calibrated gravity vectors")
    projected = g - np.expand_dims(g @ n, -1) * n
    if np.any(np.linalg.norm(projected, axis=-1) < .25):
        raise CalibrationError("Weak gravity projection into the fitted hinge plane")
    return -np.degrees(np.arctan2(g @ v, g @ u))


def source_analysis(path, summary_only=False, require_all=True):
    path = Path(path).resolve()
    data = read_json(path)
    if data.get("all_settled_windows_pass") not in (True, False) or (require_all and data["all_settled_windows_pass"] is not True):
        raise CalibrationError("Analysis contains unqualified settled windows")
    hashes = data.get("provenance", {}).get("input_sha256", {})
    if not hashes:
        raise CalibrationError("Analysis lacks input-artifact hashes")
    for name, expected in hashes.items():
        source = (path.parent / name).resolve()
        if (not source.is_relative_to(path.parent) or not isinstance(expected, str) or len(expected) != 64
                or any(c not in "0123456789abcdef" for c in expected)):
            raise CalibrationError(f"Invalid recorded input hash: {name}")
        if not summary_only and (not source.is_file() or sha256(source) != expected):
            raise CalibrationError(f"Analysis input hash mismatch: {name}")
    if summary_only:
        restored = data.get("provenance", {}).get("servo_restored_exact") is True
    else:
        run = read_json(path.parent / "run.json")
        fc_run = read_json(path.parent / "fc-run.json")
        restored = run.get("sequence_complete") is True and fc_run.get("servo_restored_exact") is True
        if run.get("profile_sha256") != hashes.get("calibration-profile.json"):
            raise CalibrationError("Run sensor profile does not match recorded input hash")
    if not restored:
        raise CalibrationError("Source sequence is incomplete or exact restoration is unverified")
    return data


def extract_branches(analysis, normal):
    """One settled observation per approach/PWM; reject unexpected duplicates."""
    branches = {"increasing_pwm": [], "decreasing_pwm": []}
    seen = {key: set() for key in branches}
    excluded = []
    for row in analysis["plateaus"]:
        if row["approach"] not in ("increasing", "decreasing"):
            excluded.append(row["plateau_id"])
            continue
        if not row["adxl"]["paired_observation_qualified"] or not row["adxl"]["settled_mean_screen_pass"]:
            raise CalibrationError("A branch observation is not qualified and settled")
        key = row["approach"] + "_pwm"
        command = row["target_us"]
        if command in seen[key]:
            raise CalibrationError(f"Duplicate {key} observation at {command}us; explicit selection is required")
        seen[key].add(command)
        count = float(row["fc"]["corrected_pb0_minus_pc0_counts"]["mean"])
        angle = float(basket_roll(row["adxl"]["corrected_mean_g"], normal))
        if not np.isfinite(count) or row["fc"]["fresh_matching_rows"] < 5:
            raise CalibrationError("Invalid or insufficient fresh PB0-PC0 observations")
        branches[key].append({"plateau_id": row["plateau_id"], "command_us": command,
                              "corrected_counts": count, "basket_roll_deg": angle})
    for key, points in branches.items():
        points.sort(key=lambda r: r["corrected_counts"])
        if len(points) < 3:
            raise CalibrationError(f"Insufficient observations for {key}")
        counts = np.array([r["corrected_counts"] for r in points])
        angles = np.array([r["basket_roll_deg"] for r in points])
        differences = np.diff(angles)
        if np.any(np.diff(counts) <= 0) or not (np.all(differences > 0) or np.all(differences < 0)):
            raise CalibrationError(f"{key} counts/angles are not strictly monotonic")
    return branches, excluded


def error_statistics(errors):
    values = np.asarray(errors, dtype=float)
    if not len(values):
        return None
    return {"count": len(values), "mean": float(values.mean()),
            "rms": float(np.sqrt(np.mean(values**2))), "max_abs": float(np.max(np.abs(values))),
            "p95_abs": float(np.percentile(np.abs(values), 95))}


def paired_command_regression(branches, physical_middle_us):
    if any(len({p["command_us"] for p in values}) != len(values) for values in branches.values()):
        raise CalibrationError("Unexpected duplicate branch/command observation; explicit selection is required")
    lookups = {name: {p["command_us"]: p for p in values} for name, values in branches.items()}
    a, b = lookups["increasing_pwm"], lookups["decreasing_pwm"]
    commands = sorted(set(a) & set(b))
    if len(commands) < 3 or not np.isfinite(physical_middle_us):
        raise CalibrationError("Need at least three shared commands and a finite physical middle")
    pairs = [{"command_us": u, "increasing_plateau_id": a[u]["plateau_id"],
              "decreasing_plateau_id": b[u]["plateau_id"],
              "increasing_angle_deg": a[u]["basket_roll_deg"], "decreasing_angle_deg": b[u]["basket_roll_deg"],
              "mean_angle_deg": (a[u]["basket_roll_deg"] + b[u]["basket_roll_deg"]) / 2,
              "increasing_corrected_counts": a[u]["corrected_counts"],
              "decreasing_corrected_counts": b[u]["corrected_counts"],
              "mean_corrected_counts": (a[u]["corrected_counts"] + b[u]["corrected_counts"]) / 2,
              "half_branch_separation_deg": abs(a[u]["basket_roll_deg"] - b[u]["basket_roll_deg"]) / 2}
             for u in commands]
    x = np.asarray(commands, dtype=float) - physical_middle_us
    y = np.asarray([p["mean_angle_deg"] for p in pairs])
    centered_x, centered_y = x - x.mean(), y - y.mean()
    sst = float(centered_y @ centered_y)
    if np.dot(centered_x, centered_x) <= 0 or sst <= 0:
        raise CalibrationError("Regression commands/angles have no variance")
    slope = float(centered_x @ centered_y / (centered_x @ centered_x))
    intercept = float(y.mean() - slope * x.mean())
    forced_slope = float(x @ y / (x @ x))
    pearson = float(centered_x @ centered_y / np.sqrt((centered_x @ centered_x) * sst))

    def report(k, intercept_value):
        residual = y - (k * x + intercept_value)
        return {"slope_deg_per_us": k, "intercept_deg": intercept_value,
                "r_squared_centered": float(1 - residual @ residual / sst),
                "residual_deg": residual.tolist(), **error_statistics(residual)}

    free = report(slope, intercept)
    free["fitted_zero_crossing_us"] = float(physical_middle_us - intercept / slope) if slope else None
    free["formula"] = "angle_deg=slope_deg_per_us*(command_us-physical_middle_us)+intercept_deg"
    forced = report(forced_slope, 0.)
    forced["formula"] = "angle_deg=slope_deg_per_us*(command_us-physical_middle_us)"
    return {"physical_middle_us": physical_middle_us, "paired_command_points": len(pairs),
            "branch_observations": 2 * len(pairs), "command_bounds_us": [commands[0], commands[-1]],
            "pairing_rule": "Exactly one increasing and one decreasing settled mean per shared command; equal branch weighting. Initial/unchanged and unpaired endpoints are excluded. Unexpected duplicate branch/command observations are rejected.",
            "pearson_r": pearson, "free_intercept": free, "forced_physical_zero": forced,
            "r_squared_definition": "1-SSE/sum((paired_mean_angle-mean(paired_mean_angle))^2), including for the constrained fit; constrained R^2 may be negative.",
            "paired_means": pairs,
            "unpaired_commands_excluded_us": sorted(set(a) ^ set(b)),
            "maximum_full_branch_separation_deg": max(2 * p["half_branch_separation_deg"] for p in pairs),
            "interpretation": "Descriptive regression of paired settled means from one sweep; individual ADC samples are not independent regression observations. Branch hysteresis remains separate from mean-line residuals."}


def paired_count_regression(pairs):
    x = np.asarray([p["mean_corrected_counts"] for p in pairs], dtype=float)
    y = np.asarray([p["mean_angle_deg"] for p in pairs], dtype=float)
    dx, dy = x - x.mean(), y - y.mean()
    if len(x) < 3 or not np.isfinite(x).all() or not np.isfinite(y).all() or dx @ dx <= 0 or dy @ dy <= 0:
        raise CalibrationError("Invalid paired count/angle regression observations")
    slope = float(dx @ dy / (dx @ dx))
    intercept = float(y.mean() - slope * x.mean())
    residual = y - (slope * x + intercept)
    return {"paired_command_points": len(x), "source_position": "signed PB0-PC0 only",
            "slope_deg_per_count": slope, "intercept_deg": intercept,
            "fitted_zero_count": float(-intercept / slope) if slope else None,
            "count_bounds": [float(x.min()), float(x.max())],
            "pearson_r": float(dx @ dy / np.sqrt((dx @ dx) * (dy @ dy))),
            "r_squared_centered": float(1 - residual @ residual / (dy @ dy)),
            "residual_deg": residual.tolist(), **error_statistics(residual),
            "formula": "angle_deg=slope_deg_per_count*mean_corrected_counts+intercept_deg",
            "pairing_rule": "Exactly two settled means per shared PWM: mean(PB0-PC0_increasing,PB0-PC0_decreasing) against mean(angle_increasing,angle_decreasing). Equal weight per command; no knot-grid or individual ADC-sample weighting.",
            "interpretation": "Descriptive linear approximation only; fitted zero does not replace the operator-defined reference or the bounded monotonic measured mean map."}


def mean_count_map(branches, max_knots=64):
    if type(max_knots) is not int or not 2 <= max_knots <= 64:
        raise CalibrationError("Maximum knots must be an integer in2..64")
    arrays = {}
    for name, points in branches.items():
        x = np.asarray([p["corrected_counts"] for p in points], dtype=float)
        y = np.radians([p["basket_roll_deg"] for p in points])
        if (len(x) < 3 or not np.isfinite(x).all() or not np.isfinite(y).all()
                or np.any(np.diff(x) <= 0)
                or not (np.all(np.diff(y) > 0) or np.all(np.diff(y) < 0))):
            raise CalibrationError("Each major branch must have finite strictly monotonic count/angle points")
        arrays[name] = x, y
    xi, yi = arrays["increasing_pwm"]
    xd, yd = arrays["decreasing_pwm"]
    if np.sign(yi[-1] - yi[0]) != np.sign(yd[-1] - yd[0]):
        raise CalibrationError("Major branch angle directions disagree")
    low, high = max(xi[0], xd[0]), min(xi[-1], xd[-1])
    if low >= high:
        raise CalibrationError("Major branches have no common measured count domain")
    knots = np.unique(np.r_[xi[(xi >= low) & (xi <= high)], xd[(xd >= low) & (xd <= high)], low, high])
    if len(knots) > max_knots:
        raise CalibrationError("Union-of-knots exceeds configured limit; no silent decimation is performed")
    inc, dec = np.interp(knots, xi, yi), np.interp(knots, xd, yd)
    angles = (inc + dec) / 2
    differences = np.diff(angles)
    if not (np.all(differences > 0) or np.all(differences < 0)):
        raise CalibrationError("Mean angle map is not strictly monotonic")
    return {"count_bounds": [float(low), float(high)],
            "angle_bounds_rad": [float(angles.min()), float(angles.max())],
            "knots": [{"corrected_counts": float(c), "angle_rad": float(a),
                       "half_branch_separation_rad": float(abs(i - d) / 2)}
                      for c, a, i, d in zip(knots, angles, inc, dec)],
            "knot_count": len(knots), "maximum_knots": max_knots,
            "interpolation": "Piecewise linear in measured counts; no extrapolation; union of both major-branch knots in their common count domain",
            "uncertainty_interpretation": "Half-branch separation describes the measured major-branch envelope. It is not a confidence interval, a bound on minor-loop/reversal error or a claim of absolute/loaded accuracy."}


def export_profile(analysis_path, mounting_path, zero_path, max_knots=64, summary_only=False):
    analysis = source_analysis(analysis_path, summary_only)
    if analysis["rotation_plane"].get("identified") is not True:
        raise CalibrationError("Training sweep does not identify the gravity rotation plane")
    mounting, zero = read_json(mounting_path), read_json(zero_path)
    if mounting.get("motor_present") is not False or mounting.get("airframe_roll_level_operator_report") is not True:
        raise CalibrationError("Expected operator-confirmed roll-level basket reference with motor absent")
    if not zero.get("operator_alignment_reference") or not zero.get("configuration_restored_exact"):
        raise CalibrationError("Live zero reference/restoration is not established")
    normal = analysis["rotation_plane"]["normal_sensor_axes"]
    n, u, v = projection_basis(normal)
    if not np.allclose(n, zero["projection"]["normal_sensor_axes"], atol=1e-10, rtol=0):
        raise CalibrationError("Zero and full-sweep hinge normals disagree")
    if zero["provenance"]["hinge_analysis_sha256"] != sha256(analysis_path):
        raise CalibrationError("Zero reference does not bind this frozen training sweep")
    branches, excluded = extract_branches(analysis, n)
    physical_middle = float(np.mean([zero["branches"][b]["interpolated_command_us"] for b in branches]))
    mapping = mean_count_map(branches, max_knots)
    regression = paired_command_regression(branches, physical_middle)
    profile_hash = analysis["provenance"]["input_sha256"]["calibration-profile.json"]
    return {"schema_version": 1, "kind": "tricopter_basket_measured_angle_mean_map",
            "status": "offline measured settled map; not installed into flight control",
            "source_position": "signed PB0-PC0 only", "input_units": "ADC counts", "angle_units": "radians",
            "operator_defined_reference": True, "motor_present_during_characterization": False,
            **mapping,
            "angle_convention": "Mechanical basket rotation is negative gravity-vector phase about the fitted positive hinge normal; physical airframe yaw sign is unassigned.",
            "reference": {"definition": mounting["zero_definition"], "operator_defined_basket_roll_zero": True,
                          "motor_present": False, "external_motor_shaft_vertical_verified": False,
                          "absolute_angle_accuracy_independently_proven": False,
                          "normal_sensor_axes": n.tolist(), "projected_positive_x": u.tolist(),
                          "in_plane_quadrature": v.tolist(), "physical_middle_us": physical_middle,
                          "live_zero_brackets": zero["branches"],
                          "curve_warp_or_live_refit_applied": False},
            "branches": branches, "regression": regression,
            "count_regression": paired_count_regression(regression["paired_means"]),
            "training": {"source": str(Path(analysis_path).resolve()), "excluded_initial_or_unchanged_plateau_ids": excluded,
                         "settled_major_branch_points": sum(map(len, branches.values())),
                         "source_position": "Fresh per-plateau signed PB0-PC0 means only; raw PB0/PC0 and commanded PWM are not measured-position fallbacks"},
            "validation": {"reversal_data_used_for_fitting": False, "held_out_reversal_validation": "pending"},
            "provenance": {"analysis_sha256": sha256(analysis_path), "sensor_profile_sha256": profile_hash,
                           "recorded_training_input_artifact_sha256": analysis["provenance"]["input_sha256"],
                           "source_mode": "tracked-summary-only; underlying raw input hashes retained but not reverified" if summary_only else "all recorded input-artifact file hashes reverified; source analysis is not refitted",
                           "training_adxl_capture_sha256": analysis["provenance"]["input_sha256"]["adxl/capture.bin"],
                           "training_fc_samples_sha256": analysis["provenance"]["input_sha256"]["fc-samples.csv"],
                           "mounting_reference_sha256": sha256(mounting_path), "live_zero_result_sha256": sha256(zero_path),
                           "export_tool_sha256": sha256(__file__)},
            "limitations": "Frozen unloaded, settled observations with operator-defined basket zero. Branch-mean lookup does not instantaneously switch branches on PWM reversal. No dynamic, motor-load or absolute-angle qualification is implied."}


def validate_profile(profile):
    if (profile.get("schema_version") != 1 or profile.get("kind") != "tricopter_basket_measured_angle_mean_map"
            or profile.get("source_position") != "signed PB0-PC0 only" or profile.get("angle_units") != "radians"
            or profile.get("operator_defined_reference") is not True):
        raise CalibrationError("Unsupported measured-angle profile schema/source/reference")
    knots = profile["knots"]
    if not 2 <= len(knots) <= 64:
        raise CalibrationError("Measured-angle profile requires2..64knots")
    x = np.asarray([k["corrected_counts"] for k in knots], dtype=float)
    y = np.asarray([k["angle_rad"] for k in knots], dtype=float)
    e = np.asarray([k["half_branch_separation_rad"] for k in knots], dtype=float)
    if (not all(np.isfinite(a).all() for a in (x, y, e)) or np.any(e < 0) or np.any(np.diff(x) <= 0)
            or not (np.all(np.diff(y) > 0) or np.all(np.diff(y) < 0))
            or profile["count_bounds"] != [float(x[0]), float(x[-1])]
            or profile["angle_bounds_rad"] != [float(y.min()), float(y.max())]):
        raise CalibrationError("Invalid measured-angle knots/bounds")
    return x, y, e


def interpolate(profile, counts):
    x, y, e = validate_profile(profile)
    value = float(counts)
    if not np.isfinite(value) or value < x[0] or value > x[-1]:
        raise CalibrationError("Measured counts outside the common calibrated domain")
    return float(np.interp(value, x, y)), float(np.interp(value, x, e))


def validate_reversals(profile, analysis_path, summary_only=False):
    validate_profile(profile)
    analysis = source_analysis(analysis_path, summary_only, require_all=False)
    input_hashes = analysis["provenance"]["input_sha256"]
    if (sha256(analysis_path) == profile["provenance"]["analysis_sha256"]
            or input_hashes["adxl/capture.bin"] == profile["provenance"]["training_adxl_capture_sha256"]
            or input_hashes["fc-samples.csv"] == profile["provenance"]["training_fc_samples_sha256"]):
        raise CalibrationError("Training sweep cannot serve as held-out reversal validation")
    if input_hashes["calibration-profile.json"] != profile["provenance"]["sensor_profile_sha256"]:
        raise CalibrationError("Held-out sensor profile differs from training")
    normal = profile["reference"]["normal_sensor_axes"]
    rows = []
    last_direction, last_target = None, None
    for source in analysis["plateaus"]:
        counts = float(source["fc"]["corrected_pb0_minus_pc0_counts"]["mean"])
        measured = float(basket_roll(source["adxl"]["corrected_mean_g"], normal))
        direction = source["approach"]
        reversal = direction in ("increasing", "decreasing") and last_direction is not None and direction != last_direction
        context = "direction_reversal" if reversal else "initial" if last_direction is None else "continuation"
        excluded = []
        if not source["adxl"].get("paired_observation_qualified"):
            excluded.append("ADXL/PWM plateau association is unqualified")
        if not source["adxl"].get("settled_mean_screen_pass"):
            excluded.append("Settled gravity block stability/norm screen failed")
        if source["fc"].get("fresh_matching_rows", 0) < 5:
            excluded.append("Too few fresh matching canonical position means")
        row = {"plateau_id": source["plateau_id"], "command_us": source["target_us"], "approach": direction,
               "previous_command_us": last_target, "context": context, "corrected_counts": counts,
               "measured_basket_roll_deg": measured, "in_common_count_domain": profile["count_bounds"][0] <= counts <= profile["count_bounds"][1],
               "qualified_for_scoring": not excluded, "excluded_reasons": excluded,
               "errors_diagnostic_only": bool(excluded)}
        predictions = {}
        for name, points in profile["branches"].items():
            x = [p["corrected_counts"] for p in points]
            if x[0] <= counts <= x[-1]:
                predictions[name] = float(np.interp(counts, x, [p["basket_roll_deg"] for p in points]))
        row["major_branch_predictions_deg"] = predictions
        count_line = profile["count_regression"]
        row["count_regression_in_training_domain"] = count_line["count_bounds"][0] <= counts <= count_line["count_bounds"][1]
        if row["count_regression_in_training_domain"]:
            row["count_line_prediction_deg"] = count_line["slope_deg_per_count"] * counts + count_line["intercept_deg"]
            row["count_line_error_deg"] = row["count_line_prediction_deg"] - measured
        regression = profile["regression"]
        command_low, command_high = regression["command_bounds_us"]
        row["command_regression_in_training_domain"] = command_low <= source["target_us"] <= command_high
        if row["command_regression_in_training_domain"]:
            x = source["target_us"] - regression["physical_middle_us"]
            row["command_predictions_deg"] = {key: model["slope_deg_per_us"] * x + model["intercept_deg"]
                                               for key in ("free_intercept", "forced_physical_zero")
                                               for model in (regression[key],)}
            row["command_prediction_errors_deg"] = {k: value - measured for k, value in row["command_predictions_deg"].items()}
        if row["in_common_count_domain"]:
            predicted, separation = interpolate(profile, counts)
            predicted = float(np.degrees(predicted))
            row.update(mean_map_prediction_deg=predicted, error_deg=predicted - measured,
                       descriptive_half_branch_separation_deg=float(np.degrees(separation)))
        if len(predictions) == 2:
            low, high = min(predictions.values()), max(predictions.values())
            row.update(major_branch_hull_deg=[low, high], within_major_branch_hull=low <= measured <= high,
                       distance_outside_major_branch_hull_deg=max(low - measured, measured - high, 0.))
        rows.append(row)
        if direction in ("increasing", "decreasing"):
            last_direction = direction
        last_target = source["target_us"]
    covered = [r for r in rows if "error_deg" in r and r["qualified_for_scoring"]]
    matched = [r for r in covered if "command_prediction_errors_deg" in r]
    return {"schema_version": 1, "status": "held-out settled reversal validation; fit unchanged",
            "fit_updated": False, "analysis_sha256": sha256(analysis_path),
            "held_out_input_artifact_sha256": input_hashes,
            "source_mode": "tracked-summary-only; raw input hashes retained but not reverified" if summary_only else "recorded raw input-artifact hashes reverified",
            "source_position": "fresh signed PB0-PC0 only", "points": rows,
            "input_points": len(rows), "qualified_points": sum(r["qualified_for_scoring"] for r in rows),
            "qualified_mask": [r["qualified_for_scoring"] for r in rows],
            "excluded_unqualified_plateau_ids": [r["plateau_id"] for r in rows if not r["qualified_for_scoring"]],
            "covered_points": len(covered),
            "out_of_domain_points": sum(r["qualified_for_scoring"] and not r["in_common_count_domain"] for r in rows),
            "unqualified_points": sum(not r["qualified_for_scoring"] for r in rows),
            "mean_map_error_deg": error_statistics([r["error_deg"] for r in covered]),
            "count_line_error_deg": error_statistics([r["count_line_error_deg"] for r in covered if "count_line_error_deg" in r]),
            "matched_model_comparison": {
                "points": len(matched),
                "selection": "Only held-out points inside both the common count domain and the paired-command training domain; no extrapolation",
                "measured_count_map_error_deg": error_statistics([r["error_deg"] for r in matched]),
                **{key + "_command_error_deg": error_statistics([r["command_prediction_errors_deg"][key] for r in matched])
                   for key in ("free_intercept", "forced_physical_zero")},
                "command_is_measured_position": False},
            "by_context": {key: error_statistics([r["error_deg"] for r in covered if r["context"] == key])
                           for key in ("initial", "continuation", "direction_reversal")},
            "major_branch_hull_coverage_fraction": float(np.mean([r["within_major_branch_hull"] for r in covered])) if covered else None,
            "interpretation": "No held-out data are pooled into fitting and no branch is instantaneously selected on command reversal. Errors describe these settled unloaded holds only; hull membership is descriptive, not a confidence or absolute-accuracy statement."}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    export = sub.add_parser("export")
    export.add_argument("--analysis", required=True, type=Path)
    export.add_argument("--mounting-reference", required=True, type=Path)
    export.add_argument("--zero-result", required=True, type=Path)
    export.add_argument("--max-knots", type=int, default=64)
    export.add_argument("--summary-only", action="store_true", help="Reproduce from recorded summary attestations; preserve raw hashes without claiming raw files were reverified")
    export.add_argument("--output", required=True, type=Path)
    validate = sub.add_parser("validate")
    validate.add_argument("profile", type=Path)
    validate.add_argument("--analysis", required=True, type=Path)
    validate.add_argument("--output", required=True, type=Path)
    validate.add_argument("--summary-only", action="store_true")
    args = parser.parse_args(argv)
    try:
        if args.output.exists():
            raise CalibrationError("Output already exists")
        if args.command == "export":
            result = export_profile(args.analysis, args.mounting_reference, args.zero_result, args.max_knots, args.summary_only)
        else:
            result = validate_reversals(read_json(args.profile), args.analysis, args.summary_only)
            result["frozen_mean_map_sha256"] = sha256(args.profile)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x") as handle:
            json.dump(result, handle, indent=2, allow_nan=False)
            handle.write("\n")
        print(json.dumps({k: result[k] for k in ("status", "count_bounds", "angle_bounds_rad", "knot_count", "mean_map_error_deg") if k in result}, indent=2))
        return 0
    except (CalibrationError, OSError, ValueError, TypeError, KeyError) as error:
        print(f"Basket calibration failed: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
