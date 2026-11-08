"""Recompute descriptive stationary ADXL345 statistics without fitting offsets."""
import argparse
import csv
import hashlib
import json
import sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def describe(values):
    a = np.asarray(values, dtype=float)
    return {"count": len(a), "mean": float(a.mean()), "std_population": float(a.std()),
            "min": float(a.min()), "median": float(np.median(a)),
            "p95": float(np.percentile(a, 95)), "p99": float(np.percentile(a, 99)),
            "max": float(a.max())} if len(a) else None


def analyze(folder):
    folder = Path(folder)
    summary = json.loads((folder / "summary.json").read_text())
    for name, record in summary["files"].items():
        assert sha(folder / name) == record["sha256"], name
    with (folder / "records.csv").open() as handle:
        rows = list(csv.DictReader(handle))
    samples = [r for r in rows if r["type"] == "sample"]
    statuses = [r for r in rows if r["type"] == "status"]
    assert len(samples) > 1 and statuses
    axes = np.array([[int(r["raw_" + k]) for k in "xyz"] for r in samples])
    timestamp = np.array([int(r["time_us"]) for r in samples], dtype=np.int64)
    valid = np.array([r["sensor_configured"] == "1" and r["drdy_timestamp_qualified"] == "1"
                      for r in samples])
    qualified = axes[valid]
    assert len(qualified) > 1
    latency = np.array([int(r["read_latency_us"]) for r in samples])
    pair = valid[:-1] & valid[1:]
    intervals = np.diff(timestamp)[pair]
    dt = int(timestamp[-1] - timestamp[0])
    host_dt = int(samples[-1]["host_rx_monotonic_ns"]) - int(samples[0]["host_rx_monotonic_ns"])
    registers = sorted({(int(r["devid"]), int(r["bw_rate"]), int(r["data_format"]), int(r["fault_code"]))
                        for r in statuses})
    counters = {k: {"first": int(rows[0][k]), "last": int(rows[-1][k]),
                    "delta_modulo_uint32": (int(rows[-1][k]) - int(rows[0][k])) & 0xffffffff}
                for k in ["sensor_overrun_events", "service_gap_events", "queue_dropped_frames"]}
    result = {
        "input_sha256": {n: sha(folder / n) for n in ["capture.bin", "records.csv", "summary.json"]},
        "analysis_sha256": sha(__file__),
        "software": {"python": sys.version, "numpy": np.__version__, "matplotlib": matplotlib.__version__},
        "samples": len(samples), "status_rows": len(statuses),
        "qualified_samples": int(valid.sum()), "qualified_fraction": float(valid.mean()),
        "statistics_mask": "configured and qualified DRDY timestamp; all rows retained in raw/CSV",
        "qualified_axis_counts": {k: describe(qualified[:, i]) for i, k in enumerate("xyz")},
        "all_axis_counts": {k: describe(axes[:, i]) for i, k in enumerate("xyz")},
        "qualified_mean_nominal_g": (qualified.mean(axis=0) / 256).tolist(),
        "qualified_mean_vector_norm_nominal_g": float(np.linalg.norm(qualified.mean(axis=0)) / 256),
        "qualified_sample_norm_nominal_g": describe(np.linalg.norm(qualified, axis=1) / 256),
        "sample_span_mcu_us": dt, "sample_span_host_ns": host_dt,
        "all_row_effective_rate_mcu_hz": (len(samples) - 1) * 1e6 / dt,
        "all_row_effective_rate_host_hz": (len(samples) - 1) * 1e9 / host_dt,
        "normal_qualified_adjacent_interval_us": describe(intervals),
        "normal_qualified_cadence_hz": float(1e6 / intervals.mean()),
        "all_read_start_latency_us": describe(latency),
        "qualified_read_start_latency_us": describe(latency[valid]),
        "status_register_tuples_devid_bw_format_fault": registers,
        "all_status_healthy": registers == [(0xe5, 0x0d, 0x08, 0)],
        "counter_snapshots": counters,
        "unqualified_times_us": timestamp[~valid].tolist(),
        "unqualified_row_flags": [{k: r[k] for k in ["time_us", "read_latency_us", "sensor_overrun", "service_gap", "poll_fallback"]}
                                  for r, good in zip(samples, valid) if not good],
        "parser": summary["parser"], "quality": summary["quality"],
        "unexpected_fresh_pwm_rows": sum(int(r["pwm_fresh"]) for r in samples),
        "maximum_pwm_sequence": max(int(r["pwm_seq"]) for r in samples),
        "angle_calibrated": False, "offsets_fitted": False,
        "interpretation": "Uncalibrated counts at nominal 256 counts/g. One stationary pose cannot separate bias, sensitivity and mounting tilt. The measured cadence is distinct from configured nominal 800 Hz. ISR/sample/filter latency and electrical rail integrity are not independently measured."
    }
    (folder / "analysis.json").write_text(json.dumps(result, indent=2) + "\n")
    seconds = (timestamp - timestamp[0]) / 1e6
    fig, panels = plt.subplots(4, 1, figsize=(11, 8), sharex=True, constrained_layout=True)
    for i, k in enumerate("xyz"):
        panels[i].plot(seconds, axes[:, i], lw=.4, label=f"raw {k.upper()}")
        panels[i].scatter(seconds[~valid], axes[~valid, i], s=15, color="crimson", zorder=3,
                          label="unqualified read")
        panels[i].set_ylabel(f"{k.upper()} counts")
        panels[i].grid(alpha=.25)
        panels[i].legend(loc="upper right")
    panels[3].plot(seconds, latency, lw=.5, color="black")
    panels[3].scatter(seconds[~valid], latency[~valid], s=15, color="crimson", zorder=3)
    panels[3].axhline(625, color="gray", ls="--", label="625 µs read-completion deadline")
    panels[3].set_ylabel("Read-start latency µs")
    panels[3].set_xlabel("MCU seconds from first sample")
    panels[3].grid(alpha=.25)
    panels[3].legend(loc="upper right")
    fig.suptitle(f"{folder.name}: uncalibrated ADXL345 stationary recording\n"
                 f"{valid.sum():,}/{len(samples):,} qualified; normal cadence {result['normal_qualified_cadence_hz']:.3f} Hz; no offset fit")
    fig.savefig(folder / "stationary.png", dpi=150)
    plt.close(fig)
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path)
    args = parser.parse_args()
    report = analyze(args.folder)
    print(json.dumps({k: report[k] for k in ["samples", "qualified_fraction", "qualified_mean_nominal_g",
        "normal_qualified_cadence_hz", "counter_snapshots", "all_status_healthy"]}, indent=2))
