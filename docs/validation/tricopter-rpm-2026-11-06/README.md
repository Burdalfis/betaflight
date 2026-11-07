# RPM telemetry replay inputs — 2026-11-06

These two immutable Blackbox records preserve the props-off motor test and its earlier aborted attempt. They support offline observer compatibility, causality and stale-telemetry tests. They do not provide true shaft acceleration, a calibrated command/voltage model, loaded response or torque coefficients.

The successful record contains 20,208 frames over 20.162233 s. Each of the three motors was driven separately with external commands 1050, 1075 and 1100 after a stopped hold. The aircraft supply was set to 7 V with a 2 A current limit. Recorded RPM ages peak at about 121 ms during startup; the aborted record contains a 1.3356 s gap while RPM remains cached at zero. A zero invalid-packet percentage does not establish fresh telemetry.

## Provenance

[The archive manifest](capture/archive-manifest.json) records byte offsets, sizes and SHA-256 hashes. Both raw files were verified byte for byte against slices of the original 1,816,576-byte `bench-all.bbl`, whose two complete downloads have identical SHA-256 `2749e0ca1a2affda8b2c354bd94bb784d948a6af031f25c2e2e6415087a8be6d`. The combined file remains local; its motor records are self-contained here. The original cloud firmware and the corrected servo datasets are separate evidence.

| Record | Original half-open byte range | Included file |
|---|---|---|
| Aborted motor test | `[747520, 940032)` | [record-02-aborted-motor.bbl](raw/record-02-aborted-motor.bbl) |
| Successful low-power test | `[940032, 1816576)` | [record-03-low-power-motors.bbl](raw/record-03-low-power-motors.bbl) |

[Capture summaries and MSP samples](capture/motor-bench-summary.json) retain the exact external commands and host timestamps. The original [capture program](tools/motor_bench.py) and [serial helper](tools/bench_io.py) are historical hardware-operation provenance; offline reproduction does not run them. The earlier attempt has matching `attempt1-*` metadata. [Native-log summary](capture/native-log-summary.json) preserves the existing decoded descriptive statistics, rather than identifying a motor model.

## Offline decoding

Verify `checksums.sha256` from this directory with `sha256sum -c checksums.sha256`. Use the official decoder at the commit in [decoder provenance](tools/blackbox-decoder-provenance.json), applying the included [integer P-interval compatibility patch](tools/blackbox-tools-integer-p-interval.patch), then build `obj/blackbox_decode`. The captured decoder binary hash is pinned in that provenance. This patch changes parsing of the modern integer decimation field; raw records and firmware are unchanged.

From the repository root, pass each raw file separately to the decoder, writing outside this evidence directory, for example:

```sh
DECODER=/path/to/pinned/obj/blackbox_decode
mkdir -p .local-artifacts/tricopter/rpm-replay/decoded
"$DECODER" --unit-vbat raw --unit-flags raw --save-headers \
  --output-dir .local-artifacts/tricopter/rpm-replay/decoded \
  docs/validation/tricopter-rpm-2026-11-06/raw/record-03-low-power-motors.bbl
```

These records use firmware `bd6a5d727`, `TRI_SERVO` (`debug_mode=102`), 14 magnetic poles, a 125 µs gyro loop, PID denominator 2 and P interval 4. Native `eRPM[i]` units are 100 electrical RPM per count; mechanical RPM is `eRPM[i] * 100 / 7`. `debug[4..6]` are the individual logical motors' decoder-observation ages in floored 100 µs units; `-1` is unseen and 32767 is saturated. Status seen bits do not establish freshness. The historical raw PB0 channel in this schema is diagnostic only and must not be used as servo position.

## Timestamp and validation boundary

No sample sequence was recorded. Ages were read using `micros()` in `servosTricopterLog()`, later within the loop than the PID-task timestamp stored as Blackbox time. Their phase offset and jitter were not measured. Consequently `(tBB-(age+1)*100, tBB-age*100]` is only a timestamp proxy before allowing for that phase uncertainty; it is not a proven exact observation interval. Replay must declare this uncertainty and test sensitivity. Overlapping plausible observation intervals may be conservatively treated as cached/ambiguous samples. Equal RPM values alone never imply duplicates: genuine new equal-valued or zero-RPM frames refresh telemetry. These logs cannot establish a ground-truth acceleration error or exact new-packet count.

Native `motor[]` clips low disarmed test values at its idle encoding, so it does not recover the exact external commands here. MSP samples and events retain those commands on a host clock, with no measured host-to-FC clock alignment. Command-to-RPM delay identification requires a separate timing contract.

A future pure-C observer should accept mechanical RPM with an actual decoder-observation timestamp and sequence, deduplicate by identity, and preserve separate estimate/freshness status through gaps. EDT, invalid and missing frames must not refresh observation age. The current firmware only tracks this metadata in `TRI_SERVO` and `TRI_GROUND`; use in another mode requires an explicit integration change. No firmware behavior changes or hardware operations are part of this archive.
