# Operational TRI feedback bench record — 2026-11-07

The first default feedback TRI controller was built, flashed to the MATEKF405TE and checked while disarmed. The motor is absent and the ADXL occupies its place. The controller and these checks use only the corrected PB0−PC0 input; the external ADXL logger was not used. No motor arming or motor commands were issued.

Source is the working-tree snapshot based on `1206c78f7b3addf1a41b2bb115187c222af9868b`. The reproducible build reports `norevision`; use the image/source hashes in [manifest.json](manifest.json) to identify this build. Implementation, settings, optional calibration and the bench procedure are described in [the firmware guide](../../TRICOPTER_FEEDBACK_MIXER.md).

## Build and focused checks

```sh
SOURCE_DATE_EPOCH=1789594937 make -j8 CONFIG=MATEKF405TE CCACHE= fwo
GCOV_PREFIX=/tmp/tricopter-operational-final-gcov CC=gcc CXX=g++ src/test/run_tricopter_checks.sh
```

The target build passes: flash body 545,076 bytes, vector segment 392 bytes, RAM 94,620 bytes and CCM 13,440 bytes. All **231 tests across 17 host targets pass**, including 17 control-core, 10 calibration-core, 15 live-adapter and 24 diagnostics tests. Existing PID, motor-output, ADC-pair, telemetry, Blackbox and offline regression checks pass; original input hashes remain unchanged. The new controller/calibration cores also passed strict/fast-math host checks and Cortex-M4F compilation. Advanced prediction, reaction-torque feedforward, dense angle mapping and finite-horizon allocation are absent from the flight source path.

## Flash and configuration

The original full 1 MiB flash and complete CLI configuration were backed up. Both programmed firmware segments match a full flash readback byte-for-byte. The configuration sector also matches the pre-flash bytes before the first boot.

Betaflight's existing boot policy resets the full configuration when the old tricopter parameter-group version cannot load. The aircraft configuration was restored from the backup. A first save attempt lacked an acknowledgement and did not persist; an explicit acknowledged `save noreboot`, followed by an independent reboot, resolved this. The failed attempt is retained separately. **All 1,472 existing exported command lines match exactly after reboot**, including the servo table, resources, motor ordering/rotation settings, PID/rate profiles and center. The only added commands are the 13 default TRI feedback settings. Subsequent temporary bench changes were saved/rebooted and restored with the same comparison.

HEX SHA-256: `cf1bdcf2e80700dc6d745a1ba3a340174a4eea35c503414ff6ceb90cc1e042a6`.

## Disarmed observations

- With the restored TRI configuration, status reports `config=1 feedback=1 vector=1`.
- After saving `debug_mode=NONE` and rebooting, all three remain valid. Feedback therefore does not depend on diagnostic logging.
- Small trim holds produce the following corrected-count-derived angle observations. These are internal feedback tracking checks, not independent absolute angle metrology. Target variation includes the existing yaw PID signal while disarmed.

| Requested trim | Target range | Measured range | Final PWM range | Largest observed tracking error |
|---|---:|---:|---:|---:|
| +5° | +4.9 to +5.1° | +4.3 to +5.8° | 1552–1554 µs | 0.7° |
| −5° | −5.3 to −4.9° | −5.7 to −5.1° | 1465–1468 µs | 0.8° |
| 0° return | −0.2 to +0.1° | −1.1 to −0.1° | 1507–1513 µs | 1.2° |

The optional calibration completed center → 1100 → 1700 → center, learned ADC **1936 / 1487 / 1280**, and reported a valid observed speed of **362°/s**. This estimate describes movement under its bounded command ramp, not maximum or loaded speed. Completion held exactly 1509 µs; applying the result in RAM succeeded. A second start/abort also returned to center. Learned values were not persisted: existing defaults **1936 / 1490 / 1281**, speed 0 and trim 0 were restored.

Final saved debug mode is the original `TRI_GROUND`. After settling, status shows a 125 µs gyro cycle, approximately 57% CPU and arming flags `RXLOSS NOPREARM CLI`, with no `TRI_FEEDBACK` or load flag. Thirty further observations confirm disarmed state, all three motor commands at 1000, bounded servo PWM and fresh coherent raw channels whose difference exactly equals the canonical position value. The final servo table is byte-identical to the pre-flash table, including **1100 / 1509 / 1700 µs, rate 100**.

## Evidence and remaining boundary

Full recovery images, firmware, raw CLI transcripts, intermediate failures and JSON observations remain under `.local-artifacts/tricopter/operational-feedback-2026-11-07/`. The manifest records their hashes and the final source hashes. [bench-evidence.tar.gz](bench-evidence.tar.gz) preserves that complete session, the preceding 1509 µs neutral-trim session and textual strict/fast-math control-core checks. [The archive manifest](bench-evidence-manifest.json) lists every member's size/hash and the excluded rebuildable caches/host objects/executables. [Independent archive verification](archive-verification.json) checks all member hashes and repeats the firmware-segment/config-sector comparisons directly from the archive. Extract from the repository root with `tar --keep-old-files -xzf docs/validation/tricopter-feedback-operational-2026-11-07/bench-evidence.tar.gz` to restore the original relative evidence paths without replacing existing files.

A harmless first readback attempt encountered a busy DFU interface while programming continued; the later full readback passed. Empty-file DFU leave was rejected, then leaving with the identical vector segment succeeded.

The motor must be reinstalled for normal prop-off output/rotation and stabilizing-yaw checks, followed by conservative hover testing. Loaded response, yaw gain tuning, motor headroom and flight behavior have not been validated here. Propeller/motor rotation was not changed. `tri_yaw_direction` and `tri_yaw_trim` provide independent polarity/bias adjustments for the planned later reversal while preserving physical PWM/ADC calibration.
