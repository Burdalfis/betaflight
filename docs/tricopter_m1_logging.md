# Tricopter M1 logging — 2026-11-06

M0 is complete: the unchanged local build is reproducible, the historical inputs are preserved, and the user confirmed the working cloud build and physical motor/servo mapping. See [M0 evidence](tricopter_m0_baseline.md) and [hardware mapping](tricopter_hardware.md).

M1 `bd6a5d727` has been flashed, read back, and bench-tested with PB0 servo feedback and low-power motor telemetry. **The remaining electrical/timing exit gates stay open.** See [the bench record](tricopter_m1_bench_2026-11-06.md). No new allocation, feedforward, angle compensation, PID changes, or CONTROL mode exists in this implementation.

The subsequent [unloaded characterization](tricopter_servo_characterization_2026-11-06.md) found an early common-positive wiper transient in both requested directions, persisting with the case clamped. The tap is observable, but early wiper voltage is not yet established as actual shaft/rotor motion. Servo supply/ground/reference measurements remain necessary before active position-dependent control.

**Wiring revision — 2026-11-06:** the user connected the wiper to **PB0 / ADC1_IN8** for proximity to the servo, replacing the earlier PC0/AirS proposal. M1 already reads configurable `ADC_EXTERNAL1`; the existing binary supports PB0 without a rebuild. `ADC_EXT 1 B00`, `ADC_RSSI 1 NONE`, and `RSSI_ADC` disabled were applied and verified. Earlier cloud captures remain unedited historical evidence.

## Modes and output path

- **LEGACY:** any debug mode other than `TRI_SERVO` leaves this diagnostic logger inactive. Existing debug modes retain their values because `TRI_SERVO` was appended to the enum.
- **LOG_ONLY:** `set debug_mode = TRI_SERVO` selects instrumentation while retaining the existing motor mixer and yaw-PID-to-servo path. There is no experimental control mode to enable.

The servo hook runs in `writeServos()` immediately after the TRI rudder hardware write, after servo mixing, filtering, limits and disarmed signal suppression. `debug[1]` is the exact PWM value passed to the driver, including zero when pulses are suppressed. It is the register-write request, **not** an independently captured pin edge or the measured mechanical position. At 498 Hz the timer emits pulses asynchronously; a scope is still required to establish output timing.

Diagnostics run at the existing servo-write/PID cadence and Blackbox samples them at its configured recording rate. No scheduler task, ADC interrupt, control filter or output correction was added.

## Debug schema

All eight fields are signed 16-bit values. Age fields use **100 microseconds per count**, rounded down, saturating at **32767 = 3.2767 seconds**. `-1` means no observation or unavailable; zero is a valid age. Age is metadata about observations, not an ESC acquisition timestamp. The underlying microsecond subtraction handles one timer wrap; use short bench sessions rather than interpreting gaps exceeding a full 32-bit timer period (~71.6 minutes).

| Field | Value / units |
|---|---|
| `debug[0]` | Configured `ADC_EXTERNAL1` raw 12-bit count (currently PB0), or `-1` when acquisition is not ready |
| `debug[1]` | Final TRI rudder PWM write request, microseconds; zero means signal suppressed |
| `debug[2]` | Time since last observed completed ADC DMA scan, 100 µs units |
| `debug[3]` | Status bit mask below; seen bits do **not** imply current freshness |
| `debug[4]` | Time since a newly decoded RPM frame for logical motor 0, 100 µs units |
| `debug[5]` | Same for logical motor 1 |
| `debug[6]` | Same for logical motor 2 |
| `debug[7]` | Worst existing DShot invalid-packet percentage among the three motors, 0–10000 = 0–100.00%; `-1` when unavailable |

| `debug[3]` bit | Meaning |
|---|---|
| 0 (`1`) | ADC backend supports DMA progress observation (currently STM32F4 only) |
| 1 (`2`) | External ADC channel initialized, DMA allocated/enabled, no DMA error flag |
| 2 (`4`) | At least one completed scan observed in this logging session |
| 3 (`8`) | A scan has been observed and the latest cached raw value is strictly between 0 and 4095 |
| 4 (`16`) | Bidirectional DShot telemetry is enabled |
| 5 (`32`) | At least one new RPM frame observed for logical motor 0 |
| 6 (`64`) | Same for logical motor 1 |
| 7 (`128`) | Same for logical motor 2 |

`debug[7]` reuses Betaflight's rolling packet statistics. On the active STM32 bitbang path **no-edge and aborted receptions are skipped by those statistics**. A low percentage does not prove good reception. Always inspect per-motor ages: a cached nonzero RPM and a rising age indicate that no new RPM has been received. EDT temperature/voltage/current/event frames and the EDT-enable handshake do not reset RPM age. Valid newly received zero RPM and repeated equal RPM values do reset age.

### ADC interpretation

The F4 backend polls the existing circular-DMA transfer-complete flag without enabling interrupts. The first call discards any completion flag left since boot, then waits for another completed scan. It reports **time since progress was observed**, rather than time since the physical channel conversion. Multiple scans may occur between polls, and observing progress does not prove that the analog input is healthy. A constant angle can correctly produce constant raw counts while scan progress remains fresh.

Other ADC backends expose raw values through their existing API but report progress observation as unsupported. The TRI schema reports unavailable acquisition until a supported backend is present; no fabricated freshness is supplied.

The non-rail indication is only a coarse electrical plausibility check. A disconnected floating wiper can sit inside the normal range indefinitely. **This circuit cannot guarantee disconnection detection.** M1 does not claim a flight-capable angle sensor or use its values for control. Electrical fault detection/protection and calibrated sensor limits are prerequisites for later control work.

The existing external-channel enable boolean is now exposed as `adc_external1_enabled`. It does not change the persistent parameter-group layout or its defaults. Changing it or `resource ADC_EXT` takes effect after save/reboot; the logger never remaps a pin or enables an ADC channel by itself.

## Existing Blackbox channels retained

Enable and inspect the native `motor[0..2]`, `servo[5]`, `eRPM[0..2]`, battery, gyro, setpoint and time fields alongside `debug[0..7]`. Motor commands remain in Betaflight's native output units (DShot commands for this configuration), rather than physical thrust. Logical motor 0 is rear, 1 front-right, 2 front-left under the verified aircraft configuration.

Native `eRPM` uses **100 electrical RPM per count**. With `motor_poles = 14`:

```text
mechanical_RPM = eRPM * 100 / 7
mechanical_rad_per_second = mechanical_RPM * 2*pi / 60
```

The cached native RPM fields may remain nonzero after reception stops. Freshness comes from the new debug ages. `getDshotRpm()` already performs the mechanical conversion; do not convert it again.

## Bench setup

Keep propellers removed for mapping, raw-input and motor/RPM checks. Save a complete current CLI export and retain the known-good cloud recovery image/configuration before using the local image. The [complete CLI text supplied in chat](tricopter_cloud_cli_2026-11-06.txt) includes the modern `diff all` and active resource listing; it is not a `dump all` export or a capture of cloud-build defaults. The shorter [cloud excerpt](tricopter_cloud_2026-11-06.txt) is a reference, not a full replayable configuration backup. Do not apply the old 4.3.1 dump or reset board defaults over the working aircraft mapping.

**Observed migration behavior:** this full local build links 91 configuration groups versus the cloud build's 77. Betaflight reset all settings on first boot despite an intact configuration sector. A fresh current-version `dump all` was restored successfully and verified after reboot. Preserve such a dump before moving between feature sets; matching version/source revisions alone do not guarantee configuration retention. The full flash backup and migration analysis are linked from the bench record.

After installing the local build, compare its resources against the confirmed mapping. Read these before changing settings:

```text
version
status
diff all
dump all
resource show all
get adc_external1_enabled
get debug_mode
get dshot
get motor_poles
get blackbox
```

The earlier cloud export shows PC0 as active `ADC_EXT`; that is the baseline before the user's PB0 wiring change. For the **current PB0 connection**, remap the external channel and remove the board's default PB0 RSSI assignment. This changes the sensor resource only; keep the verified motor/servo resources intact. The M1-specific setup is:

```text
feature -RSSI_ADC
resource ADC_RSSI 1 NONE
resource ADC_EXT 1 B00
set adc_external1_enabled = ON
set debug_mode = TRI_SERVO
set blackbox_disable_debug = OFF
set blackbox_disable_rpm = OFF
set blackbox_disable_motors = OFF
set blackbox_disable_servos = OFF
set blackbox_disable_bat = OFF
set blackbox_sample_rate = 1/4
save
```

After reboot, inspect both configured `resource` and active `resource show all`. Expect `ADC_RSSI 1 NONE`, `ADC_EXT 1 B00`, active `B00: ADC_EXT`, and PC0 free. Confirm ADC1 and its DMA2 Stream4 ownership are unchanged. A cross-resource conflict warning alone does not clear the other resource, which is why `ADC_RSSI` is explicitly removed. Do not enable `RSSI_ADC` to obtain potentiometer readings; the logger uses `ADC_EXTERNAL1` directly.

Check the actual PID rate before calling `1/4` approximately 1 kHz. The supplied status reports gyro ~8 kHz and `pid_process_denom=2`, consistent with a ~4 kHz PID loop. Keep receiver, failsafe, runaway protection, motor mapping, servo limits and motor protocol settings as already verified.

The original cloud configuration used EDT OFF. The bench session saved **`set dshot_edt = ON`** so normal arming requests Bluejay's required handshake. Disarmed MSP motor tests bypass that arming path, so the session explicitly sent inline EDT-enable command 13 to stopped ESCs first. `FORCE` changes decoder assumptions and was not needed. Recordings still show ~121 ms startup RPM reception gaps; the handshake alone does not establish continuous reception.

For a short disarmed ADC/servo recording, `blackbox_mode = ALWAYS` can be selected temporarily; it consumes flash whenever normal logging conditions permit. Stop/export the recording promptly and restore the previous mode (normally `NORMAL`). For motor-test recordings, `MOTOR_TEST` is available. Verify actual recording on the board: CLI interaction and USB motor control can affect logging and output behavior. Do not arm simply to obtain a log before completing the usual mapping/failsafe checks.

To leave LOG_ONLY, restore the previous `debug_mode` (or `NONE`), any temporary Blackbox mode and telemetry settings, then save/reboot. The actuator mixer is legacy in either mode.

## Required hardware results before closing M1

- [x] Flash/read back the local image and recover the current aircraft configuration. Source is `bd6a5d727`; backup and comparisons are in the bench record.
- [ ] Reconfirm logical motor identity/direction and PB1 servo output after local flashing; no default resource-map overwrite.
- [ ] Connect the direct wiper with common ground; compare voltages and servo behavior before/after attachment, motors disabled.
- [x] Verify PB0 as `ADC_EXT`, clear its `ADC_RSSI` duplicate and release PC0 while preserving actuator/DMA configuration.
- [x] Measure PB0 at 1100/1420/1700 µs: initial steady means were approximately 1942/1588/1280 counts. At reported 3.29 V Vref these imply 1.5605/1.2759/1.0288 V, consistent with prior wiper measurements; an independent voltmeter/loading comparison remains open.
- [ ] Verify ADC polarity, noise/loading, power sequencing and acquisition progress. Record open-wire/rail behavior without claiming reliable floating-wire detection.
- [ ] Confirm `debug[1]` versus scope-captured servo output and native `servo[5]`, including unarmed signal suppression if used; measure actual pulse timing/latency.
- [x] Verify indexed nonzero RPM at 5–10% one-motor-at-a-time command, EDT response, and explicit age growth during missing reception. Startup gaps remain observed limitations, not a continuous-freshness guarantee.
- [x] Verify ~1 kHz Blackbox schema, clean record endings and identical repeated reads; retain immutable raw bench recordings.
- [ ] Compare `status`/`tasks` and measured loop/output timing in LEGACY versus LOG_ONLY, including worst-case logging load. The supplied 39% cloud CPU snapshot is not a local performance measurement.

Independent physical rotor-angle calibration remains M2. No voltage-to-angle or PWM-to-angle calibration was invented here.

## Local verification

Run the focused local check script from the repository:

```sh
CC=gcc CXX=g++ src/test/run_tricopter_checks.sh
```

The six suites pass **44 host tests**: 9 new RPM-freshness cases, 7 TRI diagnostic cases, and 28 existing motor-ordering/Blackbox/encoding/PID regressions. They exercise the real RPM decoder and pole conversion, unchanged cached behavior, valid zero/equal packets, stale/invalid/missing/EDT data, independent motor ages, timestamp wrap, debug-mode isolation, raw/rail/unavailable flags, age encoding, final PWM including zero, and legacy mixer passthrough. Host diagnostics use mocked ADC/clock I/O; they do not test physical DMA or pulse timing.

Clang is absent in this environment, so these checks used the repository's experimental GCC/G++ fallback. Full target builds use the pinned Arm GCC and normal warning-as-error settings.

The initial full instrumented `MATEKF405TE` build passed: text **528605**, data **6620**, bss **100004** bytes, versus M0's 527457/6620/99900 (+1148 text, +104 bss). No on-board timing or stack measurement is implied. Build logs, exact source patch/fingerprints, hashes and images are preserved in [the M1 artifact folder](../.local-artifacts/tricopter/m1/manifest.json). Prefer the final manifest and its matching image when additional checkpoints are built.
