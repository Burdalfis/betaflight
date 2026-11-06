# Tricopter M1 bench record — 2026-11-06

The M1 image at `bd6a5d727` was flashed to the connected `MATEKF405TE`, the working aircraft configuration was recovered and verified, and the direct potentiometer input was moved to **PB0**. Servo feedback and low-command, propeller-free motor telemetry were recorded successfully. The existing legacy flight-control path remains active; no physical allocation or compensation was introduced.

These are bench results, not full M1 qualification or flight approval. Physical angle calibration, loaded dynamics, independently captured PWM edges, and sensor fault qualification remain open.

**Fixture correction:** the user subsequently identified that the servo case was free to rotate under internal motor reaction torque during the first recordings. Those response times describe the free-case fixture, not a rigidly mounted servo. Initial wrong-way wiper movement must not automatically be dismissed as electrical noise or attributed to gear lash. The case was then clamped, with the output arm free and linkage disconnected, and the same response/hysteresis tests were repeated. See [the servo characterization](tricopter_servo_characterization_2026-11-06.md) for the condition-specific results.

All raw evidence referenced below is under [`.local-artifacts/tricopter/m1-bench-2026-11-06/`](../.local-artifacts/tricopter/m1-bench-2026-11-06/). That folder is intentionally local and ignored by Git; this report records the useful results and their limits.

## Backup, flashing and configuration recovery

Before flashing, the session captured the current cloud firmware's CLI export and a complete **1 MiB internal-flash image**, including the 16 KiB configuration sector at `0x08004000`–`0x08007FFF`:

- [Preflash CLI transcript](../.local-artifacts/tricopter/m1-bench-2026-11-06/preflash-cli.txt), [current-version full dump](../.local-artifacts/tricopter/m1-bench-2026-11-06/preflash-dump.txt), and [diff](../.local-artifacts/tricopter/m1-bench-2026-11-06/preflash-diff.txt).
- [Cloud full-flash backup](../.local-artifacts/tricopter/m1-bench-2026-11-06/cloud-full-flash.bin), SHA-256 `bd157b4d4f67fc1362ca502854f3e3960b8264ea28a6a4793ea24140e6b4bedd`.
- [Original configuration-sector backup](../.local-artifacts/tricopter/m1-bench-2026-11-06/cloud-config-sector.bin).

Programming used the two actual firmware ranges from the HEX: **392 bytes at `0x08000000`**, and **534817 bytes at `0x08008000`**. It did not program the configuration gap as a contiguous raw binary. The [readback verification](../.local-artifacts/tricopter/m1-bench-2026-11-06/flash-verification.json) confirms both firmware segments matched exactly and the configuration sector was byte-identical to its backup **before first boot**. The [segment manifest](../.local-artifacts/tricopter/m1-bench-2026-11-06/flash-segments.json) contains per-segment hashes; [DFU logs and complete readback](../.local-artifacts/tricopter/m1-bench-2026-11-06/m1-readback-full.bin) are retained.

Despite that preservation, the first local-firmware boot reset configuration to defaults. This was detected before actuator testing. The reason is the firmware's configuration-loader policy, combined with a larger local feature set:

| Configuration evidence | Result |
|---|---:|
| Original EEPROM version / magic | 178 / `0xBE`, valid |
| Original CRC residue | `0x1D0F`, valid |
| PG records in cloud configuration | 77 |
| PGs linked into local M1 build | 91 |
| Local PGs absent from cloud configuration | 14 |
| Shared PG version mismatches | 0 |
| Added PG record bytes | 486 |
| Shared same-version payload growth | 16 bytes |
| Total size growth | 502 bytes, exactly matching CLI 4113 → 4615 |

`loadEEPROM()` defaults a missing local PG but also returns failure; `fc/init.c` then resets **all** configuration on that failure. The 14 absent groups include optional GPS/rescue/hold, LED, optical flow/rangefinder, transponder, dashboard and PWM/PPM facilities. No M1 PG-layout change or damaged config sector is needed to explain the reset. See [the parsed binary/source analysis](../.local-artifacts/tricopter/m1-bench-2026-11-06/config-load-analysis.json).

The session restored the **fresh 2026.6.2 dump from this same aircraft**, applying **1269 commands with zero reported command errors**, saved it, rebooted, and compared the result. [The comparison](../.local-artifacts/tricopter/m1-bench-2026-11-06/restored-setting-comparison.json) found **zero differences among 1210 shared settings**. [Restore transcript](../.local-artifacts/tricopter/m1-bench-2026-11-06/restore-cloud-cli.txt), [error list](../.local-artifacts/tricopter/m1-bench-2026-11-06/restore-errors.json), [save/reboot output](../.local-artifacts/tricopter/m1-bench-2026-11-06/restored-save.txt), and [post-reboot verification](../.local-artifacts/tricopter/m1-bench-2026-11-06/restored-verify.txt) are preserved. The old 4.3.1 dump was not used for this recovery.

This establishes a migration requirement: a same-version firmware build with additional PGs can still reset configuration. Preserve a fresh textual dump and binary backup before such changes, and verify configuration after the first boot rather than relying on intact flash bytes alone. A subsequent same-feature build has all current PG records available after the restore/save.

## PB0 acquisition configuration

The user changed the physical fourth-wire connection from PC0 to **PB0**, for proximity to the servo. PB0 supports ADC1 channel 8. The existing M1 code reads the configured `ADC_EXTERNAL1` resource; changing the pin required a runtime resource change, not a new firmware binary.

The [configuration transcript](../.local-artifacts/tricopter/m1-bench-2026-11-06/pb0-configure.txt) records:

```text
feature -RSSI_ADC
resource ADC_RSSI 1 NONE
resource ADC_EXT 1 B00
set adc_external1_enabled = ON
set debug_mode = TRI_SERVO
set blackbox_disable_debug = OFF
set blackbox_disable_rpm = OFF
set blackbox_disable_servos = OFF
set blackbox_disable_motors = OFF
set blackbox_disable_bat = OFF
save
```

RSSI ADC was already disabled; its default PB0 resource was explicitly freed to avoid a duplicate owner. The external resource changed from PC0 to PB0. Motor/servo resources, receiver settings and the arming procedure were not changed to make these tests work. MSP's arming-disable guard and disarmed motor-test commands were used for bench operation.

## Servo feedback and Blackbox recording

The servo test temporarily used servo 5's RAM configuration with mixer rate zero and a commanded middle, allowing deterministic disarmed positioning. It retained the established 1100–1700 µs limits. The tested sequence was 1420 → 1100 → 1260 → 1420 → 1560 → 1700 → 1420 → 1100 → 1700 → 1420 µs. The complete original servo record `(1100, 1700, 1420, 100, 255, 0)` was restored exactly afterward. This temporary bench mechanism was not saved as a new normal control mode.

The [servo summary](../.local-artifacts/tricopter/m1-bench-2026-11-06/servo-bench-summary.json) records 920 live MSP samples, the exact restoration, and all three final motor commands at 1000. The independent Blackbox timebase provides the higher-rate evidence:

| Blackbox check | Result |
|---|---:|
| Main frames / duration | 17232 / 17.192089 s |
| Mean recording rate | 1002.26 Hz |
| Frame spacing, median / min / max | 998 / 992 / 1003 µs |
| Logged PID-iteration stride | 4 |
| Native `servo[5]` versus final-write `debug[1]` mismatches | 0 |
| ADC scan-progress age | 0 in all recorded samples |
| Status flags | 255 throughout |
| Reported corrupt frames | 0; clean log-end event |

Two complete serial reads of the **747520-byte** servo recording were byte-identical, SHA-256 `daa1b8c539892b7fecf0ff2242e48dce23f5ec41ab7acb41f91c4bb8c7626048`. See [download verification](../.local-artifacts/tricopter/m1-bench-2026-11-06/blackbox-download.json), [first recording](../.local-artifacts/tricopter/m1-bench-2026-11-06/bench-servo.bbl), and [repeat read](../.local-artifacts/tricopter/m1-bench-2026-11-06/bench-servo-repeat.bbl).

The table pools the last 0.5 s of each plateau at the same PWM. Its standard deviation includes repeatability between repeated plateaus, not only sample noise:

| PWM request | Mean ADC count | Pooled standard deviation | Observed steady range | Inferred volts at 3.29 V Vref |
|---|---:|---:|---:|---:|
| 1100 µs | 1942.37 | 3.98 | 1931–1955 | 1.5605 |
| 1260 µs | 1769.29 | 3.27 | 1760–1781 | 1.4215 |
| 1420 µs | 1588.09 | 4.27 | 1576–1637 | 1.2759 |
| 1560 µs | 1443.59 | 2.93 | 1434–1453 | 1.1598 |
| 1700 µs | 1280.47 | 3.77 | 1268–1293 | 1.0288 |

Counts decrease as PWM increases, consistent with the original wiper measurements. The voltage column is calculated using the reported Vref, **not an independent voltmeter measurement**. Individual plateau standard deviations were approximately 2.52–3.93 counts. These data establish useful direct-PB0 observability, but do not establish calibrated rotor angle or prove that connecting the ADC has no effect on the servo's internal feedback loop.

[Servo analysis JSON](../.local-artifacts/tricopter/m1-bench-2026-11-06/servo-log-analysis.json), [steady summary CSV](../.local-artifacts/tricopter/m1-bench-2026-11-06/servo-steady-summary.csv), [transition CSV](../.local-artifacts/tricopter/m1-bench-2026-11-06/servo-transition-summary.csv), and [plot](../.local-artifacts/tricopter/m1-bench-2026-11-06/servo-feedback-bench.png) are retained. Descriptive 10–90% wiper response intervals ranged from about **43.9 to 123.7 ms** across these different step sizes and directions. The analysis includes causal five-sample-median smoothing and five-consecutive-sample thresholding. Those intervals include filtering delay and run from the logged PWM register-write request to wiper response; they are not an identified transport delay, physical pin-edge timing, rotor-angle response, or loaded servo model.

### Decoder and stopped-motor caveats

The upstream Blackbox decoder initially interpreted the modern integer `P interval:4` as though only the old fractional syntax existed, reporting intentionally unlogged PID iterations as missing. A **tool-only parser adaptation** treats integer 4 as 1/4. Decoded main fields were numerically unchanged; only `loopIteration` reconstruction changed, yielding the verified stride of four. [Tool provenance](../.local-artifacts/tricopter/m1-bench-2026-11-06/tools/blackbox-decoder-provenance.json) pins upstream commit `d89f96b983d0f5c8f053acf003ee2998b02b5b0f` and the adaptation patch/hash. No firmware was altered for this decoder issue.

The servo-only log's decoded native motor fields all show **158**, the header's `motorOutputLow`. That representation cannot establish the below-idle stopped output by itself. The MSP commands reported 1000 for all three motors and RPM was zero; those observations establish the recorded stopped state. Do not interpret native value 158 as a running motor.

## Propeller-free motor telemetry test

The user confirmed that propellers were removed and supplied a **7 V, 2 A current-limited** bench source. The test used one logical motor at a time at external commands **1050, 1075 and 1100**, corresponding to 5%, 7.5% and 10% of the external command range. Each step lasted about 1.5 s, with stopped intervals between motors. No simultaneous three-motor run or high-power sweep was performed. Measured FC battery voltage stayed at or above **6.84 V**; this is not an independent supply-current measurement.

The first attempt stopped on an overly strict script assertion expecting an invariant 1420 µs write; a normal legacy write of **1421 µs** tripped it. The script's `finally` path stopped every motor and retained the original servo record. The revised guard required the servo command to remain inside the existing limits. Separately, stale RPM observations before the successful test required a manual EDT-enable handshake. The [handshake record](../.local-artifacts/tricopter/m1-bench-2026-11-06/edt-enable.json) preserves inline command 13 without saving ESC settings. Do not treat the handshake alone as a reception test; the following measured run supplies the motor/RPM evidence. Persistent `dshot_edt` setup and reboot verification are recorded in the final-state section when complete.

The successful [motor test](../.local-artifacts/tricopter/m1-bench-2026-11-06/motor-bench-summary.json) captured **247 sequential MSP samples**. Each median below includes its step's startup samples and is descriptive, not a fitted command-to-RPM model:

| Logical motor / physical position | Median RPM at 1050 | Median RPM at 1075 | Median RPM at 1100 | Peak mechanical RPM |
|---|---:|---:|---:|---:|
| 0 / rear | 714 | 1286 | 1829 | 1871 |
| 1 / front-right | 657 | 1243 | 1843 | 1857 |
| 2 / front-left | 629 | 1229 | 1786 | 1829 |

Every active-step sample reported zero RPM for the two other motors. New-RPM observation ages briefly reached **80.2–80.7 ms** around the 5% startup steps. During the 7.5% and 10% steps the largest sampled active-motor age was **0.4–0.6 ms**. Invalid-packet percentages were zero in these samples, but the existing statistic omits missing-edge/aborted receptions; use the ages alongside it. The MSP replies were sequential queries and do not provide synchronized physical acquisition timestamps.

At test end, all three MSP motor commands were **1000**, all three measured RPM values were **zero**, the complete servo configuration was unchanged, and the final servo write was **1420 µs**. See [events](../.local-artifacts/tricopter/m1-bench-2026-11-06/motor-events.json), [sample CSV](../.local-artifacts/tricopter/m1-bench-2026-11-06/motor-mspsamples.csv), [execution log](../.local-artifacts/tricopter/m1-bench-2026-11-06/motor-bench.log), and [first-attempt trace](../.local-artifacts/tricopter/m1-bench-2026-11-06/attempt1-motor-bench.log).

## Remaining acceptance and final session record

The following limits remain after the recorded servo and low-command motor checks:

- No independent ADC loading comparison, oscilloscope noise/power-sequencing test, or reliable floating-wire disconnection detection.
- No physical rotor-angle zero/travel calibration or aerodynamic-load characterization.
- No scope capture of actual servo pulse edges or guaranteed command-to-pulse timing.
- No thrust, drag torque, rotor-inertia, motor-acceleration or 6S operating-envelope calibration.
- No flight-control correction or flight test.

The combined first three recordings contain **41737 frames**, with clean endings, no reported corruption, correct iteration stride four, and no final-write/native-servo mismatch. Two full **1816576-byte** reads matched SHA-256 `2749e0ca1a2affda8b2c354bd94bb784d948a6af031f25c2e2e6415087a8be6d`. The successful motor recording shows nonzero RPM on at most one channel, with native Blackbox peaks **1885.7 / 1871.4 / 1828.6 mechanical RPM**. Higher-rate logging exposes startup RPM-age peaks of **121.3–121.6 ms**, beyond the slower MSP sample peaks. The aborted attempt exposes **1.3356 s** stale reception while cached RPM remains zero and the invalid-packet statistic stays zero. See [combined analysis](../.local-artifacts/tricopter/m1-bench-2026-11-06/combined-log-analysis.json).

Matched short idle observations used the same disarmed/stopped-motor state, inactive NORMAL Blackbox mode, and 10 Hz MSP monitoring. Diagnostics-off CPU readings were **43%** and LOG_ONLY readings **45%** at whole-percent resolution. Median PID periods were **249 / 250 µs**, with observed ranges **249–252 / 246–251 µs**. These sequential eight-second observations are not a worst-case execution-time or full jitter qualification. See [legacy](../.local-artifacts/tricopter/m1-bench-2026-11-06/idle-performance-legacy.json) and [LOG_ONLY](../.local-artifacts/tricopter/m1-bench-2026-11-06/idle-performance-logonly.json) records.

The saved configuration retains the aircraft's original receiver, arming-switch, motor-resource and servo settings. [The final comparison](../.local-artifacts/tricopter/m1-bench-2026-11-06/final-setting-comparison.json) checks 1210 original settings and finds only four intentional differences: `ADC_EXT C00→B00`, `ADC_RSSI B00→NONE`, `debug_mode NONE→TRI_SERVO`, and `dshot_edt OFF→ON`. The external ADC remains enabled. Blackbox mode is NORMAL; temporary rate-zero servo positioning was never saved.

After all clamped-case tests and requested inactivity-beacon resets, the FC was rebooted normally. [Final handoff](../.local-artifacts/tricopter/m1-bench-2026-11-06/final-handoff.json) confirms all three motor commands 1000, all RPM zero, the exact original servo record `(1100,1700,1420,100,255,0)`, final servo write 1420 µs, PB0 reading 1591 counts and status 255. Temporary MSP arming overrides were not reapplied after that reboot. No receiver or persistent arming-guard changes were needed.

Onboard Blackbox retains all bench recordings: 13137920 of 16777216 bytes used, leaving approximately **3.47 MiB free**. Nothing was erased. The servo raw recordings and analysis are also preserved in the repository [validation package](validation/tricopter-servo-2026-11-06/README.md); the exact original cloud firmware/configuration backup remains local under the evidence folder.
