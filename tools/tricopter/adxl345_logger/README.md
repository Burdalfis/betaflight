# ADXL345 basket-angle bench logger

Standalone PlatformIO firmware for a Teensy LC, with optional Teensy 3.6 and 4.0 build environments. It records raw acceleration and observes the FC's existing servo PWM. It generates no servo or motor commands, and makes no Betaflight, ADC or aircraft-configuration changes. PB0−PC0 remains the FC's sole servo position measurement.

The default is four-wire SPI at 1 MHz, SPI mode 3, 800 Hz output rate, normal power, full-resolution ±2 g, FIFO bypass and active-high DATA_READY on INT1. User offset registers are explicitly cleared; this does not calibrate the sensor's physical bias. Device ID and configuration readback are checked; faults and recovery are logged. No third-party sensor library is required. The onboard pin-13 LED is also the SPI clock and is not used for status.

Startup verifies all ten configuration registers. Periodic verification reads one register per foreground iteration and yields to pending samples, so the audit does not stall acquisition as one long SPI burst. Nominal configured output rate and measured sample cadence remain distinct; use the recorded timestamps when analyzing data. See the [first hardware check and before/after evidence](../../../docs/validation/tricopter-adxl345-bench-2026-11-07/README.md).

## Wiring

These are **Teensy digital pin numbers**, not connector positions. The SPI pins are the [official PJRC defaults](https://www.pjrc.com/teensy/td_libs_SPI.html), shared by the three supplied environments.

| ADXL345 breakout label | Teensy LC | Function |
|---|---|---|
| `3V3` / sensor 3.3 V supply rail | `3.3V` | Sensor power; see power note below |
| `GND` | `GND` | Common ground |
| `SCL` / `SCLK` / `CLK` | **13** | SPI clock |
| `SDA` / `SDI` / `MOSI` | **11** | Data from Teensy to sensor |
| `SDO` / `MISO` | **12** | Data from sensor to Teensy |
| `CS` / `nCS` | **10** | Chip select, controlled by firmware |
| `INT1` | **2** | Data-ready interrupt |
| `INT2` | Unconnected | Unused |

`SDA` and `SCL` on these modules are also the SPI data-input and clock pins. **CS must be driven by pin 10, and SDO must connect to MISO**; remove any I²C-mode/address straps. Disconnect the previous printer controller and its power/data connections.

Generic modules differ in their regulator and level-shifter circuitry. The table's power connection is to the **sensor's 3.3 V rail**. A pin marked only `VIN` or `VCC` does not identify whether it is before or after a regulator. If that is the only power pin, identify the regulator/board power path first: feeding 3.3 V through a regulator can undervolt the sensor, while blindly using 5 V can expose unshifted logic. The bare ADXL345 uses 2.0–3.6 V supply and its I/O supply must not exceed its sensor supply. [ADI datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/adxl345.pdf)

For optional PWM observation, connect the **FC servo signal** to Teensy **pin 3** and FC ground to Teensy GND. Pin 3 is input-only in this firmware; the FC continues driving the servo. Confirm that the signal is 3.3 V or translate it to 3.3 V first: [LC inputs are not 5 V tolerant](https://www.pjrc.com/store/teensylc.html). Do not connect the servo's 5 V power wire to an input or the Teensy's 3.3 V pin. Power the Teensy from USB; a shared ground does not require joining its USB/VIN supply to the aircraft supply. PWM capture is optional, so the acceleration logger works with pin 3 unconnected.

Mount only the small sensor board rigidly on the moving basket, preferably near the pivot. Keep the Teensy stationary and use short, flexible sensor leads that do not constrain travel. Hold the airframe fixed during calibration. The tilt axis must change the gravity projection; rotation about a vertical axis cannot be measured this way.

## Build and upload

From the repository root, using the installed PlatformIO executable:

```sh
/home/alex/.platformio/penv/bin/platformio run -d tools/tricopter/adxl345_logger -e teensylc
/home/alex/.platformio/penv/bin/platformio run -d tools/tricopter/adxl345_logger -e teensylc -t upload
```

Use `-e teensy36` or `-e teensy40` for those boards; the default remains LC. The project pins the Teensy platform version. PlatformIO downloads its toolchain/framework on the first build. Upload only with the intended Teensy connected; pressing its Program button may be needed for the loader. The firmware uses native USB serial, not a UART adapter. Do not use Serial Monitor for the binary data stream.

### Static calibration without an interrupt

`-e teensylc_static_fifo` selects a separate stationary-calibration profile. It disables sensor interrupt outputs and polls the stream FIFO over SPI. The existing INT2-to-pin-2 wire can remain connected. This profile reads only nonempty entries as complete XYZ bursts, waits at least 5 µs after each pop, and checks FIFO depth and overrun state. Depth 31 or greater, overflow or a stalled read discards the uncertain segment and resets the FIFO; counters/status preserve the event. The one-slot reserve and bounded read window guard against an overrun cleared by the pop itself; depth 31 alone is a service-gap event, not proven sample loss.

Clean samples carry `FifoCoherent` (flag 7), with FIFO depth before the pop in the auxiliary high byte and `INT_SOURCE` in the low byte. They never carry qualified DRDY timestamps. Their `time_us` is SPI read/pop start, with unmeasured sensor-sample age. Use them for steady gravity-vector means and static offset/scale calibration; they do not support millisecond motion timing. The decoder exposes `fifo_coherent`, `fifo_depth_before_read` and `sample_time_source` separately from `drdy_timestamp_qualified`.

The default interrupt profile remains available. `teensylc_int2_probe` routes DATA_READY to INT2; `teensylc_int2_pullup_probe` additionally enables the MCU's weak 3.3 V input pull-up for diagnosis. These profiles do not establish that the connected interrupt signal is healthy.

## Record and decode

The capture script uses PySerial for USB; offline decoding uses only the Python standard library. Install `requirements.txt` into your chosen Python environment if PySerial is absent. List serial ports, identify the **Teensy** rather than the flight controller, then record:

```sh
python3 -m serial.tools.list_ports -v
python3 tools/tricopter/adxl345_logger/capture.py \
    --port /dev/ttyACM1 --output .local-artifacts/tricopter/angle-test-01 --seconds 60
```

Replace the example device with the Teensy's actual port (or stable `/dev/serial/by-id/` path). Omit `--seconds` to record until Ctrl+C. The script does not upload, reset the board or send actuator commands. The USB stream starts when the port is opened. Output directories must not already contain the capture's artifact filenames.

- `capture.bin`: original received bytes, including malformed or incomplete data.
- `records.csv`: every CRC-valid sample and status record, with timestamp/PWM validity and quality flags.
- `summary.json`: source/artifact hashes, stream epochs, sequence gaps and quality counters. Startup with a sensor fault is visible; a capture with no samples returns an error.

Decode the preserved binary again into a new directory with:

```sh
python3 tools/tricopter/adxl345_logger/capture.py \
    --input .local-artifacts/tricopter/angle-test-01/capture.bin \
    --output .local-artifacts/tricopter/angle-test-01-decoded
python3 -m unittest discover -s tools/tricopter/adxl345_logger -p 'test_*.py' -v
```

Frames are fixed 64-byte little-endian records with `ADXL` magic, version/type checks and IEEE CRC32 over bytes 0–59. The parser resynchronizes after corruption and reports an incomplete tail. CRC-valid status frames are retained rather than being interpreted as acceleration.

| Byte offset | Field | Format |
|---:|---|---|
| 0 / 4 / 5 | Magic / version / type (1 sample, 2 status, 3 diagnostic) | 4 bytes / u8 / u8 |
| 6 / 8 / 12 | Flags / attempted record sequence / MCU time µs | u16 / u32 / u64 |
| 20 / 22 / 24 / 26 | X / Y / Z / auxiliary | i16 / i16 / i16 / u16 |
| 28 / 32 / 36 | Read latency µs / PWM width µs / PWM sequence | u32 each |
| 40 | Rise timestamp of last completed PWM pulse µs | u64 |
| 48 / 52 / 56 | Sensor overrun / service gap / queue-drop counters | u32 each |
| 60 | CRC32 | u32 |

On samples, XYZ are raw counts and auxiliary is `INT_SOURCE`. On status frames, XYZ hold `DEVID`, `BW_RATE`, `DATA_FORMAT` and auxiliary is fault code: 0 OK, 1 bad device ID, 2 configuration mismatch, 3 sample timeout, 4 DRDY polling fallback, 5 FIFO segment discarded. Flags 0–6 indicate configured sensor, qualified DRDY timestamp, fresh PWM, current-read overrun, current-read service gap, polling fallback and USB-session start, respectively. Counters are cumulative within a boot; a sensor overrun event is not an exact count of lost samples. Record sequence counts attempted records during an open USB session, including records dropped by the firmware queue.

Type 3 records add read-only interrupt diagnostics after each status record. They use the same version-1 envelope with an explicit new type; older decoders skip that unsupported type. The current decoder gives them `type=diagnostic` and separate `diagnostic_*` columns, leaving acceleration, PWM and status aliases blank. They are not calibration samples. Their wire-slot meanings are:

| Wire slot | Diagnostic meaning |
|---|---|
| X | State bits: 0 current pin-2 level; 1/2 pin levels before/after the last XYZ read; 3 pending DRDY; 4/5 LC NVIC enabled/pending; 6 PRIMASK before snapshot guard; 7 LC raw-state available; 8 post-read source available |
| Y / Z | Cached `INT_ENABLE` / `INT_MAP` readback |
| Auxiliary low / high byte | Cached last pre-/post-XYZ `INT_SOURCE`; high byte only valid with diagnostic bit 8 |
| Read-latency slot | Raw DRDY ISR sequence counter, not a latency |
| PWM-width / PWM-sequence slots | LC pin-2 PCR / whole `PORTD_ISFR`, only valid with diagnostic bit 7 |
| PWM-rise-time slot | Zero-extended raw last ISR `micros()` low 32 bits; no known event when ISR sequence is zero |

Pin/source read observations and interrupt-register readbacks are **cached**, with no standalone cache-age field; boot/fault zeros do not establish a known fresh sensor state. Correlate them with preceding samples and healthy configuration status. Post-read `INT_SOURCE` is inspected only during already-unqualified polling fallback, after the XYZ completion-time snapshot. Current GPIO/PCR/NVIC values are read at diagnostic construction. NVIC pending is shared PORTC/D state, not proof of a pin-2 event; LC pin 2 is PTD0, so its raw PORTD flag mask is bit 0. Diagnostic records do not drive GPIOs, automatically rearm interrupts, or change calibration.

## Offline bias and sensitivity calibration

`calibrate.py` fits a constant bias and positive counts/g for each axis from stationary pose means. It uses the complete 3D gravity magnitude, so approximately aligned poses need not be treated as exact axis vectors. NumPy is required for this analysis; it is included in `requirements.txt`. Firmware and raw recordings remain uncorrected.

Save a copy of the session manifest beside its relative pose folders, then write a new profile:

```sh
python3 tools/tricopter/adxl345_logger/calibrate.py fit SESSION_SNAPSHOT.json --output PROFILE.json
python3 tools/tricopter/adxl345_logger/calibrate.py validate PROFILE.json CAPTURE_FOLDER \
    --output VALIDATION.json --stationarity-policy static-mean-v2
```

The correction is `acceleration_g = (raw_counts - bias_counts) / counts_per_g`. The tool checks artifact hashes, replays the binary against the CSV, verifies configuration and sample coherence, rejects duplicate training/validation captures, and checks fit rank, conditioning and positive scales. Six pose means constrain six parameters, so a near-zero training residual is expected and does not validate the model. Reference repeats and independently recorded tilted poses remain outside the fit. Output files are not silently overwritten.

The default validation policy retains the original per-axis standard deviation below 5 counts and full one-second axis-mean span below 3 counts. The explicit `static-mean-v2` policy targets averaged stationary measurements: it preserves the original noise-gate result, retains all integrity/configuration/coherence/counter checks and the 3-count block-mean-span check, and requires both the overall corrected mean and every complete one-second corrected mean to have gravity magnitude within the declared limit (default ±0.03 g). Raw scatter and block wander are reported. This is a practical session screen, not a sensor specification or an absolute angular-accuracy bound.

The current basket application uses four independent diagonal directions across Y/Z and X/Y, with sign/plane coverage recorded separately from the radial screen. X/Z diagonal checks remain optional and untested under that application gate. The fitted profile stays fixed throughout validation. Do not compensate drift by zeroing each pose or refitting the held-out data.

The 2026-11-07 session targets roughly ±2° for averaged, settled relative basket angles. That target remains to be checked through mounted return-to-position measurements; unknown-angle gravity-norm tests do not prove absolute angle accuracy. Recheck after final mounting because strain, temperature and wiring can change the baseline. Allow handling transients to settle and keep the board fixed during basket measurements.

## Preparing a settled basket recording

`basket_capture.py` defaults to a plan-only preview. `--inspect` identifies both explicit USB devices and reads the FC configuration without actuator writes. Use separate, identified FC and Teensy ports, a saved calibration profile and its independent static-validation aggregate, and a new output directory:

```sh
python3 tools/tricopter/adxl345_logger/basket_capture.py \
    --fc-port FC_PORT --teensy-port TEENSY_PORT --teensy-serial TEENSY_SERIAL \
    --profile PROFILE.json --validation-report VALIDATION_SUMMARY.json \
    --sample-hz 20 --output NEW_SESSION --inspect
```

Replace the placeholders with the actual identified ports and artifacts. Omitting `--inspect` previews the plan without opening serial ports. Execution additionally requires `--execute --fc-uid UID --setup-confirmed`, after current physical setup and installed clearances have been confirmed. The initial plan is **1420→1400→1420→1440→1420 µs**, with one second of settling and two seconds of measurement per plateau. The host sampling rate is configurable; the initial fixture uses 20 Hz to allow for real MSP request/guard latency.

After that center record is reviewed, `--plan-kind full-sweep --range-authorization FILE --authorization-sha256 SHA` permits the explicitly authorized **1420→1100→1700→1420 µs** sequence in 20 µs steps. Its 61 plateaus have 183 seconds of nominal holds plus MSP/guard overhead. The hashed authorization binds the exact plan, FC UID, calibration, successful center evidence and response thresholds. Progress is flushed after each plateau; cumulative 100 µs checks require canonical position and external gravity-vector response. The first reversal step permits hysteresis. The child capture has a per-plateau overhead allowance and startup margin; the parent stops it when the sequence finishes. The separate `sweep-preview` plan remains non-executable.

With a measured hinge and a known sensor-to-basket reference, `--plan-kind roll-zero-check` selects a fixed 19-plateau 1420→1560→1420 µs neighborhood check. It requires the same explicit execution/setup/UID arguments, live ADXL observations and a gross mean-norm screen of 1 ±0.06 g. Steps remain at most 20 µs; all identity, disarmed, stopped-motor, ADC and exact-restoration guards still apply. This mode does not require the full-sweep authorization manifest.

`--plan-kind reversal-check` selects a fixed 75-hold validation sequence with ±5/10/20/40 µs loops near 1512 µs and ±10 µs loops at 1300/1650 µs. `reversal-high-check` is a fixed 29-hold high-angle repeat. Both preserve the same paired-observer, bounded-step, identity and restoration guards and tag the data as held-out, with refitting disabled. The high repeat uses `--settle-seconds 2 --measure-seconds 4`; measurement-quality failures remain recorded separately from transport and restoration success.

Mount only the ADXL on the moving basket near its pivot; keep the Teensy fixed and the wiring flexible. Confirm the linkage is connected, the aircraft secured with props removed, and the sensor/wiring clear of travel. After mounting, record and validate a stationary sensor baseline before any commanded movement. Existing SPI wiring is sufficient for long settled plateaus; optional PWM observation can be added separately.

The capture helper verifies the FC UID/API, TRI mixer, existing TRI_GROUND mode, disarmed state, stopped motors and existing 1100/1700/1420 servo limits. It records the ADXL stream alongside canonical PB0−PC0 FC samples and host-timed plateau IDs, retains isolated stale ADC rows as unavailable, and stops on sustained invalid acquisition. Raw file chunks are flushed so the live health monitor can observe incoming data. The helper temporarily changes only the selected servo's middle/rate fields, checks exact readback, and restores the original raw servo configuration in `finally`, including polite interruption. It implements no arming, motor-test, flash-erase, EEPROM, debug-mode or Blackbox writes.

Host timestamps associate settled windows; they are not exact FC/Teensy clock synchronization. The first data will establish direction and return repeatability before a larger sweep. Fit the actual gravity-rotation plane from basket data rather than assuming sensor axes exactly match the hinge. Treat 1420 µs as an operating reference until a separate mounting reference assigns basket angle zero.

Analyze a completed record with:

```sh
python3 tools/tricopter/adxl345_logger/basket_analyze.py RUN_FOLDER --output ANALYSIS.json
```

The analyzer verifies restoration and capture integrity, then pairs fresh canonical FC measurements with interior ADXL windows and fresh input PWM matching the commanded plateau. It retains exclusions and reports host association limits. A small center arc leaves the hinge plane unidentified; a broad, well-conditioned arc can supply a fitted rotation plane and relative gravity-vector angles. Mechanical sensor/basket rotation has the opposite sign to gravity-vector rotation about the same fixed hinge; aircraft yaw convention and physical vertical zero remain separate. Failed measurement windows remain in the diagnostic output and return a nonzero status.

## Measurement boundary

Raw X/Y/Z counts are retained without software zeroing, filtering or angle conversion in the firmware. Full-resolution nominal sensitivity is approximately 256 counts/g; offsets and scale differences still require calibration. A data-ready timestamp is a **software interrupt-service timestamp**, not a hardware pin-edge capture or the sensor's internal sampling instant. USB arrival time is not measurement time. Record order can include a status/sample timestamp backstep when the sample's DRDY edge preceded construction of a status frame; the decoder reports that separately from sequence/session discontinuities.

FIFO bypass gives a direct DRDY/sample association when serviced promptly. Qualification requires the DATA_READY line to remain asserted before XYZ and the read to finish within 625 µs of the observed edge, with no detected overrun or new edge during the read. An overrun means unread data was replaced at least once; it cannot reveal an exact lost-sample count. A delayed read, polling fallback or detected timing ambiguity is marked unqualified, with SPI read-start time in place of a qualified DRDY timestamp. Late asynchronous bypass reads can also have uncertain XYZ coherency, so exclude those rows from precise timing and calibration statistics. The explicit static FIFO profile supplies coherent vectors independently of DRDY timing; only clean configured FIFO rows are eligible for stationary calibration. Sensor overrun events, service-gap events and firmware queue drops are separate counters.

USB output uses a bounded firmware queue and checked, best-effort writes through the Teensy core. It is not a lossless or hard real-time transport guarantee. Core/host stalls must remain visible through timing qualification, event counters and sequence gaps; physical timing still needs a bench check.

PWM observations are completed high pulses in the accepted 500–2500 µs interval, with a fresh flag only when the pulse completed by the row's timestamp and its rise is no older than 100 ms. Raw PWM fields may describe a pulse completed between the sensor edge and frame construction; its fresh flag is false for that earlier sample. Width/age qualification is not a complete electrical integrity test. With PWM connected, both signals share one MCU clock. A distinctive PWM staircase can align the Teensy recording with the FC's final-PWM/PB0−PC0 log; the two MCU clocks still need offset/drift alignment.

The ADXL345 is suitable for **settled angle calibration**: approach a sequence of PWM values from both directions, allow motion to settle and average gravity vectors. Fast motion adds tangential/centripetal acceleration and vibration. Filtering introduces additional delay, so accelerometer-derived tilt is not independent ground truth for millisecond servo movement. [ADI inclination guidance](https://www.analog.com/en/resources/app-notes/2020/07/22/16/49/an-1057.html)

The initial test should record settled basket angle, commanded/captured PWM and the FC's PB0−PC0 over both approaches. This supplies the missing physical count-to-angle map and exposes linkage hysteresis. It does not identify loaded thrust, motor gyroscopic torque or in-flight angle accuracy.

## First installed-range result

The 2026-11-07 installed recording completed all 61 plateaus over 1100–1700 µs, with 143,938 coherent accelerometer samples and exact original servo-configuration restoration. The broad gravity arc identifies a hinge normal close to sensor Z. Relative mechanical travel is approximately 69.33°; the initial 1420 µs position is only an operating reference.

Approach-dependent same-PWM angle differences reach 3.06°. Matching measured PB0−PC0 instead reduces, but does not eliminate, the difference: the two measured count/angle branches separate by up to 1.71°. Preserve approach state rather than treating a command or one fitted curve as exact measured angle. The exported per-branch interpolation tables are offline artifacts with no extrapolation beyond measured count bounds; their 0.23–0.25° within-run interpolation RMS is not independent absolute-angle accuracy. Physical vertical zero, airframe sign and loaded dynamic response remain to characterize.

The complete session and derived tables are retained in `.local-artifacts/tricopter/adxl345-calibration-2026-11-07/basket-full-sweep-02/`. Earlier attempts with host buffering/startup and capture-watchdog faults remain preserved separately. The final capture watchdog includes per-plateau MSP overhead, and raw chunks are flushed for the live monitor.

## Flat-basket roll reference

The operator clarified that the motor is absent and the ADXL occupies its place, with PCB +X aligned up when the basket is flat and the frame level in roll. This defines a basket roll reference. Project gravity and sensor +X into the measured hinge plane, then take the negative gravity phase as mechanical basket roll. The hinge-parallel component from slight frame pitch is excluded. The apparent hinge inclination is about −0.68°; mounting and sensor drift limit interpreting this as precise airframe pitch.

A subsequent 19-plateau check records 45,166 coherent samples; every settled window passes, all 703 FC rows retain fresh PB0−PC0, and the exact original servo configuration restores with output at 1420 µs and motors stopped. Interpolated zero is **1517.74 µs / 1489.00 PB0−PC0 counts** approaching with increasing PWM and **1505.75 µs / 1492.08 counts** with decreasing PWM. Direct holds at 1519/1504 µs measure +0.169°/−0.175°. The original curves predict the live zero counts within 0.20°; across the new points, transfer RMS is about 0.27° and maximum error below 0.50°. The original curve shapes are preserved with a common zero reference; live checks remain separate.

The zero check is in `basket-roll-zero-check-01/`; `basket-full-sweep-02/measured-angle-tables-roll-zero.json` contains the updated offline tables. The tooling passes **79 hardware-free tests**. Zero follows the operator's PCB alignment; motor-shaft vertical, aircraft yaw sign, independently proven absolute accuracy and loaded response remain unqualified. No angle calibration is installed into flight control.

The subsequent [measured-angle integration and reversal results](../../../docs/tricopter_measured_angle.md) export a bounded 58-knot mean count-to-angle profile and paired command/ADC regressions for offline models. The original reversal run has 74/75 qualified windows, and a separate longer high-angle repeat passes 29/29. No reversal observations enter fitting. The complete logger suite now passes **84 tests**; the FC ends with the exact original configuration restored at 1420 µs and motors stopped.
