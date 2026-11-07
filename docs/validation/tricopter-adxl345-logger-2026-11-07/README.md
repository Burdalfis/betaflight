# External ADXL345 logger build validation — 2026-11-07

See the [standalone PlatformIO project and wiring guide](../../../tools/tricopter/adxl345_logger/README.md). It reads the ADXL345 and observes an optional servo PWM input; it has no actuator-output, FC-configuration or PB0−PC0 measurement changes.

All three environments compile with PlatformIO Core 6.2.0, Teensy platform 6.0.0, Teensyduino framework 1.162.0 and Arm toolchain 1.150201.0 (15.2.1). Relevant platform packages are pinned in `platformio.ini`; dependencies were installed into the repository's ignored `.local-artifacts/tricopter/adxl345-logger-2026-11-07/platformio` cache, leaving the user's existing PlatformIO platforms untouched. The LC build uses **3,020 / 8,192 bytes static RAM** and **10,308 / 63,488 bytes flash**. This is the linker's static allocation, not a measured stack high-water mark.

`build-final.log` records successful `teensylc`, `teensy36` and `teensy40` builds. `build-validation.json` records the exact command, current project source hashes, ELF/HEX hashes and resolved dependency manifests. The binaries remain in the ignored project `.pio/build/<environment>/` directories; none was uploaded.

All **17 host tests** pass, recorded in `tests.log`. They cover actual shared-C++-encoder/Python-decoder interoperability, the independent CRC known vector, fragmented/corrupted/unknown-version streams, truncated tails, signed/64-bit fields, sequence rollover/reset/session boundaries, stale PWM/unqualified samples, event-counter interpretation, original-byte/file-hash preservation, read-failure artifacts and overwrite protection. The actual C++ helpers also pass host undefined-behavior instrumentation for timestamp rollover, late-read rejection and causal PWM freshness boundaries.

Independent read-only review checked the SPI pin map, LC interrupt-capable pins 2/3, ADXL345 configuration and neutral offset-register readback, read-completion timing qualification, watchdog/retry, PWM snapshots, queue/session handling and host capture. The review caught and corrected qualification that originally checked only read start, and freshness that could admit a PWM pulse completed after the sample timestamp. The final build includes both fixes.

**Validation boundary:** compile and host tests only. No Teensy or sensor was flashed/tested, no physical wire was verified and no FC was accessed. USB writes are buffered best effort, with explicit timing/loss metadata; sensor/filter/interrupt latency, throughput under host stalls and electrical accuracy need a bench check. Generic breakout VIN/VCC circuitry is unidentified, so the wiring guide conditions power on a verified sensor 3.3 V rail. Static gravity inclination can calibrate settled basket angle; it does not establish accurate dynamic angle during acceleration.

To reproduce with the downloaded cache, from the repository root:

```sh
PLATFORMIO_CORE_DIR="$PWD/.local-artifacts/tricopter/adxl345-logger-2026-11-07/platformio" \
    /home/alex/.platformio/penv/bin/platformio run \
    -d tools/tricopter/adxl345_logger -e teensylc -e teensy36 -e teensy40
python3 -m unittest discover -s tools/tricopter/adxl345_logger -p 'test_*.py' -v
sha256sum --check docs/validation/tricopter-adxl345-logger-2026-11-07/checksums.sha256
```

Use that same cache prefix with `-e teensylc -t upload` when the intended LC is connected. Upload has deliberately not been run as part of these checks.
