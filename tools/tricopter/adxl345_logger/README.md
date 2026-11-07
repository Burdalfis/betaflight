# ADXL345 basket-angle bench logger

Standalone PlatformIO firmware for a Teensy LC, with optional Teensy 3.6 and 4.0 build environments. It records raw acceleration and observes the FC's existing servo PWM. It generates no servo or motor commands, and makes no Betaflight, ADC or aircraft-configuration changes. PB0−PC0 remains the FC's sole servo position measurement.

The default is four-wire SPI at 1 MHz, SPI mode 3, 800 Hz output rate, normal power, full-resolution ±2 g, FIFO bypass and active-high DATA_READY on INT1. User offset registers are explicitly cleared; this does not calibrate the sensor's physical bias. Device ID and configuration readback are checked; faults and recovery are logged. No third-party sensor library is required. The onboard pin-13 LED is also the SPI clock and is not used for status.

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
| 0 / 4 / 5 | Magic / version / type (1 sample, 2 status) | 4 bytes / u8 / u8 |
| 6 / 8 / 12 | Flags / attempted record sequence / MCU time µs | u16 / u32 / u64 |
| 20 / 22 / 24 / 26 | X / Y / Z / auxiliary | i16 / i16 / i16 / u16 |
| 28 / 32 / 36 | Read latency µs / PWM width µs / PWM sequence | u32 each |
| 40 | Rise timestamp of last completed PWM pulse µs | u64 |
| 48 / 52 / 56 | Sensor overrun / service gap / queue-drop counters | u32 each |
| 60 | CRC32 | u32 |

On samples, XYZ are raw counts and auxiliary is `INT_SOURCE`. On status frames, XYZ hold `DEVID`, `BW_RATE`, `DATA_FORMAT` and auxiliary is fault code: 0 OK, 1 bad device ID, 2 configuration mismatch, 3 sample timeout, 4 DRDY polling fallback. Flags 0–6 indicate configured sensor, qualified DRDY timestamp, fresh PWM, current-read overrun, current-read service gap, polling fallback and USB-session start, respectively. Counters are cumulative within a boot; a sensor overrun event is not an exact count of lost samples. Record sequence counts attempted records during an open USB session, including records dropped by the firmware queue.

## Measurement boundary

Raw X/Y/Z counts are retained without software zeroing, filtering or angle conversion in the firmware. Full-resolution nominal sensitivity is approximately 256 counts/g; offsets and scale differences still require calibration. A data-ready timestamp is a **software interrupt-service timestamp**, not a hardware pin-edge capture or the sensor's internal sampling instant. USB arrival time is not measurement time. Record order can include a status/sample timestamp backstep when the sample's DRDY edge preceded construction of a status frame; the decoder reports that separately from sequence/session discontinuities.

FIFO bypass gives a direct DRDY/sample association when serviced promptly. Qualification requires the XYZ read to finish within 625 µs of the observed edge, with no detected overrun or new edge during the read. An overrun means unread data was replaced at least once; it cannot reveal an exact lost-sample count. A delayed read, polling fallback or detected timing ambiguity is marked unqualified, with SPI read-start time in place of a qualified DRDY timestamp. Late asynchronous reads can also have uncertain XYZ coherency, so exclude unqualified rows from precise timing and calibration statistics. Sensor overrun events, service-gap events and firmware queue drops are separate counters.

USB output uses a bounded firmware queue and checked, best-effort writes through the Teensy core. It is not a lossless or hard real-time transport guarantee. Core/host stalls must remain visible through timing qualification, event counters and sequence gaps; physical timing still needs a bench check.

PWM observations are completed high pulses in the accepted 500–2500 µs interval, with a fresh flag only when the pulse completed by the row's timestamp and its rise is no older than 100 ms. Raw PWM fields may describe a pulse completed between the sensor edge and frame construction; its fresh flag is false for that earlier sample. Width/age qualification is not a complete electrical integrity test. With PWM connected, both signals share one MCU clock. A distinctive PWM staircase can align the Teensy recording with the FC's final-PWM/PB0−PC0 log; the two MCU clocks still need offset/drift alignment.

The ADXL345 is suitable for **settled angle calibration**: approach a sequence of PWM values from both directions, allow motion to settle and average gravity vectors. Fast motion adds tangential/centripetal acceleration and vibration. Filtering introduces additional delay, so accelerometer-derived tilt is not independent ground truth for millisecond servo movement. [ADI inclination guidance](https://www.analog.com/en/resources/app-notes/2020/07/22/16/49/an-1057.html)

The initial test should record settled basket angle, commanded/captured PWM and the FC's PB0−PC0 over both approaches. This supplies the missing physical count-to-angle map and exposes linkage hysteresis. It does not identify loaded thrust, motor gyroscopic torque or in-flight angle accuracy.
