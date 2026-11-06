# Tricopter hardware and baseline configuration

Recorded 2026-11-06 for M0/M1 of [the control plan](TRICOPTER_CONTROL_PLAN.md).

The user has confirmed that a **Betaflight 2026.6.2 cloud build** is flashed on
the flight controller and working correctly, supplied its current CLI export,
and confirmed the physical motor/servo mapping below. The earlier `2026.2.6`
wording was corrected to `2026.6.2`. This closes the M0 aircraft-mapping check.
The local build is a separate image whose build evidence must be recorded
separately. No firmware was flashed and no aircraft configuration was changed
during this source/configuration audit.

## Evidence and preservation

- `tricopter_dump.txt` and `tricopter_diff.txt`: original flown Betaflight 4.3.1
  configuration, source revision `8d4f005`, board configuration `edcd244b`.
- `btfl_001.bbl` through `btfl_006.bbl`: original historical recordings. These
  are evidence of the earlier aircraft, not recordings of the current cloud build.
- [Current cloud-build CLI evidence excerpt](tricopter_cloud_2026-11-06.txt):
  labeled transcription of supplied version/status/configuration and active
  resource facts; this is not a full raw transcript or replay configuration.
- [Complete CLI text supplied in chat](tricopter_cloud_cli_2026-11-06.txt):
  preserves the supplied `version`, `status`, `diff all` and `resource show all`
  text with LF line endings. This is a transcription of the chat message, not
  an independently captured serial byte stream or a `dump all` export. The
  user's final sentence, outside the CLI output, was: “I can confirm that the
  motor and servo mapping is correct”.
- [Original-file SHA-256 manifest](tricopter_originals.sha256): repo-relative
  names; verify from the repository root using
  `sha256sum -c docs/tricopter_originals.sha256`.
- A byte-for-byte, read-only local snapshot is held under
  `.local-artifacts/tricopter/m0/originals/`. Its files are intentionally not
  edited by analysis or logging tools.
- Local source baseline: `e0b7bb01b` / tag `2026.6.2`, branch
  `feature/tricopter-feedback` at the start of this work.
- Board configuration audited:
  `src/config/configs/MTKS/MATEKF405TE/config.h`, config repository commit
  `3a406c5c30c20d7c3bc7dd5ed547cca961baf0a9`.

The historical dumps establish software resources and output reordering. The
current cloud-build export agrees with the aircraft motor/servo mapping, and
the user has confirmed that physical mapping. Motor directions, rear ESC-output
identity, and fitted MCU are recorded in the project plan. Board photographs or
a wiring drawing have not yet been supplied.

### Current working cloud build: supplied evidence

The supplied current CLI identifies **2026.6.2 `e0b7bb01b`**, board configuration
**`3a406c5`**, board **MATEKF405TE**, manufacturer **MTKS**. It confirms TRI,
`motor_output_reordering = 2,0,1,3,4,5,6,7`, MOTOR3 PA8, MOTOR4 NONE, and
SERVO1 PB1. Servo 5 retains 1100–1700 µs, midpoint 1420 µs, and 498 Hz output;
motor protocol is DSHOT300 with bidirectional telemetry enabled.

Active ownership in the supplied export is:

| Active pin or peripheral | Owner |
|---|---|
| PA8 | MOTOR 1 (logical motor 0) |
| PC9 | MOTOR 2 (logical motor 1) |
| PC8 | MOTOR 3 (logical motor 2) |
| PB1 | SERVO 1 |
| PC0 | ADC_EXT |
| TIM3 CH4 | SERVO 1 |
| TIM8 CH1 / CH2 | DSHOT_BITBANG 1 / 3 |
| DMA2 Stream2 / Stream3 | DSHOT bitbang |
| DMA2 Stream4 | ADC1 |

Configured resource ordinals identify pin slots; active bitbang motor ownership
is reported in logical motor order. Thus active `PA8 MOTOR 1` is consistent
with configured `resource MOTOR 3 A08` and reordering value 2. The supplied
active resources show **no motor/servo/PC0/DMA conflict**. PB1 is actively owned
by SERVO1. LED_STRIP did not appear in the supplied diff; its configured resource
should be compared separately rather than assumed NONE solely from that diff.

## Aircraft and logical motor mapping

The controller is a Matek F405-miniTE, using the `MATEKF405TE` target. The plan
records the fitted MCU as STM32F415RGT6; the board target selects STM32F405.
Successful compilation alone does not verify every peripheral on the fitted
device.

Current STM32 DShot and PWM drivers select the output pin as
`ioTags[motorOutputReordering[logicalMotorIndex]]`. Reordering values and model
indices are zero-based; CLI resource ordinals are one-based.

Historical `motor_output_reordering = 2,0,1,3,4,5,6,7` therefore means:

| Logical/model motor | Reordering value | CLI resource | FC pin | Aircraft position | Rotation viewed from above |
|---|---:|---|---|---|---|
| 0 | 2 | MOTOR 3 | PA8 | Rear; physically ESC output 4 | CCW |
| 1 | 0 | MOTOR 1 | PC9 | Front right | CW |
| 2 | 1 | MOTOR 2 | PC8 | Front left | CCW |

The plan states that ESC output 3 is unused; it does not establish the numbered
physical ESC outputs for the two front motors. Those labels can be added to a
wiring diagram later rather than inferred from FC resource numbers. Their
absence does not invalidate the confirmed logical-to-physical motor mapping.

The historical `resource MOTOR 4 NONE` prevents PA8 from remaining assigned to
two motor resources. Other historical motor resources are MOTOR5 PB11, MOTOR6
PB10, MOTOR7 PB3, and MOTOR8 PA15; they are outside the three-motor TRI mixer.
The stock TRI mixer retains the expected logical order rear, right, left.

### Differences from modern board defaults

| Resource | Modern `MATEKF405TE` default | Historical flown aircraft |
|---|---|---|
| MOTOR 1 | PC9 | PC9 |
| MOTOR 2 | PC8 | PC8 |
| MOTOR 3 | PB15 | **PA8** |
| MOTOR 4 | PA8 | **NONE** |
| SERVO 1 | PB14 | **PB1** |
| LED_STRIP 1 | PB1 | **NONE** |
| ADC_EXT 1 | PC0 | PC0 |

Motor-output reordering defaults to identity in the local firmware, unlike the
confirmed aircraft configuration. Restoring board defaults can therefore change
both motor identity and the servo pin. Preserve the supplied working cloud-build
export before making any changes. The original 4.3.1 dump is reference material,
not a complete configuration import for 2026.6.2.

## Tail servo

- EMAX ES08MD II, 5 V; bearing-mounted motor tilt basket.
- Internal mixer servo index **5** (`SERVO_RUDDER`) is sent to hardware servo
  index **0**, which is CLI `resource SERVO 1 B01` on the flown aircraft.
- PB1 timer assignment: **TIM3 CH4, AF2**. The modern board's LED-strip timer
  mapping already includes this pin; changing the resource does not require
  inventing a new timer mapping.
- Historical travel: minimum **1100 µs**, maximum **1700 µs**, midpoint
  **1420 µs**, rate **100**, forward channel **-1**.
- Historical `servo_pwm_rate = 498`, `servo_lowpass_hz = 0`, and
  `tri_unarmed_servo = ON`. The last setting permits servo movement while
  disarmed; it is part of the known configuration, not a new recommendation.
- 1420 µs is an assumed vertical-rotor reference pending physical measurement.
  The approximate 1:1.5 servo-shaft-to-rotor-angle gearing needs calibration.

`writeServos()` applies the servo table and optional low-pass filter before
writing the TRI rudder servo. When unarmed movement is disabled, it can write
zero to suppress PWM. M1 command logging must observe that final written value,
including zero, rather than an earlier mixer demand.

## PC0 position-feedback input

The prototype connects the internal potentiometer wiper directly to the AirS /
PC0 pad, sharing servo and flight-controller ground. Board configuration defines
`ADC_EXTERNAL1_PIN PC0`; the CLI resource is `ADC_EXT 1 C00`.

| Command reference | Measured wiper voltage | Nominal 12-bit ADC count at 3.300 V |
|---|---:|---:|
| 1100 µs | 1.561 V | 1937 |
| 1420 µs | 1.281 V | 1590 |
| 1700 µs | 1.031 V | 1279 |

Counts are predictions, not measurements. The voltage falls as PWM rises; none
of these points independently establishes rotor angle or vertical zero.

PC0 has no duplicate pin assignment in the historical dump or audited modern
board defaults. Both use ADC1 DMA option 1, historically DMA2 Stream4 Channel0.
The old motor timer DMA assignments use different streams (PC9 Stream7, PC8
Stream2, PA8 Stream1). The current cloud build uses DShot bitbang, whose active
DMA2 Stream2/Stream3 allocations are separate from ADC1's Stream4; the supplied
runtime ownership confirms no active conflict. Repeat the ownership/timing
capture on the instrumented local image.

The F4 ADC driver initializes external channels in analog mode without pull-up
or pull-down, with a 480-cycle sample time and continuous DMA acquisition.
Check the acquired channel and actual sample behavior on the local image. A
read of a cached ADC value is not an independent proof of fresh conversion.

Initial measurements must compare wiper voltage with and without PC0 connected,
ADC noise, and servo behavior. The optional TLV9001 is a 3.3 V unity-gain buffer
if measurements justify it. Do not add the earlier proposed raw-wiper 100 kΩ
pull-down or assume that a disconnected, floating input can always be detected
from its value. Establish electrical fault detection separately.

## Motor telemetry and historical logging

The intended interface is bidirectional DSHOT300, 14 magnetic poles / 7 pole
pairs, and Bluejay 0.21 with its EDT arming-handshake requirement. The historical
configuration already used DSHOT300, bidirectional telemetry, and 14 poles.
Verify the modern `dshot_edt` setting against the selected firmware and ESCs.

RPM must remain indexed by logical motor, using the mapping above. In this
firmware `getDshotRpm()` supplies mechanical RPM, while Blackbox `eRPM[]` uses
100 electrical RPM per unit. With 14 poles:

```text
mechanical RPM = logged eRPM * 100 / 7
```

The historical Blackbox headers inspected (001, 004, 005) contain three
`motor[]` command fields and `servo[5]`, with no RPM or measured-angle fields.
Their approximately 1 kHz recording rate and servo commands are useful
references, but cannot validate modern RPM decoding or servo position.

An EDT arming handshake does not establish continuing RPM reception. M1 bench
evidence must include packet quality, RPM sample age, and behavior when updates
stop.

## Read-only export from the current working firmware

With the aircraft disarmed and props removed, capture a complete terminal
transcript before flashing or changing settings. Every command below is a
query; run each on its own line without adding configuration arguments. A
command may be unavailable if its feature was omitted from the cloud build;
retain that response as useful build evidence. Entering the CLI prevents normal
flight operation, so perform this on the bench.

```text
version
status
diff all
dump all
resource
resource show all
timer
timer show
dma
dma show
feature
serial
mixer
servo
map
get motor_output_reordering
get motor_pwm_protocol
get motor_poles
get motor_idle
get dshot
get servo
get tri_unarmed_servo
get adc
get blackbox
get debug_mode
get align_board
get pid_process_denom
dshot_telemetry_info
flash_info
tasks
```

Save the transcript with the firmware version, date, and board identity. Keep
the cloud firmware image/build identifier if available. This export gives a
rollback reference for the known working modern setup; preserve it alongside
the original 4.3.1 evidence. No `defaults`, assignment, `save`, motor-test, or
flash-erase command is needed to collect this export.

## Remaining physical acceptance evidence

The cloud-build confirmation and supplied runtime export are recorded above.
Completed M0 aircraft evidence and remaining local-image measurements are:

- [x] Current 2026.6.2 cloud-build CLI evidence and firmware/build identifier
  supplied and preserved as complete chat-transcribed CLI text plus a labeled
  excerpt; resources compared with this historical mapping. A full `dump all`
  export and the original cloud image remain useful rollback evidence.
- [x] User confirmed that current physical motor/servo mapping is correct;
  active output resources and ADC/DMA ownership agree without conflicts.
- [ ] Board/ESC/servo wiring photographs or labeled diagram recorded, including
  the numbered physical ESC outputs for both front motors.
- [ ] After flashing an instrumented local image, repeat the props-off output
  check against the confirmed cloud baseline and record the image identifier.
  Add the two front ESC connector numbers when documenting the wiring.
- [ ] PB1 servo output, command polarity, 1100–1700 µs travel, midpoint, and
  mechanical clearance verified on the local image.
- [ ] PC0 voltage, acquired ADC count, loading/noise, and servo stability checked
  before and after attaching the direct wiper connection.
- [ ] Rotor angle independently measured against ADC values; vertical zero,
  stops, and linkage direction documented.
- [ ] Runtime resource/DMA ownership, CPU/task timing, Blackbox flash capacity,
  all three RPM channels, and continuing EDT/RPM reception recorded.
- [ ] Local firmware rollback to the known working modern configuration
  exercised before any later control-enabled milestone.

M0/M1 software build evidence does not substitute for these physical checks.
No actuator-aware control law is enabled by this hardware documentation.
