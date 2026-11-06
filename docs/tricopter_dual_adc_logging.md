# Dual ADC servo-ground logging

This follow-up to [the servo characterization](tricopter_servo_characterization_2026-11-06.md) records the tapped wiper and local servo ground separately, then subtracts their ADC counts. It remains **logging-only**: the legacy PID/mixer and servo/motor outputs are unchanged.

## Wiring and configuration

| Signal | Physical input | ADC source / configured resource |
|---|---|---|
| Potentiometer wiper | PB0 / ADC1_IN8 | `ADC_EXTERNAL1`, `ADC_EXT 1 B00` |
| Servo PCB ground sense | PC0 / ADC1_IN10 | Existing `ADC_RSSI` acquisition slot, `ADC_RSSI 1 C00` |

The additional PC0 wire is a sense input; it does not replace the servo's normal power ground. Both pins remain ordinary unipolar FC ADC inputs. PC0 can measure a positive servo-ground offset but cannot represent negative excursions below FC ground.

The new debug mode is **`TRI_GROUND`**. It enables acquisition of the existing `ADC_RSSI` slot without enabling the RSSI feature or receiver RSSI processing. The ADC parameter-group structure/version is unchanged, and no PG was added. Other debug modes retain the old RSSI/FrSky-D behavior. Save/reboot is required to initialize the additional ADC channel.

After backing up the current configuration and installing the dual-ADC build:

```text
feature -RSSI_ADC
resource ADC_EXT 1 B00
resource ADC_RSSI 1 C00
set adc_external1_enabled = ON
set debug_mode = TRI_GROUND
set blackbox_disable_debug = OFF
set blackbox_disable_servos = OFF
set blackbox_disable_motors = OFF
set blackbox_disable_rpm = OFF
set blackbox_disable_bat = OFF
save
```

Verify both configured `resource` and active `resource show all`: PB0 should be `ADC_EXT`, PC0 `ADC_RSSI`, and the aircraft's motor/servo resources unchanged. Keep `RSSI_ADC` disabled. The resource name describes the reused acquisition slot; its signal is servo ground in this diagnostic mode.

## New Blackbox schema

The existing **`TRI_SERVO` mode and its recorded fields are unchanged**. `TRI_GROUND` uses the following separate eight-field schema:

| Field | Meaning |
|---|---|
| `debug[0]` | Raw PB0 wiper count from the coherent pair; `-1` before a pair is available |
| `debug[1]` | Final TRI servo PWM write request, µs, including zero for suppressed pulses |
| `debug[2]` | Raw PC0 servo-ground count from the same pair; `-1` before available |
| `debug[3]` | **Signed PB0 − PC0**, in ADC counts; unavailable sentinel **−32768** |
| `debug[4]` | Time since the last accepted coherent pair, 100 µs/count; `-1` before available |
| `debug[5]` | Status mask below |
| `debug[6]` | Worst new-RPM observation age among all three logical motors, 100 µs/count; `-1` unless all three have observations |
| `debug[7]` | Worst existing DShot invalid-packet statistic, 0–10000 = 0–100.00%; `-1` unavailable |

The corrected range is −4095…4095. **−1 is valid corrected data**, so it is not the unavailable sentinel. PC0=0 is normal and is accepted. Cached pairs remain visible with increasing age when acquisition is stale or a safe copy cannot be obtained. Age values saturate at 32767 counts (3.2767 seconds).

| Status bit | Meaning |
|---|---|
| 0 | Backend supports coherent-pair diagnostics (currently STM32F4) |
| 1 | Both inputs/configuration and DMA are ready |
| 2 | At least one coherent pair has been accepted |
| 3 | Pair is ready and its last accepted copy was observed within 1 ms |
| 4 | Bidirectional DShot telemetry enabled |
| 5, 6, 7 | Logical RPM channels 0, 1, 2 have received an observation |
| 8 | Wiper value is at ADC floor or ceiling |
| 9 | Ground sense is at zero: valid, but negative ground excursions are unobservable |
| 10 | Ground sense is at ADC ceiling |

Bit 3 is an acquisition-progress diagnostic, not proof of ground-wire connection or exact conversion age. Existing packet-error statistics omit some missing/aborted receptions; inspect RPM age too. Native motor/RPM/servo/time fields are retained.

## Pair coherence and timing

PB0 and PC0 are adjacent, last-two channels in the F4 regular scan. The standard aircraft configuration also samples battery and current first. The pair API waits for a new completed scan after discarding any boot-sticky completion flag. It copies only during the early scan prefix, while at least one unrelated conversion remains before either pair member could be overwritten. The DMA count must remain unchanged and the copy must complete within 5 µs. Otherwise it retains the previous coherent pair and age. Duplicate, disabled or unsupported source layouts do not fabricate data.

This adds no DMA interrupt, DMA reconfiguration, or interrupt-masking section. With the current four-channel layout, two of the four DMA phases are eligible; not every PID invocation has to publish a new pair. The API deliberately reports unavailable for a two-channel-only layout with no safe prefix. Keep the existing battery/current ADC acquisition enabled for this setup.

The values belong to the **same completed regular scan**, but they are **not simultaneous**. At the current 168 MHz MCU/ADC prescaler and 480-cycle sampling, adjacent regular conversions are nominally about 47 µs apart. Injected internal ADC work can lengthen that gap. Pair age measures acceptance observation, not the physical conversion instant. Hardware timing and CPU cost still require measurement on the connected board.

## Interpretation and analysis

```text
corrected_counts = int32(PB0_raw) - int32(PC0_raw)
corrected_volts  = corrected_counts * measured_ADC_Vref / 4095
```

This should cancel a ground offset shared by the two readings to the extent allowed by sampling skew and noise. It does **not** cancel changes in the potentiometer excitation/reference, clipping, unequal input conditioning, or offsets that do not appear at the servo-ground sense point. The raw channels remain recorded so that cancellation can be tested rather than assumed. It is not yet calibrated rotor angle.

Earlier bench/analysis helpers assume the old `TRI_SERVO` schema: notably `debug[3]` was flags and `debug[4..6]` were individual RPM ages. Do not run their old validity checks unchanged on these logs. Analyze `debug[0]`, `debug[2]`, and `debug[3]` separately, using pair status/age. Preserve the prior recordings and their archived analysis unchanged.

For rollback to an older image that does not know `TRI_GROUND`, select `TRI_SERVO` or `NONE` and save before flashing that image, or restore the matching full firmware/configuration backup.

## Verification

The focused suite covers coherent-copy phase/count/preemption/wrap guards, cache age, signed and common-offset subtraction, zero ground, missing/stale pairs, diagnostics-mode isolation, PWM preservation, and both DShot diagnostic modes. Full target and optional-feature builds supplement host tests. Hardware measurements and any observed cancellation will be recorded separately after flashing; no cancellation result is assumed from these software checks.
