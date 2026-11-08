# Default feedback tricopter mixer

Selecting `mixer TRI` now selects the measured-position controller. There is no enable switch or legacy TRI servo rule in this path. The current three-motor ordering, resource/reordering settings, motor rotation settings, PID implementation and servo output hardware are retained. Logical motor 0 is the tail, 1 front-right and 2 front-left; hardware reordering remains in the motor driver. `CUSTOM_TRI` uses the same tail0 convention.

The flight code uses **three points**, not the offline 58-knot profile. PB0−PC0 is the only measured position input. The ADXL is not a runtime dependency and can be removed to reinstall the motor. No new physical characterization is required to build or operate this implementation with the existing configuration.

## Maps and defaults

PWM knots are the existing **servo record 5 min/middle/max**. Each corresponding angle is in tenths of a degree; each ADC knot is signed corrected PB0−PC0 counts:

| Point | PWM, µs | `tri_angle_*`, 0.1° | `tri_adc_*`, counts |
|---|---:|---:|---:|
| low | 1100 | −478 | 1936 |
| mid | 1509 | 0 | 1490 |
| high | 1700 | +216 | 1281 |

These initial values use our existing basket measurements. ADC points may increase or decrease; all three must be distinct and monotonic. The middle angle is zero. Hard PWM limits are honored directly; the old ±300 µs servo-rule restriction is gone.

```text
mixer TRI
servo 5 1100 1700 1509 100 -1
set tri_angle_low = -478
set tri_angle_mid = 0
set tri_angle_high = 216
set tri_adc_low = 1936
set tri_adc_mid = 1490
set tri_adc_high = 1281
set tri_yaw_direction = 1
set tri_yaw_trim = 0
save
```

The existing PB0 `ADC_EXT` and PC0 `ADC_RSSI` resource assignments remain. TRI enables both acquisition slots independently of debug mode without enabling RSSI processing. Current F4 pair acquisition uses the existing battery/current scan prefix. An unavailable backend or pair is reported rather than replaced with raw PB0 or command-derived position.

## Control behavior

The existing normalized roll/pitch/throttle mix supplies requested tail vertical effort. The existing yaw PID sum, divided by 1000, supplies yaw demand; its sign is independent of `yaw_motors_reversed`, matching the old servo convention. Servo rate and stabilized-yaw reversal still affect yaw gain/direction; reverse-thrust yaw follows the existing 3D convention.

The bounded allocator computes target tilt from lateral/vertical demand, limits it to the calibrated angle/PWM range and vertical-effort feasibility, and converts that target through the three-point PWM map. A small position P correction closes the loop on the measured angle. **Current** tail compensation and vertical/lateral vector calculations use the measured angle, never the target angle. Tail compensation is applied before native motor thrust linearization/output scaling; front motor commands and normal disarm/motor-stop/failsafe processing retain their existing paths.

Effort/vector values are normalized control proxies in Betaflight's mixer convention, not measured newtons or torque. No rotor coefficients, RPM observer, servo predictor, inertial reaction-torque feedforward or finite-horizon allocator are linked into this flight path.

| Setting | Default | Meaning |
|---|---:|---|
| `tri_yaw_gain` | 700 | Lateral-demand gain in thousandths; 700 = 0.7 |
| `tri_servo_p` | 20 | Position P gain in 0.1 µs/degree; 20 = 2 µs/degree |
| `tri_feedback_timeout` | 20 | Maximum coherent-pair age, ms |
| `tri_adc_margin` | 32 | Endpoint noise allowance, counts; clamped within this margin |
| `tri_yaw_direction` | +1 | ±1 yaw polarity, independent of physical angle mapping |
| `tri_yaw_trim` | 0 | Neutral target trim in 0.1° |
| `tri_servo_speed` | 0 | Optional observed speed estimate, degrees/s; zero means not learned |

Invalid configuration, invalid/stale feedback, channel forwarding, zero servo rate or an active calibration prevents arming with `TRI_FEEDBACK`. If feedback fails while already armed, the controller retains bounded three-point feedforward servo commands and uncompensated base motor effort; measured-vector availability becomes false. It does not fabricate an angle or forcibly disarm. Valid PC0=0 is accepted; rail samples are rejected. Acquisition freshness/range alone cannot reliably detect a floating wire at a plausible count.

## Optional disarmed calibration

This is optional: the defaults above work without running it. With props removed and existing configured servo endpoints mechanically clear:

```text
tricopter_calibrate start
tricopter_calibrate status
```

The asynchronous routine visits center → low → high → center, with bounded PWM ramps and settled averaging. It learns ADC values at the existing PWM points and estimates observed 10–90% speed from the measured trajectory. The speed is measured under that command ramp; it is not a maximum-speed or loaded-flight guarantee. Angles remain the configured three points.

After approximately five seconds, check `complete=1 result=1`, then:

```text
tricopter_calibrate apply
save
```

Completion holds center until apply/abort. `apply` changes only ADC points and the observed speed estimate in RAM; `save` persists them. No motor output is commanded. Armed state, active motor tests, stale/invalid feedback, inadequate/nonmonotonic movement or timeouts cancel the routine. Changing PWM/angle knots invalidates the result. `tricopter_calibrate abort` or leaving CLI cancels it and requests center for 300 ms before releasing the temporary unarmed-servo override.

## Straightforward bench validation

1. Save `version`, `status`, `dump all` and resources before flashing. Retain the current 1509 center and motor/resource/reordering settings. The expanded tricopter parameter group has a new version. Betaflight's boot policy resets the whole configuration when a parameter group cannot load: restore the complete pre-flash dump, save/reboot, and compare it before actuator tests. The new three-point settings load their defaults without requiring another calibration.
2. With props removed, flash the MATEKF405TE image and inspect `get tri`, `servo` and `tricopter_calibrate status`. Expect `config=1 feedback=1 vector=1`. All physical motor outputs must remain stopped while disarmed. Check arming flags after normal receiver/gyro conditions are satisfied.
3. Check that feedback remains valid with `debug_mode=NONE`; it must not depend on `TRI_GROUND` logging.
4. With `tri_unarmed_servo=ON`, use temporary `set tri_yaw_trim = 50` and `-50` to request small ±5° positions without changing receiver/arming settings. Inspect actual/target angle and PWM using `tricopter_calibrate status`. Restore trim to 0. Confirm correct measured direction and bounded PWM; no ADXL is required.
5. Optionally run the calibration commands above. Confirm completion holds center, abort returns center, and motor-test/armed conditions reject starting. Keep the learned result only when its endpoints are sensible.
6. For logging, select `debug_mode=TRI_CONTROL`, enable Blackbox debug/servo/motor fields and save. Fields 0–7 are measured angle (0.1°, unavailable −32768), target angle (0.1°), final PWM, tail effort ×1000, current vertical ×1000, current lateral ×1000, status bits, calibration stage. Vector fields use −32768 when unavailable. Status bits are config-valid, feedback-valid, yaw-saturated, vertical-saturated, calibration-active and input-valid (bits 0–5). Existing `TRI_GROUND` layout is unchanged.
7. Before the first flight, reinstall the motor and verify the usual prop-off motor order/rotation and stabilizing yaw direction. Begin with small hover/yaw commands using existing PID settings and conservative rates. Log tracking, saturation and motor headroom; increase demands gradually. Bench/unit success is not a claim of completed flight validation.

## Later propeller/motor reversal

Motor rotation remains configured through the existing ESC/motor settings. Changing prop rotation reverses steady reaction torque; it does not by itself reverse the geometric servo yaw direction. Keep the physical PWM/ADC maps intact. `tri_yaw_direction` changes yaw polarity if the stabilizing direction requires it, while `tri_yaw_trim` adjusts the neutral bias to favor the useful side of the asymmetric basket range. The rate PID's integral term handles remaining steady bias; advanced reaction feedforward stays absent.
