# Offline servo prediction and RPM observation

This stage uses saved recordings and synthetic signals. It does not access the aircraft, alter configuration, or connect model predictions to actuator outputs.

The subsequent [reaction-torque stage](tricopter_reaction_torque.md) adds placeholder-coefficient yaw equations and synthetic tests, with no active control integration. The original servo/RPM results below remain unchanged; the expanded combined suite passes 129 host/regression and 28 Python/native-C harness tests.

The later [configurable allocation stage](tricopter_allocator.md) adds a direct virtual-tail effectiveness interface and optional SI geometry adapter. It uses synthetic maps and bounds, requires no airframe dimensions in its shared solver, and preserves the same measurement and active-output boundaries.

The [integrated simulation](tricopter_simulation.md) now connects those four portable cores in a causal synthetic actuator/response loop, with source-pinned MATLAB parity and Simulink signal playback.

## Measurement and calibration contract

Servo position is **signed PB0−PC0 only**, in corrected ADC counts. Raw PB0 and PC0 are diagnostic channels. PC0 senses the servo PCB power GND input, not the potentiometer low terminal. Position and velocity remain counts and counts/s; physical shaft/tail angle calibration is unavailable. PWM is a command input, never a measured position.

The servo predictor is initialized once from a fresh corrected sample per declared replay unit. Every later prediction uses only timestamped commands and internal state. Missing/stale position samples are excluded from scoring while command time continues. They do not trigger a measurement correction or a new initialization at a command edge.

The two paired-input datasets are used for servo identification. Older single-input recordings are excluded from servo fitting. Whole cycles/repetitions are assigned to training or validation before parameter fitting, rather than randomly splitting adjacent samples. These recordings were previously explored during characterization, so this is a frozen replay holdout, not a pristine prospective experiment.

## Portable cores and integration boundary

The servo model is a monotonic piecewise-linear command/count map, a continuous PWM play operator, a fixed command delay and bounded first-order motion. Its finite command queue and parameter/state validation make missing command history explicit. Invalid configuration, out-of-range commands, backward time, queue overflow and excessive command gaps invalidate prediction until reset. Fitted rate and delay parameters describe these electrical bench recordings; they are not independently measured mechanical limits or uncertainty bounds.

Both model and observer cores are allocation-free C with explicit timing and units. They have no scheduler, ADC, motor, servo, configuration or Blackbox side effects. Offline adapters handle data provenance and quality. Future live integration needs a separate reviewed adapter, timing measurements, angle calibration where applicable and tested fallback; placing a portable file in the flight source directory does not enable it in the flight controller.

RPM observations use mechanical RPM, with acceleration in RPM/s. True new observations are identified by sequence and decoder observation time, including equal-valued and zero-RPM observations. Cached values, failed reception and non-RPM extended telemetry do not refresh the state. Decoder observation time is distinct from the ESC's physical measurement time.

Historical Blackbox logs record quantized ages rather than exact per-packet sequences/times. Their age snapshot and native frame timestamp also have an unmeasured phase offset within the PID loop. Such replay can test compatibility and stale-data behavior, but cannot establish absolute RPM acceleration accuracy. Exact-timing synthetic cases provide the observer's numerical validation.

Physical angle conversion, rotor thrust/drag coefficients, rotor inertia, flight-load dynamics and torque compensation remain uncalibrated. No value inferred from unloaded servo or low-voltage propeller-free motor recordings is promoted to a flight coefficient.

## Servo fit and replay result — 2026-11-07

The frozen split uses five complete training units and three held-out units: the last ground-test cycle, last approach repetition and second staircase. Each model is fitted independently on training units only, with the same fixed optimizer budgets and one initialization per unit. There is no post-initialization measurement correction or validation-driven parameter tuning.

| Predictor | Training RMS error, counts | Held-out RMS error, counts |
|---|---:|---:|
| Instantaneous command map | 58.127 | 52.531 |
| Command map with common first-order lag | 15.580 | 14.156 |
| Play, delay, directional lag and bounded slew | 4.520 | **4.765** |

These aggregate errors give equal weight to each complete unit, rather than treating adjacent ADC samples as independent experiments. The full model's held-out MAE is 3.559 counts; individual held-out RMS errors are 5.412, 4.660 and 4.136 counts for the large steps, center approaches and staircase respectively. Every predicted history remains valid; an invalid model suffix would fail evaluation instead of disappearing from its score. Stale/unavailable measured targets remain unscored without freezing model time.

The full fit has a common effective delay of 16.409 ms, lag constants of 19.133/20.883 ms and slew parameters of 3587/3782 counts/s for increasing/decreasing PWM. Its jointly fitted play width is 7.056 µs. These coupled fit parameters describe command-write-to-corrected-signal behavior. The play parameter does not replace the separately observed 10–11 µs staircase branch separation, and the delay is not a scope-measured servo transport delay. No confidence bound, calibrated angle or loaded mechanical limit is claimed.

The fit, frozen-parameter replay, native C build fingerprints, raw/decoded input provenance, per-unit metrics, compressed replay samples and plots are in [the offline validation package](validation/tricopter-offline-2026-11-07/README.md). The raw inputs remain in their original immutable paired-input packages. A separate invocation that re-decodes those raw files and replays frozen parameters reproduces all model parameters and evaluation metrics exactly in this environment.

## RPM observer behavior

`tricopter_rpm_observer.[ch]` implements a bounded causal alpha-beta observer at the actual observation interval. The filter tracks speed and acceleration from observation-anchored state, projecting a copy to each query timestamp. Extra cached queries cannot integrate the same acceleration twice. The update gains place repeated error poles at `exp(-dt/trackingTau)` before bounds; they do not differentiate raw RPM samples directly.

Sequence identity and modulo-32-bit time are separate from RPM value. Genuine equal/zero observations are accepted. Invalid measurements do not refresh age; inconsistent duplicate identity, backward/ambiguous time or sequence, and an excessive scheduler-call gap latch invalid until reset. Telemetry loss preserves explicit age, decays acceleration after the configured stale interval, bounds prediction at its horizon and permits reacquisition from a genuinely new observation with zero initial acceleration. Saturating elapsed age prevents an old sample becoming fresh after a full clock wrap.

The illustrative replay configuration is frozen, not identified from the motor data: 20 ms tracking constant, 30 ms stale-acceleration decay, 10 ms stale interval, 100 ms prediction horizon, 20000 RPM and 100000 RPM/s software guards. These are test settings rather than calibrated motor limits or flight settings.

Exact-timestamp synthetic tests validate numerical behavior separately from historical replay. On the specified irregular ramp, speed/acceleration RMS errors are 0.000304 RPM and 0.015768 RPM/s after the declared warm-up. On the ramp with alternating ±25 RPM measurement noise, the observer's acceleration RMS error is 31.237 RPM/s versus 49999.996 RPM/s for raw differentiation. This constructed example checks the implementation; it is not a measured acceleration accuracy claim for the aircraft.

The motor inputs are newly preserved in [the RPM evidence package](validation/tricopter-rpm-2026-11-06/README.md). Their recorded age fields and frame time are replayed with explicit phase-allowance sensitivity. The old raw PB0 field is ignored for position, and these motor recordings never enter the servo fit.

## Verification and remaining hardware work

The focused host/regression suite passes **111 tests**, including 16 servo-model and 18 RPM-observer cases. The Python/native-C harness suite passes **19 tests**. Separate strict and fast-math runs exercise numeric-fault handling; both portable cores compile for Cortex-M4F with the firmware's fast-math assumption and warning-as-error settings. Representation-based finite checks preserve NaN/Inf rejection under those flags. Frozen replay rejects changed split/settings metadata, so earlier training cannot silently be relabeled as a new holdout.

No new source is added to the active flight build's source list, and no model/observer call is added to actuator, scheduler, ADC or DShot paths. No hardware was accessed or firmware flashed during this offline stage. The installed aircraft image and its saved configuration are untouched.

Before physical validation: calibrate corrected counts against actual tail angle from both approaches, collect per-motor exact observation identities/timestamps for latency/acceleration work, and measure loaded behavior. Existing electrical/fault and independent PWM timing gates remain open. The future live RPM adapter must also explicitly enable tracking outside the currently supported `TRI_SERVO`/`TRI_GROUND` diagnostic modes if required; the offline observer does not change that firmware policy.
