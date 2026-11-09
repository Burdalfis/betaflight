# Controlled Gazebo response comparison

This document records the original approximate-plant baseline. For the updated CAD/inertia/DC-prop plant, use `python3 tools/tricopter/sitl/check_comparison_fresh.py --hover-throttle 1345` and the [updated-model characterization](/home/alex/betaflight/docs/TRICOPTER_UPDATED_MODEL_CHARACTERIZATION.md). The shared measurement inputs and model identity are recorded separately for each campaign; results are not pooled across plants.

Run `python3 tools/tricopter/sitl/check_comparison_fresh.py` from the repository root with no interactive SITL session active. The helper uses the same exclusive session lock as the viewer, performs software-only arming and finishes by disarming/stopping its processes. It never opens a physical FC. It leaves firmware, model coefficients and common PID/rate settings unchanged. Every individual control/pulse case starts with a fresh world and EEPROM, removing PID/maneuver history from earlier cases.

After completion, `python3 tools/tricopter/sitl/plot_comparison.py /absolute/path/to/the/run/directory` exports JSON/CSV summaries and standalone PNG/SVG figures. The plot exporter uses NumPy and Matplotlib (`python3-numpy` and `python3-matplotlib` on Ubuntu). Raw trial data, actual RC packets and frozen source copies remain beside those exports.

The default experiment uses three repetitions of each firmware in counterbalanced group order: stock, modified, modified, stock, stock, modified. Each group contains seven independent fresh flights, for 42 flights in total. World/plugin/binary/protocol hashes and actual common MSP readbacks bind each trial. The frozen protocol and driver source copies are written before trials begin. `--repetitions 1 --cases yaw_cw,yaw_cw_collective_up` runs a smaller four-flight fresh-state check.

An external ground-truth RC pilot performs takeoff, setup, recapture and landing. Its recovery attitude authority is 12°; the separate flight smoke check retains its existing 5° default. Setup requires a stable initial altitude, heading, velocity and attitude gate. During measurement all four flight inputs are atomically replaced by the declared fixed RC waveform. Background pilot calculations do not reach the receiver. Normal ARM/ANGLE channels and immediate disarming remain active. Native altitude/position/autopilot modes must remain off.

## Measurement matrix

The shared hover input is 1350 µs. Every case starts with 1 s of fixed neutral roll/pitch/yaw and hover throttle, then ends with a 1.2 s neutral-yaw recovery observation. Roll and pitch remain at 1500 µs throughout measurements.

| Case | Middle stages |
|---|---|
| Neutral control | 0.4 s with unchanged hover inputs |
| Collective increase / decrease | 0.4 s at 1375 / 1325 µs throttle |
| Clockwise / counterclockwise yaw control | 0.8 s turn, 0.4 s unchanged turn, 0.6 s unchanged turn |
| Clockwise / counterclockwise yaw with collective increase | 0.8 s turn, 0.4 s turn at 1375 µs throttle, 0.6 s turn at hover throttle |

The yaw RC values are 1368 / 1632 µs. Their exact raw ACTUAL-rate targets are calculated from live rates/deadband settings: approximately +60.30 / −60.30°/s in the native body-FRD convention, clockwise positive. World Euler heading is counterclockwise positive. The requested rate is unsmoothed; measured response includes native receiver smoothing, PID/feedforward, servo and motor dynamics.

Actual RC values and send timestamps are recorded at 50 Hz; actual plant state is recorded at 100 Hz. The RC sender owns an absolute simulation-time stage plan using its own plant-trace reader, so MSP polling in the main thread cannot extend a pulse. A completed run must meet a 30 ms stage-duration tolerance. Analysis aligns to recorded transitions and reports actual durations, rather than claiming identical continuous input timing. Millisecond-level interpolation is a numerical convenience, not sub-millisecond experimental resolution. An earlier exploratory batch used main-thread transitions and overran some stages by 75 ms; it is retained separately and is not the primary coupling comparison.

## Response measures and controls

Yaw rise time uses 10% and 90% of the change from measured baseline to **requested rate**. Peak overshoot and tracking error remain referenced to that requested rate, preserving gain errors. End-of-pulse averages are not called steady-state measurements. Settling requires at least 250 ms continuously within the greater of 2°/s or 5% of the target; an unreached/unsettled result is null. After release, the report separately measures rate decay, opposite-rate excursion and continued heading rotation within the finite recovery window. A normal yaw stick commands rate, not a heading step.

Roll/pitch measures use actual ground-truth angles and body rates relative to a pre-event baseline. Neutral collective pulses have a same-duration neutral control. Combined yaw/collective cases have same-duration yaw-only controls. Control traces are aligned to the same actual elapsed time from yaw onset (or neutral-baseline onset), rather than their no-op pulse marker, to avoid introducing an extra input-tick phase difference. Paired control subtraction is accompanied by raw pre-event difference/slope diagnostics. It does not establish a linear or independently identified physical coupling coefficient.

The earlier `check_comparison.py` sequential campaign retained controller state across cases. Despite close physical setup gates, yaw-control and yaw-plus-collective cases already had different roll/pitch trends before the throttle pulse. Its yaw step measurements remain useful, but its apparent degree-scale paired coupling was dominated by maneuver history. Those source-bound records and first-pass plots are preserved separately; the fresh-per-case campaign is the primary coupling comparison.

Reports retain repeated-run ranges rather than treating three trials as a confidence interval. Receiver timing, actual input values, initial states, motor/servo endpoint occupancy, native mode/arming status and sampled modified feedback/saturation flags accompany the metrics. Failed qualification attempts remain separate from the frozen repeated experiment.

This is a full firmware/ANGLE-mode comparison: accelerometer effects during translation are included. The current 500 g plant still has estimated inertia/geometry, static nominal-voltage thrust and motor lag, with no battery sag or enabled rotor-inertia/gyroscopic model. Results establish behavior in this shared model, not physical flight performance.

## Servo fidelity and independently tuned comparisons

The current campaign retains the shared yaw P/I/D/F values (50/10/20/47). It measures the two implementations as configured, including their different mapping from PID output to servo command. A faster response with those gains does not establish superiority over an independently tuned stock controller.

Gazebo's servo is still an approximation: measured three-point static calibration, a 20 ms command-target lag, a symmetric 362°/s target-rate limit, and estimated joint PD/torque parameters. The 362°/s observation came from a bounded command ramp, not a measurement of maximum servo speed. The simulator does not yet reproduce the fitted effective delay, reversal play or directional dynamics, and the disabled rotor-inertia term supplies no gyroscopic motor load.

The [existing unloaded PB0−PC0 servo model](tricopter_offline_models.md) includes jointly fitted effective delay, directional lag/rate limits and input play. Its recorded fit is descriptive electrical-position behavior, not separately identified physical constants or loaded performance. It can be reused without new hardware, but actual Gazebo joint traces must be replay-checked against the retained recordings. Adding its lag beneath the current filter/joint dynamics without that check would risk counting actuator delay twice.

A fair PID-only follow-up can set yaw feedforward to zero for both implementations and tune each P/I/D separately with the same objective, constraints and evaluation budget. Tuning should use hover cases; held-out evaluation should freeze those gains and change collective, climb demand and translational speed. The angle-to-yaw-acceleration gain itself depends on tail thrust. Betaflight's reference feedforward can still anticipate changes in requested yaw rate, so zero feedforward is a useful experimental restriction rather than a consequence of angle/acceleration proportionality.

Rapid forward-flight tests also need a clear model boundary: the present plant has simple linear drag and fixed thrust/reaction coefficients, without identified propeller inflow or advance-ratio effects. Such tests can expose geometric/control sensitivity, but cannot yet establish accurate high-speed aerodynamics or physical PID settings.

## Recorded baseline — 2026-11-08

The primary package is `.local-artifacts/tricopter/gazebo-sitl/runs/20261108T190358-both-controlled-comparison-fresh-140371`. All 42 fresh-case flights passed the declared execution checks. Three repetitions per case/firmware used the same PIDs, including yaw feedforward 47. These are **as-configured results**, not an independently tuned comparison.

Mean pure-yaw results for the approximately 60.30°/s commands:

| Measurement | Stock CW / CCW | Modified CW / CCW |
|---|---:|---:|
| 10–90% rise time | 314 / 308 ms | 217 / 215 ms |
| Overshoot relative to requested rate | 26.7 / 25.8% | 23.3 / 23.3% |
| Held-command settling time | 1.41 / 1.38 s | 1.07 / 1.07 s |
| Maximum continued heading after release, within 1.2 s | 13.22 / 13.54° | 10.32 / 10.33° |

Neither firmware met the return-to-zero settling criterion within the 1.2 s recovery window. This is a censored observation, not a measured full stopping time. Rate/heading trajectories and all individual ranges are in `comparison-yaw.png`, `comparison-analysis.json` and `comparison-aggregate.csv` in the package.

The recorded servo drive confirms the effective-gain concern. During the pure-yaw command window (onset through release, excluding recovery), mean peak commanded angle excursions from each pre-command baseline were 9.52/9.68° CW/CCW for stock and 16.78/16.44° for modified. Mean actual joint excursions were 9.19/9.29° versus 13.65/13.36°. These are excursions under the common measured static PWM map, not PWM-to-angle linearizations fitted separately per controller. The source-bound `independent-servo-comparison.json` retains individual repeats. Faster yaw cannot be uniquely attributed to feedback compensation when the command authority differs this much.

Mean peak norm of the baseline-corrected, matched-control roll/pitch difference:

| Collective perturbation | Stock | Modified |
|---|---:|---:|
| +25 µs at neutral yaw | 0.077° | 0.105° |
| −25 µs at neutral yaw | 0.112° | 0.097° |
| +25 µs during CW yaw | 0.065° | 0.061° |
| +25 µs during CCW yaw | 0.116° | 0.066° |

These effects are small, with overlapping repeated-run ranges in most comparisons and residual pre-event trends. Baseline-corrected pre-event norm differences reached 0.029° stock and 0.026° modified; one stock neutral-down repeat had an axis differential slope of 0.151°/s. No repeat was excluded using these retrospective diagnostics. The package retains raw offsets, trends and pre-event plots. The result does not support a blanket claim of reduced coupling. Neutral-yaw throttle pulses also produced larger matched yaw-rate disturbances in modified firmware: mean peaks 0.97/0.81°/s for up/down versus stock 0.31/0.26°/s.

The independent stdlib audit (`independent-fresh-protocol-audit.json` and its replay script) found no execution issues: all frozen sources and common configuration readbacks matched, all 14,345 measurement plant frames had valid actuator input, all 1,037 sampled firmware statuses were armed in ANGLE mode without faults, and all 499 sampled modified feedback records were valid. No motor upper bound or servo endpoint was occupied. All cases ended disarmed with motor outputs at 1000 µs. Maximum stage-duration error was 30 ms, at the declared tolerance; maximum RC send gap was 34.58 ms. Actual nonzero throttle pulses lasted 380–410 ms. This is timestamped real-time UDP execution, not deterministic lockstep. The focused Python harness suite passed 44 tests.

Source inspection also identified a concrete allocator/plant interface mismatch. `tricopter_control.c` applies tail compensation as `verticalEffort / cos(actualAngle)` and treats normalized effort as thrust. Its requested-angle calculation also divides requested lateral effort by that command-space vertical effort. With `thrust_linear = 0`, the resulting motor correction reaches the configured 5%-idle PWM mapping without thrust linearization. The Gazebo plugin instead makes settled thrust quadratic in normalized PWM. Consequently the compensation overcorrects rear vertical thrust when tilted in this plant, and yaw normalization retains a throttle-dependent gain. The exact correction must account for both idle and the chosen command-to-thrust law; changing a CLI linearization coefficient is not automatically an exact remedy. This finding does not establish the real motors' thrust curve or explain the measured coupling magnitude by itself. No controller or plant change was made during this baseline.

Before the tuned comparison, reconcile that effort/thrust interface with a focused end-to-end regression and replay-check the measured unloaded servo dynamics in the actual joint simulation. Then independently tune each firmware at hover with the same objective and budget. Freeze those gains for larger collective changes, sustained climbs and forward-flight yaw tests. Report yaw tracking, damping, settling, actuator use and roll/pitch coupling together rather than selecting rise time alone.
