# Updated Gazebo model characterization — 2026-11-08

Historical scope: this report used electrical coasting on motor reductions. The newer [default active-braking approximation](TRICOPTER_ACTIVE_BRAKING.md) supersedes that behavior; the results below remain bound to their original frozen model and have not been overwritten.

The updated CAD/component-mass and coupled motor/propeller model passes its property-consistency checks, actual fixed-body Gazebo checks in both rotor directions, stock/modified flight smoke checks, and all **42 fresh-case response flights**. These checks characterize the supplied simulation; they do not identify the physical aircraft or establish independently tuned controller performance.

This work retains the firmware binaries and shared firmware PID settings used for the earlier comparison. It changes the simulated aircraft and adapts only the external RC setup pilot to that aircraft. No physical FC is involved.

## Audited model and provenance

The model inputs, calculations and assumptions are in `tools/tricopter/sitl/gazebo/model/`; the [Gazebo model guide](/home/alex/betaflight/tools/tricopter/sitl/gazebo/README.md) describes their derivation. The original STEP SHA256 is `58d1a5f2e55bb05799011594abee32acdc62e99ddc57694e0769f1e6ae6d9e56`. CAD dimensions are transformed to FLU as `(−X, +Z, +Y)` about the regular midplate datum.

| Quantity | Updated value / basis |
|---|---|
| Neutral aircraft mass | 0.5099032001 kg; 81 components in four disjoint physical links |
| Link masses, base/right/left/tail | 381.2033 / 38.0000 / 38.0000 / 52.6999 g |
| Neutral CG relative to body datum | (−8.811, +0.048, −7.296) mm |
| CG inertia diagonal, roll/pitch/yaw | (0.000892456, 0.001569079, 0.002008865) kg·m² |
| Significant cross term | `Ixz = +0.0000618298 kg·m²`; full tensors retained |
| Front motor spacing | 165.037 mm; rear-to-front spacings 179.158 / 178.911 mm |
| Rear geometry | Motor mount 8.8 mm above front mounts; hinge 10.7 mm above rear mount |
| Relative rotor spin inertia | Approximately 4.567 × 10⁻⁶ kg·m² per motor/prop; material/CAD estimate |
| Target motor electrical inputs | 1800 KV, 81.5 mΩ line-to-line terminal resistance, supplied 1.2 A no-load current at 10 V |
| Supply / current bound | Fixed 22.2 V; assumed model current-regulator limit 35.22 A |
| Static prop coefficients | `kT = 8.8485406662e−7 N/(rad/s)²`, `kQ = 9.3958662702e−9 N·m/(rad/s)²` |
| Predicted full-command operating point | 12.6282 N, 36,075 RPM and 26.482 A per motor |
| Ideal equal-thrust hover | 34.462121% equivalent voltage duty and 13,106 RPM, before yaw/CG/vectoring corrections |
| Servo target / joint plant | 20 ms target lag, 362°/s target-rate cap, PD 1.5 / 0.003, torque bound 0.45 N·m |

Mass and inertia are calculated estimates using the supplied dimensions/masses, generic CAD, material assumptions and allowances for missing parts. The nominal 488–530 g mass-assumption sensitivity is not a confidence interval or a bound on CG uncertainty. Rotor inertia includes estimated magnets, back iron, aluminum web, shaft and the supplied 4.2 g prop mass; it has not been measured.

The prop coefficients come from the nearby XING2 2207 1855 KV/GF51466 full-command point at 24 V, 1685 g and 35.08 A. Reference RPM and torque are inferred from KV/resistance/current, then the prop coefficients are transferred to the target motor. Partial-throttle benchmark labels are not identified ESC voltage duties. Actual loaded RPM, ESC duty mapping, active braking, battery sag, temperature and propeller inflow remain uncharacterized. The 35.22 A bound is a model choice, not a measured ESC current limiter.

The active `dc_prop` backend integrates rotor speed from electrical torque, internal loss, prop load and inertia. It does **not** layer the previous 30 ms speed lag on top. Thrust and aerodynamic reaction use actual rotor speed and actual articulated rotor orientation. The carrier receives `−spin × Qprop × axis` and `−d(spin × J × omega × axis)/dt`; their axial sum is the shaft-torque reaction, so electromagnetic torque is not added again. Locked rotor mass/inertia appears once in the SDF tensors. The RPM ODE neglects carrier angular acceleration projected on the shaft, as documented in the model guide.

The read-only `--check` modes of `rotor_inertia.py`, `calculate_mass.py`, `motor_performance.py` and `apply_model_properties.py` passed, including `calculate_mass.py --verify-step Tricopter_motors.step`. The compiled math/protocol, motor-dynamics and SDF/asset tests passed. These verify implementation and asset consistency rather than physical accuracy.

## Actual fixed-body Gazebo checks

The body was fixed to the world while the rear joint remained articulated. All motors received 1400 µs, followed by servo commands `1509 → 1700 → 1100 → 1509 µs`, then command loss. Both checks passed:

| Rotor directions | Source run / result |
|---|---|
| Normal `+1, −1, +1` | [.local-artifacts/tricopter/gazebo-sitl/runs/20261108T220405-plant-normal-259445/plant-check.json](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261108T220405-plant-normal-259445/plant-check.json) |
| Reversed `−1, +1, −1` | [.local-artifacts/tricopter/gazebo-sitl/runs/20261108T220413-plant-reversed-259445/plant-check.json](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261108T220413-plant-reversed-259445/plant-check.json) |

Settled thrust was 2.22835 N per rotor at 1586.922 rad/s. The servo endpoints reached +21.5947° and −47.7895°, with PB0 1281/1936 and PC0 zero. Actual thrust axes followed joint angle; aerodynamic reaction changed sign on rotor reversal. Watchdog expiry removed servo drive, opened motor drive electrically, and allowed visible coast-down before thrust approached zero.

Peak sampled rear spin-up reaction was 0.1801/0.1806 N·m and precession reaction 0.04602/0.04614 N·m for normal/reversed runs. The electrical-power identity residual remained below 8.7e−10 W. These are reaction loads in an anchored test, not measured aircraft angular response.

The derived [normal anchored report](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/updated-model-characterization-review/20261108T220405-plant-normal-259445/anchored-characterization.json) and [reversed anchored report](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/updated-model-characterization-review/20261108T220413-plant-reversed-259445/anchored-characterization.json) retain trace/world/postprocessor hashes. At 100 Hz, sampled servo 10–90% movement times were:

| Command transition | Normal | Reversed |
|---|---:|---:|
| 1509 → 1700 µs | 57.0 ms | 58.0 ms |
| 1700 → 1100 µs | 153.5 ms | 153.8 ms |
| 1100 → 1509 µs | 107.5 ms | 107.5 ms |

Command onset is bracketed by adjacent recorded samples; interpolation does not establish sub-millisecond timing. These values characterize the current simulated servo with spinning simulated rotors, not the measured unloaded hardware servo or its unknown loaded response.

## Stock and modified flight smoke checks

Both firmware variants passed takeoff, hover, bidirectional yaw, landing and disarm in [.local-artifacts/tricopter/gazebo-sitl/runs/20261108T220609-both-flight-validation-261406/flight-summary.json](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261108T220609-both-flight-validation-261406/flight-summary.json). Native firmware altitude/position autopilot modes stayed off; an external RC pilot used Gazebo ground truth. Both trials ended disarmed and stopped all owned processes.

| Smoke measurement | Stock | Modified |
|---|---:|---:|
| Maximum flight absolute roll/pitch | 3.019° | 3.016° |
| Hover maximum altitude error | 0.243 m | 0.237 m |
| Hover maximum horizontal error | 0.503 m | 0.496 m |
| Yaw-positive world CCW heading change | +11.601° | +12.831° |
| Yaw-negative world CCW heading change | −11.962° | −12.657° |

The modified firmware reported valid measured feedback for every sampled armed record. Its hover angle-versus-joint error was 0.04863° RMS / 0.08528° maximum; simulated ADC quantization stayed below 0.5 count in both trials. These smoke checks establish operation with the changed plant, not precision position holding or a controlled performance ranking. Their earlier external-pilot hover estimate is not a calibrated motor/RC transfer measurement.

## Direct portable motor-model replay

[The motor replay package](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/updated-model-characterization-review/motor/README.md) compiles the actual frozen `MotorModel.hh` and reads its coefficients from the exact SDF. It uses the tail rotor inertia, 101 equilibrium duties from 0 to 1, and six independent equilibrium-start steps at 1 ms for 3 s each. This is a portable-model replay, **not a fresh Gazebo flight**.

Ten-to-ninety-percent crossings reference the requested equilibrium excursion rather than the trace's final sample. Settling means the remaining trace lies within ±2% of that excursion, with at least 250 ms retained in band. Every case retained more than 2.37 s after settling. Crossings interpolate 1 ms samples; settling is sample-quantized.

| Equivalent voltage-duty step | RPM 10–90% | Thrust 10–90% | RPM 2% settling | Thrust 2% settling | Upper current clamp |
|---|---:|---:|---:|---:|---:|
| 0 → hover | 35.55 ms | 36.58 ms | 60 ms | 69 ms | 23 ms |
| Hover → hover + 0.025 | 28.01 ms | 28.35 ms | 50 ms | 51 ms | 0 ms |
| Hover → hover − 0.025 | 28.82 ms | 28.44 ms | 52 ms | 51 ms | 0 ms |
| Hover → 0.7 | 38.79 ms | 40.45 ms | 62 ms | 66 ms | 31 ms |
| 0.7 → hover | 113.20 ms | 106.78 ms | 145 ms | 141 ms | 0 ms |
| Hover → 0, coasting | 498.23 ms | 311.78 ms | 627 ms | 484 ms | 0 ms |

All cases passed monotonicity, equilibrium settling, current-bound and discrete torque/power checks. Maximum power residual was 7.96e−12 W and torque residual 2.33e−15 N·m. The report explicitly accounts for nonnegative backward-Euler numerical dissipation; that numerical loss is not motor heat. Independent checks verified crossing brackets, last-out-of-band settling, sample intervals, artifact hashes and agreement with the separately calculated full-command equilibrium.

There is no universal motor time constant. Step size, direction, operating point and current limiting matter; the long downward responses follow the explicit absence of active braking. The package contains [JSON metrics](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/updated-model-characterization-review/motor/motor-characterization.json), [CSV metrics](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/updated-model-characterization-review/motor/motor-characterization.csv), [equilibrium samples](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/updated-model-characterization-review/motor/equilibrium.csv), complete traces, driver/source snapshots and a [SHA256 manifest](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/updated-model-characterization-review/motor/manifest.json).

![Steady thrust/current and requested-equilibrium-normalized motor steps from direct portable-model replay](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/updated-model-characterization-review/motor/motor-model-characterization.png)

[Standalone SVG figure](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/updated-model-characterization-review/motor/motor-model-characterization.svg). Panel B normalizes each signed thrust excursion against its requested equilibrium, so downward steps also progress from zero to one. The figure uses retained model outputs and performs no additional simulation; its input/output hashes are in `figure-provenance.json`.

## Response protocol and retained qualification failures

The [earlier controlled comparison](/home/alex/betaflight/docs/TRICOPTER_SITL_COMPARISON.md) at `20261108T190358-both-controlled-comparison-fresh-140371` remains a completed result for its original plant. Its 42 flights and metrics are not overwritten or pooled with the updated model. Both campaigns retain firmware roll P/I/D/F 35/16/15/47, pitch 35/17/15/49 and yaw 50/10/20/47. Neither campaign independently tunes those gains or sets yaw feedforward to zero.

The updated campaign keeps the same seven-case matrix, fresh world/EEPROM per case, three repetitions per firmware, counterbalanced order stock/modified/modified/stock/stock/modified, fixed-input measurement windows and requested-rate metrics. RC packets run at 50 Hz and plant traces at 100 Hz. Measurement channels atomically override external-pilot outputs; input timestamps and stage-duration errors remain part of qualification. The setup pilot retains 12° attitude authority and the same initial-state gates.

The updated setup adapter in `check_flight.py` inverts the steady DC/prop model and the live receiver/motor endpoint mapping for takeoff/recapture. It also supplies roll trim from actual tail lateral thrust. This adapter writes external RC inputs only; it changes neither Betaflight PID coefficients nor firmware allocation. During response measurement its calculated inputs are overridden by the declared waveform. The shared measurement hover throttle is now **1345 µs**, with ±25 µs collective pulses at **1370/1320 µs**; yaw remains 1368/1632 µs, approximately ±60.30°/s. The changed hover input is declared rather than treated as numerically identical to the original 1350 µs experiment.

Three failed qualification roots remain intact and are excluded from the repeated response result:

| Retained root under `.local-artifacts/tricopter/gazebo-sitl/runs/` | Observed failure / disposition |
|---|---|
| `20261108T220742-both-updated-model-qualification-263243` | Initial-state recapture gate timed out before measurement. The legacy external setup pilot retained approximately 0.49 m lateral offset and 0.20 m altitude bias. The DC inverse and tail-force setup trim were then added outside the firmware. |
| `20261108T221119-both-updated-model-qualification-2-266445` | The first yaw trial exceeded the 3 m altitude envelope after entering measurement with the fixed 1350 µs hover input. The primary experiment was requalified with 1345 µs; the failed flight is not discarded as a controller success. |
| `20261108T221220-both-updated-model-qualification-3-267757` | Individual stock and modified cases passed, but the aggregate equality gate rejected intentional MSP90 debug-mode/count metadata differences. This was not an actual common flight-configuration mismatch. The full reply is preserved; comparison now uses MSP90 bytes 0–7 for motor protocol/rate/idle and retains other common MSP readbacks. |

## Primary updated-model campaign — complete

The primary root is [.local-artifacts/tricopter/gazebo-sitl/runs/20261108T221429-both-updated-model-characterization-269921](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261108T221429-both-updated-model-characterization-269921). Its frozen protocol SHA256 is `22bb8ec52e1389efde4cd898bf01c2a2ff7f509800c0ac8ad0e28907b6918b1a`. The protocol/source snapshot includes the complete Gazebo source, motor model, model calculations/assets and original STEP, together with driver/configuration inputs.

All 42 fresh flights passed. The [independent protocol audit](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261108T221429-both-updated-model-characterization-269921/independent-fresh-protocol-audit.json) independently checked all 41 frozen inputs, all 32 plugin build-source hashes, the campaign-local plugin binary/build manifest, common firmware readbacks, actual scheduled RC commands, initial gates, timing, actuator and feedback validity, saturation and final disarmed command states. Both firmware binary hashes and CLI settings match the earlier plant's campaign exactly. Full MSP90 replies differed only in the declared debug-mode/count bytes. All 14,345 measurement actuator frames were valid, all 1,041 sampled statuses were healthy/armed in ANGLE mode, and all 501 sampled modified-feedback records were valid. PC0 stayed zero; maximum PB0 map error was 0.499989 count. No motor upper-PWM, servo endpoint or terminal-current limiting occurred during measurements. Sixteen servo endpoint frames occurred outside measurements in modified landing/disarm phases. Maximum measured terminal current was 9.204 A and total battery current 8.927 A; startup histories separately reached the model's 35.22 A terminal limit. Maximum stage error was 30 ms, RC send gap 26.105 ms and trace gap 11 ms. Actual collective pulses lasted 390–430 ms. All trials ended disarmed with motor commands at 1000 µs; that command check does not imply zero rotor RPM during electrical coast-down. [Host cleanup verification](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261108T221429-both-updated-model-characterization-269921/host-process-cleanup.json) found no remaining campaign processes. The Python helper suite passed 56 tests; the plugin's three CTest checks passed.

The following are means of three repetitions per firmware and direction, with the original shared PIDs and yaw feedforward 47:

| Requested yaw-rate response | Stock CW / CCW | Modified CW / CCW |
|---|---:|---:|
| 10–90% rise time | 574 / 575 ms | 416 / 401 ms |
| Peak rate overshoot | 0 / 0% | 0.035 / 0% |
| Held-command settling | 0.873 / 0.863 s | 0.690 / 0.663 s |
| Return-to-zero settling | 0.870 / 0.910 s | 0.690 / 0.713 s |
| Continued heading after release, maximum within 1.2 s | 12.574 / 12.844° | 9.261 / 9.494° |

Rise-time ranges were 572–576 / 571–578 ms for stock and 413–418 / 399–402 ms for modified. Unlike the old plant, every pure-yaw return settled within the observation window, without opposite-rate excursion. Both implementations are well damped at this operating point. The approximately 28–30% faster modified response remains an as-configured result, not a comparison after independently tuning effective loop gains.

Modified still uses more servo authority during the 1.8 s pure-yaw command window:

| Servo response, excursion from pre-command baseline | Stock CW / CCW | Modified CW / CCW |
|---|---:|---:|
| Peak commanded angle excursion | 7.43 / 7.52° | 10.56 / 11.34° |
| Peak actual joint excursion | 6.39 / 6.44° | 8.23 / 8.52° |
| Sampled peak joint speed | 202 / 198°/s | 299 / 337°/s |
| Mapped-command versus joint RMS error | 0.51 / 0.53° | 0.71 / 0.79° |

Thus faster yaw is accompanied by roughly 1.3 times the actual excursion and 1.4–1.5 times the commanded excursion. No servo torque-limit occupancy was observed during command/recovery windows. Signed-summed rotor reaction peaks, excluding thrust-arm moments, were 0.0419/0.0256 N·m stock CW/CCW versus 0.0568/0.0394 N·m modified. Their precession-only norm peaks were 0.0245/0.0231 versus 0.0368/0.0388 N·m. These sampled reactions are useful actuator/load diagnostics, not independent causal explanations for body coupling.

For the 0.4 s collective increase during an existing yaw command, actual-age-aligned subtraction of the yaw-only control gave these mean peak differences over the 1 s post-onset window:

| Collective perturbation during turn | Stock | Modified |
|---|---:|---:|
| CW roll/pitch difference norm | 0.245° | 0.249° |
| CCW roll/pitch difference norm | 0.427° | 0.433° |
| CW yaw-rate disturbance | 8.902°/s | 7.970°/s |
| CCW yaw-rate disturbance | 8.262°/s | 7.421°/s |

There is no demonstrated roll/pitch-coupling improvement: the differences between firmware means are small relative to repeated-run ranges and matching floors. Modified has about 10% smaller combined-case yaw disturbances at these gains. Pre-event combined-case baseline-corrected roll/pitch mismatch reached 0.00896°, with a maximum axis differential slope of 0.0610°/s.

Neutral-yaw collective pairs need more caution. Mean peak roll/pitch differences were 0.299/0.226° for stock up/down and 0.288/0.252° for modified. Pre-existing neutral-pair differences reached 0.231° raw and 0.0968° after constant-baseline removal, with a 0.553°/s differential slope. Consequently the neutral roll/pitch peaks cannot be cleanly attributed to the throttle pulse or used to rank small firmware differences. No repeats were excluded using these diagnostics. The signed traces and trend-sensitivity results remain exported. Neutral matched yaw-rate disturbances were 8.25/8.33°/s stock versus 7.76/7.65°/s modified.

The package contains [yaw trajectories](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261108T221429-both-updated-model-characterization-269921/comparison-yaw.png), [coupling trajectories with visible pre-event history](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261108T221429-both-updated-model-characterization-269921/comparison-coupling.png), JSON/CSV aggregates, all individual traces and the frozen inputs. The separate [actuator/current/torque characterization](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/updated-model-characterization-review/20261108T221429-both-updated-model-characterization-269921/characterization.json) reports sampled RPM, forces, currents, servo tracking, torque components and their provenance. Its rotor-reaction vectors exclude thrust-arm moments and other body forces; numerical dissipation is not heat. Native fixed-input measurement windows are complete ANGLE-mode responses, including accelerometer effects during translation, rather than an isolated rigid-body torque experiment.

The plugin SHA256 used for the anchored, smoke and primary runs is `cf79340c72f9265bc42d83ba5c8e34c607280c0334a550270ecfcf628c7e9544`. The source SDF SHA256 is `4794aba41ac01b42986780cf8440a44efbb3f8af021660168945f949c0a2961f`; materialized run worlds have different hashes because mesh paths and test-specific setup are resolved. The portable replay's frozen `MotorModel.hh` SHA256 is `cd3860b03171a222ff83f4cdd2e4ff2b3a49ae8c71807e7944d672ef392fd3e7`.

## Interpretation limits and next gate

The new geometry and rotor momentum remove important omissions from the original plant. They do not make the servo an identified actuator: the simulator still lacks the retained unloaded model's jointly fitted effective delay, direction-dependent dynamics and input play. The 362°/s observation came from a bounded hardware command ramp, not maximum-speed characterization. The smaller joint D coefficient addresses sampled-PD stability with the CAD basket inertia; it is not a measured servo constant. Reusing the recorded PB0−PC0 model requires replay-checking actual joint trajectories and avoiding duplicated lag.

The operational allocator's effort/thrust mismatch also remains. It treats normalized motor command as vertical thrust and applies `vertical / cos(actualAngle)` compensation, while the active DC/prop plant has a nonlinear command-to-thrust transfer. Its requested-angle and headroom calculations use the same command-space vertical value. The earlier quadratic example is no longer the exact active transfer law, but the domain mismatch persists. The external setup adapter does not fix it inside the firmware. This prevents attributing throttle-dependent improvements solely to measured-vector allocation and must be resolved or explicitly bounded before independent PID optimization.

Any later tuning should separately optimize both firmwares with the same objective, constraints and budget, then freeze gains for held-out collective and translational cases. Simulation gains remain conditional on the declared actuator/aerodynamic assumptions; these tests do not produce physical flight PID settings.

## Evidence retention

The raw run roots and complete motor replay remain ignored local artifacts. A compact versioned evidence subset would retain the two `plant-check.json` files and derived `anchored-characterization.json` files, smoke `flight-summary.json`, portable `motor-characterization.json`/`.csv`, `parameters.json`, `manifest.json`, the motor SVG/PNG and `figure-provenance.json`, and their small driver/replay/plot scripts, plus the completed primary `protocol.json`, summary/aggregate metrics and independent audit. Preserve their source hashes and exact raw-root names together.

Large raw `plant.csv`, `pilot.csv`, RC packets, logs, binaries and duplicated source/mesh/STEP snapshots can remain in a separately hashed retained archive rather than being repeated in the documentation tree. The model inputs/calculations/assets belong with the model source; this document links the working-checkout evidence and does not claim that ignored raw data is already versioned or remotely archived.
