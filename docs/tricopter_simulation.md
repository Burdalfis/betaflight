# Offline integrated tricopter simulation

This experiment connects the existing C servo predictor, RPM observer, reaction-torque model and virtual-tail allocator in a time-stepped synthetic actuator/response loop. It uses configurable effectiveness maps, with no CAD, CG or body-inertia requirement in the coordinator. All airframes, actuator mappings, limits and coefficients are explicit synthetic placeholders. No aircraft parameters are identified and no flight-control outputs are changed.

The subsequent [opt-in dynamic allocator](tricopter_dynamic_allocator.md) uses predicted servo/motor state and explicit collective/roll/pitch group priority over yaw. The original simulation modes and evidence below remain preserved.

The later [measured basket-angle extension](tricopter_measured_angle.md) makes measured count-to-angle conversion available as a separate explicit option. It preserves the original synthetic default and command-only predictor. Real angle mapping does not make the synthetic actuator dynamics, effectiveness or torque parameters physically calibrated.

## Controller and plant boundary

The coordinator receives requested collective/roll/pitch/yaw response proxies and timestamped mechanical-RPM observations. Reset is its only corrected-position measurement input. Its servo predictor subsequently advances from commands alone. It has no access to plant direction, position, motor acceleration, future packets or plant parameters.

The separate synthetic plant has motor lag/slew dynamics and its own C servo-model state. Plant and predictor can have different parameters. Plant RPM and its analytic derivative generate a synthetic reaction disturbance; the coordinator uses only observer estimates and explicit acceleration validity. The plant's position is a synthetic signed PB0−PC0 count proxy, with a declared counts-to-virtual-direction mapping. This is not calibration of the real servo or a raw-channel fallback.

Each tick has a fixed causal order:

1. Advance the plant to the current time under the previous commands.
2. Capture synthetic decoder RPM observations, queue/deliver packets whose software-delivery times have arrived, and retain their capture timestamps/identities.
3. Advance the command-driven servo predictor and RPM observers, then qualify acceleration and evaluate reaction.
4. Allocate a bounded response target, derive motor-speed/PWM commands, and optionally coordinate virtual tail vertical effort.
5. Hold the new commands for the following interval.

Software delivery latency is distinct from the observer's decoder timestamp. The synthetic packet timestamp is not presented as an ESC-internal measurement timestamp. Startup, stale intervals, reacquisition and repeated cached data cannot fabricate known acceleration.

## Response and reaction conventions

The plant response is an explicit surrogate:

```text
u_actual = [frontRightEffort, frontLeftEffort,
            tailEffort*cos(virtualDirection), tailEffort*sin(virtualDirection)]
effort_i = effortPerOmegaSquared_i * omega_i^2

actuatorResponse = plantEffectiveness * u_actual
totalResponse = actuatorResponse + [0,0,0,plantYawResponsePerNm * scalarReactionNm]
```

Both plant and controller effectiveness maps must **exclude the separately added reaction contribution**. A map fitted to total observed response cannot be used with the full aerodynamic/inertial reaction add-on without separating that contribution. Otherwise it would be counted twice.

This separation is supplied explicitly by the synthetic experiment. Ordinary logs may not identify steady aerodynamic reaction separately from the map's yaw effectiveness, because the effects can be collinear. A future characterized model may need to absorb steady reaction into its map and add only an independently identified transient term, or use a richer/scheduled basis. The current simulation does not solve that identification problem.

The existing reaction core remains unchanged: `reactionSign*(kQ*omega² + J*alpha)`, with separate aerodynamic/inertial terms. In this experiment it is a synthetic additive yaw disturbance independent of the mathematical tail direction. It is not a tilted-axis/gyroscopic or full rigid-body model. The simulation tests component integration, delays, quality gates and allocation behavior; it does not establish real-flight torque accuracy or stability.

The coordinator needs no airframe mass or body inertia. This experiment still supplies explicit synthetic rotor `J`, `kQ` and a torque-to-response gain; ordinary response data may identify only their combined effect rather than each physical quantity uniquely.

Using the same C servo-model family for the plant and predictor is an integration test. Deliberately different parameters expose model mismatch, but do not independently validate the physical actuator model.

## Coordinator modes and validity

All modes run the same tracking and diagnostic pipeline. Baseline allocation applies no reaction compensation. Reaction compensation subtracts the available estimated scalar reaction, converted through an explicit response-per-N·m gain, once from the requested yaw response. A configured software bound clips that correction visibly. Missing reaction causes an explicit degraded allocation; it is not published as a measured zero disturbance.

An optional virtual-tail-vertical coordination experiment sets commanded tail effort to the allocated vertical component divided by the cosine of the predicted current direction, with an explicit cosine guard and effort clipping. It changes only tail effort. For an arbitrary effectiveness matrix this need not preserve collective, yaw or the allocator's optimum. All four post-coordination residuals and clipping/quality flags are reported.

Nominal allocator output, coordinated motor targets, current predicted actuator response and predicted total response are distinct. Current predictions use observed/predicted RPM, not commanded RPM. Total-response diagnostics require a valid reaction estimate. Every unavailable numerical result has an availability flag; invalid zeros must not enter error scores as successful predictions.

**PB0−PC0 remains the sole servo position source.** Real angle calibration remains unavailable. Mathematical direction, normalized effort and response proxies stay distinct from measured mechanical angle, newtons and physical moments.

## Synthetic results — 2026-11-07

The frozen protocol runs two effectiveness maps through three scenarios (matched, telemetry faults and model mismatch) and three modes (baseline, reaction compensation and reaction plus virtual-tail-vertical coordination). Each run has 4,501 ticks at 1 ms, for **18 runs and 81,018 rows**. All commands are valid and bounded, with **zero unconverged allocations**. The motor plant integrates an analytic directional first-order lag with slew limits and a separate RPM cap; the servo plant uses its own C state.

Tests check previous-command timing, source-clock wrap, delayed-capture age, future-suffix causality, hidden-truth isolation, zero-reaction baseline equivalence and independent motor/reaction algebra. Telemetry faults include a gap beyond the observer horizon, outliers and reacquisition. The controller requires a fresh distinct-timestamp pair to requalify acceleration. The synthetic decoder capture timestamp is retained through the software queue; this delay is included in observer age. Packet periods, delay and phases must align with the uniform simulation grid.

Actual-response errors score **every row**, including saturation, faults and unavailable estimates. Predicted-total response has **85.892–99.822% coverage**, reported alongside its masked error. Unknown predictions cannot improve scores by becoming zeros. The full four-axis metrics and all traces are preserved in [the evidence package](validation/tricopter-simulation-2026-11-07/README.md).

The comparisons expose tradeoffs rather than proving a universal improvement. On matched synthetic A, yaw RMS error changes from about 0.32538 baseline to 0.32027 with reaction compensation; adding tail coordination gives 0.32068. On matched synthetic B, reaction compensation slightly reduces yaw error but increases collective error from about 0.4302 to 0.4490. These are response-proxy errors for the complete prescribed waveform, including infeasible requests. They are not flight-performance measurements or tuned-controller results.

The focused suite passes **160 host/regression tests** and **47 Python/native-C tests**, including 15 new C integration tests and ten new Python simulation tests. Strict and fast-math integration tests compile all five C modules and pass. All five also compile for Cortex-M4F with fast-math and warnings treated as errors; that is compile-only evidence, without hardware execution or timing/stack qualification.

## MATLAB and Simulink

MATLAB R2026b compiles the same C simulation bridge through [the MEX adapter](../tools/tricopter/matlab/README.md). A frozen 701-tick fixture includes startup, changing demands, a telemetry outage/recovery, outliers and an infeasible request. All **85 numeric, 27 integer and 47 flag columns** match Python exactly; six malformed transport checks pass. The consumed fixture bytes and every shared native source/header are pinned to their hashes.

The generated [Simulink model](validation/tricopter-simulation-2026-11-07/matlab/tricopter_offline_signal_replay.slx) is explicitly **signal playback and analysis**. It has a fixed discrete tick, interpolation disabled and final-value hold. It is saved, closed, reopened and checked: all 701 samples match exactly. This is not a separate Simulink dynamic plant or independent physics validation. The MEX runner is the MATLAB entry point that actually executes the C simulation.

The evidence package preserves source/compiler/protocol hashes, all 18 compressed traces, plots, MATLAB output and independent replay reproduction. Flight source lists, actuator/configuration paths and the four existing model implementations remain unchanged. No hardware was accessed or firmware flashed. Physical angle/effort characterization, loaded behavior, real telemetry timing, full body/tilted-rotor physics, uncertainty identification and live integration remain open.
