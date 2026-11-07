# Offline dynamic allocation with explicit priorities

This stage adds an opt-in finite-horizon allocator to the synthetic actuator/response simulation. It uses the current command-driven servo predictor and fresh RPM-observer state to estimate reachable response. The controller still needs a configurable virtual-tail effectiveness map, not CAD dimensions, CG or body inertia. All current maps, motor/servo parameters and direction relationships remain synthetic placeholders. No active flight-output path is connected.

## Priority means a separate optimization stage

The four response axes are collective, roll, pitch and yaw. The first three form the protected primary group:

```text
primaryCost = 0.5 * sum_{i=0..2} ((predictedResponse_i - requested_i)/scale_i)^2
yawCost     = 0.5 * ((predictedResponse_3 - requested_3)/scale_3)^2
```

The positive collective/roll/pitch scales configure their relative importance inside the group; smaller scales give an axis more weight. This is a weighted group priority, not a strict ordering between those three individual axes. Yaw is a separate secondary objective. Increasing yaw demand cannot purchase a primary cost above the global primary minimum plus the explicitly configured numerical tolerance.

After evaluating all accepted candidates, selection makes three global passes: find the minimum primary cost; minimize yaw among candidates within its absolute tolerance; minimize normalized effort/PWM movement among candidates admitted by both prior stages. Final ties use deterministic candidate order. The thresholds are anchored to global minima, avoiding cumulative tolerance drift from pairwise comparisons.

## Reachable motor and servo state

At a fixed predicted tail direction `theta`, the map from the logical motor-effort vector `[tail, frontRight, frontLeft]` to four responses is

```text
A(theta) = [B_column2*cos(theta) + B_column3*sin(theta),
            B_column0, B_column1]
```

Three motor inputs cannot generally satisfy four independent targets at that direction. Servo motion changes the map over time. The allocator samples an explicitly configured bounded PWM grid plus current and nominal PWM, with at most 19 candidates. Candidate PWM is restricted to the inverse count-map interval corresponding to the configured virtual-direction limits, which may be narrower than the servo model's full PWM-knot range.

For each candidate, the allocator copies the **current** servo-model state, including delayed command history, queues that candidate at the current timestamp and advances only the copy over the prediction horizon. It consumes no future requests, observations or plant truth. The horizon must exceed the modeled command delay; forecast subdivision obeys the servo model's call-gap limit. Missing directional authority is explicit rather than silently freezing a requested motion.

Each motor's current state is its fresh observed/predicted mechanical RPM. Explicit directional first-order lag and slew limits give minimum and maximum reachable RPM at the horizon under bounded commands. The effort law converts these into a reachable effort box, intersected with configured static effort bounds. No motor command is substituted for an unavailable current state.

For an accepted servo candidate, the protected 3×3 response matrix must be full rank and pass its condition guard. The allocator enumerates 27 lower/free/upper motor faces and solves the bounded primary least-squares problem. A monotone motor-response inversion converts that effort target into bounded float RPM commands. Those **published commands** are forwarded through the model again before scoring primary, yaw and movement costs.

## What the guarantee covers

Priority is checked over the accepted, well-conditioned sampled servo candidates and their primary motor solutions. It is not a continuous global servo-angle optimum, support for rank-deficient primary maps, or optimization over every motor point in a primary-tolerance sublevel set. Skipped candidates and an all-skipped result have explicit status. Quantization/model error is separated from continuous box-solve accuracy in validation.

The endpoint prediction assumes the current request/reaction offset and candidate commands remain held through the horizon. The simulation replans each tick, so that predicted endpoint is not a promise of the subsequently realized trajectory. The scalar reaction remains the previously declared additive yaw surrogate; this allocator does not introduce candidate-dependent future rotor-inertial/gyroscopic torque physics.

Invalid configuration, including an unsuitable horizon, is rejected during validation/reset. When runtime state or candidate quality is unavailable, the offline coordinator reports that dynamic allocation was not used and falls back to bounded nominal commands. Its dynamic priority guarantee is unavailable on those rows. Actual-response metrics still score those rows; coverage and degradation are reported separately.

The dynamic mode is optional and does not combine with the earlier virtual-tail-vertical heuristic. Existing offline modes retain their behavior. **PB0−PC0 remains the sole servo position source**, with only an explicit synthetic counts-to-mathematical-direction map here. Real angle calibration, loaded characterization and live integration remain open.

## Reproducible synthetic evidence

The [2026-11-07 validation package](validation/tricopter-dynamic-2026-11-07/README.md) records two synthetic effectiveness maps, four scenarios (ideal, matched, telemetry faults and model mismatch) and three allocation policies. Its predeclared 40 ms horizon, 1 ms tick and absolute primary/yaw/movement tie bands of `1e-8` were held fixed. All 24 runs, totalling 120,024 rows, are retained. A fresh execution reproduced every decompressed trace byte and every numerical metric.

Independent NumPy SVD/face enumeration, motor forward equations and global selection checks passed for 1,336 accepted candidate problems at 80 sampled times. Dynamic allocation covered 87.622–100% of each run: 38,754 of 40,008 dynamic-policy rows overall, with 1,254 explicit fallback rows. Fault scenarios account for 619 fallback rows per airframe; matched and mismatched runs each have four startup fallback rows. Every row, including fallback, contributes to actual-response metrics.

Static feasibility is classified per airframe using the raw actuator-only request and controller map, with the separate reaction contribution excluded. This does not establish total-response feasibility under reaction, mismatched plant parameters or finite-horizon actuator limits.

| Controller map | Positive yaw request, required direction | Reverse yaw request, required direction | Full four-axis static feasibility |
|---|---:|---:|---|
| synthetic_A | 0.353915 rad | −0.406929 rad | Positive only |
| synthetic_B | 0.583708 rad | −0.602237 rad | Neither |

The configured direction range is −0.4 to 0.55 rad. The unchanged collective/roll/pitch request in both yaw windows is statically feasible on both maps, proved by a zero-direction witness. The separate high-demand window exceeds each map's collective support bound. These classifications accompany the metrics rather than treating every yaw request as feasible.

Actual trajectory results retain tradeoffs. For synthetic_A in the ideal scenario, normalized collective/roll/pitch RMS over the positive-yaw window is **0.013528** with dynamic allocation versus **0.010868** with the soft weighted policy and **0.023688** with the earlier heuristic. In the reverse window the respective values are **0.018642**, **0.069800** and **0.037471**. The positive-window regression remains visible: endpoint priority alone does not guarantee improved transient tracking when commands are replanned each tick. Full results and plots are in the package.

## Validation boundary

- All **177 host C tests** and **59 Python tests** pass. The 14 dynamic-allocator and 18 coordinator tests also pass strict and `-ffast-math` builds with all six C model modules compiled under each variant. Coverage includes invalid input/state, bounds, saturation, delay history, directional motor dynamics, rank/conditioning rejection, priorities and fallback.
- All six C modules compile for Cortex-M4F under `-O3 -ffast-math -Wall -Wextra -Werror`. This is compile evidence only; execution time, stack use and flight-loop suitability remain unqualified.
- Independent MATLAB `lsqlin` checks pass for **88 candidate problems at five snapshots**. Maximum continuous effort difference is `2.04e-8`; maximum continuous primary-cost difference is `3.00e-7`, which includes float32 publication of the continuous effort solution. Recomputed published-command forward costs agree within `1.43e-14`, and global priority selection passes. These oracle comparison tolerances are separate from the runtime tie bands. Exported servo forecasts are inputs to this audit, so it does not independently validate servo physics.
- The original offline ABI and default modes reproduce the frozen **701-row** fixture/traces bit-for-bit, including signed zero. The existing MATLAB MEX and reopened Simulink signal-replay model retain exact parity for those original outputs. That compatibility check does not execute the new dynamic API in Simulink.

No active mixer, PID, scheduler, ADC, DShot or actuator output path changes. Physical angle calibration, loaded servo/motor identification, body dynamics, candidate-dependent future reaction torque and hardware timing remain subsequent work.
