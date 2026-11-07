# Offline tricopter control-effectiveness allocator

This stage adds a portable C allocation experiment with explicit **synthetic placeholder maps and actuator limits**. The primary interface accepts a control-effectiveness matrix; an optional geometry adapter constructs one in SI units. Basic configuration and characterization can therefore replace detailed airframe geometry. The code is separate from the active firmware source list and has no actuator, configuration or hardware I/O. Flight-control outputs remain unchanged. It solves static targets; it does not make the servo or motors respond instantaneously.

## Configuring an airframe without CAD

The shared solver operates on four virtual actuator coordinates: front-right effort, front-left effort, tail vertical effort and tail rightward effort. The tail pair shares an effort unit, so its magnitude and direction define a bounded vector. A supplied 4×4 matrix maps these coordinates to collective, roll, pitch and yaw response. Its coefficients can absorb thrust ratios, moment arms and rotational inertia. Neither CG coordinates nor mass, inertia or absolute thrust coefficients are required by this direct interface.

The response units belong to the supplied map. For example, collective may be a dimensionless thrust proxy while roll/pitch/yaw use angular acceleration per normalized effort. A virtual collective row is also possible, but then it is a command-allocation convention rather than a measurement of vertical force. Direct-map effort and response must not be labeled newtons, N·m or RPM without the corresponding calibration. The separate SI adapter retains those physical units explicitly.

| C interface | Airframe inputs | Output units |
|---|---|---|
| `tricopterAllocatorVirtualAllocate` / `VirtualForward` | Effectiveness matrix, effort/direction bounds, response scales and solver settings | Caller-defined effort and response proxies; mathematical direction |
| `tricopterAllocatorAllocate` / `Forward` | CG-relative motor positions, thrust/angle bounds, `kT`, wrench scales and solver settings | N, N·m, rad, rad/s and mechanical RPM |

Both interfaces require explicit synthetic-placeholder provenance in this offline version. No default airframe map or calibrated source is silently selected.

The minimum configuration is output identity, effort limits, a tail vector angle range, a full-rank effectiveness map, response scales and numerical solver limits. Future characterization must establish motor effort versus command/RPM and the servo's physical angle sign/curve/range, then estimate the independent motor/vector response coefficients. Logs can supply rough gains and signs, but closed-loop correlations, actuator delay, reaction torque, voltage and operating-point changes can confound identification. No aircraft map is fitted from the current logs in this stage.

This supports conventional three-motor tricopters with one tail tilt servo, including asymmetric layouts. It is not an arbitrary-actuator aircraft allocator. A constant map also assumes approximately separable effort and tail-direction effects; unusual pivots, strong nonlinearities or changing operating conditions may require a different forward model or scheduled maps. The explicit map keeps those assumptions reviewable rather than hiding them in aircraft-specific constants.

The direct map's columns must describe those same virtual coordinates, with the tail pair proportional to `tailEffort*cos(direction)` and `tailEffort*sin(direction)`. Direction is a mathematical effort coordinate until its mechanical meaning is characterized. An ordinary local Jacobian from three motor commands plus servo PWM is a different model: it needs a trim term and four interval constraints, not this circular sector. It cannot be substituted without conversion and validation.

## Optional geometry adapter, units and forward model

The body frame is right handed: x forward, y left, z up. Positive yaw is counterclockwise viewed from above. Logical motor 0 is the tail, 1 is front right and 2 is front left. Each position `r_i=(x_i,y_i,z_i)` is in metres **relative to the CG**. Positive tail angle points thrust rightward, along negative y.

Let `R` and `L` be the front-right and front-left thrusts in newtons. Write the tail force as `(0,-h,v)`, where `v` is its upward component and `h` is its rightward component. Then:

```text
u = [R, L, v, h]
tail thrust = hypot(v,h)
tail angle  = atan2(h,v)

force = [0, -h, R+L+v]
moment = sum(r_i cross F_i) + [0,0,knownReactionYaw]

            [ 1      1       1       0       ]
A =         [ y_1    y_2     y_0     z_0     ]
            [-x_1   -x_2    -x_0     0       ]
            [ 0      0       0      -x_0     ]

achieved [Fz,Mx,My,Mz] = A*u + [0,0,0,knownReactionYaw]
residual = achieved - requested
```

This includes roll coupling from a tail mount above/below the CG and pitch/roll contributions from all three thrusts. Front motor z offsets do not contribute for purely vertical thrust. It assumes fixed thrust application points and a tail thrust axis rotating in the body yz plane. A moving pivot, tilted front mounts, variable moment arms, aerodynamic interference and gyroscopic terms require a later model.

Only vertical force and three moments are allocation targets. The lateral force is reported explicitly; it is not independently controllable in this model. The optional known yaw reaction is an externally supplied, frozen disturbance for each solve. No RPM observer or reaction-torque feedback loop is connected. In particular, this does not silently extend the previous near-vertical reaction equations to a tilted rotor.

Each explicit coefficient `kT_i` converts thrust through `T_i=kT_i*omega_i^2`, with omega in mechanical rad/s. The returned speeds are algebraic synthetic targets, not DShot commands, achieved RPM, voltage compensation or a motor-response model.

## Constraints and saturation policy

Front efforts lie between zero and their configured maxima. The tail vector lies in a circular sector: its magnitude is bounded, and its direction is between explicit lower and upper bounds containing zero, strictly inside ±90 degrees. In the SI adapter these become thrust and angle limits. They are software experiment limits, not measured aircraft limits. Minimum effort is zero; a nonzero minimum tail magnitude would make this feasible set nonconvex and is outside this implementation.

An exactly feasible inverse solution reconstructs the requested wrench. For an infeasible request, the allocator minimizes the dimensionless weighted error

```text
f(u) = 0.5 * sum_j(((A*u + disturbance - requested)_j / scale_j)^2)
```

The four positive scales use their corresponding response units; in the SI adapter these are N, N·m, N·m and N·m. Smaller scales give an axis greater relative weight. This is a simultaneous weighted compromise, not a strict hierarchy that guarantees collective or attitude priority. Saturation can trade an error on one axis against another; achieved response and residual must be used together.

The bounded projected-gradient solver uses the normalized matrix and a step equal to the reciprocal of its squared Frobenius norm. Front projection clips thrust; tail projection finds the nearest point in the angle wedge and then caps its radius. The feasible set is convex. A finite iteration budget and a separate convergence diagnostic prevent an unfinished solve from being represented as a proven optimum.

The Python validation independently forms the forward model with vector cross products. For constrained results it also evaluates a first-order objective-gap bound by minimizing the gradient's linear form over the feasible set. The bound follows directly from the differentiable convex inequality in [Boyd and Vandenberghe, section 4.2.3](https://www.stanford.edu/~boyd/cvxbook/bv_cvxbook.pdf). This checks the optimization separately from the core's own stopping flag.

## Numerical contract

Inputs and public outputs are binary32; matrix inversion, projection and optimization arithmetic use binary64. Finite-input checks remain effective under fast-math. Explicit magnitude, rank and weighted condition-number guards reject unsupported maps/configurations. They are numerical guards, not measured airframe limits.

The returned effort magnitudes and direction are canonical. Controls and achieved response are reconstructed from those published values, with ordinary float rounding. Invalid input/configuration clears all numerical outputs and returns `valid=false`. A bounded solve that misses the declared convergence tolerance retains feasible finite controls with `valid=true`, `converged=false` and `NOT_CONVERGED` status. Neither validity nor a small wrench residual alone proves an optimal constrained allocation.

`exact` records that the unconstrained inverse was feasible before float publication, using a `1e-10 * max(1,maxAbsControl)` feasibility tolerance. It does not promise bit-exact response reconstruction or override the returned convergence check. The iteration loop checks the published canonical solution before stopping; SI reaction subtraction remains in binary64 through the shared solve.

The independent certificate projects only tiny float-publication roundoff onto the feasible set, reports that correction and rejects larger violations. Its objective-gap bound applies to that nearby feasible point; it is a numerical cross-check, not an interval-arithmetic proof. An infeasible target can have a large residual while its constrained solution has a small certified objective gap.

## Actual angle and measurement boundary

The forward evaluator also accepts independently specified thrusts and a tail angle. Evaluating the allocated thrusts at a held previous angle exposes the force/moment error while the servo is moving. It is not a second allocation that can produce four arbitrary wrench components with only three fixed-angle thrusts. Actuator delay, slew, closed-loop tracking and coordinated timing remain future work.

**PB0−PC0 remains the sole servo position measurement.** No ADC counts or PWM values are converted into physical angle. All directions/angles in this experiment are synthetic inputs. The direct-map route needs characterized effectiveness, effort/vector mapping, loaded response and physical limits before integration. CG geometry and absolute thrust coefficients are additional requirements only for the optional SI interpretation.

## Synthetic validation — 2026-11-07

The frozen protocol contains a direct map with no geometry/`kT` dependency and a separate asymmetric SI example with nonzero CG offsets. All maps, positions, coefficients, effort/thrust limits and direction/angle limits are arbitrary placeholders. There is no aircraft fit or calibrated provenance. The example ±direction limits are not inferred from the servo's PWM endpoints.

Five direct-map and eleven SI cases, plus 81 static yaw requests for each interface, give **178 valid, converged allocations**. Cases cover feasible reconstruction, both yaw signs, zero/low effort, negative collective, front thrust saturation, both tail direction limits and mixed infeasible demands. Independent convex objective-gap bounds are at most **7.993e-6** for direct mode and **5.015e-5** for the SI adapter, below the declared dimensionless `1e-4` cutoff. The distinct gradient-mapping tolerance is `1e-5` in inverse effort units. These are floating-point checks of the chosen optimization problem, not physical accuracy claims.

A further seeded **1,024-request stress check** covers an identity map with an independent Euclidean-projection oracle and the coupled direct map. All returned controls are bounded and all solves converge; the largest independent gap is **1.465e-5**. Deliberate one-iteration exhaustion retains a feasible result with nonconvergence status. Singular/ill-conditioned maps, invalid numbers/configurations, unknown requested reaction and out-of-bounds forward inputs are rejected separately.

Unit tests rescale response units and effort units without changing the represented allocation. A noiseless synthetic characterization demonstration estimates a full-rank map from 64 training excitations, then checks 24 held-out requests through the C allocator. Maximum held-out response error is 5.77e-8 proxy units. This demonstrates the algebraic workflow without CAD; it does not establish identifiability or accuracy on closed-loop aircraft logs.

The focused suite passes **145 host/regression tests**, including 16 new C allocator tests, and **37 Python/native-C tests**, including nine new allocator tests. Separate strict and fast-math C runs pass; Cortex-M4F warning-as-error compilation passes. Cortex validation is compile-only: CPU cost, stack use, scheduling and actual actuator tracking have not been qualified.

See [the reproducible evidence package](validation/tricopter-allocator-2026-11-07/README.md), [the direct-map saturation plot](validation/tricopter-allocator-2026-11-07/response/direct-effectiveness-sweep.png) and [tool commands](../tools/tricopter/README.md). The active build source list and existing actuator/measurement paths are unchanged. No firmware was flashed, configuration changed or hardware accessed.
