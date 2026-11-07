# Offline reaction-torque model

This stage implements the control plan's first near-vertical yaw model as a stateless, allocation-free C function. All coefficients are explicit **synthetic placeholders**. It has no actuator, scheduler, configuration, ADC, DShot or Blackbox side effects and is not included in the active firmware source list.

## Equations, units and signs

Inputs are nonnegative mechanical rotation magnitudes in RPM and signed changes of those magnitudes in RPM/s. Electrical RPM must already have been converted at the input boundary; no motor-pole factor belongs inside this model.

```text
omega_i = mechanicalRPM_i * pi/30               [rad/s]
alpha_i = accelerationRPMperSecond_i * pi/30    [rad/s^2]

aerodynamic_i = reactionSign_i * kQ_i * omega_i^2
inertial_i    = reactionSign_i * J_i * alpha_i
reaction_i    = aerodynamic_i + inertial_i
reaction_z    = sum(reaction_i)                  [N*m]
```

`kQ` has units N·m/(rad/s)² and `J` has units kg·m². The quadratic drag law with constant `kQ` is an explicit simplifying assumption, not an identified propeller model. Positive body yaw is CCW/nose-left with body z up. The logical mapping is:

| Logical motor | Physical position | Rotor spin viewed from above | Reaction sign |
|---|---|---|---:|
| 0 | Rear | CCW | −1 |
| 1 | Front right | CW | +1 |
| 2 | Front left | CCW | −1 |

Equal positive speeds on all three motors **do not cancel**: two CCW rotors oppose one CW rotor. Flipping acceleration changes the inertial contribution's sign; aerodynamic drag retains its sign for the same speed. Strong braking can therefore reverse the net frame reaction.

## Mechanical interpretation and inertia bookkeeping

For a fixed body and rotor spin sign `sigma_i`, the rotor balance is `Mshaft_i − Q_i = J_i*alpha_i`; reaction on the frame is `−sigma_i*Mshaft_i`. Thus `reactionSign_i = −sigma_i`. The fixed-axis relation between torque, angular acceleration and angular-momentum change follows [MIT's angular-momentum treatment](https://web.mit.edu/8.01x/Fall01/expt/expt_am.pdf).

During body yaw motion, RPM is rotor-relative-to-stator speed. Absolute rotor angular acceleration also includes body angular acceleration. The implemented term can be used as the simplified yaw equation's right-hand-side contribution when `Izz_locked` is the inertia of the whole aircraft with rotors locked to the body:

```text
Izz_locked * bodyYawAcceleration
    = otherYawTorque + sum(reactionSign_i * (kQ_i*omega_i^2 + J_i*alpha_i))
```

Equivalently, it approximates physical mount reaction for fixed/negligible body motion. It is **not total external torque on the aircraft plus rotors**. A later full angular-momentum formulation must not add a separate rotor `−dH/dt` term after using this inertial contribution. Tilt projections, changing shaft directions, body-rate gyroscopic terms, thrust-vector moments, rigid-body cross terms and body-inertia identification remain outside this first model.

## Explicit placeholder coefficients

The synthetic protocol supplies every coefficient; zero-initialized or unspecified provenance is rejected. Its illustrative values are:

| Quantity | Per-motor synthetic value | Provenance |
|---|---:|---|
| `kQ` | 1e−7 N·m/(rad/s)² | Arbitrary test constant |
| `J` | 2e−5 kg·m² | Arbitrary test constant |
| Reaction signs | [−1, +1, −1] | Verified aircraft spin convention |

The 20000 RPM, 100000 RPM/s, 10 ms age and 2 N·m limits in the protocol are software test guards. They are not measured operating limits. No coefficient is fitted from RPM, servo ADC, current, voltage or aircraft recordings. Both `coefficientsCalibrated` and `forFlight` remain false even when equation evaluation is valid.

## Input quality and numerical behavior

Every motor must be seen, fresh and valid, within the configured age/range limits, with acceleration validity explicitly established. An RPM observer's initialized, reacquired or coalesced zero derivative does not establish known acceleration. The model never substitutes zero for missing acceleration and never silently publishes a partial sum. Any failed input or component/sum bound clears every numerical output and returns invalid status plus the affected motor mask. Those zeros are invalid results, not a measured zero torque.

Public inputs/outputs use binary32 floats. Finite input validation uses representation checks that survive fast-math; products and sums use binary64 intermediates and are bounded before float output conversion. Opposing large terms cannot hide a component-bound violation through cancellation. Coefficient source, numerical equation validity and calibration/flight-use status remain separate.

PB0−PC0 remains the sole servo position measurement. This model has no servo-angle input or counts-to-angle conversion, so angle calibration stays unavailable.

## Synthetic validation — 2026-11-07

The synthetic runner executes the same C core used by the host tests. A separate double-precision equation oracle checks six frozen cases: equal-speed steady rotors, rear/front-right acceleration ramps, hard rear braking, a single-instant cancellation snapshot and a kinematically consistent ideal free-coast trajectory. No aircraft recording is converted into an apparent torque measurement.

With the arbitrary coefficients above, equal 6000 RPM gives a summed reaction of about **−0.0394784 N·m**, showing the odd-rotor residual. Rear and front-right acceleration produce opposite-signed inertial terms. Hard rear braking reverses the net reaction. In free coast, `J*alpha = −kQ*omega^2`, so the modeled shaft reaction cancels; integrating inertial torque also agrees with `reactionSign*J*deltaOmega`.

Across these cases, C outputs agree with the independent oracle to within 5.18e−9 N·m; free-coast residual is at most 1.16e−9 N·m. The largest integrated angular-impulse difference is 2.01e−9 N·m·s. These quantify floating-point agreement for synthetic equations, not physical torque accuracy or coefficient identification.

Eleven stored fault cases verify complete suppression for missing/stale/invalid RPM, unknown acceleration, invalid numbers, software bounds and unspecified coefficient provenance. Eighteen C tests additionally cover per-motor sign/permutation, isolated terms, speed-squared and coefficient scaling, freshness boundaries, extreme finite inputs and cancellation that must not hide a component violation. Strict and fast-math builds pass. The full focused suite passes **129 host/regression tests** and **28 Python/native-C harness tests**; Cortex-M4F warning-as-error compilation passes too.

See [reproducible synthetic results, protocol and plots](validation/tricopter-reaction-2026-11-07/README.md) and [tool usage](../tools/tricopter/README.md). The prior servo/RPM validation packages remain unchanged. This step completes offline equation validation while coefficient identification, angle calibration, loaded measurements and live integration remain open.
