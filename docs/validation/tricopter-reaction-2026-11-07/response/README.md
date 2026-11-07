# Synthetic vertical-axis reaction torque

The actual portable C core passes independent analytic checks of body reaction signs, SI units and software guards. Every coefficient here is an arbitrary **synthetic placeholder**, uncalibrated and unavailable for flight.

Logical motor order is rear/front-right/front-left, with body reaction signs [-1, 1, -1]. Positive body yaw is CCW/nose left. Mechanical RPM is a nonnegative rotation magnitude; acceleration is its signed RPM/s derivative. The core converts both by pi/30.

Placeholder kQ values are [1e-07, 1e-07, 1e-07] N m/(rad/s)^2 and rotor inertias J are [2e-05, 2e-05, 2e-05] kg m². Arbitrary illustrative constants chosen for synthetic testing. Neither kQ nor inertia is estimated from this aircraft, its unloaded bench recordings, current, voltage, RPM or servo ADC data.

For each rotor, body reaction is sign*(kQ*omega² + J*alpha). The aerodynamic and inertial terms are separate components of the shaft reaction; the free-coast case alpha=-kQ*omega²/J cancels them. The inertial impulse independently matches sign*J*deltaOmega.

This scalar formula approximates motor-mount/frame reaction when body angular motion is negligible. Equivalently, it supplies the simplified yaw-equation RHS when the base vehicle inertia includes the rotors’ locked-rotation axial inertia. It is not the total external moment on the aircraft-plus-rotors system. A later full angular-momentum model must account for rotor spin momentum exactly once; do not add this inertial term again alongside -dH_spin/dt. Axes are near-vertical; there is no rear angle, tilted-axis projection, gyroscopic vector model, thrust calculation or allocator.

Software guards are 20000 RPM, 100000 RPM/s, 10000 us age, and 2 N m for every component and sum. They are not measured physical limits. All motors must be seen/fresh/valid and independently acceleration-valid. Missing, stale or unknown-acceleration input clears every numerical output; no partial motor sum is returned.

| Synthetic case | Rows | Maximum total torque magnitude (N m) | Maximum torque oracle error (N m) |
|---|---:|---:|---:|
| steady_equal | 1001 | 0.0394784175 | 1.18e-10 |
| rear_ramp | 2001 | 0.0316043571 | 2.08e-09 |
| front_right_ramp | 2001 | 0.0316043571 | 2.08e-09 |
| rear_braking | 2001 | 0.0628318414 | 5.18e-09 |
| cancellation_snapshot | 1 | 1.8819292e-10 | 2.2e-10 |
| free_coast | 1001 | 1.16423415e-09 | 1.22e-09 |

The cancellation case is one algebraic instant, not a constant-speed trace with nonzero acceleration. Ramp, braking and free-coast trajectories have consistent speed/derivative inputs; the braking derivative changes at its stopping boundary.

All 11 stored invalid-input/provenance/limit checks suppress the complete torque result. Independent Python/native tests also cover speed-squared scaling, acceleration reversal, full motor-tuple permutation, SI conversion, angular-momentum impulse and malformed transports.

Only synthetic trajectories are evaluated. Historical RPM logs have no acceleration ground truth and are not converted into apparent aircraft torque. Servo PB0-PC0 remains the sole measured-position source elsewhere; this model consumes no servo position or angle.

Reproduce from the repository root with `python3 tools/tricopter/reaction_replay.py --output /path/to/output` and `python3 -m unittest discover -s tools/tricopter -p test_reaction_replay.py -v`. `reaction-replay-summary.json`, `native-build.json` and `checksums.sha256` pin protocol, sources, compiler and artifacts. No hardware or active actuator path is used.
