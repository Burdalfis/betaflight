# Offline configurable allocation validation — 2026-11-07

See [the model and airframe configuration contract](../../tricopter_allocator.md) and [tool usage](../../../tools/tricopter/README.md). This package validates the actual portable C allocator with explicit **synthetic placeholder maps, geometry, coefficients and limits**. It identifies no aircraft parameters and enables no flight-control outputs.

The primary virtual-tail interface requires an effectiveness matrix, effort/direction bounds and response scales. It needs no CAD dimensions, CG, mass, inertia or `kT`. Its units are declared effort/response proxies; direction is mathematical until characterized. The optional SI adapter separately checks CG-relative `r × F` geometry and explicit thrust-to-speed coefficients. A local motor/servo-PWM Jacobian cannot be substituted for the virtual-tail matrix without changing the model and constraints.

`response/` contains the frozen protocol inside `allocator-replay-summary.json`, source/compiler/script fingerprints, compressed case/sweep data, two plots and numerical certificates. Five direct-map cases, eleven SI cases and 81 static yaw requests per interface give **178 valid, converged allocations**. Cases cover both signs, zero/low effort, exact reconstruction, both direction bounds, thrust saturation, negative collective and mixed infeasible requests. All independent objective-gap bounds meet the dimensionless `1e-4` cutoff; maxima are 7.993e-6 direct and 5.015e-5 SI. The separate C gradient-mapping tolerance is `1e-5` in inverse effort units.

The independent oracle checks geometry through vector cross products and verifies constrained solutions through primal feasibility and a linear support-function gap. Tiny publication roundoff is projected to a nearby feasible certificate point and its correction is reported; this is a numerical check, not an interval-arithmetic proof. Saturated wrench error is distinct from solver nonconvergence. Deliberate one-iteration exhaustion, singular maps and unknown requested reaction are stored explicitly.

The noiseless synthetic identification example recovers a rank-four map from 64 training excitations and checks 24 held-out requests. It demonstrates a characterization workflow without dimensions; it does not validate identification from real closed-loop logs. The held-angle example uses the same thrusts at two independently specified angles to show that allocated target angle is not achieved angle. Neither example converts PB0−PC0 or PWM into angle.

`capture/` preserves **145 host/regression tests**, **37 Python/native-C tests**, strict and fast-math C checks, Cortex-M4F compile settings/source hashes, unchanged active-path fingerprints and an independent replay reproduction. An additional seeded **1,024-request stress check** passes bounds/convergence and independent projection/gap checks, with maximum gap 1.465e-5. Its source and exact seed are preserved in `capture/stress_validate.py` and `stress-validation.json`.

From the repository root, with a C11 compiler, Python, NumPy and Matplotlib:

```sh
CC=gcc CXX=g++ src/test/run_tricopter_checks.sh
python3 -m unittest discover -s tools/tricopter -p 'test_*replay.py' -v
python3 tools/tricopter/allocator_replay.py --output .local-artifacts/tricopter/allocator-reproduction
python3 docs/validation/tricopter-allocator-2026-11-07/capture/stress_validate.py
sha256sum --check docs/validation/tricopter-allocator-2026-11-07/checksums.sha256
```

The strict/fast-math commands are recorded in `capture/validation.json`; Cortex settings are in `execution-validation.json`. Cortex validation is compile-only. No hardware, flash, receiver/arming configuration, scheduler, PID, actuator or ADC path was changed. PB0−PC0 remains the sole position source and angle calibration remains unavailable. Real effectiveness/effort mapping, operating-region validation, physical limits, loaded dynamics and execution cost remain integration gates.
