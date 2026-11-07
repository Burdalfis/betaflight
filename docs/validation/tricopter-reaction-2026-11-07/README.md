# Synthetic reaction-torque validation — 2026-11-07

See [equations, units, signs and inertia bookkeeping](../../tricopter_reaction_torque.md) and [tool usage](../../../tools/tricopter/README.md). This package validates the real portable C algebra with explicit **synthetic placeholder coefficients**. Nothing here identifies aircraft torque or enables control output. PB0−PC0 remains the sole measured servo position source; this model uses no angle or servo input.

`response/` contains the frozen protocol and coefficients inside `reaction-replay-summary.json`, native build/source fingerprints, six synthetic cases, eleven fault checks, compressed per-row replay and two component plots. The example values are `kQ=1e-7 N*m/(rad/s)^2`, `J=2e-5 kg*m^2` per rotor, with reaction signs `[-1,+1,-1]`. Every valid evaluation still reports `coefficientsCalibrated=false` and `forFlight=false`.

The cases include steady equal speeds, rear/front-right ramps, rear braking, one algebraic cancellation snapshot and an analytically consistent ideal free-coast trajectory. Rotor acceleration is specified independently as known synthetic input. No raw RPM differencing or assumption that an initialized observer's zero is known acceleration is used. The oracle is independent double-precision test algebra; every reported model prediction executes the C core.

Free-coast cancellation and the inertial impulse/rotor angular-momentum balance catch sign or double-counting mistakes. The formula approximates fixed-body mount reaction, or the moving-body yaw RHS with locked-rotor inertia included in the base inertia; it is not total external torque of the vehicle plus rotors. Tilt projection, body-rate gyroscopic coupling, thrust, allocation and parameter identification are outside scope.

`capture/` preserves the combined **129 host/regression tests**, **28 Python/native-C tests**, strict/fast-math C checks, Cortex-M4F compile settings/source fingerprints and independent replay reproduction. Numeric agreement around nanonewton-metres reflects floating-point equation agreement only; it is not physical measurement precision. Cancellation is plotted at component scale with numerical-residual annotation.

With C11 compiler, Python, NumPy and Matplotlib, run from the repository root:

```sh
CC=gcc CXX=g++ src/test/run_tricopter_checks.sh
python3 -m unittest discover -s tools/tricopter -p 'test_*replay.py' -v
python3 tools/tricopter/reaction_replay.py --output .local-artifacts/tricopter/reaction-reproduction
```

All inputs are generated from the explicit protocol; no drone, decoder or network access is needed. Sources and tool versions are pinned in the captured provenance. Cortex-M4F validation is compile-only; execution cost and live integration have not been measured. No active firmware source-list/caller change, hardware operation, configuration change or flash occurred.

Verify this package from the repository root:

```sh
sha256sum --check docs/validation/tricopter-reaction-2026-11-07/checksums.sha256
```
