# Offline tricopter model replay

This harness fits and replays the portable C model in `src/main/flight/tricopter_servo_model.c`. Python handles input validation, deterministic optimization and reporting; each prediction calls the same C implementation. It sends no hardware commands and is not linked into the active controller.

Position is **signed TRI_GROUND `debug[3]`, PB0−PC0, in ADC counts**. Velocity is counts/s. Angle calibration remains unavailable. PC0 senses the servo power ground, not an independently measured potentiometer-low terminal. Raw PB0/PC0 enter only diagnostics and validity checks; there is no raw-position fallback.

From the repository root, with a C11 compiler, Python, NumPy and Matplotlib:

```sh
python3 -m unittest discover -s tools/tricopter -p 'test_servo_replay.py' -v
python3 tools/tricopter/servo_replay.py --prepare-inputs --output-dir .local-artifacts/tricopter/servo-replay
```

The input-preparation command uses the existing pinned adapted official decoder at its default local path. Supply `--decoder /path/to/blackbox_decode` when necessary; its binary hash must match the archived provenance. It verifies the archived raw hashes, saves decoded CSV/headers and decoder logs, and writes `inputs/input-provenance.json`. No network operation is needed. Existing decoded CSV inputs require that provenance:

```sh
python3 tools/tricopter/servo_replay.py --provenance /path/to/input-provenance.json --ground-csv /path/to/servo-ground.01.csv --hysteresis-csv /path/to/servo-hysteresis.01.csv --output-dir /path/to/output
```

Both CSV overrides are optional when the paths in the provenance remain valid. To replay previously fitted parameters without optimization, add `--replay-report /path/to/servo-model-report.json`; the C source hashes must match.

`servo_replay_protocol.json` freezes complete-unit splits before fitting: ground cycles0/1 and approach repetitions0/1 train; their final complete cycle/repetition validates. Staircase cycle0 trains and cycle1 validates. The optimizer budget, bounds and engineering-prior starts are fixed in `servo_fit_settings.json`. All recordings were previously explored for characterization, so this is a holdout from parameter fitting, not a pristine prospective experiment.

Each complete unit receives exactly one initialization from its first fresh corrected sample and prior command-only history after a documented pre-step hold. That initial sample is excluded from scoring. Every subsequent timestamp advances C state even when ADC data is stale; measurements never correct predicted state. Invalid model histories fail scoring instead of improving results through a dropped suffix. No target filtering or interpolation is performed.

Three predefined baselines are independently fitted on training data only: instantaneous three-knot command map, map plus common lag, and the full play/delay/directional-lag/slew model. Reports contain per-unit and equally weighted aggregate held-out errors, transient/plateau subsets, parameter/source/tool/input provenance, compressed replay samples and held-out plots. Fitted lag/play/slew parameters are a joint description of an unloaded electrical proxy; they are not separately identified mechanical constants or flight control limits.

Frozen-parameter replay requires matching protocol/settings hashes and bodies, as well as matching C sources. Fixed bridge map knots and command-gap metadata are checked explicitly. A changed split cannot silently relabel previously trained parameters as a new holdout.

## RPM observer replay

The independent per-motor C observer is in `src/main/flight/tricopter_rpm_observer.[ch]`. `rpm_native.py` builds and executes it through the batch C bridge; Python does not implement a second observer. Exact synthetic tests exercise source identity, causality, repeated queries, missing data and transport validation:

```sh
python3 -m unittest discover -s tools/tricopter -p 'test_*replay.py' -v
CC=gcc CXX=g++ src/test/run_tricopter_checks.sh
python3 tools/tricopter/rpm_replay.py \
  --decoder .local-artifacts/tricopter/m1-bench-2026-11-06/tools/blackbox-tools/obj/blackbox_decode \
  --output .local-artifacts/tricopter/rpm-replay
```

The RPM command verifies and decodes the two archived motor records, runs all three motors at phase allowances 0/250/500 µs, and writes a report, provenance, compressed per-row replay and plots. `rpm_replay_settings.json` freezes illustrative filter settings before evaluation; none are fitted physical coefficients.

These historical `TRI_SERVO` records preserve per-motor quantized ages but no exact observation sequences or timestamps. The logger's within-loop phase is also unmeasured. Interval overlap supplies only a conservative cache-identity/timestamp proxy; phase allowances are sensitivity assumptions. Equal-valued or zero-RPM observations can be genuinely new, while cached zeros preserve age. Real replay checks compatibility/freshness behavior, and has no acceleration ground truth. The historical raw PB0 field is unused for position. Exact-timing synthetic cases separately validate observer mathematics.

Future live integration requires a coherent RPM/identity/timestamp adapter and explicit tracking support where needed; the current firmware only tracks those metadata in `TRI_SERVO`/`TRI_GROUND`. No code here changes that policy or invokes actuator, scheduler, serial or flash operations. See [full results and limits](../../docs/tricopter_offline_models.md).

## Synthetic reaction-torque model

`reaction_native.py` executes `src/main/flight/tricopter_reaction_torque.[ch]` through the C batch bridge. The protocol in `reaction_synthetic_protocol.json` supplies explicit arbitrary `kQ` and rotor-inertia placeholders, software guards and logical motor signs. It rejects calibrated/flight provenance and preserves the unit convention. No aircraft logs are fitted or converted into measured torque.

```sh
python3 -m unittest discover -s tools/tricopter -p 'test_reaction_replay.py' -v
python3 tools/tricopter/reaction_replay.py --output .local-artifacts/tricopter/reaction-replay
```

The command saves six independent analytic cases, eleven all-output-suppression fault checks, source/compiler/protocol fingerprints, compressed replay data and component plots. Tests cover mechanical RPM conversion, the odd-rotor steady residual, aerodynamic speed-squared scaling, acceleration/deceleration signs, tuple permutations, coast cancellation and inertial angular impulse.

The model is fixed/negligible-body-motion near-vertical shaft reaction, or the simplified yaw RHS when locked-rotor inertia is included consistently. It is not total external aircraft-plus-rotor torque. Every motor needs fresh valid speed and explicitly valid acceleration; initialized observer zeros do not establish acceleration. All numerical outputs are cleared on any failed input/bound, and calibration/flight-use flags always remain false. No active flight-control source or caller is added. See [equations, inertia convention and validation](../../docs/tricopter_reaction_torque.md).

## Configurable offline thrust-vector allocation

`allocator_native.py` executes `src/main/flight/tricopter_allocator.[ch]`. The shared virtual-tail solver accepts an explicit 4×4 control-effectiveness map, effort limits, tail vector direction bounds and response scales. It requires no CAD dimensions, CG, mass, inertia or absolute thrust calibration. A separate SI adapter builds the map from CG-relative positions and supplies thrust-to-speed conversion using explicit `kT` coefficients.

```sh
python3 -m unittest discover -s tools/tricopter -p 'test_allocator_replay.py' -v
python3 tools/tricopter/allocator_replay.py --output .local-artifacts/tricopter/allocator-replay
```

The frozen protocol contains synthetic direct-map and geometric examples. The runner checks feasible reconstruction, effort/thrust and direction/angle bounds, saturation, independent convex objective-gap certificates, unit rescaling and a held-angle forward evaluation. Static request sweeps are not dynamic servo simulations. Direct-map proxy units remain distinct from N/N·m/RPM, and mathematical vector direction is not automatically calibrated mechanical angle.

The direct map must use the solver's virtual front/tail-vector coordinates; a local three-motor-plus-servo-PWM Jacobian has different constraints and cannot be substituted. No airframe coefficients are fitted here. Calibration and flight-use flags remain false; PB0−PC0 stays the sole servo position measurement and no count/PWM-to-angle conversion is added. See [configuration, equations and validation](../../docs/tricopter_allocator.md).

## Integrated synthetic simulation

`simulation_native.py` builds the actual servo, RPM, reaction and allocation cores together with `tricopter_simulation.[ch]`. The separate C plant has synthetic motor lag/slew, its own servo state and a configurable response map. The coordinator sees only its initial corrected count and causal RPM packets. It uses no CAD, CG or body-inertia data, and never consumes later plant truth.

```sh
python3 -m unittest discover -s tools/tricopter -p 'test_simulation_replay.py' -v
python3 tools/tricopter/simulation_replay.py --output .local-artifacts/tricopter/simulation-replay
```

The frozen protocol exercises multiple synthetic maps, baseline/reaction/virtual-tail-vertical coordination modes, motor/servo mismatch, saturation and telemetry faults. Actual response errors include all rows; prediction errors include their availability coverage. Unavailable reaction remains explicit. Maps exclude the separately added scalar yaw disturbance, which is a declared surrogate independent of virtual direction, not a full tilted-rotor or flight-dynamics model.

The runner exports a source-pinned MATLAB parity fixture. [The MATLAB adapter](matlab/README.md) compiles the same C bridge as MEX and checks numeric/status/quality outputs; its optional Simulink model is discrete signal playback only. All mappings/coefficients remain synthetic, PB0−PC0 remains the sole position source, and no active firmware output/configuration path is connected. See [architecture, results and limits](../../docs/tricopter_simulation.md).

## Dynamic allocation with protected response priorities

`dynamic_native.py` links the actual finite-horizon allocator and opt-in coordinator mode to the shared synthetic plant. The controller forecasts copied servo state and reachable motor effort, minimizes configured collective/roll/pitch group error, then yaw and movement in separate stages. The old simulation ABI and modes retain their behavior.

```sh
python3 -m unittest discover -s tools/tricopter -p 'test_dynamic_replay.py' -v
python3 tools/tricopter/dynamic_replay.py --output .local-artifacts/tricopter/dynamic-replay
```

The pinned protocol compares soft-weighted, vertical-heuristic and dynamic policies across synthetic airframes, faults and mismatch. Whole-wave metrics and predeclared transition windows are separate; pure-map static feasibility is checked per airframe. All actual rows are scored; dynamic use/fallback coverage is explicit. The guarantee concerns predicted horizon response over accepted sampled candidates, not every realized transient or a continuous servo optimum.

Full-time traces retain compact dynamic summaries. Detailed candidate tables are archived only at predeclared audit times, with independent NumPy face/box certificates and an optional [MATLAB `lsqlin` cross-check](matlab/README.md). See [the algorithm, limits and results](../../docs/tricopter_dynamic_allocator.md).
