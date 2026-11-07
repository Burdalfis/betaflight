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
