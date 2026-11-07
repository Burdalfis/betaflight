# Offline servo model and RPM observer — 2026-11-07

See [the model contract, results and remaining gates](../../tricopter_offline_models.md) and [tool usage](../../../tools/tricopter/README.md). This stage uses saved inputs and synthetic signals only. No aircraft access, configuration change, flash operation or active-controller integration occurred. Physical angle calibration is unavailable.

## Servo results

Position is signed PB0−PC0, with raw inputs used only for diagnostics/quality checks. The same portable C model performs optimization and replay. Five complete units train and three complete units validate; one corrected sample initializes each unit, and subsequent measured samples never correct predictions. Previously explored recordings make this a frozen replay holdout rather than a prospective experiment.

Held-out equal-unit RMS error is 52.531 counts for an instantaneous map, 14.156 for a common-lag model and **4.765 for the play/delay/directional-lag/slew model**. Full-model MAE is 3.559 counts. `servo/` preserves all baseline parameters/metrics, frozen protocol, parent fitted-parameter report, native source/build fingerprints, sample-level compressed replay and plots. The final report adds defensive replay-integrity guards while reusing exactly the original fitted parameters. Frozen parameters and evaluation reproduce exactly in this environment.

The immutable raw inputs remain in the [paired-ground](../tricopter-ground-2026-11-06/README.md) and [corrected-hysteresis](../tricopter-hysteresis-2026-11-06/README.md) packages. Raw/decoded file hashes and original workspace paths are recorded in `capture/servo-input-provenance.json`; full decoded CSVs remain local and are reproducible from those raw files.

## RPM results

The C observer tracks mechanical RPM and RPM/s with explicit source identity, observation time, freshness, bounded prediction and reacquisition. Exact synthetic tests validate mathematics; the two archived low-power recordings validate compatibility and stale handling with 0/250/500 µs phase assumptions. Approximate packet identities/times do not establish physical acceleration accuracy.

At the illustrative 250 µs allowance, the aborted test's motor0 age reaches about 1.336 s; its cached zero cannot refresh the observer. The successful test exposes about 122 ms startup gaps on each channel. No observation whose conservative native age exceeds the 100 ms prediction horizon is accepted in any of the six sensitivity cases. `rpm/` preserves all cases, plots, compressed sample-level replay, frozen settings and source/build/input provenance. Its report states the assumptions explicitly.

Raw motor inputs and capture provenance are preserved separately in [the RPM archive](../tricopter-rpm-2026-11-06/README.md). Those historical recordings contain raw PB0 diagnostics; they are never used as servo position or included in servo fitting.

## Verification and reproduction

`capture/` records 111 passing host/regression tests, 19 Python/native-C harness tests, strict/fast-math checks, Cortex-M4F compile settings and independent package replay checks. Binary32 finite guards preserve NaN/Inf rejection under fast-math. Portable cores are not added to the active firmware source list.

Use the pinned adapted official decoder recorded in the input packages. From the repository root, with C11 compiler, Python, NumPy and Matplotlib:

```sh
CC=gcc CXX=g++ src/test/run_tricopter_checks.sh
python3 -m unittest discover -s tools/tricopter -p 'test_*replay.py' -v
python3 tools/tricopter/servo_replay.py --prepare-inputs \
  --replay-report docs/validation/tricopter-offline-2026-11-07/servo/servo-model-report.json \
  --output-dir .local-artifacts/tricopter/servo-frozen-reproduction
python3 tools/tricopter/rpm_replay.py \
  --decoder .local-artifacts/tricopter/m1-bench-2026-11-06/tools/blackbox-tools/obj/blackbox_decode \
  --output .local-artifacts/tricopter/rpm-frozen-reproduction
```

Omit `--replay-report` to rerun the fixed-budget servo fit on the declared training units. Provide `--decoder` to the servo command if the pinned decoder lives elsewhere. Source/tool/dependency versions are captured; compiler/platform changes can affect floating-point results. Model status and freshness are software contracts, not physical sensor validity or angle/load qualification.

From the repository root, verify this package with:

```sh
sha256sum --check docs/validation/tricopter-offline-2026-11-07/checksums.sha256
```
