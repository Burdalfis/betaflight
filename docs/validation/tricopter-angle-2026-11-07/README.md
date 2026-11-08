# Measured basket-angle evidence — 2026-11-07

See [methods, equations and scope](../../tricopter_measured_angle.md) for the complete interpretation and reproduction commands.

- `angle-profile.json`: frozen 58-knot measured map, both approach tables and command/ADC linear regressions.
- `training-analysis.json`, `roll-zero-result.json`, `mounting-reference.json`: same-byte input summaries; raw-source hashes are retained.
- `frozen-protocol.json`, `frozen-reversal-plan.json`, `frozen-reversal-high-plan.json`: training/validation separation and predeclared capture plans.
- `reversal-analysis.json`, `reversal-validation.json`: all 75 points retained, failed stationary window 61 excluded from scoring.
- `reversal-high-analysis.json`, `reversal-high-validation.json`: separate longer 29-hold repeat, all passing.
- `native-integration-check-final.json`: actual C replay of qualified corrected counts, plus two synthetic-airframe integration checks. The earlier native check is retained separately.
- `core-strict-fast-math-arm-checks.json`, host/Python/logger logs: compiler commands, sources and test evidence.
- `independent-reversal-audit.json`: independent raw/CSV/quality/restoration and minor-loop checks.
- `calibration-and-reversals.png`: paired mean regressions and first-run held-out errors.

The ADXL occupies the absent motor's place. Angle zero follows operator PCB alignment; PB0−PC0 is the sole measured servo position source. Fits use the earlier sweep only. Raw bench streams remain in the local artifact directory named in the methods document. Summary-only reproduction does not independently recheck those raw bytes.

Flight-control outputs and configuration are unchanged after exact restoration. Synthetic motor/effectiveness/reaction models and the historical command-driven servo dynamics are not promoted to physically validated flight models.
