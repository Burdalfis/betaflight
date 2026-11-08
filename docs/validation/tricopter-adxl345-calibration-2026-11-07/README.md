# Preserved ADXL calibration and basket recordings — 2026-11-07

`raw-session.tar.gz` preserves **467 files**, 263,425,003 uncompressed bytes, from the calibration session. Its compressed size is 47,018,981 bytes. `raw-session-manifest.json` records every member's original SHA-256/size, the archive SHA-256 and the excluded rebuildable files.

`archive-verification.json` records an independent check of every archive member and safe extraction. Fitting from the extracted six-pose session reproduces both frozen bias and scale vectors exactly. Extracted full-sweep hashes/restoration checks and both held-out reversal replays also pass, retaining the original failed-window exclusion.

The archive includes original ADXL binary **and CSV** recordings, FC differential-ADC/PWM observations, all six training poses, reference repeats, tilted validation, thermal observations, mounted baselines, both sweep attempts, zero and reversal recordings, failed attempts, diagnostic history and historical source snapshots. CSV host-arrival times are needed for joint settled-window analysis and cannot be recovered from binary alone. Host executables, shared libraries, object files and Python caches are excluded; the measured-angle package separately retains native build/test evidence.

These exact files are also directly available here:

- `calibration-profile-v1.json`: frozen ADXL biases/scales, SHA-256 `d068869cd9034fcf418337a8c45f8dc4f8bb2e77454bbcc086d37aad5191d573`.
- `fit-session-snapshot.json`: original training/holdout selection and input hashes. Its pose paths resolve relative to the session root inside the archive.
- `tilted-validation-summary-v2.json`: independent validation against the frozen sensor profile.

Historical provisional/pending labels remain unchanged in frozen construction-time artifacts. Subsequent validation and operator corrections are preserved separately. In particular, the first purported mounted hold was actually a bench hold; the original failed sweep and failed reversal window are not promoted to passing data.

Restore into a checkout from its repository root. `--keep-old-files` prevents replacing an existing local session:

```sh
sha256sum docs/validation/tricopter-adxl345-calibration-2026-11-07/raw-session.tar.gz
tar --keep-old-files -xzf docs/validation/tricopter-adxl345-calibration-2026-11-07/raw-session.tar.gz

python3 tools/tricopter/adxl345_logger/calibrate.py fit \
  .local-artifacts/tricopter/adxl345-calibration-2026-11-07/fit-session-snapshot.json \
  --output /tmp/replayed-adxl-profile.json

python3 tools/tricopter/adxl345_logger/basket_analyze.py \
  .local-artifacts/tricopter/adxl345-calibration-2026-11-07/basket-full-sweep-02 \
  --output /tmp/replayed-basket-analysis.json
```

Compare archive/member hashes with the manifest before replay. Re-fitting creates new time/provenance metadata; compare numerical coefficients and diagnostics with the frozen profile rather than expecting its complete JSON hash to repeat. Outputs require new filenames. This is host-only replay and opens no device ports.

See [measured-angle results](../../tricopter_measured_angle.md) and the [angle validation package](../tricopter-angle-2026-11-07/README.md). This archive adds remote preservation of the raw evidence; the original files remain untouched in the ignored local session directory.
