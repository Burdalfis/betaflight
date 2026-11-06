# Servo ground-reference experiment

See [wiring, schema, hardware results and interpretation](../../tricopter_dual_adc_logging.md). This package records the clamped, unloaded servo after adding PC0 servo-PCB ground sense beside the PB0 wiper input. Firmware `a15ed3065` retains the legacy controller and logs raw PB0, raw PC0 and signed PB0−PC0 in `TRI_GROUND`. Motors remained stopped.

`raw/servo-ground.bbl` is the original 772096-byte recording, read twice with identical SHA-256 `1596d8165add47cdadb929626c76efc033ae87df0c5d553f7c8c1fb09cba7701`. The onboard log range is 13137920–13910016. Earlier logs were not erased. `capture/` preserves the run schedule, host samples, idle/final-state checks, readback metadata, build provenance and configuration comparisons. The full flash/configuration recovery images and serial transcripts remain local under `.local-artifacts/tricopter/m1-ground/bench-2026-11-06/`.

`response/` contains the offline analysis, unchanged historical response-metric helper, declared methods, synthetic checks, per-trial summaries, raw early traces and plots. The experiment supports ground rise as a substantial component of the original initial kick, with incomplete cancellation. It does not independently measure mechanical angle. The first 20 ms of every transition has complete fresh paired data; full-hold metrics retain a strict validity gate and should not be pooled into a new general servo specification.

PB0 and PC0 belong to the same completed scan, with nominally 47 µs sequential sampling skew. PC0=0 is valid but hides negative servo-ground excursions. Pair freshness is an acquisition-progress check, not proof of connection or simultaneous conversion. Earlier single-input evidence is preserved separately in [the original characterization package](../tricopter-servo-2026-11-06/README.md).

Build the official decoder revision pinned in `tools/blackbox-decoder-provenance.json`, applying `tools/blackbox-tools-integer-p-interval.patch`. From this directory, reproduce offline results using Python, NumPy and Matplotlib:

```sh
mkdir -p work
blackbox_decode --unit-vbat raw --unit-flags raw --save-headers --output-dir work raw/servo-ground.bbl
python3 response/analyze_ground.py --synthetic-only --output-dir work/synthetic
python3 response/analyze_ground.py --csv work/servo-ground.01.csv \
  --bbl raw/servo-ground.bbl --run-json capture/servo-ground-run.json \
  --output-dir work/response
```

`tools/` also preserves the bench acquisition scripts and their MSP transport. These scripts operate live hardware and depend on the captured fixture/power conditions; the reproduction commands above are offline only. Their original workspace import path is historical; to run a copied capture script, provide the accompanying `bench_io.py` on its Python import path. Absolute paths in recorded analysis provenance identify the original workspace; hashes and package-relative locations identify archived inputs.

Verify packaged evidence from the repository root:

```sh
sha256sum --check docs/validation/tricopter-ground-2026-11-06/checksums.sha256
```
