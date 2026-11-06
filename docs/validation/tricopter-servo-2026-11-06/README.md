# Servo characterization evidence

See [the interpretation and results](../../tricopter_servo_characterization_2026-11-06.md). Firmware under test was `bd6a5d727`, with runtime PB0 feedback, unchanged legacy control, a 5 V unloaded servo and disconnected linkage. All motors were stopped during these servo recordings.

`manifest.json` pins six raw BBL files and their fixture conditions. Every raw file was read twice from the FC with identical hashes. `capture/` preserves run conditions and readback manifests. Free-case data were collected before the user identified housing movement; the full sequence was then repeated with the housing clamped and arm free. Do not pool those fixtures without labeling them.

`response/free-case/` and `response/clamped-case/` contain per-trial and grouped step metrics. `response/fixture-comparison/` retains unfiltered early traces and common-signed-transient diagnostics. `hysteresis/` contains the separate small-step analysis. Methods and amendments distinguish first ANY excursion, target-directed excursion and established target-directed response; short wrong-way signals are not automatically classified as electrical or removed from raw evidence.

The analysis uses Python and NumPy; the plot source additionally uses Matplotlib. The official decoder source is pinned in `tools/blackbox-decoder-provenance.json`. Apply `tools/blackbox-tools-integer-p-interval.patch` to that revision before decoding modern integer `P interval:4` headers. This fixes iteration reconstruction; it does not modify raw logs.

Example reproduction, after building that decoder, from this directory:

```sh
mkdir -p work
blackbox_decode --unit-vbat raw --unit-flags raw --output-dir work raw/clamped-cyclic.bbl
blackbox_decode --unit-vbat raw --unit-flags raw --output-dir work raw/clamped-paired.bbl
blackbox_decode --unit-vbat raw --unit-flags raw --output-dir work raw/clamped-dither.bbl
python3 response/analysis.py --synthetic-only
python3 response/analysis.py \
  --dataset work/clamped-cyclic.01.csv cyclic12 48 \
  --dataset work/clamped-paired.01.csv paired6plus6 24
python3 hysteresis/analyze.py work/clamped-dither.01.csv
```

The scripts write derived files beside themselves; run on a copy when preserving this checkpoint. Packaged JSON also contains fixture/raw-input provenance and presentation summaries added by `response/package_results.py`. Absolute paths in historical provenance identify the original workspace; raw hashes and relative package paths are authoritative for locating copied inputs.

`checksums.sha256` covers all packaged files except itself. From the repository root:

```sh
sha256sum --check docs/validation/tricopter-servo-2026-11-06/checksums.sha256
```
