# Corrected servo hysteresis and approach test

See [results and interpretation](response/hysteresis-report.md) and [the sole position measurement contract](../../tricopter_dual_adc_logging.md). All position metrics use signed **PB0−PC0**, `debug[3]`, with metric-local paired-data freshness checks. Raw channels are diagnostic only. PC0 senses the servo PCB power GND input, not its potentiometer low terminal.

The case was clamped, arm free, linkage disconnected and propellers removed. All motors stayed stopped. Three repeats approached 1420 µs from 1100, 1400, 1440 and 1700 µs in varied order, then two 2 µs staircases traversed 1400–1440 µs. The exact plan/events, supply minimum and restoration checks are in `capture/`. No onboard recording was erased.

`raw/servo-hysteresis.bbl` is the original 1660928-byte recording from firmware `a15ed3065`, read twice with matching SHA-256 `2532d8ff189aba16e64ce4dc53008e358071585962496734eed55cce02ab7f3b`. It occupies onboard offsets 13910016–15570944. The canonical API refactor at `f6e496fc1` followed this recording and preserves the same corrected arithmetic and debug schema; its build/test and subsequent flash/idle verification are separately labeled `capture/canonical-*`.

`response/` contains the portable analyzer, declared methods, synthetic checks, per-plateau/paired data, plots and complete results. The measured directional branch separation is about 10–11 µs input-equivalent. This is descriptive combined servo hysteresis and threshold behavior, without independently isolating gear lash or calibrated angle.

`tools/` preserves the acquisition scripts, MSP transport and official-decoder provenance/compatibility patch. Live capture scripts require the recorded fixture and power conditions; the following reproduction is offline only. Build the pinned official decoder with the included patch, then run from this directory using Python, NumPy and Matplotlib:

```sh
mkdir -p work
blackbox_decode --unit-vbat raw --unit-flags raw --save-headers --output-dir work raw/servo-hysteresis.bbl
python3 response/analyze_hysteresis.py --synthetic-only --output-dir work/synthetic
python3 response/analyze_hysteresis.py --csv work/servo-hysteresis.01.csv \
  --events capture/servo-hysteresis-events.json --run-json capture/servo-hysteresis-run.json \
  --bbl raw/servo-hysteresis.bbl --output-dir work/response
```

Decoded full CSVs, full firmware/configuration recovery images and serial transcripts remain local under `.local-artifacts/tricopter/`. The raw BBL and pinned decoder reproduce the decoded inputs; saved headers, logs and hashes are packaged. Absolute analysis paths identify the original workspace, while file hashes and package-relative locations identify archived inputs.

Verify the archived bytes from the repository root:

```sh
sha256sum --check docs/validation/tricopter-hysteresis-2026-11-06/checksums.sha256
```
