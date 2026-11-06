# Paired ADC servo-ground experiment, 2026-11-06

The new paired recording supports a substantial electrical contribution to the earlier positive wiper kick. PC0 rises with PB0 immediately after each command; subtracting PC0 removes most of that early positive excursion in the ADC-decreasing transitions. The data does not establish that all remaining excursions are shaft motion, or exclude gear lash under other conditions.

Three unloaded, clamped-case cycles used 1420→1100→1420→1700→1420 µs, with 1.2 s holds and the linkage disconnected. Root's capture records the motors stopped throughout. The two independent 772,096-byte Blackbox downloads are identical, SHA-256 `1596d8165add47cdadb929626c76efc033ae87df0c5d553f7c8c1fb09cba7701`.

The following are descriptive medians and ranges over three trials. Each positive peak is the maximum baseline-subtracted reading in the predeclared **[0,20) ms** window. Peaks can occur at different times in each channel. This window contains early target-directed movement in the ADC-increasing transitions, so its peak-reduction percentage is not a universal electrical cancellation factor.

| PWM transition (µs) | PB0 positive peak (counts) | PC0 positive peak (counts) | PB0−PC0 positive peak (counts) | Per-trial peak reduction |
|---|---:|---:|---:|---:|
| 1420→1100 | 49 (47–52) | 40 (39–41) | 43 (17.5–54) | 12.2% (−3.8–62.8%) |
| 1100→1420 | 51 (43–55) | 40 (40–42) | 11 (5–23) | 78.4% (58.2–88.4%) |
| 1420→1700 | 48 (45–49) | 40 (38–47) | 11 (10–25) | 75.6% (49.0–79.2%) |
| 1700→1420 | 48 (47–48) | 40 (39–44) | 23 (19–27) | 52.1% (42.6–60.4%) |

The **1420→1700** transition is especially useful because its eventual ADC displacement is negative. Its initial positive PB0 kick is therefore opposite the established response direction. At the raw PB0 peak sample, the three trials show PB0/PC0/corrected increments of **49/38/11**, **45/39/6**, and **48/38/10** counts. Early PB0/PC0 correlation is 0.73–0.95. The positive corrected peaks elsewhere in the same window remain 10–25 counts; subtraction improves the signal but does not remove all short spikes. At an approximate 3.28–3.29 V ADC reference, 40 counts represent about **32 mV**. This conversion is indicative, not a calibrated voltage or shaft-angle measurement.

All four groups have **63/63 usable paired rows** in the [0,20) ms window, from 21 rows per trial. See the JSON for the exact status-mask counts and maximum observed pair age of each group. The baseline uses valid paired rows over the preceding 200 ms.

The complete log has 17,641 frames over 17.599483 s, a median 998 µs frame interval and a maximum 1115 µs interval. The decoder reports the expected 75% rate decimation and no unexpected missing-loop report. Signed `PB0−PC0` equals the firmware field exactly in every available frame. The corrected value is positive throughout this experiment, so negative-data support is checked by the synthetic validation rather than demonstrated by these hardware samples. No wiper rail or PC0 ceiling was observed.

The pair freshness flag is set in **17,623/17,641 frames (99.898%)**. Eighteen cached frames have encoded observation ages 12 or 17 (1.2–1.3 ms or 1.7–1.8 ms after quantization); all other frames are marked fresh. Observation age is not physical conversion age. PC0 is at its zero floor in **2,374 frames (13.46%)**. Zero is valid but cannot reveal negative servo-ground excursions below FC ground.

The strict full-response gate accepts only trials 5 and 9, both 1420→1100. The other holds contain isolated stale rows, sometimes well after the initial response. Their early electrical transients remain usable, but this experiment does **not** replace the prior complete characterization or provide new pooled latency, movement-time, overshoot or settling specifications. The JSON retains the two uninterrupted reference trials and flags the other ten. No stale data was interpolated, and plotted invalid rows are gaps.

The sense channels are sequential, nominally about **47 µs apart**, and can be further separated by injected ADC work. Subtraction cannot cancel excitation/reference changes, different conditioning, clipping or offsets absent at the ground sense point. PWM time zero is a logged final write request, not an externally measured pulse edge. The unloaded fixture does not identify flight torque or load dependence.

## Reproduction

The portable analyzer requires Python, NumPy and Matplotlib. Keep `analyze_ground.py`, `response_metrics_v4.py` and `ground-analysis-methods.json` together. `response_metrics_v4.py` is an unchanged copy of the prior v4 electrical-response pipeline; earlier evidence is not edited.

```sh
python3 analyze_ground.py --csv /path/to/servo-ground.01.csv --bbl /path/to/servo-ground.bbl --run-json /path/to/servo-ground-run.json --output-dir /path/to/new-output
```

The local official Betaflight decoder is pinned to commit `d89f96b983d0f5c8f053acf003ee2998b02b5b0f`, with the previously documented integer-P-interval compatibility patch. Decoding uses `--unit-vbat raw --unit-flags raw --save-headers`. The saved header identifies `Betaflight 2026.6.2 (a15ed3065) STM32F405`, `debug_mode=103`, and `P interval=4`. The date embedded in that deterministic build is the fixed build epoch, not the capture date.

Outputs include the paired early sample CSV, per-trial metrics CSV, complete JSON, PNG/PDF comparison, synthetic validation and hashes. The declared methods explain all gates and limitations.
