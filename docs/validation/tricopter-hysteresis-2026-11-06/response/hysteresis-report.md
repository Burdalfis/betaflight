# Corrected servo approach and hysteresis experiment

The corrected electrical position proxy shows repeatable dependence on command direction. At an identical 1420 µs command, approaching from below gives a higher `PB0−PC0` endpoint than approaching from above. Two 2 µs staircase cycles show a directional branch separation equivalent to about **10–11 µs of command** using their measured local slopes. These observations are consistent with servo hysteresis/deadband, but do not isolate gear lash from the servo's controller, friction or remaining electrical effects.

**Every endpoint, curve, speed and reversal calculation uses `debug[3]` only.** Raw PB0 and PC0 are retained for diagnostics and data validity. PC0 senses the servo PCB **power ground**, not an independently sensed potentiometer-low terminal; subtraction therefore remains an electrical position proxy.

The case was clamped, arm free, linkage disconnected and motors stopped. Root recorded three repeats of four center approaches, changing their order between repeats. Each pre-position was held for 0.4 s and each 1420 µs center for 0.6 s. After a 1380 µs precondition and 1400 µs start, two triangular staircases traversed 1400–1440 µs in 2 µs increments, holding each command for 0.25 s. Actual PWM plateau durations include communication overhead. All 108 event labels match the recorded PWM plateaus exactly.

The two independent 1,660,928-byte downloads match, SHA-256 `2532d8ff189aba16e64ce4dc53008e358071585962496734eed55cce02ab7f3b`. The firmware is the same `a15ed3065` dual-ADC diagnostic build used for the earlier paired transient experiment.

## Center endpoint dependence

Endpoints are medians over the final 200 ms of each center hold. Each accepted metric window contains only fresh coherent pairs, with no stale interpolation. All accepted tails pass the descriptive drift check.

| Previous command →1420 µs | Accepted repeats | Center median (counts) | Repeat range (counts) |
|---|---:|---:|---:|
| 1100→1420 | 3/3 | 1591 | 1587–1591 |
| 1400→1420 | 3/3 | 1590 | 1590–1591 |
| 1440→1420 | 2/3 | 1583 | 1583–1583 |
| 1700→1420 | 3/3 | 1583 | 1582–1583 |

Pairing approaches within each repetition gives **−4, −9, −8 counts** for the `1700 approach minus 1100 approach` center difference, and **−7, −8 counts** for `1440 approach minus 1400 approach`. One small-approach pair is rejected because one final center tail contains a stale sample. Thus the observed direction dependence persists for both large and small approach distances. Three repeats give descriptive repeatability, not a population specification.

## Staircase branches

Each endpoint uses the final 100 ms after excluding at least the first 150 ms of its plateau. Branch comparisons pair identical interior commands, 1402–1438 µs, within each cycle. Missing endpoints are omitted without interpolation.

| Cycle | Accepted paired commands | Median up-minus-down (counts) | Midpoint slope (counts/µs) | Fit R² | Absolute input-equivalent separation |
|---|---:|---:|---:|---:|---:|
| 1 | 16 | 10.25 | −1.0150 | 0.9901 | 10.10 µs |
| 2 | 15 | 10.50 | −0.9802 | 0.9924 | 10.71 µs |

The input-equivalent quantity is branch separation divided by the magnitude of the fitted **local corrected-count/command slope**. It is not calibrated shaft angle or a direct measurement of mechanical play. Per-command differences and both cycle curves are preserved in the CSV and JSON; two cycles do not justify an independent-sample confidence interval.

Only the lower reversal between the cycles has a completely accepted reference and uninterrupted early endpoint windows. Its center-independent staircase reference is 1400 µs, corrected median 1607 counts. The first 2, 4 and 6 µs command reversals have unchanged endpoint medians; 8 µs gives a 0.5-count shift, 10 µs gives 5.5 counts, and 12 µs gives 6 counts. The predeclared noise threshold is crossed at **14 µs cumulative command change**, with a 9-count target-directed shift, and confirmed at **16 µs**, with 11 counts. The threshold is about 8.90 counts at those two steps.

Accordingly, **12–14 µs is a thresholded detectable-response bracket, confirmed at 16 µs**. It is not a physical no-motion interval: smaller endpoint changes are present before that threshold. Both upper-reversal reference tails contain stale rows and are rejected rather than filled.

## Approach timing

Timing uses corrected raw five-row sustained crossings, with each metric's search prefix checked separately. It is a new descriptive analysis, not a drop-in replacement for the older raw-wiper characterization pipeline.

| Transition | First target-directed electrical excursion | 10–90% interval | Mean 10–90% slew |
|---|---:|---:|---:|
| 1100→1420 | 16.96–20.95 ms, n=2 | 84.81–87.81 ms, n=2 | 3157–3367 counts/s |
| 1700→1420 | 11.97–18.96 ms, n=3 | 72.84–73.84 ms, n=2 | 3315–3372 counts/s |

The different step spans contribute to the different movement intervals. Mean slew ranges overlap; these small samples do not establish intrinsic mechanical direction asymmetry. The ±20 µs approaches are too small relative to the predeclared raw-noise timing threshold, so no latency or 10–90% time is reported for them.

## Data quality and limits

The log contains **38,157 frames over 38.071970 s**, median frame interval 998 µs, maximum 1004 µs. Exact signed subtraction holds in every available frame. **38,115 frames (99.890%)** are usable; the 42 other frames are cached pairs outside the 1 ms freshness criterion. The maximum encoded observation age is 19 (1.9–2.0 ms after quantization). There are no unexpected frame gaps at the declared threshold.

The analyzer accepts **93/108 local plateau tails**; all accepted tails are quiet by the recorded drift test. This local gating allows a late stale row elsewhere in a hold to leave an earlier timing measurement usable, and vice versa. PC0 is zero in 5156 frames (13.5%); no PC0 ceiling is observed. Negative servo-ground voltage is therefore still unobservable when the unipolar sense input clips at zero.

The channels belong to the same completed ADC scan but are sequential, nominally about 47 µs apart. Power-ground subtraction does not cancel excitation/reference changes or identify the potentiometer's low terminal independently. Logged PWM requests are not externally measured pulse edges. The unloaded fixture does not identify flight load or gyroscopic torque.

## Reproduction

Keep `analyze_hysteresis.py` and `hysteresis-methods.json` together. Python, NumPy and Matplotlib are required.

```sh
python3 analyze_hysteresis.py --csv /path/to/servo-hysteresis.01.csv --events /path/to/servo-hysteresis-events.json --run-json /path/to/servo-hysteresis-run.json --bbl /path/to/servo-hysteresis.bbl --output-dir /path/to/new-output
```

The decoder is the same pinned official Betaflight blackbox-tools build and integer-P-interval compatibility patch as the paired transient experiment. Its stdout/stderr, decoded CSV and saved headers are retained. Synthetic checks cover metric-local freshness, rejection of missing data before a first-event claim, a known 80 ms movement interval and recovery of a known 4 µs directional gap. Earlier firmware/source and historical artifacts are unchanged by this offline analysis.
