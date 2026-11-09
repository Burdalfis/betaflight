# Independent yaw tuning in Gazebo — 2026-11-08

This experiment reruns the active-braking plant's original-gain baseline, automatically tunes yaw P/I/D independently for stock and modified Betaflight, then evaluates the selected gains. It is simulation tuning with the existing approximate servo and nonlinear motor/allocator interface, not a physical flight PID recommendation. Roll/pitch P/I/D/F, filters, limits, geometry, motor/prop parameters and firmware binaries remain fixed.

## Three evaluation cohorts

Each cohort uses three repetitions of the same seven fresh-world/EEPROM cases, in stock/modified/modified/stock/stock/modified group order. The shared fixed-input collective is 1345 µs; yaw commands are approximately ±60.2976°/s. Native position/altitude autopilot modes stay off. The external ground-truth RC pilot performs setup and landing; its four inputs are overridden throughout each measurement.

1. **Original/F47:** yaw P/I/D 50/10/20, F47, with the original yaw-hold setting. [Repeated baseline](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261108T233514-both-braking-pretune-baseline-323682).
2. **Original/F0:** the same P/I/D with yaw F and yaw-hold gain explicitly zero. This isolates feedforward removal from PID tuning.
3. **Tuned/F0:** separate automatically selected yaw P/I/D for each firmware, with both yaw feedforward settings zero. Selection is frozen before this cohort starts.

Original/F0 versus Tuned/F0 measures the PID-tuning change. Original/F47 versus Tuned/F0 measures the combined feedforward/PID change. The final stock-versus-modified result compares two independently tuned implementations with the same search budget and criteria; it does not establish a global optimum.

## Search protocol

Training uses approximately ±90.3136°/s (RC1333/1667), with a 1 s neutral baseline, 1 s command and 1.5 s release observation. Each direction starts with a fresh world and EEPROM. The final 60°/s waveform and collective cases are excluded from selection; they provide different evaluation excitations as well as repeated trials.

Each firmware receives exactly **24 fresh CW/CCW pairs**, or 48 training flights:

- Nine coarse candidates: P={25,50,100}, D={0,20,40}, I=10; original 50/10/20 runs first.
- Six integral candidates: the best two coarse seeds, each with I={5,20,40}.
- Three bounded local refinements: incumbent P×1.25, P×0.75, then D−10 if D≥20 or D+10 otherwise. Duplicate/boundary proposals use a declared deterministic coordinate fallback.
- The best three explored candidates each receive two additional paired confirmations. Selection uses all three pairs, including every failure.

Bounds are P15–150, I3–40 and D0–50. Missing feasible seed/finalist slots are filled by lowest penalized candidates so a poor starting I does not prevent later exploration. Only candidates passing every recorded trial and confirmation may win. No failed flight is discarded or selectively retried. Firmware order and first direction are counterbalanced across the fixed budget. Independent checks verify actual directories and timestamps rather than treating a candidate's inherited `variant_order` metadata as a standalone two-firmware cohort.

For each direction, the scalar objective is:

```text
0.35 * normalized command IAE + 0.15 * normalized command ITAE
+ 0.25 * normalized release IAE + 0.10 * normalized release ITAE
+ 2 * overshoot_ratio^2 + opposite_return_ratio^2
+ 0.05 * mean normalized command/release settling
+ 0.05 * absolute last-200ms tracking error / commanded rate
```

IAE normalization is rate magnitude × window duration; ITAE uses rate magnitude × duration²/2. Errors remain referenced to requested rate rather than each response's own plateau. Unobserved settling stays null and contributes an explicit censored score of 1.25 window durations. CW/CCW are combined as mean score plus 0.25 times their absolute difference. Failed trials retain a finite ≥1000 penalty and make their candidate ineligible.

Common constraints reject more than 25% command overshoot or opposite return, sustained above-band oscillation, more than 1% motor/servo endpoint occupancy, more than 5° baseline-relative roll/pitch excursion, more than 60°/s roll/pitch rate norm or more than twice the target yaw rate. The armed motor lower endpoint is 1050 µs in this configuration. Normal timing, arming, actuator-input and ADC-validity gates also apply. Modified-only allocation flags are diagnostic and do not create an unequal constraint.

## Configuration and provenance

Every run retains a separate yaw-only CLI overlay, final firmware CLI, EEPROM, provisioning log and native PID readbacks. Yaw P/I/D come from MSP112 bytes `[6,9)` and yaw F from MSP94 `[36,38)`; the zero hold gain is verified using its native CLI `get` result. No shared CLI file is mutated. Raw MSP replies remain unchanged; only those declared yaw bytes are masked in the additional shared-settings comparison. All other supplied settings, source/model assets, plugin and firmware hashes are checked.

The original baseline's 42 flights and the following 180-flight reference/search/evaluation campaign are separate immutable packages. The latter is [the completed campaign](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261108T235059-both-yaw-autotune-338473); `search-protocol.json`, `search-progress.json`, `selection.json` and the per-cohort summaries retain the decision history. Both packages completed successfully, for **222 fresh flights** in total.

Reproduction from this checkout:

```sh
python3 tools/tricopter/sitl/check_comparison_fresh.py --repetitions 3 --hover-throttle 1345 --label braking-pretune-baseline
python3 tools/tricopter/sitl/autotune_yaw.py --baseline /absolute/path/to/completed/baseline
```

## Results

Both original-gain cohorts are complete (42 flights each), and all 96 training flights passed. Both independent searches selected **P31 / I20 / D10 / F0**, with yaw-hold gain zero, after three complete CW/CCW pairs per finalist. Their balanced confirmed training objectives are 0.084084 for stock and 0.064634 for modified; these describe the 90°/s training waveform and are not compared numerically to the different evaluation waveform.

The equal raw gain values were an outcome of separate searches, not a requirement imposed on the candidates. They are the best observed points in this finite coarse/local search, not proven global optima. Training, candidate selection and its confirmation were independently reconstructed from the raw records.

The final 42-flight evaluation completed with those selected gains frozen. The tables below give means across three repetitions, with CW/CCW separated. The [complete three-condition report](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/yaw-autotune-review/20261108T235059-both-yaw-autotune-338473/README.md) includes every range, raw measurement reference, finite recovery horizon and machine-readable aggregate.

| Firmware and measurement | Original 50/10/20, F47 | Original 50/10/20, F0 | Tuned 31/20/10, F0 |
|---|---:|---:|---:|
| Stock CW/CCW 10–90% rise, ms | 586.4 / 571.6 | 553.6 / 537.0 | **194.7 / 193.3** |
| Modified CW/CCW 10–90% rise, ms | 416.6 / 400.0 | 386.4 / 372.4 | **139.9 / 140.6** |
| Stock CW/CCW command settling, s | 0.910 / 0.853 | 0.857 / 0.823 | **0.270 / 0.270** |
| Modified CW/CCW command settling, s | 0.700 / 0.663 | 0.670 / 0.637 | **0.200 / 0.200** |
| Stock CW/CCW overshoot, % of requested rate | 0 / 0 | 0 / 0 | **1.48 / 1.96** |
| Modified CW/CCW overshoot, % of requested rate | 0 / 0 | 0.10 / 0.02 | **2.67 / 2.86** |
| Stock CW/CCW release settling, s | 0.867 / 0.907 | 0.840 / 0.900 | **0.270 / 0.273** |
| Modified CW/CCW release settling, s | 0.683 / 0.720 | 0.673 / 0.667 | **0.200 / 0.200** |
| Stock CW/CCW continued heading after release, ° | 12.66 / 12.50 | 12.25 / 12.95 | **7.39 / 7.67** |
| Modified CW/CCW continued heading after release, ° | 9.22 / 9.52 | 9.23 / 9.38 | **5.58 / 5.58** |

Settling requires staying within max(2°/s, 5% of target) for the rest of the recorded window, with at least 0.25 s remaining. All these settling results are observed, not censored. Rise thresholds use the measured baseline and requested rate, not a reduced plateau. Continued heading is finite-window heading drift after release, distinct from rate overshoot. The tuned responses have a small opposite-rate return peak: approximately 0.87–1.11°/s stock and 1.69–1.71°/s modified.

![Three-condition yaw response](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/yaw-autotune-review/20261108T235059-both-yaw-autotune-338473/yaw-rate.png)

PID tuning materially reduces rise and settling times in both implementations. Removing feedforward alone gives only a modest rise-time change and does not uniformly improve tracking RMS. Relative to the F0 reference, tuned CW/CCW tracking RMS is 12.81/13.15°/s versus 13.45/13.72 stock, and 11.62/11.35 versus 11.75/11.48 modified. Relative to F47, tracking RMS improves only in stock CW; the other three direction/variant means increase slightly. Thus faster 10–90% rise is not a blanket tracking improvement.

## Collective-change tradeoffs

The held-out matrix applies a ±25 µs collective pulse lasting 0.4 s, both from neutral yaw and during a held turn. The values below are peak yaw-rate differences against the matched sham or pure-yaw trajectory, after alignment by actual command age. They describe disturbances added by the collective pulse, not absolute tracking error.

| Firmware and collective case | Original F47, °/s | Original F0, °/s | Tuned F0, °/s |
|---|---:|---:|---:|
| Stock neutral up / down | 8.36 / 8.39 | 8.33 / 8.30 | **8.68 / 9.37** |
| Modified neutral up / down | 7.60 / 7.50 | 7.61 / 7.49 | **8.90 / 8.97** |
| Stock up during CW / CCW | 8.95 / 8.41 | 9.06 / 8.38 | **10.42 / 7.49** |
| Modified up during CW / CCW | 7.99 / 7.37 | 8.01 / 7.38 | **9.38 / 8.37** |

Hover tuning therefore **generally worsens collective-induced yaw disturbances**; the exception is stock's CCW combined case. Modified does not consistently beat tuned stock in these disturbance measures. The held-out tests were not used to retune either controller after selection.

Roll/pitch coupling shows little consistent change. Tuned peak paired roll/pitch angle norms during CW/CCW collective-up are **0.240/0.403° stock** and **0.229/0.414° modified**. Original F47 values were 0.244/0.409° and 0.250/0.399°. Neutral up/down tuned values are 0.233/0.201° stock and 0.211/0.186° modified, with overlapping repetition ranges. Raw matched trajectories are not identical before the pulse: maximum pre-event roll/pitch norm mismatch across all conditions is 0.2384°, and maximum absolute axis differential slope is 0.5577°/s. These bounds and the retained prehistory prevent interpreting small differences as identified physical coupling coefficients.

## Actuator use and validation boundary

The modified controller still uses more basket excursion at these selected gains: actual CW/CCW excursions are **7.56/7.47° modified** versus **6.60/6.63° stock**, measured relative to each pre-command baseline. This is a remaining authority difference, not an isolated demonstration of the ADC feedback's benefit. Tuning reduces the modified controller's command excursion from 10.33/11.52° at F47 to 7.79/8.08°, and sampled joint speed peaks from 319/326 to 206/222°/s. It does not achieve its improvement by increasing the earlier servo command peaks.

All 96 training flights and all **72/72 yaw-containing evaluation flights** across the three conditions satisfy the frozen physical/stability/timing constraints. No motor/servo endpoint or drive/braking current-limit occupancy was observed in the nominal 100 Hz evaluation samples; this is a sampled result, not proof of full-rate or hardware extrema. All three cohorts retain N=3 per variant/case. Source, physical world/assets, firmware/plugin and non-yaw settings identity checks pass. Independent raw training reconstruction reproduces the metrics, objective, candidate ranking and selected gains within numerical tolerance.

The [independent execution audit](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261108T235059-both-yaw-autotune-338473/independent-full-campaign-audit.json) passes all 222 flights, including 76,764 planned-window actuator frames, 5,595 healthy armed ANGLE status samples and 2,694 valid modified feedback samples. The 180 new overlay provisions verify their native yaw fields and zero hold gain. PC0 remains zero; actual-joint ADC mapping error stays within rounding (0.5 count). Selection and evaluation protocol file timestamps tie at the filesystem's observed resolution; the frozen driver's sequential control flow establishes selection before evaluation. Final read-only host inspection found no remaining campaign processes or active simulation sockets.

The [independent held-out constraint reconstruction](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/yaw-autotune-review/20261108T235059-both-yaw-autotune-338473/independent-heldout-feasibility.json) independently passes all 72 yaw-containing evaluation flights and agrees on 885 numeric plus 649 exact checks, with maximum numeric difference 2.3×10⁻¹⁶. The [independent training math audit](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/yaw-autotune-review/20261108T235059-both-yaw-autotune-338473/independent-training-math-audit.md) covers all 96 search flights and confirms both selected candidates.

Before the campaign, the 85-test Python suite and three native Gazebo CTests passed. Focused synthetic tests cover objective normalization, censored settling, saturation/failure penalties, constrained candidate selection, equal budgets, confirmation retention, native profile offsets, narrow shared-settings masking and provisioning readbacks. To rerun the Python checks:

```sh
python3 -m unittest discover -s tools/tricopter/sitl -p 'test_*.py' -v
```

The tested CLI overlays are exported as [stock_yaw_autotuned_sitl.cli](/home/alex/betaflight/tools/tricopter/sitl/configs/stock_yaw_autotuned_sitl.cli) and [modified_yaw_autotuned_sitl.cli](/home/alex/betaflight/tools/tricopter/sitl/configs/modified_yaw_autotuned_sitl.cli). Both exactly match the tested overlay, SHA256 `92e2e1a29e491bc0cc9dbb3bdcb57e05f5ed347ef10b1d5c9dbb810a135170d7`. The [SITL README](/home/alex/betaflight/tools/tricopter/sitl/README.md) gives launch commands. Shared simulator defaults and physical FC configuration have not been changed by this tuning experiment.

The finite search found useful **hover yaw tunes**, not globally optimal or full-envelope tunes. The existing servo remains an approximate lag/rate/torque model rather than a validation against every fitted unloaded response. The nonlinear DC plant and firmware's command-to-thrust assumption still differ. The present collective pulses do not test sustained high-rate climb or rapid forward flight. The next comparison should freeze these gains and add those operating points, reporting yaw tracking, available authority and roll/pitch coupling without using them to revise the hover tune.
