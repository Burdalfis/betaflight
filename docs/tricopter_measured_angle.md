# Measured basket-angle calibration and offline integration

The 2026-11-07 calibration now supplies settled basket angle from **signed PB0−PC0 only**. The ADXL occupies the motor's place. Its operator-aligned +X-up position defines flat-basket roll zero with the frame level in roll. Gravity is projected into the previously fitted hinge plane, excluding hinge-parallel gravity from slight pitch. The motor is absent; rotor-shaft alignment, aircraft yaw sign, loaded dynamics and independent absolute accuracy are not established.

The [validation package](validation/tricopter-angle-2026-11-07/README.md) retains the frozen input summaries, measured profile, regression points, held-out reports, source hashes and native checks. The [raw-session archive](validation/tricopter-adxl345-calibration-2026-11-07/README.md) preserves the underlying recordings, timing CSVs, sensor calibration and historical source snapshots. Original files remain under `.local-artifacts/tricopter/adxl345-calibration-2026-11-07/`.

## Linear correlations

Each regression uses **29 equal-weight points**, commands 1120–1680 µs in 20 µs increments. Each point averages exactly one increasing-PWM and one decreasing-PWM settled observation. For the ADC regression, both corrected counts and angle are averaged within that pair. Initial holds, unpaired 1100/1700 endpoints and all new reversal recordings are excluded from fitting. The many raw samples inside a hold are not treated as independent regression points.

Let `u` be commanded PWM in µs, `c` signed PB0−PC0 counts and `theta` basket roll from flat in degrees. The unconstrained fits are:

```text
theta =  0.11490966915 * (u - 1508.80510892)
theta = -0.10442633291 * (c - 1489.89677893)
```

| Fit | Pearson r | Centered R² | RMS residual | Maximum absolute residual |
|---|---:|---:|---:|---:|
| Command → paired mean angle | +0.999724907 | 0.999449891 | 0.4511° | 1.1681° |
| Paired mean PB0−PC0 → paired mean angle | −0.999771044 | 0.999542140 | 0.4115° | 1.0133° |

The arithmetic mean of the two physically referenced approach-zero commands is **1511.74648922 µs**. Expressed about that reference, the free command fit has a **+0.337993° intercept**; its fitted zero crossing is 1508.805109 µs. These are distinct quantities. If zero must be fixed at the measured middle, the separately reported constrained fit is:

```text
theta = 0.11397679499 * (u - 1511.74648922)
```

Its training RMS/max residuals are 0.5315°/1.1955°. The linear ADC fit's measured paired-count domain is 1299.311–1919.289 counts. Neither correlation removes approach dependence or supplies dynamic position from a command.

## Bounded measured map

The offline models use a continuous **58-knot mean angle map** rather than the command regression. At every knot in the union of the two branch count grids, each measured approach curve is interpolated and their angles averaged. Only their common count domain is retained: **1295.789–1924.737 counts**, approximately **−46.55° to +19.96°**. No extrapolation is permitted.

Half the local approach separation is retained as a descriptive envelope. It is not a confidence interval or a bound on future reversal error. The mean map does not switch branches instantly when command direction reverses. Freshness and canonical ADC quality must be established by the caller; raw ADC channels and PWM are never substituted for corrected position.

The portable allocation-free [C map](../src/main/flight/tricopter_angle_calibration.h) supports up to 64 strictly monotonic knots, finite-value validation, interpolation and inverse conversion. The simulation and dynamic allocator accept it through an explicit `measuredAngleConfigured` option. It converts current predicted counts, inverse direction goals and future candidate counts consistently. Command-delay history and the command-only servo predictor remain intact; no new feedback correction is introduced.

The Python transport exposes `NativeDynamic.angles(profile, counts)` for actual recordings and `NativeDynamic.run(..., angle_profile=profile)` for hybrid offline simulation. Existing packed transport and disabled/default numerical behavior remain unchanged. Overall model calibration and flight-use flags remain false; motor/servo dynamics, effectiveness and reaction coefficients are still synthetic or retain their explicitly historical characterization scope. No actuator, PID, ADC configuration, mixer or flight-output path is changed.

## Prospective reversal checks

The frozen 75-plateau sequence tests ±5/10/20/40 µs loops near 1512 µs and ±10 µs loops at 1300/1650 µs. It records **176,754 coherent ADXL samples** without framing/sequence faults. One window, plateau 61 at 1660 µs, fails the unchanged stationary gate: X quarter-mean span 4.008 counts exceeds the 3-count limit. Its data remain visible but are excluded from scoring. The other **74 windows pass**. Original servo configuration restores exactly.

Across those 74 held-out points, the frozen measured map has **0.7513° RMS / 2.0888° maximum** error against settled ADXL observations, versus **1.2751° / 2.9309°** for the free command line. The linear ADC approximation has **0.9274° / 2.3238°** error. No held-out observations update the fit. Only about 61% lie inside the original major-branch hull, confirming that its width is descriptive rather than a guaranteed error bound.

The actual 5 µs pulse changes are independently visible in Teensy pulse medians. Small 5–10 µs return reversals show little resolved movement in this sequence; 20 µs returns move clearly. Nine returns/holds at 1512 µs span about **1.44°**. These observations do not distinguish universal deadband, electrical behavior and gear/linkage play, and do not measure millisecond motion timing.

A targeted **29-plateau high-angle repeat**, with two seconds settling plus four seconds measurement per hold, records **131,696 coherent samples** and passes all 29 windows. Frozen-map RMS/max errors are **0.5797°/1.4737°**. This separate repeat leaves the original failed window and its exclusion intact. Both runs finish with the complete original 96-byte servo table restored, 1420 µs output, disarmed state and three motor outputs at 1000.

## Validation and reproduction

The complete host suite passes **186 tests**, the offline Python/native-C suite **69**, and logger/calibration/capture suite **84**. The 41 focused angle/simulation/dynamic tests pass with all seven C modules compiled under both strict and fast-math variants. All seven compile for Cortex-M4F with fast math and warnings treated as errors; this is compile evidence only.

Actual held-out corrected counts run through the C map agree with the Python interpolation within **0.000011°**. Two 2501-tick synthetic-airframe runs consume the measured map in current and horizon predictions and pass global priority-selection checks. These use explicit −0.35 to +0.30 rad limits and current/nominal PWM candidates only; they verify integration, not physical allocator performance. Separate tests retain exact agreement with the original 701-row default fixture. MATLAB's source list is updated for the new module; measured-profile MATLAB/Simulink execution was not performed.

```sh
python3 tools/tricopter/basket_calibration.py export \
  --analysis docs/validation/tricopter-angle-2026-11-07/training-analysis.json \
  --mounting-reference docs/validation/tricopter-angle-2026-11-07/mounting-reference.json \
  --zero-result docs/validation/tricopter-angle-2026-11-07/roll-zero-result.json \
  --summary-only --output NEW_PROFILE.json

python3 tools/tricopter/basket_native_check.py \
  docs/validation/tricopter-angle-2026-11-07/angle-profile.json \
  docs/validation/tricopter-angle-2026-11-07/reversal-validation.json \
  --build /tmp/tricopter-angle-check --output NEW_NATIVE_REPORT.json

CC=gcc CXX=g++ src/test/run_tricopter_checks.sh
python3 -m unittest discover -s tools/tricopter -p 'test_*.py'
python3 -m unittest discover -s tools/tricopter/adxl345_logger -p 'test_*.py'
```

Summary-only export reproduces the measured fit from retained summaries and recorded hashes without claiming to replay the raw acquisition. Validation without `--summary-only` checks the recorded raw input-artifact hashes. Outputs refuse overwriting existing reports. The frozen profile's validation field records its construction-time state; the separate held-out reports contain completed validation results.
