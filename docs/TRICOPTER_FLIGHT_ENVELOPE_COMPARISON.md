# Frozen hover gains: sustained climb and forward-flight comparison

This extends the [independent hover yaw tuning](TRICOPTER_YAW_AUTOTUNE.md) without changing gains, firmware binaries or the aircraft plant. Stock and modified retain their independently selected yaw **P31 / I20 / D10 / F0**, with yaw hold gain zero. Roll/pitch gains, ANGLE settings, motor braking and all physical coefficients remain fixed.

## Protocol

The main comparison schedules **54 fresh simulated flights**: three repetitions of hover, sustained climb and rapid forward-flight inputs, each with a no-yaw control and CW/CCW yaw probes, for both firmwares. Variant group order is stock/modified/modified/stock/stock/modified. Each individual flight starts with a fresh world and EEPROM, using the same hover setup gates as the earlier campaign. All 54 planned cases remain in the denominator, including interrupted or unsuccessful cases.

| Phase | Duration | Input |
|---|---:|---|
| Baseline | 1 s | Level roll/pitch, collective 1345 µs, zero yaw |
| Operating lead | 3 s | Fixed operating-point roll/pitch/collective, zero yaw |
| Yaw probe / matched control | 2 s | Same operating input, yaw stick approximately ±60.3°/s or zero |
| Yaw release | 1.5 s | Same operating input, zero yaw |

| Operating point | Collective | Pitch input |
|---|---:|---:|
| Hover | 1345 µs | 1500 µs, level |
| Sustained climb | 1400 µs | 1500 µs, level |
| Forward flight | 1380 µs | 1903 µs, approximately 20° native ANGLE pitch target |

Roll remains 1500 µs. The RC sender overrides all four flight channels atomically throughout the entire baseline/lead/probe/release program. There is no external speed, position, altitude or heading hold in that interval, and native autopilot modes stay disabled. The same predeclared lead duration applies to both variants; no flight is delayed until it reaches a favorable operating point. Achieved speed, climb rate, acceleration and attitude are measured rather than inferred from stick input. Forward yaw probes describe curved flight with a fixed body pitch command, not straight flight with independent heading control.

The setup pilot's original 3 m altitude and comparison runner's 5 m horizontal limits are extended to 200 m for these disposable simulator episodes; the 40° attitude protection and 0.35 m measurement ground clearance remain. After the release observation, each simulator receives a normal RC disarm, records its final status and stops its owned processes. This is explicitly simulated in-air termination, not a landing test. No physical FC connection or motor/servo output override is used.

All failed attempts remain recorded; there is no gain retuning or selective retry of poor performance. Identity drift aborts the campaign. Execution faults are retained and other planned cases continue. Actual stage/send timestamps, full traces, native PID settings, raw ADC and source/model/binary hashes are preserved.

## Coordinate and measurement definitions

Native ANGLE earth reference remains 100. Betaflight attenuates the body-yaw PID target by the cosine of the largest commanded roll/pitch angle and adds coordinated roll/pitch rate terms. Thus approximately 60.3°/s stick yaw at the final 20° pitch target requests approximately 56.7°/s body-FRD yaw. The integer pitch command gives a reconstructed held-command native target of approximately 19.98°. The native earth-reference field is queried and verified rather than assumed. Body-yaw tracking uses this corrected held-command reference. Transient RC and angle filtering are not reconstructed from the plant response.

World heading rate is reported separately. Nonzero body roll/pitch gyro rates in a tilted turn can be required by coordinate kinematics. Euler attitude tracking and matched no-yaw controls describe unwanted roll/pitch movement; body-rate peaks are diagnostics, not automatically coupling errors. Matched trajectories are aligned by actual command age, with pre-event mismatch and slope retained. Shaded plot ranges describe three repetitions, not confidence intervals.

The current plant uses RPM-only thrust/prop torque and a fixed linear translational drag coefficient. It does not include advance-ratio/inflow-dependent propeller aerodynamics. The default SITL build runs the real Mahony attitude estimator against virtual gyro/accelerometer/magnetometer feeds. `USE_IMU_CALC` is enabled in `src/main/target/common_pre.h`; the apparent `#undef USE_IMU_CALC` in the SITL target header is conditional on an explicit `SITL_ATTITUDE_DIRECT` build. The tested default binaries do not select that bypass. Firmware attitude can therefore differ from Gazebo ground truth during acceleration; both are retained and compared. The existing servo and braking models remain approximations. This comparison measures the current model's behavior at different operating conditions; its speeds and PID values are not validated real-aircraft predictions.

## Shared SITL attitude inconsistency

The [independent attitude-mode and transform audit](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/envelope-review/attitude-mode-and-transform-audit.json) verifies the real estimator in **both** pinned binaries using preprocessing and ELF disassembly. It also supplies a reproducible consistency counterexample for the angular-rate input chain:

1. [The bridge](/home/alex/betaflight/tools/tricopter/sitl/gazebo/TricopterMath.hh:284) converts body FLU angular velocity to packet FRD as `(wx, -wy, -wz)`.
2. [The native SITL adapter](/home/alex/betaflight/src/platform/SIMULATOR/sitl_gyro.h:45) converts that packet to native gyro as `(wx, +wy, -wz)` relative to the original FLU vector.
3. [Mahony receives those native gyro values](/home/alex/betaflight/src/main/flight/imu.c:767), while the bridge/adapter restores accelerometer specific force to FLU. The remaining single-axis gyro reflection has determinant −1 and cannot represent the same frame through a proper rotation.

For constant ground-truth 20° pitch and clockwise 60°/s world yaw with zero Euler roll, the independent gyro-only example predicts a false roll derivative of approximately **41.0°/s**. After 100 ms, truth remains roll 0° / pitch 20° / world yaw −6°, whereas native propagation gives roll +4.095° / pitch 19.798° / quaternion yaw +5.992°. This demonstrates inconsistent kinematics in the shared simulator input chain. It is a consistency counterexample, not a complete causal replay of the closed-loop bank excursions.

The current experiment preserves both firmware binaries and this input chain. Forward-turn failures therefore establish an operating boundary of the present simulator integration; they cannot establish an allocator disadvantage or predict the real aircraft's coupled response. Correcting and validating the shared attitude/sensor conventions, or using a separately labelled common direct-attitude build, requires a new comparison reference rather than replacing these retained results.

## Retained qualification attempts

Qualification is separate from the final N3 study. It checks execution and whether the chosen RC input reaches a useful operating condition, without changing the hover PID gains, firmware binaries, plant or measurement protections. The artifacts below retain both the original failures and the reason for the one shared forward-input adjustment.

| Attempt | Inputs and outcome |
|---|---|
| [399603 — initialization](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261109T010939-both-envelope-qualification-399603) | A harness `KeyError('plugin_sha256')` occurred before any flight or simulator process started. The historical plugin hash is stored in each firmware group, rather than at the summary's top level. The lookup was corrected and a focused regression test added. This attempt contains zero flights. |
| [400580 — six-flight qualification](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261109T011025-both-envelope-qualification-400580) | Hover CW and climb CW completed for both firmwares: four completed flights. The original forward input, 25° native pitch and 1362 µs collective, reached the 0.35 m ground-clearance protection during the yaw probe in both variants: two interrupted flights. Firmware pitch remained near 24.5° while ground-truth pitch differed substantially. |
| [404561 — revised forward qualification](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261109T011502-both-envelope-forward-qualification-404561) | The shared forward input was reduced to 20° native pitch and increased to 1380 µs collective. Both CW probes exceeded the unchanged 40° ground-truth attitude protection during the yaw pulse. At the final planned-input samples, stock/modified ground-truth roll was approximately −40.6°/−40.1°, while the latest firmware roll reports were both approximately −0.2° (40–70 ms older). These are two interrupted flights, not successful forward-response measurements. |

The final main study retains **20° / 1380 µs** for all forward cases. It does not relax the 40° protection, shorten the yaw probe to avoid the observed failure, change to direct ground-truth attitude, or retune either firmware. The mismatch between estimated and actual attitude is an observed limitation of these accelerated flights; it does not by itself isolate a mixer defect or establish an estimator root cause. It affects interpretation of both variants.

Interrupted records retain raw plant, receiver and firmware telemetry. Their command timestamps come from the transmitted RC ledger even when the normal completed-stage summary is absent. An aborted yaw probe cannot establish complete rise, overshoot or settling performance over the prescribed window. Such values remain unavailable or censored, and partial trajectories must be labelled as interrupted rather than included among successful full responses.

## Reproduction and results

From the repository root:

```sh
python3 tools/tricopter/sitl/check_envelope.py --dry-run
python3 tools/tricopter/sitl/check_envelope.py
```

The runner requires the completed hover campaign referenced by its default `--reference`, or an explicitly supplied compatible campaign. It verifies physical inputs and per-variant binaries against that record. A preliminary six-flight qualification used only CW cases at each operating point; the retained additional forward qualification is described above.

The [main 54-case campaign](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261109T011856-both-frozen-hover-envelope-408126) finished all scheduled attempts: **42 complete programs and 12 interrupted forward turns**. Hover and climb each completed all 18 flights; the six forward no-yaw controls completed. Every forward CW/CCW probe, three repetitions per direction per firmware, exceeded the unchanged 40° ground-truth attitude protection before yaw release.

The runner exits 1 and preserves `pass: false` because some flight programs failed. This is not an unfinished campaign: all 54 outcomes remain recorded and all owned simulator processes were stopped. The separate analysis reports outcome-ledger integrity independently of flight success. Failed programs retain partial trajectories rather than fabricated full-step metrics; successful programs retain their final native disarmed status.

## Hover versus sustained climb

The values below are means across three repetitions. Gains remain P31 / I20 / D10 / F0 for both firmwares; requested level-flight body yaw is approximately ±60.30°/s. Rise thresholds refer to the requested rate, not each observed plateau. Settling uses max(2°/s, 5% of requested rate), with a continuously settled observed suffix of at least 0.25 s.

| Firmware / direction | Hover rise, ms | Climb rise, ms | Hover overshoot, % | Climb overshoot, % | Hover command settling, s | Climb command settling, s |
|---|---:|---:|---:|---:|---:|---:|
| Stock CW | 194.44 | 143.67 | 1.685 | 3.008 | 0.273 | 0.210 |
| Stock CCW | 192.07 | 147.19 | 1.856 | 3.095 | 0.263 | 0.210 |
| Modified CW | 139.89 | 123.88 | 2.787 | 3.178 | 0.200 | 0.180 |
| Modified CCW | 140.37 | 124.53 | 2.986 | 3.350 | 0.200 | 0.183 |

Averaging the two directions, stock rise changes from **193.3 to 145.4 ms** (24.7% shorter), while modified changes from **140.1 to 124.2 ms** (11.4% shorter). Thus the modified controller's response changes less with this collective increase. Both controllers remain controlled in the tested climb: stock does **not** fall apart at this operating point. Overshoot grows in both, and the modified controller still has somewhat greater overshoot. These are finite, model-specific comparisons with independent hover tunes, not isolated attribution to angle feedback or a global controller ranking.

After yaw release, stock settles in 0.270 s at hover and 0.217/0.210 s in climb (CW/CCW). Modified settles in 0.200 s at hover and 0.190/0.183 s in climb. All these complete-program settling values are observed rather than censored. No comparable forward-turn release measurement exists because those programs abort before release.

![Complete and interrupted yaw responses](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261109T011856-both-frozen-hover-envelope-408126/envelope-yaw-rate.png)

![Hover/climb response summary](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261109T011856-both-frozen-hover-envelope-408126/envelope-response-summary.png)

## Achieved operation, coupling and authority

The no-yaw climb controls average **6.31/6.39 m/s** vertical speed in the last 0.3 s before the probe marker (stock/modified), and **7.66/7.76 m/s** during the 2 s probe window. They continue accelerating; this is a sustained collective input, not regulated steady climb speed. Full-program height gains are approximately 40.0/40.5 m.

The forward controls average **8.01/7.98 m/s** horizontal speed before the probe and **10.32/10.29 m/s** during it, reaching approximately **14.27/14.25 m/s** at the end. They are also climbing: probe-window vertical speeds average 2.80/2.92 m/s. Physical pitch changes from approximately 22° before the probe to 26.7° at the end, despite the held 19.98° native command. These are accelerating pitched-flight comparisons, not level, constant-speed flight. Ground-speed and vertical-speed traces retain this distinction.

Matched no-yaw controls quantify the extra physical Euler roll/pitch movement during yaw. Means of the peak angle-vector norms over the yaw pulse are:

| Operating point / direction | Stock, ° | Modified, ° |
|---|---:|---:|
| Hover CW | 1.004 | 0.946 |
| Hover CCW | 1.322 | 1.456 |
| Climb CW | 0.309 | 0.321 |
| Climb CCW | 0.673 | 0.907 |

There is no consistent roll/pitch coupling improvement. The largest raw pre-event mismatch is approximately 0.0365° in hover and 0.0582° in climb; the corresponding largest differential roll/pitch slope norms are 0.0813/0.0226°/s. These diagnostics and the shared attitude inconsistency limit attribution. The figures retain prehistory rather than presenting subtraction as identified physical coupling coefficients.

![Matched yaw-induced roll/pitch differences](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261109T011856-both-frozen-hover-envelope-408126/envelope-attitude-coupling.png)

The modified mixer still uses more basket excursion: approximately 7.50–7.52° versus stock's 6.54–6.64° in hover, and approximately 6.16° versus 5.68° in climb, relative to each pre-yaw baseline. Thus raw PID values do not equalize effective actuator authority. During completed hover/climb probe windows, no motor endpoint, servo endpoint/torque-limit or drive/braking current-limit occupancy was observed in the nominal 100 Hz samples. This is a sampled observation, not proof about sub-sample extrema or hardware saturation.

## Interrupted forward turns

All **12/12** forward turns stop during the yaw pulse. Observed prefixes last **1.21–1.38 s** after yaw onset. Their physical roll approaches ±40°, while sampled native roll peaks are only **0.6–0.9°**. At associated telemetry times the peak native-versus-truth roll error is approximately 37–40°. The trajectories are closely matched between the two firmware variants; the shared gyro/IMU convention inconsistency dominates this result.

An observed prefix ends at the first subsequent disarm packet's associated state time. This can be just before the first ≥40° sample in the independent 100 Hz plant trace. Consequently a precise sampled threshold-crossing timestamp may be unavailable even when the runtime guard reports the 40° violation. No crossing is extrapolated. Native MSP attitude samples are about 10 Hz with quantization and state-association latency, so these are diagnostic sampled errors, not synchronized full-rate estimator errors.

No forward turn provides a complete prescribed yaw pulse or release observation. Full-window rise/overshoot/tracking and release/settling statistics are therefore unavailable for those cases; they are not replaced with the successful prefix or extrapolated from the last sample.

![Forward-turn actual versus estimated attitude](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261109T011856-both-frozen-hover-envelope-408126/envelope-forward-attitude.png)

## Artifacts and verification

The [full numerical report with repetition ranges](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261109T011856-both-frozen-hover-envelope-408126/envelope-results.md), [JSON analysis](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261109T011856-both-frozen-hover-envelope-408126/envelope-analysis.json) and [aggregate CSV](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261109T011856-both-frozen-hover-envelope-408126/envelope-aggregate.csv) retain all outcomes. The five figure families are `envelope-yaw-rate`, `envelope-response-summary`, `envelope-achieved-operation`, `envelope-attitude-coupling` and `envelope-forward-attitude`, each available as standalone PNG and SVG. [Achieved speed and climb-rate plots](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261109T011856-both-frozen-hover-envelope-408126/envelope-achieved-operation.png) show actual operation throughout the fixed inputs. Figures use three repetitions per variant/case, with min–max ranges rather than confidence intervals; interrupted trajectories remain individually labelled.

Analysis reproduction after the campaign:

```sh
python3 tools/tricopter/sitl/plot_envelope.py /absolute/path/to/completed/campaign
```

An analysis pass means all scheduled outcomes were recognized and processed without an analysis error. It does not mean every flight succeeded. `--allow-partial` produces explicitly provisional artifacts while a campaign is running. Inputs consumed by analysis and the analysis source are hashed separately from the frozen execution sources.

The **114 Python tests** and the runner's three native Gazebo CTests pass. The [independent response/math audit](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/envelope-review/20261109T011856-both-frozen-hover-envelope-408126/independent-response-math.md) checks all 54 outcome records, reproduces 528 yaw and achieved-operation values (maximum difference 2.13×10⁻¹⁴), and compares an independent rotation-Jacobian reconstruction against world-angle finite differences over 33,661 samples (maximum trial RMS component error 0.0580°/s). Those finite differences exclude 100 ms around command edges/termination and fit no time offset. Failed forward responses remain excluded from full-step statistics.

The [independent execution audit](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/envelope-review/20261109T011856-both-frozen-hover-envelope-408126/independent-envelope-audit.json) passes all 54 attempted records with zero integrity issues, checking 46 frozen execution inputs and 32 plugin/model sources. All 54 native yaw gain, hold-gain and earth-reference readbacks agree. Completed windows contain 31,564 valid actuator frames and 2,300 healthy armed status samples; interrupted prefixes contain another 6,339 valid actuator frames and 461 healthy status samples. PC0 remains zero and PB0 follows the actual joint to within 0.5-count rounding. Maximum stage-duration error is 20 ms and maximum active-program RC send gap is 21.44 ms. The [case ledger](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/envelope-review/20261109T011856-both-frozen-hover-envelope-408126/independent-case-ledger.csv) includes every attempted case.

Final MSP disarm acknowledgments are verified for the 42 completed programs. For the 12 interrupted programs, the evidence is the observed low-throttle/AUX-disarm commands and owned-process cleanup; no missing final-status acknowledgment is invented. A [read-only host process/socket inspection](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/envelope-review/20261109T011856-both-frozen-hover-envelope-408126/host-process-socket-cleanup.json) across the main and three retained qualification attempts found no remaining campaign processes or active simulator sockets.

The next useful step is to correct and validate the shared simulator sensor/attitude conventions, establish a new hover reference with these gains fixed, and repeat the envelope comparison. Further PID tuning before resolving that inconsistency would risk tuning around a simulator defect.
