# Shared SITL frame correction and frozen-gain flight tests

The [previous flight-envelope comparison](TRICOPTER_FLIGHT_ENVELOPE_COMPARISON.md) completed 42 programs and interrupted all 12 forward turns at the 40° physical attitude limit while native estimated roll stayed below 1°. A shared gyro/attitude convention mismatch made those forward-turn results unsuitable for ranking the mixers. This record covers the correction and the subsequent tests with the same selected hover yaw gains, **P31 / I20 / D10 / F0**, hold gain zero.

## Correction scope

The [versioned common patch](/home/alex/betaflight/tools/tricopter/sitl/patches/corrected-frame.patch), SHA256 `7ce82ec3438216c6bc8046dab3ea5995918849a2ca278c7af854fc7589ad2807`, changes four existing files in isolated simulator source copies:

- `sitl_gyro.h` adds a conversion from native clockwise-positive yaw to the right-handed attitude convention, retaining the original PID gyro mapping.
- `imu.c` applies that conversion only to the local Mahony yaw-rate input in Gazebo SITL.
- `pid.c` applies the conversion only when ANGLE consumes the coordinated-turn yaw rate. Stored native yaw demand, yaw PID polarity and cosine attenuation remain unchanged.
- `sitl.c` corrects the ENU→NWU quaternion reference from +90° to −90°, including the virtual-magnetometer and optional direct-attitude references.

The live working-tree firmware files are unchanged. Hardware and legacy branches preserve their original formulas. Both experiment binaries use the **identical common SITL patch**; the comparison baseline is therefore labelled **stock controller with common SITL coordinate correction**, not a byte-unchanged upstream binary. No direct-attitude bypass is used: both run the real Mahony estimator. The tricopter mixer changes remain exclusive to the modified variant.

The build tool retains pristine stock input and verifies all 4,750 Git blobs. Separate content-addressed stock/modified snapshots are patched only after exact four-file preimage checks, and their complete contents, modes and symlink targets are retained in snapshot manifests. Exact postimages and patch bytes are pinned. Other historical build-critical source hashes must remain unchanged. Existing plain stock/modified binaries and prior experiment packages remain retained.

The first adapter preparation stopped before compilation because `git apply` rewrote archive permission modes under umask 0002. The builder now restores the original modes of the four approved regular files, then checks the complete tree and approved postimages. Real-patch tests cover both 0644 and 0664 inputs under that umask. No firmware experiment was run with that failed preparation.

## Native coordinate validation

Focused tests apply the actual patch to temporary copies, compile the real patched helper and extracted consumer blocks, and exercise Gazebo, legacy and hardware guards. They cover tilted CW/CCW quaternion propagation, coordinated roll/pitch signs, unchanged native yaw PID demand, and cardinal East/North/West/South reconstruction. This is compiled source-level guard validation, not an embedded hardware-target build.

Both corrected native SITL processes also pass [the coherent tilted-yaw sensor check](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261109T022205-both-corrected-attitude-check-460331): **40 segments**, ±20° roll/pitch and mixed tilt, both yaw directions. Maximum roll/pitch error norm is **0.1414°** for each firmware; maximum relative heading error is **1.4234° stock / 1.4683° modified**. The independent audit checks 2,820 samples and confirms the original native PID gyro/yaw polarity in 1,268 steady-rate samples.

These runtime checks establish relative attitude propagation. The shared flight configuration leaves `trust_mag=false`, so an initial East pose still reports heading zero; no artificial compass calibration is introduced. Absolute cardinal reconstruction is covered by the compiled quaternion test, not claimed as an absolute-heading result of these unmagnetized process checks.

Independent evidence: [patch/guard review](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/corrected-frame-review/independent-patch-review.json), [build/estimator-mode review](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/corrected-frame-review/independent-build-attitude-mode-review.json), [complete source snapshot review](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/envelope-review/corrected-frame-build-review.json), and [native estimator audit](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/corrected-frame-review/20261109T022205-both-corrected-attitude-check-460331/independent-native-attitude-audit.json).

## Flight protocol

After qualification, repeat the previous nine-case matrix with corrected binaries: hover, 1400 µs sustained climb and 20°/1380 µs accelerating forward flight, each with no-yaw, CW and CCW probes, three repetitions per firmware (**54 flights**). Then run the climb triplet at **1450 µs collective** (**18 additional rapid-climb flights**) and the forward triplet with a longer **6 s fixed lead** (**18 higher-speed forward flights**), again three repetitions per firmware. All **90 comparison flights** and all **six qualification flights** complete without an abort.

Every case starts with a fresh world/EEPROM and the original setup gates. Measurement inputs remain fixed and identical across variants: 1 s baseline, 3 s operating lead, 2 s yaw probe and 1.5 s yaw release at the same operating input. Yaw stick corresponds to ±60.30°/s before ANGLE earth-reference attenuation. All four flight channels override the external setup pilot throughout measurement; there is no native altitude/position autopilot, speed hold or gain retuning. The 40° attitude and 0.35 m ground-clearance protections remain. Normal RC disarm and simulator termination follow the observation; these disposable episodes are not landing tests.

The original 1400 µs cases support before/after interface comparisons. The additional 1450 µs cases test the frozen hover tune at a stronger sustained collective. Achieved speeds and rates of climb, attitude tracking, yaw response and actuator bounds are reported from the traces. RPM-only prop loads, linear drag, fixed voltage and approximate servo dynamics remain the same model assumptions as before; no new high-speed aerodynamic model is introduced.

## Results

The [six-flight corrected qualification](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261109T022645-both-corrected-frame-qualification-464007) passes all six programs. Both firmware variants complete the 2 s forward CW yaw probe that previously reached the attitude guard. Physical roll peaks are approximately 4.51° stock / 4.36° modified; physical pitch peaks are approximately 23.6°. The 1450 µs climb reaches approximately 19.3–19.4 m/s vertical speed and 83 m height, within the 200 m scenario bound.

This removes the severe handedness/propagation defect rather than making accelerated attitude estimation perfect. Qualification still shows approximately 4.2° sampled native-versus-truth attitude error during acceleration. Final analysis retains actual physical attitude, estimated attitude and achieved velocity separately. [Independent qualification audit and flight closure](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/corrected-frame-review/20261109T022645-both-corrected-frame-qualification-464007/native-truth-flight-closure.json).

The [54-flight corrected comparison](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261109T022947-both-corrected-frame-envelope-467162) completes **54/54 programs with no aborts**. All 12 forward turns complete their prescribed pulse and release, compared with 12/12 interrupted turns before correction. The independent execution audit verifies the identical common patch, unchanged aircraft and native gains, timing/ADC validity and final MSP disarm for every program.

The [additional 18-flight rapid-climb cohort](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261109T024822-both-corrected-frame-rapid-climb-481988) completes **18/18 programs** at 1450 µs collective. It uses the exact same frozen execution helper versions and builds as the corrected 1400 µs campaign.

### Corrected hover, climb and forward yaw responses

The table gives means across three repetitions, separated by direction. Rise is body-yaw 10–90% of requested rate, not a response-relative plateau. Overshoot is relative to that requested rate. Settling requires an observed continuously settled suffix within max(2°/s, 5% of request), with at least 0.25 s remaining. Forward body-yaw demand includes the unchanged ANGLE cosine attenuation to approximately ±56.7°/s; hover/climb demand is approximately ±60.3°/s.

| Variant / condition | CW / CCW rise, ms | CW / CCW overshoot, % | CW / CCW command settling, s | CW / CCW release settling, s |
|---|---:|---:|---:|---:|
| Stock hover | 193.6 / 191.6 | 1.65 / 1.85 | 0.273 / 0.267 | 0.270 / 0.273 |
| Stock 1400 climb | 143.6 / 148.1 | 2.99 / 2.94 | 0.210 / 0.213 | 0.210 / 0.210 |
| Stock forward | 163.8 / 167.3 | 6.20 / 6.23 | 0.530 / 0.530 | 0.230 / 0.227 |
| Modified hover | 139.3 / 139.5 | 2.77 / 2.87 | 0.200 / 0.200 | 0.200 / 0.200 |
| Modified 1400 climb | 123.3 / 125.3 | 3.33 / 3.41 | 0.180 / 0.183 | 0.180 / 0.180 |
| Modified forward | 135.6 / 138.1 | 5.57 / 5.67 | 0.390 / 0.387 | 0.190 / 0.190 |

Both controllers retain controlled yaw in these operating conditions. Modified remains faster, with a smaller rise-time change from hover to forward flight and lower forward overshoot/settling time; it has somewhat greater hover/climb overshoot. Stock does not become unstable in the tested climb or forward cases. These are as-configured comparisons with separately selected hover gains, not proof of a globally optimal tune or isolated attribution to feedback alone. Matched roll/pitch coupling and actual servo authority are reported separately.

![Corrected operating-point response summary](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261109T022947-both-corrected-frame-envelope-467162/envelope-response-summary.png)

![Corrected forward physical and estimated attitude](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261109T022947-both-corrected-frame-envelope-467162/envelope-forward-attitude.png)

Across the completed 54 programs, physical roll remains within approximately 5.68°, while the largest sampled native-versus-truth roll/pitch error norm is approximately 6.71° under acceleration. The severe tilted-yaw divergence is removed; residual estimator effects remain measurable. Model speeds, servo loads and translational aerodynamics retain their earlier approximation boundaries.

### Rapid-climb yaw injection

The 1450 µs controls enter the yaw window at approximately 13.2 m/s vertical speed, accelerate through approximately **13–17 m/s during the 2 s yaw pulse**, and reach approximately 19.3–19.4 m/s by the end of recovery. The pulse-average control climb speeds are 15.51 m/s stock and 15.64 m/s modified. Those phase distinctions matter: the whole yaw probe is not described as a 19 m/s steady climb.

| Variant | CW / CCW rise, ms | CW / CCW overshoot, % | CW / CCW command settling, s | CW / CCW release settling, s |
|---|---:|---:|---:|---:|
| Stock, 1450 climb | 116.6 / 118.7 | 4.34 / 4.22 | 0.173 / 0.180 | 0.177 / 0.193 |
| Modified, 1450 climb | 112.2 / 113.2 | 3.79 / 3.78 | 0.167 / 0.170 | 0.170 / 0.167 |

Both frozen hover tunes remain controlled. Relative to corrected hover, stock's mean rise time shortens by approximately 39%, while modified shortens by approximately 19%. Modified is therefore less sensitive to collective in this test, but neither has invariant response. The rise-time advantage narrows at high collective; modified has somewhat lower rapid-climb overshoot. No full-envelope optimum or physical gain-transfer claim follows from these finite tests.

Matched pulse roll/pitch angle-norm peaks are approximately 0.565/0.485° stock and 0.517/0.489° modified (CW/CCW), with small pre-event differences retained. This does not establish a consistent coupling winner. Actual servo excursions are about 5.13° stock and 5.24–5.27° modified. [Full rapid-climb report and repetition ranges](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261109T024822-both-corrected-frame-rapid-climb-481988/envelope-results.md).

### Higher-speed forward yaw injection

The original forward probe begins at approximately 8 m/s and averages about 10 m/s during yaw. To test the same tune at a higher entry speed, a final [18-flight forward-only cohort](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261109T025623-both-corrected-frame-fast-forward-488576) uses a **6 s fixed operating lead instead of 3 s**. Pitch/collective remain 20°/1380 µs and yaw/release remain 2 s/1.5 s. Both variants receive that same predeclared timing; no speed hold, adaptive delay, gain or physical coefficient is added. All other hover/climb waveforms remain unchanged.

The longer-lead cohort completes **18/18 programs**. Actual yaw probes run at approximately **12.25–14.30 m/s horizontal ground speed**; the matched no-yaw controls run at 13.45–15.57 m/s during the pulse and reach approximately 16.3 m/s by the end. Those are separate trajectories, not interchangeable speed measurements. Forward controls also climb at about 3 m/s. These fixed-body-pitch yaw turns follow curved trajectories; initial forward motion does not imply constant heading-aligned velocity throughout a 120° yaw maneuver. The comparison report separately measures body-heading and ground-velocity-course changes rather than treating these as coordinated racing turns.

| Variant, 6 s forward lead | CW / CCW rise, ms | CW / CCW overshoot, % | CW / CCW command settling, s | CW / CCW release settling, s |
|---|---:|---:|---:|---:|
| Stock | 160.4 / 165.4 | 8.53 / 8.30 | 1.067 / 0.898 | 0.224 / 0.217 |
| Modified | 133.8 / 136.4 | 7.41 / 7.20 | 0.765 / 0.630 | 0.189 / 0.192 |

Rise changes little from the shorter forward lead, but overshoot and command settling worsen in both controllers. Modified retains the shorter rise/settling times and lower forward overshoot; this does not establish invariant response over the envelope. All reported settling values are observed within the prescribed window, not extrapolated. [Full higher-speed forward report](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261109T025623-both-corrected-frame-fast-forward-488576/envelope-results.md).

Matched pulse roll/pitch norm peaks are approximately 8.74/8.77° stock and 8.53/8.80° modified. Small inter-variant differences do not support a consistent decoupling winner: raw pre-event mismatches reach 0.179° with differential slope up to 0.112°/s, and residual estimated-versus-physical attitude errors reach approximately 6.96° stock / 6.82° modified under acceleration. These sampled native errors use the original approximately associated 10 Hz state times, without fitted lag correction. The unaccelerated coherent sensor check's 0.14° error must not be substituted for flight estimator accuracy.

![Forward entry-speed comparison](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/corrected-frame-review/final-envelope-comparison/corrected-forward-entry-speed.png)

![Frozen hover gains at two climb collectives](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/corrected-frame-review/final-envelope-comparison/corrected-climb-vs-hover.png)

## Artifacts, final checks and reproduction

The [four-cohort comparison](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/corrected-frame-review/final-envelope-comparison/README.md) retains the old defect baseline, corrected 1400 µs cases, rapid climb and higher-speed forward cases, with N3 mean/min–max tables, actual speeds, servo authority, body heading versus velocity course and native attitude diagnostics. The legacy artifacts remain unchanged. Each corrected cohort also retains JSON/CSV, exact analysis sources, five standalone PNG/SVG figure families, the full RC/plant/native logs and copied firmware binaries/manifests.

Across all completed cases and controls, no motor endpoint, servo endpoint/torque-limit or motor terminal-current-limit occupancy is observed in the **nominal 100 Hz full-program samples**. This is a sampled result, not a statement about sub-sample transients: full trace histories can reach the declared braking current bound. Current, energy and speed figures refer to the fixed-voltage, RPM-only propeller and linear-drag model. Propeller unloading/inflow versus forward speed is not modelled, so these trials do not validate real-aircraft high-speed aerodynamics or physical PID transfer.

The [aggregate independent audit](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/corrected-frame-review/independent-corrected-campaign-audit.json) passes **90/90 comparison flights, 6/6 qualification flights and 40 native sensor segments**. It verifies identical common patch/builds, source snapshots and old physical inputs; all native P31/I20/D10/F0/hold0/earth-reference readbacks; 72,736 valid measured actuator frames; 5,326 healthy native status samples; 2,559 valid modified feedback samples; and 90 final MSP disarm acknowledgments. PC0 remains zero throughout 151,037 retained ADC history rows, with PB0 matching actual joint position to within 0.5-count rounding. Maximum measured stage-duration error is 20 ms and maximum active RC send gap is 22.52 ms. Final host inspection finds no owned processes or active simulator sockets.

The three [independent raw-response audits](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/envelope-review/20261109T022947-both-corrected-frame-envelope-467162/independent-response-math.md) independently reproduce **1,260 numeric yaw/speed measurements** across all 90 trials, with maximum difference 1.42×10⁻¹⁴, and compare rotation-Jacobian reconstruction to raw-angle finite differences over **65,276 samples**. [Rapid-climb audit](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/envelope-review/20261109T024822-both-corrected-frame-rapid-climb-481988/independent-response-math.md); [higher-speed forward audit](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/envelope-review/20261109T025623-both-corrected-frame-fast-forward-488576/independent-response-math.md). All final audits bind the final analysis bytes after plot-layout validation.

The **144 Python tests** pass, including frame-consumer/guard compilation, builder/snapshot integrity, stale-build rejection and the declared climb/forward waveform changes. The runner's three native Gazebo CTests also pass. No embedded hardware-target build or physical aircraft test is claimed.

Default SITL firmware build, interactive launch, native protocol/estimator checks and free-flight checks now select the corrected-frame binaries. The anchored plant-only check continues to use the shared canonical plant plugin, since it loads no firmware. Historical plain builds remain explicit alternatives and cannot overwrite corrected outputs. The standalone check defaults were aligned only after all experiment guards and audits completed; their frozen execution snapshots remain unchanged.

From the repository root:

```sh
# Both defaults use the identical common simulator correction.
python3 tools/tricopter/sitl/build.py
python3 tools/tricopter/sitl/check_attitude.py

# Reuse the completed hover tuning as gain provenance; no new tuning occurs.
python3 tools/tricopter/sitl/check_envelope.py
python3 tools/tricopter/sitl/check_envelope.py --climb-throttle 1450 \
  --cases climb_control,climb_cw,climb_ccw
python3 tools/tricopter/sitl/check_envelope.py --forward-lead 6 \
  --cases forward_control,forward_cw,forward_ccw
python3 tools/tricopter/sitl/plot_envelope.py /absolute/path/to/completed/run
```

Use the selected simulator yaw overlays in [the SITL README](/home/alex/betaflight/tools/tricopter/sitl/README.md) for manual controller flying. The experiments do not change the physical FC or silently retune the shared interactive profiles.
