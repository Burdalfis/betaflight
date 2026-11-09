# Tricopter yaw authority correction and nonlinear-drag comparison

This revision removes the measured-position allocator's high-collective yaw lockout. Selecting `mixer TRI` continues to use the feedback allocator by default. The current basket angle still comes exclusively from corrected **PB0−PC0**. Motor order, rotation conventions, native PID code, servo hardware and the three-point calibration remain unchanged.

## Correction

The previous allocator restricted the target angle to `±acos(requested_tail_vertical)`. At full requested tail effort this becomes zero, so even a configured nonzero operating trim is clamped away. The servo cannot oppose the steady rotor reaction torque. Adding trim alone does not repair that constraint.

The replacement bounds servo targets by the calibrated mechanical/PWM endpoints. At the **measured** angle θ, let `c=cos(θ)` and requested normalized tail vertical effort be `V`. The allocator uses:

```text
common_scale = min(1, c/V)                 # 1 when V <= c
motor_vertical[i] *= common_scale         # all three motors
tail_motor_effort = motor_vertical[0]/c
target_angle = trim + atan2(yaw_gain * signed_yaw_PID,
                           max(V * common_scale, minimum_vertical))
```

This reserves enough motor headroom to compensate the measured tail tilt, accepting reduced collective. The common multiplier preserves requested roll/pitch vertical-effort ratios in Betaflight's normalized mixer convention. These quantities precede native thrust linearization and endpoint scaling; they are not measured forces in newtons. Current vector compensation never uses a predicted or requested future angle.

Collective shortfall and mechanical/PWM yaw saturation are reported separately. Native PID integration, gains and limits are unchanged. The old servo-saturation hook has no callers in the pinned native PID; this correction does not claim to repair a previously frozen integral term.

The simulator's heading-balanced hover gives **−6.1396°** measured basket angle, agreeing with the independent static force/torque solution of **−6.1375°**. The modified simulator preset therefore explicitly sets **`tri_yaw_trim=-61`**, or −6.1°. Both the archived old allocator and the corrected allocator receive that same trim in before/after trials. The generic firmware default remains zero because this trim belongs to the airframe and rotation configuration.

Physical zero stays **PWM1509 / ADC1490 / 0°**, and the endpoints stay 1100/1700 µs, −47.8/+21.6°. A trim is an operating angle, not a redefinition of the sensor or PWM map. With zero yaw PID demand, focused tests prove that the trim remains commanded as collective changes. The recorded trials do not log the separate PID integral component.

The user's cancellation argument holds when reaction torque and the relevant rotor thrusts scale together with a fixed distribution: their ratio, and therefore the required basket angle, remains constant. Acceleration/braking torque, changing rotor distributions and aerodynamic yaw moments need not follow that relationship. The existing PID supplies those residual corrections; prediction, inertial reaction feedforward and finite-horizon optimization remain absent.

## Shared plant and visuals

Both controllers now use a new common body-axis quadratic drag law:

```text
F_body[i] = -0.5 * rho * CdA[i] * abs(v_body[i]) * v_body[i]
rho = 1.225 kg/m³
CdA = [0.010, 0.012, 0.025] m²             # body FLU axes
```

The coefficients are explicit **unmeasured engineering placeholders**. Drag is dissipative and attitude-dependent. Its resultant acts through the instantaneous whole-aircraft CG, including basket motion; mass-proportional application at the four physical link CGs avoids an invented drag moment. The existing angular drag is retained. `airframe_drag_model=linear_world` reproduces the historical linear-force branch. Previous linear-drag campaigns remain separate evidence.

The new model has no propeller inflow, aerodynamic lift or resolved flow. Its approximately 43 m/s forward operating speed is a synthetic model result, not a claim about the real craft. Motor electrical/propeller dynamics, active braking, servo dynamics, rotor inertia and all physical mass/CG/inertia inputs are unchanged. A 56-field preservation check, independent force/moment/power checks and CTest cover that boundary. See [the complete aerodynamic assumptions](../tools/tricopter/sitl/gazebo/model/airframe_aerodynamics.json).

Three thin translucent propeller discs replace the blade visuals. They remain attached to their motor links, including the moving tail carrier. Disc radius is 64.77 mm; they add no collision geometry, mass, inertia or aerodynamic force. The original prop meshes remain available as provenance.

## Comparison protocol

All trials use Betaflight 2026.6.2 and the same explicitly recorded four-file SITL coordinate correction. “Stock” means the upstream stock controller with that common simulator correction, not a byte-pristine upstream executable. Stock keeps its native tricopter mixer. The corrected native allocator differs from the archived modified controller only in five flight/mixer source files; native PID and physical calibration sources are identical.

Each flight starts with a fresh world and EEPROM. All four primary RC channels are overridden by the fixed measurement program; the external pilot only handles setup/recovery. Common world, plugin, receiver, roll/pitch configuration, timing guards and native first-arm gyro calibration are verified from retained artifacts. Firmware/source/adapter hashes and native gain/trim readbacks are checked for every trial. Programs retain every failed attempt.

| Profile | Yaw P / I / D | Feedforward | Heading hold gain |
|---|---|---:|---:|
| Betaflight factory yaw | 45 / 80 / 0 | 120 | 15 |
| Frozen hover tune | 31 / 20 / 10 | 0 | 0 |

These are the existing profiles, with no retuning for the new allocator or drag. The factory profile deliberately includes the native factory feedforward; the hover-tuned comparison uses zero feedforward as requested in the earlier tuning study.

The throttle programs retain the earlier nominal 70/85/100% punchouts and 90/95/100% holds. The new drag law requires a new **shared** forward roll/pitch trim: RC1332/1903, throttle1715. Both controllers use these same inputs. The old linear-plant forward trim is not reused as if it were a level-flight point. CW/CCW probes retain RC1368/1632 and the existing three-second pulse.

Throttle case percentages name the RC demand above `min_check`. Native integer throttle-lookup quantization subsequently gives mixer collective 0.699/0.849/0.899/0.949/1.000 for RC1715/1858/1905/1953/2000. These are not identical to modeled ESC duty or force. The retained entry-gate field named `native_collective_at_least_70_percent` checks the pre-lookup demand; the separate actual mean motor-duty check uses observed PWM. The forward case therefore requests nominal 70%, not an exact post-lookup 0.700.

Forward setup starts at 20 m and uses the unchanged hover acceptance gate. The fixed forward lead lasts 25 s. The strict two-second steady-entry gate retains its original acceleration, speed, vertical-motion and angular-rate thresholds. A gentle curved trajectory can have almost constant speed while failing its acceleration threshold; such a trial is not reported as qualified steady straight flight.

## Results and artifacts

The main comparison grid contains **60 completed flights**: 20 throttle-envelope cells, 12 repeated 95% holds, 12 forward control/yaw cells and 16 high-collective yaw cells. Four preflight recording failures remain alongside that grid. Including the engineering check, forward qualifications, failed sandbox launch and illustrative video retakes gives 85 indexed attempts, 69 completed programs and 16 infrastructure aborts. These counts exclude the separate initial heading-balance hover. `study-index.json` binds every retained ledger; execution completion and straight-flight qualification remain separate fields.

The initial matched before/after 95% engineering trial already distinguishes the defect from tuning: with the frozen hover tune and the same −6.1° trim, settled body-yaw RMS was **383.26°/s** for the archived allocator, **0.239°/s** after the correction, and **0.232°/s** for stock. The old allocator drove its target back toward zero despite the trim; the corrected allocator retained approximately −6.14° measured basket tilt.

At that operating point the new common reserve is approximately **0.574%** of normalized motor effort. This is specifically the allocator's measured-angle reserve, not the total collective reduction already introduced by native roll/pitch allocation. High-throttle transition kicks remain and are included in the full-window metrics.

The completed 20-flight throttle envelope has one fresh trial per controller/profile/case. “Settled” excludes the first 0.5 s after the throttle change; for one-second punchouts this leaves only 0.5 s and is not a steady-state qualification.

| Profile | Case | Stock body-yaw RMS, °/s | Corrected body-yaw RMS, °/s |
|---|---|---:|---:|
| Factory | 70% punch | 0.598 | 0.298 |
| Factory | 85% punch | 11.822 | 0.314 |
| Factory | 100% punch | 212.660 | 0.290 |
| Factory | 90% hold | 207.615 | 0.212 |
| Factory | 100% hold | 219.148 | 0.258 |
| Hover-tuned | 70% punch | 0.154 | 0.176 |
| Hover-tuned | 85% punch | 0.293 | 0.335 |
| Hover-tuned | 100% punch | 0.316 | 0.438 |
| Hover-tuned | 90% hold | 0.171 | 0.214 |
| Hover-tuned | 100% hold | 0.297 | 0.248 |

The retained scorer flags sustained oscillation in the factory-stock 90/100% holds. It does not flag sustained oscillation or unwanted spin for the corrected controller. **The hover-tuned stock controller also remains quiet**: these results do not establish an unavoidable stock instability after tuning.

The correction is not uniformly better during transitions. With the hover tune, the full 100% punch window has yaw RMS **22.83→28.42°/s** and peak **99.27→113.90°/s** for stock→corrected. The brief acceleration/braking kicks remain; inertial feedforward was deliberately excluded. The corrected factory-gain controller also retains approximately 4° late recovery roll/pitch angle norm after the long holds, versus approximately 0.9–1.4° for stock; the tuned controllers both have approximately 3.95°. Those are trajectory/controller responses, not isolated measurements of aerodynamic coupling.

The 20-flight execution audit verifies 13,979 transmitted measurement frames, maximum send gap 23.712 ms, maximum stage-duration error 20 ms, matching plant/settings, valid electrical/feedback samples and final disarmed PWM1000 for every completed flight. PWM1000 does not assert an instantaneously stopped coasting rotor.

### Repeated 95% hold

The 95% confirmation has **three measured fresh trials per controller/profile**, assembled from eight valid original trials and four replacement video trials. All 16 attempts remain indexed: 12 completed flights and four preflight capture-window failures. Only the recorder helper changed between cohorts; the native controllers, common settings, plant and measurement program match. Recording-world paths differ by cohort root; independent XML comparison verifies equality after that path relocation alone.

| Profile | Controller | Settled body-yaw RMS mean (range), °/s | Full high-window yaw RMS mean, °/s |
|---|---|---:|---:|
| Factory | Stock | 219.838 (219.058–220.913) | 212.518 |
| Factory | Corrected | 0.264 (0.244–0.280) | 15.155 |
| Hover-tuned | Stock | 0.234 (0.229–0.237) | 12.159 |
| Hover-tuned | Corrected | 0.263 (0.234–0.320) | 15.079 |

Ranges describe three observed trials, not confidence intervals. All 12 pass independent execution, electrical/feedback, timing and final disarm checks. The corrected allocator's settled status includes vertical reserve and clears yaw saturation. Its transient penalty with the hover tune is visible in the full-window column.

### Fast forward flight with yaw pulses

All **12 forward programs completed**, with one control/CW/CCW flight per controller/profile. Entry speeds were **42.84–42.93 m/s**. However, **0/12 passed the strict straight steady-flight gate**: acceleration RMS was 0.641–0.685 m/s², dominated by turning, and three entries also narrowly exceeded the 1°/s roll/pitch-rate threshold. First-arm gyro calibration did not remove the curvature. The native versus ground-truth entry roll/pitch error was approximately 5.7–6.0°; the cause is not isolated by this study.

The held native ANGLE body-yaw reference is ±36.054°/s. It is different from physical world-heading rate at this tilt. Metrics below use that native body reference; they do not impose an invented world-heading target.

| Profile / direction | Overshoot stock → corrected | Rise 10–90%, ms | Settling, ms | Body-error RMS, °/s |
|---|---:|---:|---:|---:|
| Factory CW | 52.84% → 35.02% | 20 → 28 | 390 → 290 | 4.530 → 3.682 |
| Factory CCW | 46.46% → 33.88% | 22 → 28 | 320 → 290 | 3.910 → 3.948 |
| Hover-tuned CW | 11.07% → 5.73% | 69 → 90 | 180 → 200 | 3.555 → 3.798 |
| Hover-tuned CCW | 10.80% → 4.58% | 70 → 94 | 190 → 120 | 4.064 → 4.002 |

Rise/overshoot use the actual first RC packet and each trial's own preceding 0.5 s baseline. Settling requires a recorded 0.25 s suffix within the larger of 2°/s or 5% of target. The 100 Hz trace makes the fastest factory rises only two or three samples long; millisecond differences are descriptive interpolations, not precision timing measurements.

The corrected controller reduces observed overshoot in both directions/profiles, with slower rise and mixed settling/error differences. **Tuned stock is already well damped.** No sampled servo endpoint or modified allocation cap appears in these forward probes.

Matched-control-subtracted physical roll/pitch excursions are approximately **9–10.5° during the pulse and 34–47° during recovery**, with similar trends for both controllers. Native versus ground-truth recovery attitude errors also become large. These include the altered trajectory, attitude estimation and anisotropic drag after the turn; they are not isolated PID cross-axis terms. Good body-yaw tracking does not establish satisfactory physical attitude control or qualified straight flight. The recovery interval must not be hidden by reporting only pulse metrics.

Twelve earlier sandbox attempts failed before takeoff and are retained separately as infrastructure failures. The successful host cohort is not merged with them as controller outcomes. No acceptance gate was relaxed, and no neighboring trim trial was selected to improve the result.

### Explicit yaw authority at 95% and full collective

Sixteen additional fresh flights apply CW/CCW yaw commands during nominal 95% and 100% holds, for both controllers/profiles. All pass the independent execution audit. Their held native body target is ±60.2976°/s. This tests active yaw authority rather than only a zero-demand trim.

With the hover tune at full collective:

| Direction | Settled body-error RMS stock → corrected, °/s | Overshoot stock → corrected | Rise stock → corrected, ms | Settling stock → corrected, ms |
|---|---:|---:|---:|---:|
| CW | 0.217 → 0.253 | 26.4% → 6.3% | 31 → 55 | 120 → 150 |
| CCW | 0.185 → 0.109 | 23.1% → 6.2% | 32 → 55 | 120 → 170 |

The corrected allocator retains nonzero measured basket angles of approximately **−5.17° CW / −7.11° CCW**, with no sampled mechanical/PWM yaw cap or endpoint hit. The extra inferred common reserve is approximately **0.406% / 0.770%**, respectively. These are separate from the larger pre-existing native roll/pitch collective reduction and are not physical thrust-loss measurements. Settled physical roll/pitch norms remain approximately 0.92–1.10° for the corrected controller in these full-collective probes.

Factory-stock yaw is already oscillating before the yaw step, so a quiet-start rise time is unavailable. Its full-collective settled body-error RMS is approximately **223.36/216.55°/s** CW/CCW, versus **0.330/0.169°/s** corrected. The corrected controller still has approximately **63–68%** factory-gain step overshoot; the allocator correction does not replace PID tuning.

These cells are N=1 finite three-second pulses. “Settled” excludes the first 0.5 s. High-collective settling requires a recorded 0.5 s quiet tail, unlike the shorter forward metric. Recovery releases yaw and returns throttle to hover simultaneously, so it does not isolate yaw release.

Artifact root: `/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/`.

- Before/after engineering check: `runs/20261109T054113-both-authority-fix-smoke-625197/`.
- Twenty-flight throttle envelope: `runs/20261109T060855-both-authority-fix-throttle-envelope-642473/`; see `independent-throttle-comparison.json/.md` and `independent-plots/`.
- Repeated 95% confirmation: `runs/20261109T060412-both-authority-fix-95-confirmation-637471/` and `runs/20261109T063249-both-authority-fix-95-qhd-confirmation-659013/`; the latter contains `independent-95-aggregate-n3.json/.md` and four QHD videos.
- Primary forward comparison: `runs/20261109T062031-both-authority-fix-forward-comparison-host-650381/`; independent tables/plots are in `authority-fix-review/forward650381-independent-audit.json/.md`, `forward650381-native-yaw.png` and `forward650381-control-subtracted-rp.png`.
- Explicit high-collective yaw commands: `runs/20261109T063452-both-authority-fix-high-yaw-probes-663307/`; see `independent-high-yaw-comparison.json/.md` and `independent-plots/`.
- Illustrative QHD forward CCW retakes: `runs/20261109T064040-both-authority-fix-forward-qhd-668043/`; these remain curved-flight demonstrations and are not added as repeated samples to the primary N=1 forward cohort.
- Original study identity, static trim and MCU build: `runs/20261109T052246-both-yaw-authority-study-3/`.

The [study index](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261109T052246-both-yaw-authority-study-3/study-index.json) and [validation manifest](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261109T052246-both-yaw-authority-study-3/final-validation.json) provide exact paths, hashes and validation boundaries.

Review plots:

- [Stock / archived allocator / corrected allocator, same hover tune](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261109T054113-both-authority-fix-smoke-625197/independent-plots/compare-hold_95_control-hover_tuned.png).
- [Full-collective hold, factory yaw gains](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261109T060855-both-authority-fix-throttle-envelope-642473/independent-plots/compare-hold_100_control-factory_yaw.png).
- [Full-collective CW yaw command, hover-tuned gains](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261109T063452-both-authority-fix-high-yaw-probes-663307/independent-plots/compare-hold_100_cw-hover_tuned.png).
- [Forward body-yaw responses](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/authority-fix-review/forward650381-native-yaw.png) and [control-subtracted physical roll/pitch, including recovery](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/authority-fix-review/forward650381-control-subtracted-rp.png).

[QHD videos and capture verification](TRICOPTER_VIDEO_CAPTURE.md#qhd-allocator-study-videos) include the new prop discs and exact live Nvidia renderer evidence. Capture defaults to 2560×1440 at 60 fps with NVENC. A rotating-geometry probe demonstrates 60 distinct scene states per second even while the owned Gazebo window is unfocused; resolution and encoder metadata alone are not used to claim real scene cadence.
All six delivered clips pass full-clip aircraft visibility (10,299 frames),
exact owned-window/camera and hardware checks. Six separate moving-ground
intervals contain 720/720 distinct frames at 60 Hz, with repeat controls rejected.
The linked quality report binds the actual videos and all review artifacts.

## Build and validation boundary

The SITL executable and the **MATEKF405TE** target build successfully. No physical FC was flashed and no powered hardware test was performed for this revision. Unit/SITL success is not physical-flight validation. The existing [bench procedure](TRICOPTER_FEEDBACK_MIXER.md#straightforward-bench-validation) applies; this change adds no calibration prerequisite.

The [MATEKF405TE HEX](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261109T052246-both-yaw-authority-study-3/mcu-build/bin/betaflight_2026.6.2_STM32F405_MATEKF405TE.hex) has SHA-256 `e8c2ce2c36db038a8135269d68fb3b7efdb3f772e61de81ec42db0bde484003a`. The generic firmware keeps trim zero until configured; the bundled simulation preset supplies the model-specific −6.1° trim.

Completed checks include 239 native tests in 17 suites, 42 additional strict/fast-math control-test executions, 205 simulator Python tests, three CTest targets and independent plant/trace audits. A final read-only native-code review found no blocking defect. The target image and exact hashes are retained with the study artifacts. Coverage collection emitted a stale-gcov checksum warning; no coverage claim is made.

The failed sandbox launch also exposed a runner defect: a ledger containing only failed execution attempts could still exit with status zero. After the completed measurement campaigns, the CLI wrapper was corrected to return status one when execution fails, with a focused regression check. That change does not alter measurement packets, control configuration or native firmware. Completed execution remains distinct from straight-flight qualification; both results stay explicit in each ledger.
