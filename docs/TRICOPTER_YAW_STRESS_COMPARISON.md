# High-collective yaw stress comparison

This study deliberately searches beyond the earlier 38–45% collective envelope.
It keeps the existing aircraft physics, as requested. It does not change flight
firmware, motor mapping, rotation conventions, servo behavior or PB0−PC0 feedback.
The only world changes extend the ground and add visual grid lines.

**Result:**95% collective reproduces factory-stock yaw oscillation in3/3
matched trials. Modified loses yaw authority in3/3 trials with either profile.
The requested paired steady70% forward condition was not achieved; its failures
and separate gyro-only diagnostics remain in the record. No firmware fix is
included in this study.

## Comparison contract

Both controllers use the same previously verified four-file SITL coordinate
adapter. “Stock” means the stock controller with that common adapter, not a
byte-pristine upstream executable. Source/build/adapter manifests and binaries
are retained in each formal campaign.

The two tested yaw profiles are:

| Profile | Yaw P/I/D/F | Yaw hold | Roll/pitch |
|---|---|---|---|
| Factory yaw | 45 / 80 / 0 / 120 | 15 | Aircraft configuration |
| Frozen hover tune | 31 / 20 / 10 / 0 | 0 | Same aircraft configuration |

Aircraft roll is 35/16/15/F47 and pitch 35/17/15/F49. Factory yaw does **not**
mean factory roll/pitch gains. The separately defined aircraft-original yaw
profile 50/10/20/F47 is not relabelled as a Betaflight factory profile.

Native CLI/MSP readbacks verify gains, ANGLE limit80° and earth reference100.
Fresh EEPROM/world per flight, common fixed RC programs, 50Hz receiver input and
100Hz plant traces are retained. Every attempt and failure stays in the ledger.
The setup pilot is overridden on all four primary RC channels during each test.

With min_check1050 and native integer normalization, throttle RC1715,1858,1905,
1953,2000 correspond to 70%,85%,90%,95%,100% native collective. Actual motor
duties are recorded separately. A completed program is not a stability claim.

## Completed punchout screen

Campaign:
`.local-artifacts/tricopter/gazebo-sitl/runs/20261109T034433-both-yaw-stress-punch-screen-525474`

All12 one-second70/85/100% punchouts completed, covering both controllers and
both yaw profiles. Every case returned to hover collective for four seconds.
Readback/model/input identity checks passed. An independent raw-data audit made
108 numeric comparisons, maximum discrepancy5.68e−14.

70/85% caused sharp transient yaw kicks in both controllers. They decayed after
release. At100%, stock develops rapid alternating yaw/servo reversals; modified
instead loses approximately339–355° of heading during the pulse, reaching about
386°/s. Both recover their yaw rate after collective is reduced. Recovery does
not undo the heading loss.

The modified yaw allocator reports saturation throughout the full-throttle
pulse. Its `acos(verticalEffort)` feasible-angle bound collapses toward zero at
full requested vertical effort; the simulated basket then stays near center.
The observed steady spin agrees with the model's unbalanced propeller reaction
torque/angular-drag balance. This is evidence of authority allocation at the
collective limit, not proof of unstable unsaturated PID poles. Rotor momentum
reaction is also consistent with the onset/release kicks; no counterfactual
identifies it as their exclusive cause.

Independent reports:
`.local-artifacts/tricopter/gazebo-sitl/stress-review/punch-independent-audit.md`
and `punch-independent-audit.json`. Original inflight metrics are preserved;
schema2/3 reanalyses are separate files in that directory.

## High-speed qualification and interpretation

Existing linear drag and static propeller coefficients predict roughly75–76°
pitch and126–131m/s for nearly level70% flight. These extreme speeds are a
synthetic stress condition, not a credible real-aircraft speed prediction.
No quadratic drag or inflow correction has been added.

The common initial forward program uses roll1440/pitch1986/throttle1715us,
a four-second ramp, then a fixed settling lead. ANGLE limit80° and an85°
attitude guard permit this trim. The common horizontal envelope is10km and
height envelope800m. Final two seconds before the yaw probe are assessed from
actual motion; a large stick input is never taken as proof of steady flight.

The first normal-fusion qualification failed:
`.local-artifacts/tricopter/gazebo-sitl/runs/20261109T035114-both-steady70-qualification-536654`.
Stock completed its program but was not steady; modified hit the attitude guard.
Native attitude diverged from ground truth. Local high-tilt gyro propagation
tests passed at±75°. At one acceleration event, inferred specific force was
0.993g, inside the native0.9–1.1g trust gate, but pointed along apparent pitch
36° while true pitch was78°. The source's Mahony correction predicts about
−8.9°/s of erroneous pitch correction there. This inference uses100Hz velocity
differences, not a recorded raw-accelerometer replay.

A separate common `imu_dcm_kp=0`, `imu_dcm_ki=0` diagnostic removes accelerometer
correction while retaining native gyro integration, ANGLE coordination and
PIDs. It is explicitly **not normal native sensor fusion or a real-flight
configuration recommendation**. Initial diagnostic data is retained at:
`.local-artifacts/tricopter/gazebo-sitl/runs/20261109T035551-both-steady70-gyro-only-qualification-542698`.
Stock has small yaw error; modified shows a slow yaw/servo oscillation. Neither
passed the original strict steady-entry gate in that first qualification.
These preliminary runs do not establish a paired steady-flight yaw comparison.

## Classification and video evidence

`stress_metrics.py` separates sustained alternating oscillation, decaying
ringing, growing excursions, unwanted spin and sampled actuator limits.
Sustained oscillation requires at least four complete above-band cycles over
one second, confirmed peak reversals on both sides, persistent tail activity
and a final/initial amplitude ratio≥0.8,
after a0.5s input-transition exclusion. Guard aborts and saturation alone are
not classified as unsaturated PID instability. Failed prefixes stop at the
first post-measurement disarm; post-disarm falls are excluded.

The original punch MP4s have intermittently stale Gazebo scene frames despite
updating telemetry overlays, so they are not accepted as continuous visual
flight evidence. An intermediate recorder used an isolated artifact-local Xvfb display,
with explicitly recorded software OpenGL rendering and **Nvidia h264_nvenc
hardware encoding**. A continuous-motion probe passed before longer-flight
recording. This changes the display/capture path, not simulator physics. No
system packages were installed; Xvfb was extracted into local artifacts.
That low-rate capture path has since been replaced by the physical-window,
Nvidia-rendered60fps path described in [the capture report](TRICOPTER_VIDEO_CAPTURE.md).

No controller gains or flight code are changed during a frozen campaign.

## Longer holds and independent repeats

Discovery campaign (12/12 completed):
`.local-artifacts/tricopter/gazebo-sitl/runs/20261109T040048-both-yaw-stress-long-hold-screen-547588`.
These are zero-yaw90/95/100% holds:0.5s lead plus3s pulse and4s recovery.

Confirmation campaign (8/8 completed, two additional repetitions at95%):
`.local-artifacts/tricopter/gazebo-sitl/runs/20261109T040936-both-yaw-stress-95-confirmation-559999`.
The confirmation runs have no GUI or encoder; their physical inputs and native
configuration match discovery. This excludes video load as a necessary cause.

Combining the preselected95% discovery and confirmation cohorts gives N=3 per
controller/profile:

| Yaw profile | Stock controller | Modified controller |
|---|---|---|
| Factory yaw | Sustained oscillation3/3 | Unwanted spin3/3 |
| Frozen hover tune | Sustained oscillation0/3; ringing2/3 | Unwanted spin/reversals3/3 |

Factory stock has approximately5Hz yaw/servo oscillation. Discovery body-yaw
RMS is243°/s, with227°/s in its final0.5s of high throttle. Factory modified
has approximately384°/s one-direction spin and yaw-allocation saturation in
every sampled high-throttle MSP reply. The hover-tuned profile has different
transient behavior; it must not be presented as the same persistent stock
heading oscillation. Its discovery run includes substantial roll/pitch
deviation. All these high-throttle failures coincide with actuator/authority
limits; they do not identify unstable unsaturated PID poles.

The varied hover-stock transients include one actual roll/pitch norm83.3° and
native/ground-truth error RMS49.5°. Its quiet yaw tail does not mean acceptable
overall flight. The complete per-replicate table is in
`stress-review/95-confirmation-independent-audit.md`.

Independent raw-data audit:
`.local-artifacts/tricopter/gazebo-sitl/stress-review/long-hold-independent-audit.md`.
Schema4 confirmed-extremum reanalysis is retained separately as
`stress-review/confirmed-cycles-metrics-v4.json`; original inflight records are
unchanged. It corrects the older hover-stock95 classification to decaying
ringing. Factory-stock95 remains sustained.

Additional camera-regression flights are retained separately at
`.local-artifacts/tricopter/gazebo-sitl/runs/20261109T041308-both-yaw-stress-95-video-validation-564364`.
They are not silently added to the predefined N=3 cohort. The fixed recorder
verifies camera mode, targets, offset and actual camera position before flight.
Videos encode at30fps; actual software-rendered scene updates are approximately
9Hz. Use the100Hz CSV/plots for quantitative fast-oscillation analysis.

## Final forward diagnostic

Campaign:
`.local-artifacts/tricopter/gazebo-sitl/runs/20261109T041506-both-yaw-stress-forward70-isolated-567907`.
The common gyro-only program extends the fixed settling lead to40s; physics
remains unchanged. All four attempted flights and all powered prefixes are
retained. Both factory-yaw flights and modified hover-tuned flight reached the
ground-height guard before the scheduled probe. The completed hover-stock run
reached131.3m/s horizontal speed but descended approximately2.28m/s and failed
the declared acceleration/angular-rate steadiness limits. **No pair qualified**
for the requested steady70% yaw-response comparison, and no yaw-response probe
is claimed for those aborted setup prefixes.

Modified hover shows an approximately0.186Hz oscillatory pattern without sampled
actuator endpoints or allocator saturation, but only three confirmed cycles
before abort; it does not meet the four-cycle sustained-pattern criterion.
This diagnostic cannot establish normal-fusion behavior or real gyro drift.

The independent report is `stress-review/forward70-independent-audit.md`.
`stress-review/forward70-isolated-metrics-v5.json` corrects unavailable entry
statistics to null when the planned probe was never reached; original inflight
records and source snapshots remain untouched. A missing entry is not replaced
with an earlier ramp window.

All four final videos passed camera readback, actual aircraft-visibility,
Nvidia encoder and owned-process cleanup checks. One follows the aircraft over
6.06km. Capture success is separate from the flight qualification failures.

## Selected plots and videos

- [Replacement60fps stock95% video](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261109T044105-both-yaw-stress-95-nvidia60-587788/01-factory_yaw-hold_95_control-stock/flight.mp4)
- [Replacement60fps modified95% video](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261109T044105-both-yaw-stress-95-nvidia60-587788/01-factory_yaw-hold_95_control-modified/flight.mp4)
- [Replacement60fps stock forward diagnostic](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261109T044324-both-forward70-nvidia60-video-590929/01-hover_tuned-forward_70_control-stock/flight.mp4)
- [Replacement60fps modified forward diagnostic](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261109T044324-both-forward70-nvidia60-video-590929/01-hover_tuned-forward_70_control-modified/flight.mp4)

Earlier clips below retain their original quality limitations and are archived
for provenance. Use the replacements above for review.

- [Factory95% plot](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261109T041308-both-yaw-stress-95-video-validation-564364/plots/compare-hold_95_control-factory_yaw.png)
- [Stock95% video](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261109T041308-both-yaw-stress-95-video-validation-564364/01-factory_yaw-hold_95_control-stock/flight.mp4)
- [Modified95% video](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261109T041308-both-yaw-stress-95-video-validation-564364/01-factory_yaw-hold_95_control-modified/flight.mp4)
- [Final forward diagnostic plot](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261109T041506-both-yaw-stress-forward70-isolated-567907/plots/compare-forward_70_control.png)
- [Completed stock forward diagnostic video](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261109T041506-both-yaw-stress-forward70-isolated-567907/01-hover_tuned-forward_70_control-stock/flight.mp4)

## Reproduction

Run from the repository root:

```sh
python3 -m unittest discover -s tools/tricopter/sitl -p 'test_*.py'
python3 tools/tricopter/sitl/check_yaw_stress.py --cases punch_70,punch_85,punch_100 --video
python3 tools/tricopter/sitl/check_yaw_stress.py --cases hold_90_control,hold_95_control,hold_100_control --video
python3 tools/tricopter/sitl/check_yaw_stress.py --cases forward_70_control --forward-lead 40 --gyro-only --video
python3 tools/tricopter/sitl/plot_yaw_stress.py /absolute/path/to/campaign
```

The recorder finalizes the MP4 before stopping its owned GUI/display processes.
`video-record.json` retains GPU identity, exact encoder command, stream encoder
tag, dimensions/frame count, hashes, renderer and process cleanup evidence.
Recording now requires the physical display and Nvidia hardware OpenGL/NVENC;
Xvfb is no longer used. The complete focused Python suite passes190 tests. All owned simulator,
encoder, GUI and isolated-display processes are stopped.
