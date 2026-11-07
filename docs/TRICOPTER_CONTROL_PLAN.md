# Tricopter Model-Based Control — Development Plan

**Project:** Betaflight tricopter-feedback

**Working branch:** `feature/tricopter-feedback`

**Firmware baseline:** Betaflight `2026.6.2`, target `MATEKF405TE`

**Document version:** 0.11 — 2026-11-07

**Status:** M0 complete; M1 flashed and bench-tested; servo prediction, RPM observation, placeholder-coefficient yaw reaction and configurable allocation connected in an offline synthetic actuator/response simulation; airframe characterization, physical calibration and loaded identification remain open; **no new flight-control law validated**

> **Purpose**  
> Build an actuator-aware, physically informed controller for a thrust-vectoring tricopter. Retain Betaflight's proven gyro, receiver, PID/rate-loop, failsafe, DShot, and Blackbox infrastructure. Replace the simplistic yaw-PID-to-servo mapping with a coordinated **three-motor + tail-servo control allocator**, informed by measured tail angle, rotor speed, reaction torque, and actuator dynamics. Develop in testable stages; do not enable unvalidated corrections in flight.

## Quick-start: where we are and what to do next

**Confirmed:** Hardware and motor directions are known; six historical Blackbox logs have been analyzed; Bluejay ESCs have been updated to **0.21**; the user's Betaflight `2026.6.2` checkout is on branch `feature/tricopter-feedback`. The user confirmed a working cloud build at `e0b7bb01b`, board config `3a406c5`, and correct physical motor/servo mapping. Two clean unchanged local builds produced identical HEX/BIN/ELF files. The servo potentiometer wiper has been measured at **1.561 V (1100 µs), 1.281 V (1420 µs), and 1.031 V (1700 µs)**. Its measured voltage is already within the FC's 0–3.3 V ADC range. **Current wiring:** the wiper is directly connected to **PB0** (`ADC_EXT 1 B00`); a separate servo PCB ground-sense wire is connected to **PC0** (`ADC_RSSI 1 C00`). `TRI_GROUND` logs both raw channels and signed PB0−PC0, with `RSSI_ADC` disabled. See [dual-ADC setup](tricopter_dual_adc_logging.md). The **TLV9001 unity-gain buffer remains optional later**. A preliminary standalone C torque-model/test harness exists but is **not calibrated or flight-ready**.

**Immediate next steps (direct ADC prototype; amplifier parts not required):**

- [x] Build and record the output/configuration of **unmodified** Betaflight `2026.6.2` for `MATEKF405TE`; confirm TRI/servos, PC0 ADC, RPM telemetry, Blackbox and reproducibility. See [M0 evidence](tricopter_m0_baseline.md).
- [x] Audit the current target's pin allocation and establish the aircraft's motor/servo resource map; compare the supplied current runtime export and user-confirmed physical mapping. See [hardware record](tricopter_hardware.md). **Do not assume stock target motor order matches the aircraft.**
- [x] Implement logging-only `TRI_SERVO` diagnostics for configurable `ADC_EXTERNAL1` counts, final servo write request, ADC scan progress, logical RPM reception ages and packet-quality indication; preserve legacy outputs. Target build and 44 host tests pass. The existing binary supports the current PB0 input through configuration; no rebuild is needed for this pin change. See [M1 schema and setup](tricopter_m1_logging.md).
- [x] Flash/read back M1 `bd6a5d727`, restore and verify the current 2026.6.2 aircraft configuration, and remap `ADC_EXT` to PB0. Record servo feedback and one-motor-at-a-time 5–10% command tests with props removed. See [the bench record](tricopter_m1_bench_2026-11-06.md).
- [x] Verify EDT and all three indexed RPM channels in low-power bench recordings. New logs expose ~121 ms startup reception gaps and a 1.336 s stale interval before the required EDT handshake; a zero invalid-packet statistic is not proof of freshness. No continuous in-flight reception claim is made.
- [x] Flash/read back logging-only `TRI_GROUND` at `a15ed3065`, preserve configuration, and record three clamped unloaded servo cycles with raw PB0, raw PC0, and signed PB0−PC0. See [dual-ADC evidence](validation/tricopter-ground-2026-11-06/README.md).
- [x] Implement and validate offline corrected-count servo prediction/replay, followed by a causal per-motor RPM observer. Keep angle calibration unavailable and leave the active controller unchanged. See [offline models and results](tricopter_offline_models.md).
- [x] Implement offline aerodynamic/inertial yaw reaction equations with explicit synthetic coefficients, separate terms, validity gates and analytic tests. No active control integration. See [reaction-torque model and evidence](tricopter_reaction_torque.md).
- [x] Implement offline bounded allocation using a configurable virtual-tail effectiveness map, plus an optional SI geometry adapter. Synthetic maps/geometry/coefficients exercise reconstruction, saturation, units and held-angle behavior; detailed airframe dimensions are not required by the shared solver. See [allocator configuration and evidence](tricopter_allocator.md).
- [x] Connect all four portable modules in a causal offline actuator/response simulation with separate plant/predictor states, synthetic airframes, delayed/faulted RPM observations and explicit reaction quality. Preserve the generic map interface and active-output boundary. See [simulation architecture and evidence](tricopter_simulation.md).
- [ ] Complete electrical loading/power-sequencing/fault checks and independent PWM pin-edge timing. ADC scan progress and non-rail counts do not guarantee wiper connection.
- [ ] Port the standalone torque equations into a pure-C module with host unit tests; separately build a motor-speed/acceleration observer driven by command **and** RPM telemetry.
- [ ] Make offline replay plots/tests for Blackbox logs 001, 004, and 005, keeping original logs immutable.

**First major gate:** an instrumented Betaflight build that retains the proven legacy mixer and logs accurate per-motor RPM, servo command, signed PB0−PC0 position measurements and raw ADC diagnostics without changing aircraft behaviour. The direct-input signal must first be checked for ADC loading/noise and stable operation of the servo's original feedback loop.

---

### Revision notes — v0.11 (2026-11-07)

- Added an offline coordinator composing the servo predictor, three RPM observers, scalar reaction model and virtual-tail allocator. Only initial corrected position and causal RPM observations enter the coordinator; plant truth remains separate.
- Added synthetic motor/servo plants, telemetry delivery/fault scheduling, map/actuator mismatch and bounded experimental reaction/virtual-tail-vertical coordination modes. Targets, current predictions, achieved surrogate response and their validity remain distinct.
- The synthetic additive yaw reaction is counted once and deliberately independent of virtual direction. Effectiveness maps exclude that separately modeled contribution; no 3D/gyroscopic or body-flight-dynamics claim is introduced.
- MATLAB executes the same C simulation through a source-pinned MEX adapter; the optional Simulink model is explicitly discrete signal playback/analysis. No active-flight source list, actuator/configuration path, PB0−PC0 source or real angle calibration changes.

### Revision notes — v0.10 (2026-11-07)

- Added a pure C bounded allocator with a direct 4×4 virtual-tail control-effectiveness interface. Basic configuration and characterization can supply the map without CAD dimensions, CG, mass, inertia or absolute thrust coefficients; all current coefficients remain synthetic placeholders.
- Kept geometric SI modeling as a separate adapter: CG-relative cross products, tail `atan2`/`hypot` decomposition, full force reporting and explicit thrust-to-speed conversion. Proxy response units and mathematical direction remain distinct from calibrated N/N·m/RPM and mechanical angle.
- Saturation minimizes an explicit weighted response error and reports achievable response, residual, active constraints and convergence. Independent synthetic projection/forward/convex-gap checks cover bounds, infeasible requests and unit changes. A fixed-angle forward example exposes the gap between a requested and an actual vector.
- No active-flight source-list/caller, measurement-source, configuration or hardware change is part of this stage. PB0−PC0 remains the sole servo position source; actual angle/effort mapping, identified effectiveness, actuator timing and M5 integration remain open.

### Revision notes — v0.9 (2026-11-07)

- Added stateless near-vertical yaw reaction algebra with per-motor `kQ`, `J` and explicit signs. Mechanical RPM/RPM/s are converted to rad/s/rad/s² inside the core; aerodynamic, inertial and total components are reported separately.
- Coefficients are mandatory synthetic placeholders, with calibration/flight-use flags always false. Every motor needs fresh valid speed and independently valid acceleration; missing inputs invalidate the complete result without a partial sum.
- Synthetic steady/ramp/braking/cancellation/coast and angular-impulse tests verify signs, units and bookkeeping. The model approximates fixed-body mount reaction, or the yaw RHS with locked-rotor inertia included; it is not total external aircraft-plus-rotor torque.
- The focused suite passes 129 host/regression and 28 Python/native-C harness tests; strict/fast-math and Cortex-M4F compile checks pass. No flight-control source-list/caller change or hardware access occurred.

### Revision notes — v0.8 (2026-11-07)

- Added an allocation-free portable C servo predictor: monotonic command/count map, continuous play, command delay and bounded directional lag/slew. Fit only paired PB0−PC0 datasets, with complete units frozen for validation and one measured initialization per unit.
- Held-out equal-unit RMS error is 4.765 corrected counts, versus 14.156 for a lag-only baseline and 52.531 for an instantaneous map. These are previously explored unloaded electrical recordings; joint fit parameters are not independently identified mechanical constants or angle calibration.
- Added an offline causal RPM/acceleration observer with identity/time handling, bounded prediction, stale decay and reacquisition. Existing per-motor raw recordings are archived. Quantized-age replay is a compatibility test with explicit phase uncertainty; synthetic cases establish numerical correctness separately.
- The focused suite passes 111 host tests and 19 Python/native-C harness tests. Strict/fast-math checks and Cortex-M4F compile checks pass. No active-flight source-list or actuator/scheduler/telemetry-path change, hardware access or flash operation is part of this stage.

### Revision notes — v0.7 (2026-11-06)

- Selected signed **PB0−PC0 as the sole servo position measurement**. Raw PB0/PC0 remain diagnostic; no single-channel fallback or PWM-as-position substitution is allowed. The canonical position API exposes coherent-pair freshness and age without changing the legacy actuator path.
- Clarified the ground-sense solder point: servo PCB **power GND input**, not the potentiometer low terminal. Internal excitation/electronic behavior can remain in the corrected measurement.
- Calibration, hysteresis, velocity and future position observers must use corrected counts. Actual angle calibration and loaded response remain separate gates.
- A corrected-only clamped test records three repeats of four center approaches and two fine staircases. It finds about 10–11 µs input-equivalent directional hysteresis and a repeatable center offset by approach. See [the hysteresis/approach results](validation/tricopter-hysteresis-2026-11-06/response/hysteresis-report.md).

### Revision notes — v0.6 (2026-11-06)

- Added logging-only `TRI_GROUND`: PB0 remains the wiper and PC0 senses servo PCB ground. Record both raw channels and signed PB0−PC0 using a coherent completed-scan pair. The old `TRI_SERVO` schema is unchanged.
- Reuse the existing `ADC_RSSI` acquisition slot for PC0 only in this mode, with the RSSI feature off; no parameter-group layout or version change. See [dual-ADC wiring, schema and acquisition limits](tricopter_dual_adc_logging.md).
- Flashed/read back `a15ed3065`; all 1471 saved configuration commands matched except the intended ADC_RSSI resource and debug-mode changes. Recorded three clamped unloaded cycles with all motors stopped; the original servo configuration was restored and verified after reboot.
- The paired recording strongly supports ground rise as a major component of the early kick. On 1420→1700, the median positive peak falls from 48 raw PB0 counts to 11 corrected counts (75.6% reduction across three trials). Residual transients, potentiometer excitation effects and independent mechanical position remain unresolved.
- Ground subtraction is an experiment, not an assumption of calibrated position or cancellation of potentiometer excitation changes. No measured-angle control correction is enabled.

### Revision notes — v0.5 (2026-11-06)

- Flashed/read back the M1 binary and preserved a complete current cloud-firmware/configuration backup. The larger local feature set triggered Betaflight's whole-config reset policy; the fresh 2026.6.2 dump was restored, and all 1210 shared settings matched after reboot before intentional sensor/logging changes.
- Bench-verified PB0 readings, final servo command logging, low-power per-motor RPM, native Blackbox schema/readback and short idle CPU comparisons. Persisted `dshot_edt = ON` for Bluejay's normal arm-time handshake; disarmed motor tests explicitly sent the same handshake.
- Recorded 72 large-step trials and three small-step cycles in each of free-case and clamped-case fixtures, with linkage disconnected. See [servo characterization](tricopter_servo_characterization_2026-11-06.md). All clamped large steps initially show the same positive wiper-voltage kick, approximately 36–40 mV, even for opposite requested directions. Power/ground/reference disturbance is plausible but not proved; the early signal must not be treated as calibrated physical motion or actuator deadtime.

### Revision notes — v0.4 (2026-11-06)

- Recorded the user's actual wiper wiring to **PB0 / ADC1_IN8**, chosen for proximity to the servo. The earlier **PC0/AirS** proposal and baseline cloud captures remain historical evidence.
- Keep the existing M1 binary: its `ADC_EXTERNAL1` input follows the configured resource. Set `ADC_EXT 1 B00`, clear `ADC_RSSI 1` to `NONE`, and disable `RSSI_ADC` before save/reboot; do not change motor or servo resources.
- The existing wiper voltages and nominal count estimates remain reference predictions. No PB0 voltage, loading, noise, or acquisition result is claimed by this wiring revision.

### Revision notes — v0.3 (2026-11-06)

- Completed M0 local reproducibility and evidence preservation; recorded the user's current cloud build and physical mapping confirmation.
- Implemented M1 `TRI_SERVO` LOG_ONLY diagnostics and exposed the existing ADC external-channel enable boolean; no CONTROL mode or physical compensation was added.
- Added transport-level RPM observation tracking so cached RPM, missing packets, EDT handshakes and non-RPM EDT frames cannot fabricate fresh RPM samples.
- Added 16 focused host tests alongside 28 existing regressions. Preserved baseline and instrumented images, toolchain/config provenance and build logs locally.
- Kept M1 hardware acceptance open: analog loading/noise, open-wire limitations, pin timing, continuous telemetry, recording throughput and loop cost require bench evidence. Later phases remain unstarted.

### Revision notes — v0.2 (2026-11-06)

- **Measured** the internal servo potentiometer wiper at **1.561 V (1100 µs), 1.281 V (1420 µs), and 1.031 V (1700 µs)**.
- Changed M1/M2 prototype from a required inverting TLV9001 level shifter to a **direct wiper→PC0/AirS input**; initial instrumentation need not wait for analog parts.
- Reclassified the TLV9001 as an **optional 3.3 V unity-gain buffer**, pending evidence of ADC loading, noise or drive limitations.
- Added raw ADC-count estimates, signal-polarity notes, loading checks and the requirement to calibrate **physical rotor angle separately from servo PWM**.

---

## 1. Hardware baseline and non-negotiable mappings

| Item | Known configuration / assumption |
|---|---|
| Flight controller | **Matek F405-miniTE**; fitted MCU **STM32F415RGT6** (Cortex-M4F, 168 MHz); use Betaflight `MATEKF405TE` target unless build verification reveals a specific conflict. |
| Existing firmware reference | Betaflight **4.3.1**; preserved CLI dump `tricopter.txt` and six logs `btfl_001.bbl` … `btfl_006.bbl`. |
| Firmware development | Betaflight **2026.6.2**, working branch **`feature/tricopter-feedback`**. |
| ESC firmware | **Bluejay 0.21**, EDT arming handshake requirement enabled. Verify EDT with compatible Betaflight firmware. |
| ESC interface | Bidirectional **DSHOT300** initially; **14 magnetic poles** (7 pole pairs); ESCs are on a rotated 4-in-1 board. |
| Motors and props | Three otherwise identical **rewound 2207** motors, identical **Gemfan 5136 three-blade** propellers. Do **not** assume the original nominal KV is still correct. |
| Battery | 6S, approximately 1400 mAh. |
| Tail servo | **EMAX ES08MD II**, 5 V; existing rate approximately **498 Hz**. |
| Servo PWM output | **PB1**, previously repurposed LED pad; original `resource SERVO 1 B01`. |
| Servo configuration reference | Original `servo 5 1100 1700 1420 100 -1`: 1100–1700 µs, configured midpoint 1420 µs. Mechanical straight-up angle **believed**, not measured, at 1420 µs. |
| Servo geometry | Tail motor rides on a bearing-mounted tilt basket. Rough gearing: **1° servo shaft rotation ≈ 1.5° rotor tilt**. Mechanical stops, sign, and curve require measurement. |
| Position feedback — current prototype | Fourth wire from **internal servo potentiometer wiper directly to PB0 / ADC1_IN8**, configured as `ADC_EXT 1 B00`; `ADC_RSSI 1 NONE` and `feature -RSSI_ADC`. This replaces the earlier PC0/AirS proposal. Wiper voltage was measured inside 0–3.3 V; PB0 loading/noise still requires bench measurement. |
| Position feedback — optional revision | **TLV9001 as unity-gain, non-inverting buffer**, supplied at 3.3 V, if loading, sampling noise, wiring, or servo interference justifies it. The previously proposed inverting 5 V→3.3 V level-shift stage is **not required** by measured voltages. |
| Measured servo wiper voltage | **1100 µs → 1.561 V; 1420 µs → 1.281 V; 1700 µs → 1.031 V.** Measurements are at the potentiometer wiper; supply conditions and physical rotor angle have not yet been calibrated. |
| Original motor mapping | **Logical 0 = rear CCW, 1 = front-right CW, 2 = front-left CCW.** This was a flown, working mapping. |
| Physical ESC mapping | Rear motor physically on **ESC output 4**; output 3 unused. The original firmware used `motor_output_reordering = 2,0,1,3,4,5,6,7` and customized resources. **Revalidate complete mapping on the modern build; preserve the aircraft's behaviour.** |
| Original control/logging rates | Gyro 8 kHz, PID 4 kHz, Blackbox about 1 kHz (`1/4`). New implementation rates are proposals pending profiling. |
| Flight controller logging | SPI Blackbox flash; verify available space, recording fields, and schema on the new build. |

### Geometry and sign convention

Use a **right-handed body frame**: `x` forward (nose), `y` left, `z` up. Positive yaw is **CCW / nose-left**, as in the earlier Blackbox analysis. Positive tail angle `θ` means the rear motor thrust has a **rightward (`−y`)** component and produces **positive (CCW) yaw moment**. Define all model inputs in mechanical units (N, N·m, rad, rad/s), and convert to/from Betaflight's implementation conventions explicitly at the integration boundary.

| Motor | Logical index | Rotor spins, viewed from above | Reaction on frame while accelerating |
|---|---:|---|---|
| Rear | 0 | CCW | Clockwise / negative yaw |
| Front right | 1 | CW | Counterclockwise / positive yaw |
| Front left | 2 | CCW | Clockwise / negative yaw |

**Rules:** Never infer physical motor order from array indices after motor-output reordering. Never flip a motor or servo sign merely because a newly built target has different defaults. Verify by tracing the full software-output-to-ESC-to-physical-motor path with props removed.

## 2. Evidence, assumptions, and open measurements

### Established by existing recordings and code inspection

- **Log 005 around 33.7 s:** A commanded yaw step exhibits a strong relation between servo PWM and yaw *acceleration*. After signal alignment, a **~31–37 ms** command-to-acceleration lag was seen over the examined windows; ~33 ms was typical. This is a **closed-loop observation**, *not* a calibrated servo transport delay.
- **Logs 001 and 005:** Some throttle increases with near-zero yaw setpoint are accompanied by **negative yaw acceleration**, followed by servo correction. **Log 004:** full-throttle disturbance and runaway-takeoff disarm. These are consistent with negative reaction-torque transients, but do **not** independently prove that rotor inertial torque dominates aerodynamic torque.
- Historical `servo[5]` Blackbox data represents **commanded PWM**, not actual tail position. Historical logs do not provide the independent rotor-speed and tail-angle measurements needed for unique parameter identification.
- Betaflight `2026.6.2` retains an ordinary tricopter servo mixer driven by yaw PID output and a tricopter motor-correction stub returning zero. Useful insertion points exist, but source-level integration needs build/testing.

### Not yet measured / must not become magic constants

- Aircraft mass and **full inertia tensor** from CAD in the chosen coordinate frame, especially `Izz`, `Ixx`, `Iyy`, and cross terms.
- Accurate motor/rotor positions relative to the **actual CG**; tail lever arm; tilt-axis location and orientation; servo mechanical limits.
- Each motor's **RPM versus command, voltage, and airflow**, and acceleration/deceleration dynamics; common rotor axial inertia `J` (identical rotating assemblies, but not necessarily identical transient response).
- Rotor **thrust coefficient** `kT`, aerodynamic torque coefficient `kQ`, and their dependence on inflow, airspeed, and throttle/RPM.
- **Measured** servo wiper voltage decreases as commanded PWM increases (1100 µs: 1.561 V; 1420 µs: 1.281 V; 1700 µs: 1.031 V). **Still unknown:** potentiometer-voltage-to-actual-rotor-angle curve, slew limits, load sensitivity, deadband, latency, and changes with servo supply voltage. Do not confuse PWM-command calibration points with a measured rotor-angle calibration.
- Filtered RPM/ADC update timing and end-to-end actuation latency.

Treat these as calibration parameters or hypotheses, with confidence/validity flags. Keep parameter provenance and measurement conditions with each fitted value.

Detailed geometry, absolute thrust and full-body inertia are optional for the direct control-effectiveness route. Basic output/servo configuration and characterized effort-to-response gains can absorb these effects in declared proxy units. Use CAD/SI parameters when useful for physical interpretation; do not make them a prerequisite for every tricopter. Characterizing the actuator/vector coordinates and their operating region remains necessary.

---

## 3. Target architecture

```text
Receiver / pilot rate commands
              │
              ▼
   Existing Betaflight rate PID  ◄──── gyro / attitude processing
              │                         ▲
              │ desired normalized     │
              │ moments / demands      │
              ▼                         │
   PID demand → physical calibration   │
              │                         │
              ▼                         │
   ┌────────────────────────────────────────────────┐
   │        Coordinated tricopter allocator         │
   │  • desired Fz, roll, pitch, yaw moments         │
   │  • aerodynamic + inertial rotor reaction model  │
   │  • rear vector angle / thrust magnitude         │
   │  • motor/servo limits and time-to-actuate        │
   │  • priority, residual torque, anti-windup info   │
   └─────────────┬─────────────────┬────────────────┘
                 │                 │
             3 motors          tail servo
                 │                 │
            DShot + RPM      PWM + ADC angle
                 │                 │
                 └────────┬────────┘
                          ▼
                 Aircraft dynamics ────────────────► gyro

   Side-channel estimates: motor state observer, servo position/
   velocity/prediction, battery voltage, telemetry freshness,
   model residuals and Blackbox diagnostics.
```

**Design principle:** Separate (1) requested aircraft response, (2) feasible physical actuator allocation, and (3) actual/predicted actuator state. The aircraft needs force/moment commands, not an arbitrary servo pulse derived only from yaw PID. Keep established Betaflight rate control initially; do **not** begin by replacing the PID controller.

**Portability:** Put pure equations, estimators, and allocation logic in bounded, allocation-free **portable C** with host-side tests. Keep Betaflight-specific I/O, scheduling, configuration, and Blackbox integration behind thin adapters. MATLAB/Simulink can be the higher-fidelity 6DOF validation environment.

## 4. Physical model and allocation strategy

### 4.1 Rotor reaction torque

With identical rotor axial inertia `J`, near-vertical rotor axes, and `ω_i` as positive mechanical rotation **magnitudes**:

```text
S_Q = ω₁² − ω₀² − ω₂²
S_J = dω₁/dt − dω₀/dt − dω₂/dt
τ_reaction,z ≈ kQ·S_Q + J·S_J
```

The [offline scalar implementation](tricopter_reaction_torque.md) now tests this equation with explicit placeholder coefficients and separate per-motor terms. It approximates fixed/negligible-body-motion shaft reaction; for a moving-body yaw equation, use inertia with the rotors locked into the base `Izz`, so the relative rotor-spin inertia term is not counted twice. It is not a measurement of total external aircraft-plus-rotor torque.

For general attitude/motion/tilt, model each rotor's **signed angular momentum vector** and its derivative in the body frame; combine with aerodynamic drag torques and the full rigid-body rotational equations. Include rear rotor-axis projection (`cos θ`), the `ω₀ sin θ · dθ/dt` contribution, and gyroscopic cross terms as later refinements—not as unverified assumptions from the historical pitch traces.

```text
I·dΩ/dt + Ω × (I·Ω) = τ_actuators + τ_external
```

**Important:** The full model should treat rotor gyroscopic angular momentum consistently so signs and terms are not double-counted. The simplified yaw equation above is the first implementation, not the final general model.

### 4.2 Rear thrust vector

Use `θ > 0` for tail thrust toward aircraft right and positive yaw moment. For a rear moment arm `L > 0`, ignoring offsets beyond the primary rear arm:

```text
Fz,rear      = T_rear · cos(θ)
Fy,rear      = −T_rear · sin(θ)
τz,vector    ≈ L · T_rear · sin(θ)

required_lateral_thrust = (τz,requested − τz,reaction − τz,other) / L
θ_requested             = atan2(required_lateral_thrust, Fz,rear_requested)
T_rear_requested        = hypot(Fz,rear_requested, required_lateral_thrust)
```

The *physical actuator* must maintain the correct vertical rear thrust, using measured/predicted **actual** tail angle and motor response. Commanded angle is not actual angle; avoid compensating so late that the motor introduces a pitch disturbance, or so early that it overcompensates. The allocator must account for pitch/roll effects as well as yaw, using characterized effectiveness or explicit CG geometry.

### 4.3 Control authority / saturation

At steady state, three thrusts plus tail angle can provide four controlled quantities (collective, roll, pitch, yaw) in an appropriate operating region. **During fast transients the servo is slow**, and at near-fixed angle only three fast motor inputs remain. Full independent control is not always possible.

- Compute feasible actuator outputs and **residual unmet moment demand**.
- Respect servo angle/rate, motor min/max output, motor response rate, battery voltage and tail linkage limits.
- Preserve stable roll/pitch/collective support; yield or soften yaw demand when necessary. Define priority weights and document tradeoffs.
- Feed yaw saturation and achieved-versus-requested allocation back into the existing PID **anti-windup** path. Do not merely clip final PWM while the integrator continues winding up.
- Optional future strategy: predictively **slew-limit collective thrust** when an abrupt command would create yaw torque the servo cannot oppose in time. Make this opt-in and measurable.

### 4.4 Observer design

**Motor observer inputs:** motor output demand, battery voltage, mechanical RPM, and valid/fresh telemetry metadata. Estimate `ω_i`, `dω_i/dt`, uncertainty, and short-horizon prediction. Start with a causal first-order motor response; allow acceleration vs deceleration differences and independent per-motor response parameters despite equal rotor inertia. No direct raw-RPM finite differentiation in the active controller.

**Servo observer inputs:** actual ADC potentiometer reading, commanded PWM, calibration curve, fault limits. Estimate `θ`, `dθ/dt`, and short-horizon `θ_pred`. Calibrate nonlinear travel, sign, slew and lag. Do not treat the observed 33 ms closed-loop delay as an actuator constant.

**Model residuals:** compare expected and measured angular response through an appropriately filtered dynamics residual; use initially for **logging and slow model-bias estimation**, not an aggressive acceleration PID. Account for noise, unmodeled airflow and cross-axis coupling.

---

## 5. Hardware interfaces and timing

### 5.1 Servo feedback: measured low-voltage wiper, direct ADC prototype

**New measurement (2026-11-06):** The internal servo potentiometer wiper runs at a much lower voltage than the previously assumed 0–5 V range. Measured wiper voltages at three commanded servo PWM values:

| Servo command | Measured wiper voltage | Approx. 12-bit ADC count* |
|---|---:|---:|
| 1100 µs (configured low) | **1.561 V** | **1937** |
| 1420 µs (configured middle) | **1.281 V** | **1590** |
| 1700 µs (configured high) | **1.031 V** | **1279** |

*Counts assume a 3.300 V ADC reference, a linear 12-bit conversion and no material loading of the wiper; actual counts must be measured. The nominal span is ~530 mV, or ~658 counts. The measured voltage decreases as PWM increases. The two intervals suggest roughly −0.88 mV/µs, but do **not** establish a linear mechanical angle relationship.*

**Current prototype signal path (ground-referenced wiring, 2026-11-06):**

```text
EMAX ES08MD II potentiometer wiper (fourth wire)
    └── direct signal wire ──► Matek F405-miniTE PB0 / ADC1_IN8 / ADC_EXTERNAL1
Servo PCB power GND input (separate sense wire, not potentiometer low terminal)
    └── ground sense wire ──► Matek F405-miniTE PC0 / ADC1_IN10 / ADC_RSSI

Sole position measurement = signed PB0 counts - PC0 counts, from one scan pair.
Raw PB0 and PC0 remain diagnostic channels only.

Servo ground and FC ADC ground must share a reference.
The original three servo wires and PWM control path remain unchanged.
```

- **Resource configuration:** set `resource ADC_EXT 1 B00`, `resource ADC_RSSI 1 C00`, `adc_external1_enabled = ON` and `debug_mode = TRI_GROUND`, with `feature -RSSI_ADC`, then save/reboot. Verify active ownership and unchanged motor/servo/DMA assignments. PC0 is the servo's power GND input, not its potentiometer low terminal.
- **Position source:** use signed **PB0−PC0 only**, via the canonical position API and its freshness metadata. Raw readings are diagnostic, with no raw-PB0 fallback. Internal excitation/electronic behavior may remain; the selected source is still an electrical position proxy pending angle calibration.
- **No 5 V→3.3 V attenuation is required for the measured normal operating range.** The 1100–1700 µs samples are ~1.03–1.56 V, below the FC input limit. Confirm voltage at PB0 under actual supply conditions before enabling control; the three wiper samples do not establish PB0 loading or every power-up/fault voltage.
- The direct connection is for **early logging and position characterization**, initially with no change to motor or servo control. Check whether attaching the ADC alters the wiper voltage or servo behaviour, and inspect noise/ripple with motors disabled.
- A **100 kΩ pull-down directly on the raw pot line is not part of this prototype**: it could load the servo's internal feedback node. Optional small series resistance/ADC-side capacitor can be tested if ADC sampling noise is evident, but do not assume a large input capacitor is harmless without checking servo stability.
- **Future optional buffer:** use the ordered **TLV9001 as a non-inverting unity-gain voltage follower** powered from the FC 3.3 V supply. It presents a high input impedance to the potentiometer and provides a low-output-impedance ADC drive. The ordered resistors for the earlier inverting/offset level-shift design are **not needed** for this buffer configuration. Add local supply decoupling and validate the IC's input/supply sequencing and output loading when assembled.
- **Calibration:** verify each raw ADC-to-voltage relationship diagnostically, then independently measure **actual rotor tilt angle** at multiple positions and fit corrected **PB0−PC0 counts→angle** (accounting for linkage and mechanical limits). `1420 µs` is an assumed vertical rotor reference, **not yet confirmed**. Servo PWM, corrected wiper voltage, servo-shaft angle, and rotor tilt angle are distinct quantities.
- **Later flight-capable implementation:** introduce explicit plausibility/fault checks and any necessary electrical protection, stable wiring/strain relief, and validated fallback. Logging-only direct-wire bench testing is not evidence that the unprotected circuit is flight-ready.

### 5.2 DShot / EDT / RPM

- Bluejay **0.21** with EDT arming option; configure and verify compatible Betaflight EDT support.
- On current Betaflight, `getDshotRpm(index)` returns **mechanical RPM** after pole-count conversion. Blackbox `eRPM[index]` instead uses **100 electrical RPM per unit**. For 14 poles, `mechanical_RPM = logged_eRPM * 100 / 7`.
- RPM decoding must use **logical motor indices** and the verified physical output mapping.
- EDT handshakes are an arm-time condition, **not a continuous data-quality guarantee**. Implement real-time freshness/invalid-frame checks and defined stale-data behaviour. Simply reading the latest cached RPM repeatedly does not guarantee a new observation.
- Preserve diagnostic logging for both packet quality and model state; make motor telemetry loss visible to the pilot/flight logs where appropriate.

### 5.3 Scheduling proposals (measure, then decide)

| Operation | Initial proposal | Notes |
|---|---:|---|
| Gyro sampling / filters | Existing 8 kHz | Preserve baseline unless profiling dictates otherwise. |
| PID loop | Existing 4 kHz | Preserve baseline. |
| Motor + servo observers | Up to ~1 kHz or telemetry/ADC update cadence | Timestamp data; never presume a new sample on every invocation. |
| Physical allocator | ~1 kHz initially | Rate and phase depend on mixer integration; avoid introducing extra sample-and-hold latency. |
| Servo PWM output | Existing ~498 Hz | Respect servo electrical and mechanical behaviour. |
| Slow parameter/bias adaptation | 20–100 Hz, later | Disabled until model validated. |
| Blackbox diagnostics | ~1 kHz when practical | Check storage, channel count and CPU impact. |

Use fixed memory; profile CPU cycles, IRQ/DMA interactions, stack usage, control-loop jitter, and Blackbox write impact on the **actual STM32F415**.

---

## 6. Firmware integration approach

**Minimal behavioural delta first.** Preserve a runtime or build-time **LEGACY / LOG_ONLY / CONTROL** mode distinction and ensure reverting to the legacy mixer is straightforward.

| Area | Current Betaflight source / intended role |
|---|---|
| `src/main/flight/servos_tricopter.c` | Start with ADC and finalized-servo-command instrumentation; maintain existing servo mapping in LOG_ONLY. |
| `src/main/flight/mixer_tricopter.c` | Tricopter-specific correction/allocator integration. Currently returns zero motor correction; no physical allocation yet. |
| `src/main/flight/servos.c` | Existing yaw PID-to-servo mixer; eventually drive servo from target angle after calibration/limits. |
| `src/main/flight/mixer.c` | Apply coordinated three-motor outputs and calculate actual actuator saturation/feedback. |
| `src/main/flight/mixer_init.c` | Fixed TRI mixer reference and logical motor definitions; don't change before independent model/allocator is proven. |
| `src/main/flight/tricopter_servo_position.h`, `servos_tricopter.c` | Canonical signed PB0−PC0 position sample, coherent-pair freshness/age and diagnostic raw fields; no raw-PB0 fallback. |
| `src/main/drivers/adc.h` and ADC implementation | Coherent completed-scan PB0/PC0 pair acquisition; raw channels are diagnostic inputs to the canonical corrected-position adapter. |
| `src/main/drivers/dshot.c` / `dshot.h` | Existing `getDshotRpm()` and RPM telemetry interfaces. Add freshness adapter only as needed. |
| `src/main/build/debug.h`, `debug.c` | Create distinct TRI diagnostic debug modes or channels; preserve existing debug enum values. |
| `src/main/blackbox/blackbox.c` | Verify RPM fields and sample timestamps; introduce additional recorded model fields as needed. |
| Proposed `src/main/flight/tricopter_*.c/.h` | Pure model, state observers, allocator, coordinator, and light Betaflight adapters. Exact layout can evolve. |

**Logging caveat:** The debug channel must represent the **final servo PWM written to hardware** if being used to fit servo dynamics; logging earlier in the mixer may record a value subsequently filtered or constrained. Check the order of `servoMixer()`, filtering, and `writeServos()` before deciding where to log.

**Configuration:** Provide configurable parameters with explicit defaults, valid ranges, units, versioning, and profile persistence. Prevent enabling CONTROL without essential calibration and sensor validity. Never silently reuse the old 4.3.1 CLI dump wholesale on 2026.6.2.

### Proposed portable C module contracts

```text
tricopter_geometry      optional SI adapter: motor positions, CG, tilt geometry
tricopter_motor_model   RPM observer, angular momentum and drag torque
tricopter_servo_model   calibrated angle, velocity, predicted position
tricopter_allocator     effectiveness map -> bounded effort/vector targets
                        optional SI wrench/thrust/angle interpretation
tricopter_simulation    offline causal coordinator and synthetic test boundary
tricopter_controller    glue, modes, saturation, fault state, diagnostics
```

Design interfaces around declared units and explicit `dt`; preserve SI units for physical models and keep characterized proxy units distinct. Avoid hidden global timing and heap allocations. Keep model calculations deterministic and suitable for host tests/simulation replay.

---

## 7. Development phases and acceptance gates

### M0 — Baseline preservation and reproducible build

**Execution status (2026-11-06): complete.** Two clean unchanged local builds are byte-identical; source/config/compiler provenance, original input hashes and recovery artifacts are recorded. User-supplied cloud CLI matches the baseline revisions, and the user confirmed correct physical motor/servo mapping. See [M0 evidence](tricopter_m0_baseline.md). The unchanged M0 images remain recovery references; M1 was subsequently flashed and bench-tested.

**Implement:** Freeze original CLI dump, logs, motor resource/reordering config and board photos/notes. Build untouched `2026.6.2` with TRI/servo support; capture GCC/toolchain versions, image hash, build flags, firmware size, target/config revision. Create minimal host test framework and CI/local test script.

**Exit gate:** Repeatable build; legacy motor/servo mapping documented and verified **without props**; stock firmware remains recoverable.

### M1 — Observability / telemetry (no behaviour change)

**Execution status (2026-11-06): flashed and bench-tested; remaining electrical/timing exit gates open.** Full target build and 44 host checks pass. PB0 servo feedback, all three RPM channels, ~1 kHz Blackbox readback, and unchanged final/native servo writes were measured on hardware. The legacy controller remains active. See [bench results](tricopter_m1_bench_2026-11-06.md) and [M1 schema/setup](tricopter_m1_logging.md). Exact pin-edge timing, independent loading/power-sequencing qualification and reliable floating-wire fault detection are not established.

**Implement:** Coherent PB0 wiper/PC0 servo power-ground acquisition, signed **PB0−PC0 as the sole position measurement**, diagnostic raw channels, logged final servo PWM, all three real RPM channels, motor commands, battery voltage, timing and telemetry validity/age. Configure PB0 as `ADC_EXT` and PC0 as `ADC_RSSI`, with the RSSI feature off. Retain `LOG_ONLY` and ADC range/freshness diagnostics. No TLV9001 assembly is needed to begin.

**Exit gate:** Correct logical/physical motor indexing; sane eRPM-to-mechanical conversion; reproducible logs; PB0 reads near the three measured wiper-voltage/nominal ADC reference points without disrupting servo feedback; disconnection/invalid data recognized; servo PWM matches hardware output timing. No new allocation active. The [M1 logging guide](tricopter_m1_logging.md) records the direct wire's floating-input detection limitation; this requirement is not claimed satisfied by a non-rail count alone.

### M2 — Servo hardware and system identification

**Execution status (2026-11-07): unloaded corrected-count characterization, offline prediction and replay implemented; M2 acceptance remains open.** Both free and clamped-case measurements are archived. Paired PB0/PC0 logging identifies a substantial ground-rise component; subsequent corrected-only tests measure direction-dependent endpoints. The portable predictor gives 4.765-count held-out RMS error, with no post-initialization measurement correction. Physical angle zero/travel calibration, independent electrical interpretation and loaded response remain separate work. See [dual-ADC results](tricopter_dual_adc_logging.md) and [offline model evidence](tricopter_offline_models.md).

**Implement:** Use the potentiometer wiper **directly wired to PB0**, replacing the earlier AirS/PC0 proposal, for the first bench recording. Confirm the existing 1.561/1.281/1.031 V wiper points at the ADC; check signal loading and supply sensitivity. **Do not delay this milestone for the analog-component shipment.** Add the TLV9001 only as an optional **unity-gain buffer** if measurements show benefit. Independently calibrate ADC counts ↔ actual rotor tilt angle (zero, sign, stops, travel curve); do not derive angle calibration from PWM commands alone. Measure steps/sweeps, unloaded and under representative force if practical; characterize response, deadband, overshoot and rate limits. Compare command-to-angle with historical command-to-yaw-acceleration observations.

**Exit gate:** Direct-wiper voltage and ADC samples agree under documented supply conditions, with no detected impairment of the servo's own feedback loop; calibrated measured tail angle and bounded predicted angle; fault response tested; no unsafe voltage on PB0 during normal or planned test conditions. If the optional buffer is fitted, repeat the noise/loading/latency measurements.

### M3 — Rotor-state observer and reaction-torque model

**Execution status (2026-11-07): offline RPM observer and placeholder-coefficient yaw reaction equations implemented/tested; physical identification and M3 acceptance remain open.** The pure C observer uses mechanical RPM, observation timestamps/identity, causal acceleration estimates and explicit stale/horizon behavior. The separate torque core reports signed aerodynamic/inertial components using arbitrary synthetic `kQ`/`J`; calibration/flight-use flags remain false. Synthetic cases validate algebra, not real acceleration/torque accuracy. No command/voltage model, identified `kQ`/`J`/`kT`, thrust model or active firmware integration is introduced. See [observer results](tricopter_offline_models.md) and [reaction-torque evidence](tricopter_reaction_torque.md).

**Implement:** Pure-C observer for three motor speeds/accelerations, voltage compensation and telemetry freshness. Separate **aerodynamic** and **inertial** signed torque proxies. Introduce calibrated coefficients `kQ`, `J`, and `kT` only when defensible. Use historical motor-command logs only for qualitative consistency and synthetic replay, **not** as ground-truth RPM. Fit new RPM-equipped data with held-out validation; account for delayed servo torque.

**Exit gate:** Synthetic sign/units tests pass; observer tracks measured RPM with documented latency/error; model predicts transient torque direction and improves fit versus no-reaction-torque baseline on unseen data.

### M4 — Geometric thrust-vector mixer, initially offline

**Execution status (2026-11-07): configurable offline allocator and optional geometric adapter implemented with synthetic placeholders; aircraft characterization and integration remain open.** The primary interface accepts an effectiveness map and effort/vector bounds, reducing dependence on detailed airframe knowledge. It uses a virtual tail-vector basis, not an arbitrary motor/servo-PWM Jacobian. The SI adapter separately constructs the map from CG-relative geometry. Both report bounded targets, achievable response, residual and convergence; an independent forward evaluator can use an actual/predicted angle. No instantaneous actuator tracking or physical calibration is implied. See [model, configuration and validation](tricopter_allocator.md).

**Implement:** Inverse `atan2`/`hypot` tail vector decomposition; account for **actual/predicted angle**, rear vertical effort/thrust, and all three motor pitch/roll effects using characterized effectiveness or approximate geometry. Detailed CAD/CG/inertia knowledge is optional for the effectiveness-map route. Implement bounds and achievable-response residual reporting. Port the same C core to MATLAB/Simulink for comparison where useful.

**Exit gate:** For synthetic requests, reconstructed feasible force/moments agree with forward model within numerical tolerance; all constraints handled deterministically; no invalid outputs at zero/low thrust and near saturation.

### M5 — Controller integration: conservative, reversible

**Preparation status (2026-11-07): the four models are connected only in an offline synthetic actuator/response simulation.** Its causal timing, quality gates, bounded allocation and experimental coordination are testable without the aircraft. The controller interface still requires no CAD/CG/inertia data. MATLAB same-C parity and Simulink signal playback aid inspection; neither validates a physical airframe or enables M5 flight outputs. See [the simulation boundary](tricopter_simulation.md).

**Implement:** Connect Betaflight rate-PID demands to physical allocation (document units/scaling); feed actuator saturation back to integrator control. First enable **static geometric compensation** at conservative gain/limits, then steady-state reaction trim, then dynamic reaction feedforward, **one feature at a time**. Ensure coherent motor/servo timing and safe transition among LEGACY/LOG_ONLY/CONTROL.

**Exit gate:** Prop-off and thrust-stand tests pass; simulated yaw/pitch/roll tracking and saturation tests pass; correct sign/zero behaviour; failure detection reliably disables unsafe corrections; rollback verified.

### M6 — Controlled flight and iterative validation

**Implement:** Controlled hover, small yaw steps, symmetric rapid throttle ramps, deceleration, combined pitch/yaw/throttle, high-throttle saturation only after earlier gates. Compare against baseline and new-flight logs using identical metrics. Retune rate PID only *after* determining physical mixer behaviour.

**Exit gate:** Demonstrable improvement in at least one target metric without worsening roll/pitch stability, runaway/failsafe behaviour, actuator stress, or telemetry reliability; results reproducible across flights and battery voltages.

### M7 — Advanced model and refinement (optional)

**Consider only after evidence:** Full inertia tensor and gyroscopic terms, motor-assisted yaw allocation, aerodynamic gain scheduling, adaptive trim/coefficient identification, dynamic saturation weights, predictive collective slew control, and potentially constrained optimization/MPC. Keep simple bounded methods unless complexity earns its cost.

**Exit gate:** Each addition produces measurable benefit on held-out logs or reproducible flight tests and survives failure-injection tests.

---

## 8. Verification and test matrix

**Host/unit tests:** rotor sign permutations; equal acceleration/deceleration; zero-RPM and stale-RPM behaviour; tilted rear-axis projection; angle/thrust inversion; units and finite outputs; zero/low thrust; servo polarity and travel constraints; throttle/motor saturation; transition modes; anti-windup; deterministic results at irregular `dt`.

**Simulation:** replay identified hover/step/punchout cases; include motor lag, servo lag, battery sag, telemetry quantization/loss, mechanical limits, uncertain `kQ/kT/J`, CG shifts, cross-axis moments and artificial disturbances. Use sweeps to find unstable or unobservable parameter combinations.

**Bench (props off):** resources and motor ordering; receiver/failsafe; bidirectional DShot and EDT; **direct PB0 input at 1.561/1.281/1.031 V wiper reference positions, servo-wiper loading and noise**, ADC polarity and **independently measured rotor-angle** travel calibration; supply/ADC fault conditions; servo speed and stops; command-to-signal latency; Blackbox data integrity. If adding the TLV9001 follower, compare the same measurements before and after buffering.

**Bench (thrust stand / restrained testing):** measured motor command→RPM response, RPM→thrust, drag/torque estimate where possible, servo behaviour under representative tail load. Restraint/propeller testing requires an appropriate guarded setup; do not treat prop-off tests as validation of thrust or torque dynamics.

**Flight:** repeatable low-risk maneuvers; increase authority gradually; compare to legacy on metrics below. Preserve conventional failsafe and runaway protections.

### Performance measurements (baseline first; targets later)

- Yaw-rate step settling time, overshoot and RMS tracking error.
- Maximum uncommanded yaw rate and integrated yaw excursion during throttle steps.
- Yaw-acceleration/model-residual magnitude and transient sign prediction.
- Induced pitch/roll rate or attitude deviation during yaw maneuvers.
- Servo saturation dwell time, angle tracking error, movement speed, and commanded excursion.
- Motor output saturation dwell time; thrust redistribution during tail tilt.
- RPM packet validity, sample age, motor observer error, ADC signal integrity.
- CPU timing worst case, gyro/PID jitter, flash/RAM usage, Blackbox throughput.

Do **not** invent numerical performance targets before a consistent, repeatable baseline exists.

## 9. Failure modes and fallback policy

- **ADC invalid/out-of-range/disconnected/stale:** no measured-angle-dependent compensation; use explicitly bounded legacy or conservative model-only mode and avoid sudden actuator jumps. Fallback must be exercised before flight.
- **RPM invalid/stale/implausible:** freeze or decay observer confidence and disable dynamic torque feedforward; retain safe servo/rate-loop behaviour and log the incident. A valid arming handshake does not override this rule.
- **Servo stalled or saturated:** expose limited yaw authority and suppress yaw integrator windup; avoid increasing rear motor demand beyond thrust/ESC limits.
- **Motor at min/max or limited by dynamics:** recompute achievable forces/moments and prioritize stable flight over impossible yaw requests.
- **Unreliable parameter calibration / model divergence:** degrade or disable model compensation and return to stable conventional rate feedback.
- **Fault in logging/optional subsystem:** must not block core gyro, PID, receiver, motor output, or failsafe tasks.
- **Mode transition:** make transitions bumpless; never change to aggressive allocation merely because telemetry returns for one sample.

For early prototypes, accept *minimal circuitry* but **do not permit experimental control mode to fly without credible electrical limits, sensor sanity checks, and tested fallback**.

## 10. Repository organization and development discipline

Suggested layout (adapt to existing Betaflight tree and build rules):

```text
docs/
  TRICOPTER_CONTROL_PLAN.md        <- this document
  tricopter_hardware.md            <- pinout, motor order, servo wiring
  tricopter_calibration.md         <- measurement method, coefficients
  tricopter_test_matrix.md         <- procedures and results
src/main/flight/
  tricopter_motor_model.[ch]
  tricopter_servo_model.[ch]
  tricopter_allocator.[ch]
  tricopter_controller.[ch]
tests/tricopter/                   <- portable host tests / synthetic cases
tools/tricopter/                   <- log replay, plotting and fit tools
```

**Development rules:**

1. Commit a **known-good, unchanged build** before functional edits; keep each feature separable and bisectable.
2. Gate physical effects independently: **measurement → prediction → offline comparison → bounded feedforward → full allocation**.
3. Preserve original CLI dump and `.bbl` files; annotate experiments with firmware commit, ESC firmware, config export, battery, prop setup, and parameter set.
4. Store physical parameters and their units explicitly; annotate whether values are **measured**, **fitted**, **assumed**, or **synthetic**.
5. Keep integration paths short and measurable. Each new 1 kHz feature must justify its latency, CPU and RAM cost.
6. Make control-enabled changes observable in Blackbox, including **model terms, limits, and fallback reasons**.
7. Treat the model as a hypothesis until reproduced on **held-out flights**, not merely one attractive waveform.

## 11. Open technical questions / decisions

- [ ] Exact modern-target resource map, DMA/timer interactions, and any motor-output reordering changes versus 4.3.1.
- [ ] ADC acquisition rate, wiper source impedance/loading, and sample-and-hold behaviour with the **direct PB0 connection** (the current wiring; earlier PC0 proposal superseded). Is a **TLV9001 unity-gain buffer** actually necessary for clean, low-latency readings?
- [ ] Verify that **1420 µs / ~1.281 V** corresponds to the rear rotor's **vertical** position (needs physical measurement); fit actual ADC voltage/count to tilt angle and check supply-voltage dependence.
- [ ] Effective servo command/position latency under aerodynamic load and at different supply voltages.
- [ ] Best RPM telemetry timestamp/freshness interface and packet handling with EDT interleaving.
- [ ] Best causal `ω/dω/dt` observer and motor command→target-RPM model.
- [ ] How to identify `kQ`, `J` and `kT` without conflating servo correction and airflow-dependent effects.
- [ ] Characterized effectiveness map and its operating region; optional CG geometry/tilt-axis offsets for physical interpretation and coupling checks.
- [ ] Conversion and anti-windup mapping from Betaflight's normalized PID output to the allocator's declared response units; physical moments are an optional interpretation.
- [ ] Optimal scheduler placement to avoid an extra cycle of delay between measured angle, predicted thrust, and servo/motor output.
- [ ] Which advanced gyro/aero terms are **measurably** beneficial after simpler compensation is operating.

---

## Reference materials

- Original **Betaflight 4.3.1** CLI dump: `docs/tricopter_dump.txt`, with `docs/tricopter_diff.txt` (hardware mapping/configuration source of truth for the historical flown airframe; previously referred to as `tricopter.txt`).
- Historical Blackbox recordings: `docs/btfl_001.bbl` … `docs/btfl_006.bbl` (2022-era flights; use for baseline phenomena, not RPM/servo-angle ground truth). Their original bytes are protected by [the checksum manifest](tricopter_originals.sha256).
- Prior prototype: `tricopter_torque_v1` (standalone torque equations and unit tests; **review and integrate selectively**, not flight-ready).
- Betaflight baseline source: <https://github.com/betaflight/betaflight/tree/2026.6.2>
- Betaflight board configuration: <https://github.com/betaflight/config>
- Bluejay ESC firmware: <https://github.com/bird-sanctuary/bluejay>

**Guiding rule:** *Command physical moments. Estimate actual actuator state. Coordinate all actuators. Never assume hardware has moved simply because it was commanded to move.*
