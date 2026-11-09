# Default synchronous-PWM braking approximation

The Gazebo `dc_prop` motor backend now enables active braking by default for both stock and modified Betaflight runs. Firmware gains, geometry, inertia, static prop coefficients and positive-duty steady operating points are unchanged. The earlier [42-flight characterization](TRICOPTER_UPDATED_MODEL_CHARACTERIZATION.md) used the coasting model; its retained numerical results are historical and are not relabeled as braking results.

## Model and settings

The existing averaged motor equation now permits signed terminal current while the drive is conducting:

```text
I = clamp((u * Vbus - Ke * omega) / R, -Ibrake, +Idrive)
Qshaft = Ke * I - Qloss
J * omega_dot = Qshaft - kQ * omega^2
```

Back-EMF above requested voltage produces negative current and braking torque. The implicit mechanical integrator, actual-RPM thrust, rotor spin-up/precession reactions and current link orientation remain the same. Electromagnetic torque is not applied a second time to the airframe.

| SDF field | Default | Meaning |
|---|---:|---|
| `motor_brake_current_limit_a` | 35.22 A | Equivalent terminal braking-current bound; zero restores coasting reductions |
| `motor_brake_on_stop` | false | Whether a valid exact-zero-duty command also brakes |

The C++ parameter defaults and supplied world both enable the 35.22 A braking bound. This is an explicit approximation using the same nominal bound as forward drive, **not an identified ESC braking limit**. The bounds can be changed independently. A fraction of this current limit is not a proportional braking-strength setting: small steps that do not hit the bound can be unchanged.

Active damping during throttle reductions and a separate zero-throttle stop brake are distinct features in [Bluejay's ESC source](https://github.com/bird-sanctuary/bluejay/blob/main/src/Bluejay.asm). [BLHeli_S's manual](https://github.com/bitdump/BLHeli/blob/master/BLHeli_S%20SiLabs/BLHeli_S%20manual%20SiLabs%20Rev16.x.pdf) describes damped regenerative operation. The simulator approximates those effects through an averaged DC equivalent; it does not emulate commutation, MOSFET switching, inductance or the particular ESC firmware.

Valid positive-duty commands can brake by default. Raw PWM1000 maps to exact zero duty and coasts unless `motor_brake_on_stop` is enabled. Armed Betaflight idle normally remains above PWM1000 and therefore retains active damping. Invalid packets, watchdog expiry and electrically suppressed raw PWM0 always open the motor drive, independently of the stop-brake option.

## Energy accounting

Effective applied voltage is calculated for either current sign as `Veff = Ke * omega + R * I`; signed ideal battery current is `Veff * I / Vbus`. Forward current limiting can lower effective voltage; braking current limiting can raise it above the requested duty voltage. Negative battery current returns energy to an ideal absorbing, fixed-voltage bus. With unclipped zero-duty short braking, applied voltage and bus power are zero while copper dissipates rotor energy.

The discrete balance retains copper, mechanical loss, propeller work, rotor kinetic energy and nonnegative backward-Euler numerical dissipation. Numerical dissipation is not motor heat. Battery voltage rise/sag, charge acceptance and ESC switching losses are not modeled. The declared current-regulated bridge is a normal motor-speed-envelope approximation, not a model of externally forced overvoltage.

## Verification

The focused tests pass signed drive/brake current bounds, braking versus explicit coast, monotonic convergence, regenerative power balance, both stop settings, disabled drive, short-brake copper dissipation, unchanged steady equilibria and carrier reaction signs. The compiled suite passes 18,041 motor checks, 428 math/protocol checks and SDF consistency validation; all 56 Python helper tests pass.

Actual fixed-body Gazebo tests passed in [normal rotation](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261108T230049-plant-normal-300190/plant-check.json) and [reversed rotation](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261108T230059-plant-reversed-300190/plant-check.json). They now command a positive-duty 1700→1350 µs reduction in addition to joint travel/ADC/thrust-axis checks. Both recorded negative terminal current down to −35.22 A and returned bus current, then verified zero electrical current and passive coast after watchdog expiry. Sampled electrical-power residuals were below 2.5e−9 W.

The [portable replay](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/active-braking-review/README.md) compares 15 cases using the exact frozen header and world parameters at 1 ms. All pass finite response/settling, signed current bounds, monotonicity and discrete torque/power checks. Maximum power residual is 7.96e−12 W. These are model calculations, not physical ESC measurements.

| Equivalent voltage-duty reduction | Coast thrust 10–90% | Default braking thrust 10–90% |
|---|---:|---:|
| 0.7 → ideal hover (0.344621) | 106.78 ms | 26.88 ms |
| Hover → hover − 0.025 | 28.44 ms | 27.84 ms |

All paired upward trajectories are bit-identical. For the larger reduction, lowering the assumed brake-current bound to 17.61 / 8.805 A gives thrust times of 36.28 / 51.71 ms. This provides adjustable, explicit uncertainty rather than identifying an ESC constant. Default zero-duty stop and disabled-drive coast retain the same 311.78 ms thrust movement time; selecting the separate stop brake gives 20.18 ms. The large reduction returns 6.84 J to the ideal bus at the default bound; copper, propeller/mechanical losses and numerical dissipation account for the remainder of rotor kinetic energy.

[The standalone response figure](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/active-braking-review/motor-braking-comparison.png), SVG, complete traces, JSON/CSV metrics, source snapshots and hashes are retained beside the replay script.

All **14 fresh flights** passed in [the stock/modified verification campaign](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261108T230240-both-active-braking-validation-301633/comparison-summary.json): one repetition of each of the seven existing yaw/collective/control cases per firmware, with the same 1345 µs hover input and unchanged PID gains. This is integration verification, not a replacement for the previous three-repetition performance comparison or an independently tuned ranking.

The [independent audit](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261108T230240-both-active-braking-validation-301633/independent-fresh-protocol-audit.json) checks all 41 frozen inputs, all 32 plugin build-source hashes, common settings, current bounds, signed power identities, actual commands/timing, arming/mode health, measured ADC and cleanup records. Both firmware binaries and CLI settings match the earlier coasting campaign exactly. All 4,779 measurement actuator frames were valid, all 349 sampled statuses were healthy/armed in ANGLE mode, and all 168 modified feedback samples were valid. PC0 stayed zero and PB0 map error stayed below 0.5 count. Maximum stage error was 20 ms and RC send gap 23.715 ms. Signed power residuals stayed below 2.1e−9 W over 23,909 full plant frames.

Full flight histories contained 297 negative-current rotor samples and 297 negative-bus-power rotor samples within the declared bounds. They occurred during setup/landing; the small, receiver-smoothed measurement pulses retained positive terminal current. The stronger prescribed-input anchored reductions above directly verify braking, independently of those low-amplitude flight pulses. No motor/servo endpoint saturation occurred during measurement; 18 servo endpoint frames occurred outside measurements. All trials ended disarmed with PWM1000 commands. [Host cleanup verification](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261108T230240-both-active-braking-validation-301633/host-process-cleanup.json) found no remaining campaign processes; this does not imply instantaneous zero RPM during the selected stop/coast policy.
