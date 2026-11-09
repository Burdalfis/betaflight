# Gazebo tricopter plant

This directory contains a Gazebo **Jetty / gz-sim 10** model-system plugin, a self-contained SDF world with bundled CAD visual meshes and focused math/protocol checks. Follow [the SITL runner guide](../README.md) for stock/modified Betaflight builds and RC input.

```sh
cmake -S tools/tricopter/sitl/gazebo -B .local-artifacts/tricopter/gazebo-build
cmake --build .local-artifacts/tricopter/gazebo-build -j
ctest --test-dir .local-artifacts/tricopter/gazebo-build --output-on-failure
```

Jetty uses the unversioned CMake packages `gz-sim` (version 10) and `gz-plugin` (version 4). Set `GZ_SIM_SYSTEM_PLUGIN_PATH` to the build directory so the SDF can load `libTricopterBridge.so`. Without Gazebo installed, `-DTRICOPTER_BUILD_PLUGIN=OFF` still builds the pure math/protocol tests and checks the SDF topology. See the [official plugin example](https://github.com/gazebosim/gz-sim/tree/gz-sim10/examples/plugin/system_plugin) and [Jetty installation](https://gazebosim.org/docs/jetty/install_ubuntu/).

## Articulation and coordinates

The model frame is **FLU**: +X nose, +Y left, +Z up. Logical motor 0 is rear, 1 front-right and 2 front-left. The front motor links are fixed to the body. The rear motor link is a physical child on the bounded `tail_joint`; the plugin applies finite servo torque to that joint. It never teleports the basket or computes its thrust direction from commanded PWM.

The supplied **synthetic −X hinge** makes increasing joint angle / PWM produce leftward thrust (+Y) and clockwise yaw (−Z), matching Betaflight's positive yaw convention with `tri_yaw_direction=1`. This establishes the simulator convention only; the real aircraft's stabilizing yaw polarity remains a separate check. The historical offline plan uses the opposite physical angle convention.

Each rotor's thrust and aerodynamic reaction torque use its **actual link orientation** and are applied to that link, so physics supplies the offset moments and joint reactions. Scalar reaction torque is `−spin_sign × reaction_ratio_m × thrust`, where +1 means rotor CCW viewed from above and −1 means CW. The supplied `1 -1 1` matches rear CCW, right CW, left CCW. Negating all three signs models the planned rotation reversal without changing thrust or physical calibration.

The SDF uses the CAD-derived motor and hinge geometry and the component mass budget below. Missing parts and material distributions have explicit assumptions; drag and loaded servo coefficients remain unmeasured estimates. The default `dc_prop` motor model integrates speed from signed electrical drive/braking torque, mechanical loss, propeller load and rotor inertia. Custom worlds can still select the older `lag` backend. Servo target has first-order lag and a rate bound, followed by torque-limited joint PD. A zero servo PWM or actuator watchdog expiry removes servo drive torque; it does not synthesize a center pulse. Motor command expiry opens the electrical drive and leaves mechanical/propeller-load coasting, independently of the valid-command braking settings.

The servo plant's derivative gain is **0.003 N·m·s/rad**. The previous 0.03 value produces sampled-PD chatter at a 1 ms physics step with the CAD basket's roughly 6.8 × 10⁻⁶ kg·m² hinge inertia. The smaller value is a numerical correction to the unmeasured servo plant, not a firmware PID change or identification of the real ES08MD II's loaded response. Position gain, torque/speed limits and calibration remain as before.

The default **rotor spin inertia is approximately 4.567 × 10⁻⁶ kg·m² per motor/prop**. The component SDF tensors already include the rotating mass once in their locked configuration. The bridge additionally applies the carrier reaction from relative rotor angular momentum:

```text
H_relative = spin_sign * J * omega * actual_rotor_axis
reaction = -spin_sign * J * (omega_dot * axis + omega * (carrier_omega cross axis))
```

Aerodynamic reaction is `−spin_sign × prop_torque`; electromagnetic motor torque is not applied a second time to the aircraft. The RPM ODE neglects carrier angular acceleration projected onto the rotor axis, a small approximation relative to the aircraft yaw inertia. No aerodynamic inflow, blade-flexibility model, thermal model or battery sag is claimed.

### Airframe drag

The default `airframe_drag_model=quadratic_body` replaces the former purely linear translational drag. Air-relative velocity is expressed in body FLU, and each component produces

```text
F_body[i] = -0.5 * air_density_kg_m3 * drag_cda_body_m2[i]
            * abs(v_air_body[i]) * v_air_body[i]
```

The default density is **1.225 kg/m³**, with body-axis **CdA = [0.010, 0.012, 0.025] m²**. These are explicit, configurable engineering placeholders for the exposed frame, battery and motors, not measured drag coefficients or CAD-derived projected areas. The larger body-normal coefficient makes attitude affect the world force. [The aerodynamic assumptions](model/airframe_aerodynamics.json) record the parameters and limits. For example, forward body flow at 10/20 m/s gives 0.6125/2.45 N opposing force. This is a homogeneous quadratic, dissipative approximation; it does not add an aerodynamic lift, blade inflow, ground-effect or wind model, and it does not establish the real aircraft's maximum speed.

The bridge computes velocity at the instantaneous whole-aircraft CG from mass-weighted link-CG velocities, including the basket's motion. It distributes the resulting world force to the four link CGs in the same mass fractions. Their resultant passes through the current aircraft CG, so translational drag adds no net moment about it and its mechanical power is `F_world dot v_CG <= 0`. Angular drag remains the original pure `−0.02 * world_angular_velocity` N·m torque on `base_link`.

For historical reproduction, select **`airframe_drag_model=linear_world`**. That branch retains `−linear_drag_n_s_m * base_link_origin_world_velocity`, applied at the original `base_link` origin, with the former coefficient **0.15 N·s/m**. A world that omits the new selector uses the new quadratic default. Stock and modified firmware must use the same plant; previous linear-drag recordings remain historical results and must not be presented as results for this nonlinear baseline. Rotor/DC motor, active-braking, servo, calibration and physical mass/inertia coefficients are unchanged.

## Default aircraft approximation

The default represents the supplied **iFlight XING-e 2207 1800 KV / GF51466 / 6S** assembly. The component budget gives **0.5099032 kg** ready to fly, including the 222 g battery and 11.2 g strap. The front motor spacing is **165.037 mm**, and the rear-to-front spacings are **179.158 / 178.911 mm**. The rear mount is 8.8 mm above the front mounts.

The body datum is the centre of the regular midplate's lower face. CAD coordinates transform to FLU as **(−X, +Z, +Y)**. At neutral basket angle, the total CG is **(-8.811, +0.048, -7.296) mm** from that datum, and the CG-centred inertia diagonal is **(0.000892456, 0.001569079, 0.002008865) kg·m²** for roll/pitch/yaw. Full cross terms are retained, including `Ixz = 0.0000618298 kg·m²`; the body axes are not assumed to be principal inertia axes.

Four disjoint mass groups prevent double counting:

| SDF link | Mass | Contents |
|---|---:|---|
| `base_link` | 381.2033 g | Fixed structure, battery, electronics, fixed hardware and servo |
| `right_motor` | 38.0000 g | Right motor including wires, and prop |
| `left_motor` | 38.0000 g | Left motor including wires, and prop |
| `tail_motor` | 52.6999 g | Motor/prop, moving carbon basket, bearings and basket hardware |

Each link's inertial pose is its group CG relative to the motor-mount/body link origin; its tensor is about that CG. Thrust is applied at the motor-axis link origins. The rear joint is explicitly placed at the CAD hinge, **10.7 mm above the rear motor mount**, so the rear mass and thrust line rotate about the correct pivot. The −X joint sign retains the established simulator/PWM convention.

[The frozen CAD input](model/cad_geometry.json), [81-component budget](model/component_budget.csv), [calculation](model/calculate_mass.py) and [complete mass properties](model/mass_properties.json) are versioned together. Calculation uses Python and NumPy, without requiring FreeCAD or the original STEP at runtime:

```sh
python3 tools/tricopter/sitl/gazebo/model/calculate_mass.py --check
# Optional: check the original CAD file's identity too.
python3 tools/tricopter/sitl/gazebo/model/calculate_mass.py --check --verify-step Tricopter_motors.step
```

The calculation ignores embedded STEP densities. Carbon uses an assumed bulk density of 1.60 g/cm³; the missing bottom plate duplicates the regular `midplate` beneath the arms, while `Midplate_Modded` remains separate. The top plate retains 70% of its 40 × 20 × 4 mm envelope. The battery is 78 mm longitudinal × 42 mm transverse × 39 mm vertical, flush below the bottom plate. The open rear-arm shell was reconstructed from a mid-thickness section and extruded through its 5 mm thickness. Missing fasteners are counted from the supplied inventory, replacing unusable CAD screw surfaces.

FC/ESC masses are estimated at 8/10 g, and an additional 10 g covers power wiring, capacitor, small mounts and missing linkage. Electronics locations, top standoff locations, strap distribution and the stationary motor residual distribution remain estimates. The nominal mass-assumption sensitivity is 488–530 g; this is not a confidence interval and does not bound position uncertainty. STEP-derived visual meshes preserve the modelled frame cutouts, motor assemblies, bearings and rear basket. The duplicated bottom plate uses the regular midplate outline. Parts absent from CAD retain estimated primitive visuals; collision shapes remain simple primitives. Physics inertias come from the component budget, not the visual or collision meshes. No synthetic landing feet remain: the battery is the lowest contact surface. The rendered prop diameter is 5.1 inches. These are calculated estimates, not measured or flight-validated aircraft properties.

### Visual meshes

The [STL manifest](meshes/manifest.json) records source STEP identity, tessellation settings, link-local metre bounds, triangle counts and hashes. Meshes are grouped on the correct fixed or articulated links. `run.py` resolves their relative paths before copying a world into a session directory, so launches work from any working directory without Fuel downloads or a FreeCAD installation.

To regenerate geometry, run [export_visuals.py](model/export_visuals.py) in a FreeCAD Python environment with `Part`, `Import`, `Mesh` and `MeshPart`, passing the original STEP. The exporter checks its SHA256. This exports the supplied CAD motor/servo geometry as-is; those models remain proxies for the actual XING-e/ES08MD II hardware. Existing malformed screw/nut surface placeholders are omitted. Missing real hardware is already represented in the mass budget.

Only the three propeller blade visuals are replaced by **thin, semi-transparent discs**, radius 64.77 mm and thickness 0.8 mm, on their existing motor links. Each disc is centered on the motor axis at the corresponding CAD blade envelope's vertical midpoint (about 25.118 mm above the mount). These static rotating-envelope visuals make the basket angle readable; they do not simulate exposure blur or stopped blades. The original blade STL assets and hashes remain as provenance for the unchanged prop mass and inertia. Discs add no collisions, masses, inertia, spin animation or aerodynamic area; dynamic RPM still drives all thrust, prop torque and gyroscopic calculations independently.

### Rotor inertia

[rotor_inertia.py](model/rotor_inertia.py) and [rotor_properties.json](model/rotor_properties.json) estimate 14 evenly spaced NdFeB magnets, the steel back-iron annulus, 6061 aluminum upper bell web and hardenable stainless shaft. Magnet height is nominally 7.75 mm, gap 0.2 mm, arc length 4.75 mm at mean radius and radial thickness 1.2 mm. Back iron is 1.2 mm thick and 0.75 mm taller. The generic CAD supplies the vented upper web and stepped/hollow shaft; neither is a measured XING-e bell.

The bell/shaft estimate is **15.99 g and 1.900 × 10⁻⁶ kg·m²**. The stationary motor residual is normalized so each complete motor remains **33.8 g including wires**. The prop CAD volume is **3.275 cm³**: typical polycarbonate density 1.20 g/cm³ predicts **3.93 g**, while the supplied **4.2 g** mass is retained for physics, giving **2.667 × 10⁻⁶ kg·m²**. Rotating transverse tensors are azimuth-averaged. Combined spin inertia is **4.567 × 10⁻⁶ kg·m²**. The documented height/density sensitivity is not a complete uncertainty bound: actual bell mass, any lower aluminum skirt, bearing inner races, retainer/prop nut and material distribution remain uncertain.

### Motor and propeller dynamics

The default is an averaged **six-step, two-terminal DC-equivalent** BLDC model. For the confirmed delta winding, measured line-to-line resistance is **81.5 mΩ**, individual delta branches are **122.25 mΩ**, and equivalent wye phases would be **40.75 mΩ**. Terminal resistance stays 81.5 mΩ in `V_eff = Ke × omega + R_LL × I_eq`: replacing it with a branch resistance would overstate copper loss. `I_eq` is signed equivalent terminal current: positive drives the rotor and negative supplies braking torque. It is neither the current in one delta branch nor average battery current. Ideal PWM bus current is `V_eff × I_eq / V_battery`. Detailed sinusoidal commutation, delta circulating harmonics and PWM inductive ripple are outside this model.

The target motor uses **1800 KV** and the operator-supplied **1.2 A no-load current at 10 V**. A smoothed Coulomb loss model is calibrated to reproduce that exact no-load point; a single point does not identify high-speed iron/bearing losses. Its asymptotic loss-current coefficient is about 1.213 A, distinct from the measured 1.2 A reference. Mechanical integration is implicit and uses the material-derived rotor inertia; the older 30 ms speed lag is not layered on top.

[The performance calculation](model/motor_performance.py) and [complete calibration report](model/motor_performance.json) retain source hashes, equations and assumptions. The nearby [iFlight XING2 2207 1855 KV table](https://iflight-public.oss-cn-hongkong.aliyuncs.com/Support/lnstructions/XING2-MOTOR/XING2-2207-1855KV-EN.pdf) supplies the exact GF51466 prop point: **1685 g thrust and 35.08 A at 24 V/full command**, with 50 mΩ line resistance. Its no-load current is separately assumed at 1 A; the target motor's supplied 1.2 A value is not silently substituted. The [XING-E Pro 1800 KV sheet](https://iflight-public.oss-cn-hongkong.aliyuncs.com/Support/lnstructions/XING-E-MOTOR/XING-E-Pro-2207-1800KV-EN-1.pdf) confirms the target resistance but tests different props and does not provide loaded RPM.

The reference operating point establishes static coefficients in `T = kT × omega²` and `Q = kQ × omega²`; these are transferred to the target motor. User-supplied **PConst 1.13 / TConst 0.88** are retained as metadata: they are calculator correction factors, not direct aerodynamic coefficients, and applying them again to an already calibrated GF51466 result would double-correct it. The bench's partial-throttle labels are not identified physical ESC voltage duty: treating its 50% label as 50% duty underpredicts thrust by about 48%. No exact bench command-curve fit or identified ESC mapping is claimed.

At nominal **22.2 V**, the model predicts approximately **36,075 RPM, 12.628 N and 26.48 A per motor at full command**. Ideal equal-thrust hover is about **13,106 RPM at 34.46% voltage duty**, before tricopter yaw/CG compensation. The SDF's `max_thrust_n`, `reaction_ratio_m` and `max_rotor_speed_rad_s` are compatible steady-state diagnostics/legacy-backend fields; the active DC backend computes actual thrust, torque and speed from the circuit/load equations instead. `motor_current_limit_a = 35.22` is the positive drive-current bound.

The default now approximates the confirmed active-braking ESC behavior. At a valid positive duty, the motor can produce negative current when back-EMF exceeds requested voltage:

```text
I_eq = clamp((duty * V_battery - Ke * omega) / R_LL,
             -motor_brake_current_limit_a, motor_current_limit_a)
```

`motor_brake_current_limit_a` defaults to **35.22 A**; zero restores the former coasting-only response to downward commands. Both current bounds are explicit model assumptions, not identified hardware protection settings. A valid zero-duty command (1000 µs) coasts by default (`motor_brake_on_stop = false`). Set `motor_brake_on_stop` true to apply the same bounded braking at zero duty. Watchdog/invalid input and a suppressed raw zero-PWM output always disable electrical torque and coast, even when braking on stop is enabled. The portable `StepMotor` API exposes that distinction through its `driveEnabled` argument.

Effective voltage is reconstructed from back-EMF and the signed, limited current. Negative bus current represents ideal energy returned to a supply held at **22.2 V**; braking at zero effective voltage can instead dissipate energy in winding resistance without returning it to the bus. This is an approximate bidirectional electrical power balance, not an identified ESC control law or battery charging model. Battery acceptance, voltage rise/sag, switching losses, current-loop dynamics and active-braking strength have not been measured. Copper loss remains `R_LL × I_eq²`; implicit integration dissipation remains a separate numerical term. Aerodynamic reaction plus rotor spin-up reaction already transmit electrical acceleration/braking torque to the airframe, so no extra braking body torque is added.

The earlier updated-model motor replay and 42-flight comparison used nonnegative current and coasting downward steps. Their source-bound reports are retained as historical results and do not describe the new braking-enabled transients.

Regenerate and check all properties without FreeCAD:

```sh
python3 tools/tricopter/sitl/gazebo/model/rotor_inertia.py
python3 tools/tricopter/sitl/gazebo/model/calculate_mass.py
python3 tools/tricopter/sitl/gazebo/model/motor_performance.py
python3 tools/tricopter/sitl/gazebo/model/apply_model_properties.py
# Each command also supports --check for read-only consistency verification.
```

## Third-person view

The optional `camera_heading_model` names a separate collisionless, gravity-free kinematic world model with no geometry or aircraft mass. The bridge uses normal model pose commands to copy actual aircraft position and yaw into this target, with zero roll/pitch. The GUI follows it from local offset (−2.4, 0, 1.0) m and looks at the aircraft, with both gains set to 1. This keeps the horizon level and removes full-attitude orbiting and interpolation lag. The target does not affect aircraft force, servo, sensor or actuator calculations. The runner adds or upgrades this empty target in older world copies automatically.

## Measurement and wire interfaces

All UDP addresses are loopback. Ports are configurable in the plugin SDF:

| Direction | Default port | Format |
|---|---:|---|
| Betaflight → plugin | 9001 | Existing stock 68-byte `<H2x16f` raw PWM: motorCount must be 3; floats 0/1/2 are tail/right/left; float 3 is physical servo0 |
| Plugin → Betaflight | 9003 | Existing 144-byte `<18d` FDM |
| Plugin → modified ADC backend | 9005 | 32-byte `<IHHdIIHHI`: `TDA1` magic, version 1, size 32, simulation time, epoch, sequence, PB0, PC0, reserved zero |

The plugin does not consume quad-only normalized motor packets on port 9002. It rejects wrong-size/count, nonfinite and malformed raw PWM packets. No physical motor-resource reordering is applied again in the plant.

Pause, resume, reset and long scheduling gaps invalidate actuator liveness. The bridge discards the pending raw receive queue, waits for it to be completely empty, sends a new FDM state, then requires a newly received actuator packet. A queue larger than the bounded per-tick drain remains disabled until emptied; old queued packets cannot acquire a fresh receipt time on resume. Motor lag state is retained across pause, so zero command causes physical spin-down on resumed physics steps. The unchanged raw PWM ABI has no source timestamps or sequence numbers; this gate enforces the receive/state boundary rather than claiming packet-level age authentication.

FDM transmission is paced identically for stock and modified firmware: one normal state packet remains outstanding until a valid raw-PWM response arrives, then the bridge waits at least 1 ms of wall time before the next state. Startup/lost replies permit a retry only after 30 ms in both simulation and wall time. This prevents queued FDM bursts from corrupting native SITL's instantaneous simulation-rate estimate and causing false receiver timeouts. ADC measurements still publish every physics update with actual simulation timestamps; FDM pacing does not hold the simulated potentiometer at the last command or last FDM state.

PB0 is the bounded inverse three-point angle/ADC map of **actual joint position**, rounded to 12-bit counts; PC0 is always zero. Default knots are −47.8°/0°/+21.6°, ADC 1936/1490/1281 and PWM 1100/1509/1700 µs. Actual position remains independent of the PWM target when the joint lags, saturates or is disturbed. Reset increments the ADC epoch and restarts sequence numbers; paused/repeated simulation timestamps do not refresh samples.

The FDM gyro is body FRD `(ωx,−ωy,−ωz)`, matching native `sitlGyroBodyFromSim`. Packet acceleration is **the negative of FLU body specific force** `−Rᵀ(a_world−g_world)`, because `sitl.c` negates all three packet axes before virtual acceleration input. This acceleration wire convention differs from the gyro frame transform. Quaternion is `(w,x,−y,−z)` from the FLU→ENU link quaternion, matching native Gazebo conjugation. Geographic positions are mirrored to compensate native SITL's existing Gazebo GPS mirror; velocities remain ENU.

Set `TRICOPTER_TRACE=/absolute/path/trace.csv` to record 100 Hz simulation-time observations; this overrides optional SDF `trace_path`. The CSV includes raw actuator PWM, actual joint position/rate, filtered servo target, PB0/PC0, three thrusts and signed aerodynamic reaction torques, world position, attitude, gyro, actual rear world thrust axis, joint torque and actuator-watchdog state. Appended fields retain rotor angular speeds, world-frame spin-up/precession reaction torques, signed equivalent terminal/battery currents, effective motor voltage, shaft/prop torques and implicit integration dissipation. New airframe-drag fields retain the pre-step world force, dissipative power and body air-relative velocity. Negative terminal current denotes braking; negative bus current denotes the model's ideal returned electrical power. The requested trace path is overwritten at startup.

The native gyro helper is exercised directly by the C++ test. The static SDF test checks logical link mapping, a real revolute child joint, calibrated endpoint consistency, the offset −X hinge, offline assets, per-link physical inertia and independent reconstruction of the total mass/CG/full tensor. These checks do not replace compiling and running the actual Gazebo plugin.
