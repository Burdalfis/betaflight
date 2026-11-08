# Gazebo tricopter plant

This directory contains a Gazebo **Jetty / gz-sim 10** model-system plugin, an asset-independent SDF world and focused math/protocol checks. Follow [the SITL runner guide](../README.md) for stock/modified Betaflight builds and RC input.

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

The SDF mass uses the operator's approximate total, and thrust uses the estimate below. Inertia, geometry, drag, reaction ratio and loaded servo coefficients remain estimates. Motor speed state follows first-order lag; thrust is `max_thrust_n × state²`. Servo target has first-order lag and a rate bound, followed by torque-limited joint PD. A zero servo PWM or actuator watchdog expiry removes servo drive torque; it does not synthesize a center pulse. Motor command expiry requests zero and lets rotor state spin down.

Optional `rotor_inertia_kgm2` adds the carrier reaction from changing rotor-relative angular momentum, including axis reorientation; it defaults to zero. If enabled, the SDF link inertias must include the corresponding locked-rotor inertia. No aerodynamic inflow, propeller blade model, battery model or physically identified loaded dynamics are claimed.

## Default aircraft approximation

The operator requested approximately **500 g including battery**, 1800 KV 2207 motors, 6S and Gemfan Hurricane 51466 props. The default SDF allocates 38 g to each motor/prop assembly and 386 g to the remaining aircraft, totaling 500 g. These component allocations are estimates. Body and local motor inertias scale with their changed masses, preserving the previous assumed radii of gyration; lever arms and inertia distribution still need real geometry to establish matching rotational dynamics. The rendered prop diameter is 5.1 inches.

[iFlight's XING2 2207 1855 KV test sheet](https://iflight-public.oss-cn-hongkong.aliyuncs.com/Support/lnstructions/XING2-MOTOR/XING2-2207-1855KV-EN.pdf) reports **1685 g thrust with GF51466 at 24 V**. Its [1800 KV XING-E Pro sheet](https://iflight-public.oss-cn-hongkong.aliyuncs.com/Support/lnstructions/XING-E-MOTOR/XING-E-Pro-2207-1800KV-EN-1.pdf) tests other props, so it does not supply the exact requested combination. The default uses this explicitly approximate scaling from the nearby exact-prop result:

```text
max_thrust_n = 1.685 kgf × 9.80665 × (1800/1855)² × (22.2/24)²
             = 13.31255 N per motor
```

The KV/voltage-square scaling assumes proportional loaded speed; winding resistance, motor variant, ESC and prop differences limit its accuracy. This is a **fixed nominal-6S estimate**, without voltage sag, discharge or current simulation. At the same assumed scaling, 24 V gives 15.56 N and 25.2 V gives 17.15 N per motor. Edit `max_thrust_n` deliberately to compare those operating points. Nominal static thrust/weight is about 8.15, and ideal level hover speed command is about 35% before yaw/vector compensation. ESC duty, stick throttle and observed flight hover need not match that estimate exactly.

The default **30 ms speed time constant** is an estimate, not measured XING/51466 response. [NeuroBEM, motor dynamics Eq. 20](https://rpg.ifi.uzh.ch/docs/RSS21_Bauersfeld.pdf) identifies 33 ms on a different Betaflight aircraft with XRotor 2306 motors and 5-inch three-blade props, supporting its order of magnitude. It implies about 65.9 ms RPM 10–90% time; thrust is squared after speed lag, so a stopped-to-full thrust step takes about 77.7 ms from 10% to 90%. The project's unloaded 7 V bench recordings instead suggest roughly 50 ms speed lag, but they do not identify the loaded 6S setup. `motor_time_constant_s` remains configurable. No additional characterization is required to run either firmware.

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

Set `TRICOPTER_TRACE=/absolute/path/trace.csv` to record 100 Hz simulation-time observations; this overrides optional SDF `trace_path`. The CSV includes raw actuator PWM, actual joint position/rate, filtered servo target, PB0/PC0, three thrusts and signed aerodynamic reaction torques, world position, attitude, gyro, actual rear world thrust axis, joint torque and actuator-watchdog state. The requested trace path is overwritten at startup.

The native gyro helper is exercised directly by the C++ test. The static SDF test checks logical link mapping, a real revolute child joint, calibrated endpoint consistency, −X hinge and no external assets. These checks do not replace compiling and running the actual Gazebo plugin.
