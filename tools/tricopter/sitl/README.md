# Tricopter Gazebo SITL: stock and feedback Betaflight

Run the same articulated tricopter plant with either the stock Betaflight **2026.6.2** controller, pinned at `e0b7bb01b17b21351057e9ead2d1ab39dd44fa16`, or this checkout's measured-position feedback controller. **Both default simulator builds use the identical common SITL coordinate correction.** The stock input is exported from that commit and checked against every tracked Git blob; a separate source copy receives the four-file simulator-only patch. Stock retains its original tricopter mixer, while modified adds measured-position allocation and virtual ADC. This baseline is labelled stock controller with common SITL correction, not a byte-unchanged upstream binary. Physical STM32 firmware sources and hardware paths are unchanged by the correction. See [the patch, provenance and validation](../../../docs/TRICOPTER_SITL_FRAME_CORRECTION.md).

The default airframe uses the supplied CAD geometry and a **509.9 g component mass budget**, with iFlight XING-e 2207 1800 KV motors, GF51466 props and nominal 6S voltage. Front motor spacing is 165.0 mm; the model includes the calculated CG, full inertia tensors and offset rear hinge. Missing parts and mass distributions retain explicit assumptions. Motor/prop torque, loaded RPM and gyroscopic reactions now follow a DC-equivalent circuit/load model with explicit assumptions; drag and loaded servo behavior remain estimates; this is not an identified aircraft model. Both variants use the same world and common receiver/PID settings. Every run records its exact firmware, plugin, world and configuration hashes.

## Install and build

This host uses Ubuntu 26.04. Gazebo **Jetty / gz-sim10** supports it; the installer follows the [official binary installation](https://gazebosim.org/docs/jetty/install_ubuntu/). It adds the signed OSRF repository and installs Gazebo, development headers and input/build dependencies. It backs up existing files at the two Gazebo repository/key paths before replacing them. It does not install ROS.

From the repository root:

```sh
sudo bash tools/tricopter/sitl/install_gazebo_jetty.sh
python3 tools/tricopter/sitl/build.py
```

The corrected source snapshots, independent stock/modified object directories, binaries and build manifests live in `.local-artifacts/tricopter/gazebo-sitl/corrected-frame/`. `build.py --variant stock` and `--variant modified` build separately. Both use the identical `EXTRA_FLAGS=-DUSE_SERVOS` compile option and the same default `--sitl-adapter corrected-frame`. Bare SITL otherwise omits servo support. Full snapshot/preimage/postimage manifests record the common patch explicitly; the original pristine stock input still verifies 4,750 Git blobs. Existing FC firmware build artifacts and physical FC configuration are not used. The runner refuses changed source/binary/adapter hashes until that variant is rebuilt.

The earlier plain binaries and all old comparisons remain under the parent artifact directory. Historical byte-unchanged stock can be built explicitly with `--sitl-adapter none --artifacts /separate/plain-build-directory`; run it by passing the same `--artifacts` path. The tools refuse to overwrite plain artifacts with corrected builds or vice versa. The old default-estimator frame mismatch makes those historical binaries unsuitable for interpreting rapid tilted-turn performance.

## Xbox now, TX16S later

Pair the Xbox controller using the operating system's Bluetooth settings, then discover and inspect its input device:

```sh
python3 tools/tricopter/sitl/rc_input.py --list
python3 tools/tricopter/sitl/rc_input.py --inspect 'Xbox'
```

These commands do not send RC. If its Linux device name differs, use the printed name or `/dev/input/eventN`. Inspect each stick/trigger/button; the supplied profile checks required capabilities and uses reported kernel axis ranges. See [input controls and calibration](RC_INPUT.md).

```sh
python3 tools/tricopter/sitl/run.py --variant stock --device 'Xbox'
# Stop with Ctrl-C before starting the other variant.
python3 tools/tricopter/sitl/run.py --variant modified --device 'Xbox'
```

The runner builds/checks the Gazebo plugin, creates a fresh session EEPROM, applies the common settings plus the modified-only overlay when appropriate, and starts Betaflight, Gazebo and the HID bridge. It prevents concurrent sessions sharing the same ports and stops its own child processes on exit. Omitting `--device` opens an unarmed simulation with no RC stream. Use `--headless --duration 15` for a bounded server run.

The native Ubuntu Gazebo build uses a clean system toolchain environment and refreshes CMake's dependency cache on each launch. This prevents an active Anaconda environment from supplying incompatible Protobuf or other libraries. The controller bridge uses `/usr/bin/python3`, where the installer provides `python3-evdev`. These environment changes apply only to child processes; your shell and Conda installation are unchanged.

The viewer uses the verified **Intel/Ogre1** rendering path on this laptop. Its third-person camera stays **2.4 m directly behind and 1.0 m above** the aircraft, following position and heading with a level horizon. A separate invisible target removes aircraft roll/pitch from the camera offset, and unity follow/track gains remove the old spring-like lag. `--no-follow` keeps a fixed camera after framing. `--gui-engine ogre2` selects Ogre2 on a machine with compatible graphics drivers. Ogre2's hybrid-GPU EGL probe stalled on this host, so changing power mode or selecting the Nvidia GPU is unnecessary for this model. The XWayland environment selection applies only to the Gazebo child process. See [Gazebo's rendering troubleshooting](https://gazebosim.org/docs/jetty/troubleshooting/).

The shared configuration retains the physical aircraft's existing PID gains and rate curves. ANGLE mode is capped at **30°** for the gamepad (previously 60°). Centering the stick levels the aircraft; ANGLE mode does not brake horizontal velocity or hold position automatically.

The independently tuned **simulation yaw profiles** use P31 / I20 / D10 / F0 for both variants. They override yaw only; the shared defaults retain the original gains. To fly with the evaluated profiles:

```sh
python3 tools/tricopter/sitl/run.py --variant stock --device 'Xbox' \
  --config tools/tricopter/sitl/configs/stock_yaw_autotuned_sitl.cli
python3 tools/tricopter/sitl/run.py --variant modified --device 'Xbox' \
  --config tools/tricopter/sitl/configs/modified_yaw_autotuned_sitl.cli
```

Stop the first session before starting the second. These are simulator results, not physical-aircraft PID recommendations. See the [independent yaw tuning protocol and results](../../../docs/TRICOPTER_YAW_AUTOTUNE.md).

Xbox defaults:

- Right trigger: throttle, released = low.
- Right stick: roll/pitch. Left stick X: yaw, inverted on the wire so stick-right gives clockwise/right yaw in the declared simulator convention.
- A: toggle arm; B: disarm. Y: toggle ANGLE, initially enabled.
- Release A and throttle at startup; wait for the input interlock and firmware arming flags to clear before deliberately arming. A physical disconnect stops RC datagrams and exercises native Betaflight RX-loss/failsafe behavior. Quiet sticks remain valid.

For the TX16S, use the ExpressLRS TX module's [BLE Joystick mode](https://www.expresslrs.org/quick-start/transmitters/lua-howto/), pair it and inspect its axes/switches. The TX backpack accessory and the TX module's BLE joystick function are distinct. The ELRS mapping is provisional until the actual transmitter is checked:

```sh
python3 tools/tricopter/sitl/rc_input.py --inspect 'DEVICE NAME'
python3 tools/tricopter/sitl/run.py --variant modified \
  --device 'DEVICE NAME' --profile tools/tricopter/sitl/profiles/elrs.json
```

Both variants receive AETR channels at 50 Hz on the existing UDP receiver. Configurator/MSP is available at `127.0.0.1:5761` (TCP); runner telemetry uses a separate MSP port, 5762, so it does not occupy the configurator connection. The unchanged native SITL listeners bind all interfaces; these tools connect over loopback. They do not open a physical FC serial port.

## Model and bridge

[The standalone SDF world](gazebo/worlds/tricopter.sdf) uses bundled STEP-derived visual meshes plus primitive estimates for missing parts and collision shapes; it requires no Fuel downloads. Logical motors are **0 rear, 1 front-right, 2 front-left**. Front motors are fixed child links. The rear motor is mounted on an actual bounded revolute joint. A finite-torque servo controller follows the PWM/angle map with configured lag/speed limits, rather than teleporting the joint.

Each rotor's thrust and aerodynamic reaction torque are applied to its own link using the **current link orientation**. Thus the rear force and reaction axis move together, and physics transmits the joint loads to the frame. Rotor-relative momentum reactions use approximately 4.57 × 10⁻⁶ kg·m² per bell/shaft/prop and the load-dependent simulated RPM. Loaded speed, ESC behavior and real gyroscopic response are not physically identified. See [physics, coordinates and protocol details](gazebo/README.md).

The default DC motor model approximates the aircraft's active-braking ESCs: at a valid positive motor command, current can become negative when back-EMF exceeds the requested drive voltage. The separate `motor_brake_current_limit_a` is **35.22 A** by default; this is an assumed model bound, not a measured ESC setting. Setting it to zero restores coasting on downward command steps. A valid zero-duty command (1000 µs) coasts by default (`motor_brake_on_stop = false`); setting that field true also brakes at zero duty. Expired/invalid actuator input or a suppressed raw zero-PWM output always opens the drive and coasts. Signed bus current models ideal energy exchange with a fixed-voltage supply; battery acceptance, sag and ESC switching losses are not modelled.

The actual joint angle is mapped through the three ADC/angle points. The modified SITL receives raw **PB0** from that angle and **PC0 = 0**, then uses the existing canonical PB0−PC0 measurement path. PWM target is never substituted for actual basket position.

| Point | Servo PWM | Basket angle | PB0 counts | PC0 counts |
|---|---:|---:|---:|---:|
| low | 1100 µs | −47.8° | 1936 | 0 |
| center | 1509 µs | 0° | 1490 | 0 |
| high | 1700 µs | +21.6° | 1281 | 0 |

The synthetic hinge uses −X in the model's FLU coordinates so positive tilt produces Betaflight-positive yaw. This makes the configured simulator feedback polarity explicit; it does not validate the real aircraft's stabilizing yaw sign.

The bridge consumes stock SITL's **68-byte raw output** on UDP 9001: `<H2x16f`, with motor count 3, motor PWM in floats 0–2, and physical servo0 PWM in float 3. It does not mistake servo PWM for a fourth motor or use the fixed quad output on 9002. FDM data go to 9003; RC to 9004; virtual ADC to 9005. ADC packets are versioned/coherent, with sequence/time/epoch checks; invalid or old packets do not refresh the feedback age. Loss of actuator packets opens the DC drive, leaves mechanical/propeller-load coasting, and releases servo torque. The legacy `lag` motor backend retains its configured speed decay.

## Change the airframe or compare rotation

Edit a copy of the world and pass `--world /absolute/path/world.sdf` to **both** variants. Rotor link poses define the motor axes; the explicit rear joint pose defines its pivot, and inertials define each disjoint group's CG and full inertia tensor. The versioned [component budget and assumptions](gazebo/README.md#default-aircraft-approximation) define the default plant. Plugin fields define electrical/propeller coefficients, drive/braking current bounds, spin signs, rotor inertia, servo map/dynamics and ADC map. The older maximum-thrust, reaction-ratio and motor-time-constant fields remain available for the `lag` backend. Keep the modified firmware's three-point settings consistent with the plant's sensor map. The default total mass is 0.5099032 kg. At fixed nominal 22.2 V the motor/load model estimates **12.628 N and 36,075 RPM per rotor at full command**, using the confirmed delta winding and 1.2 A at 10 V no-load point. No dynamic battery sag model is implied. See the [thrust source and motor-response assumptions](gazebo/README.md#default-aircraft-approximation). `--config extra.cli` appends deliberate simulator firmware changes; its resulting configuration is saved with each run.

To reverse all prop/motor rotation reaction signs while preserving thrust and basket calibration:

```sh
python3 tools/tricopter/sitl/run.py --variant stock --reverse-spin --device 'Xbox'
python3 tools/tricopter/sitl/run.py --variant modified --reverse-spin --device 'Xbox'
```

The runner negates the three plant `spin_sign` values. It does not reverse the geometric servo yaw direction. The firmware's existing `tri_yaw_direction` and neutral trim remain independently configurable.
With the bundled −6.1° operating trim, the modified reverse-spin run also needs an
extra CLI overlay setting `tri_yaw_trim = 61`; `--reverse-spin` changes the plant
rotation signs and does not silently rewrite the firmware's physical trim.

## Checks and recordings

Flight videos use a viewable **2560×1440** Nvidia-rendered Gazebo window and NVENC
at 60 fps. The three propeller visuals are translucent spinning-envelope discs.
The stress recording world uses a single textured ground surface and 240 Hz
visual state updates. See [capture setup and verification](../../../docs/TRICOPTER_VIDEO_CAPTURE.md).

The default plant now uses configurable body-axis **quadratic drag**, with explicit
unmeasured CdA placeholders. The modified preset adds a **−6.1° physical operating
trim** while retaining the PWM/ADC map zero. Its allocator reserves common motor
headroom from measured tail tilt instead of clamping the yaw target to zero at
full collective. See [the correction and new shared-plant comparison](../../../docs/TRICOPTER_YAW_AUTHORITY_FIX.md).
Historical linear-drag campaigns below retain their original frozen inputs;
select `airframe_drag_model=linear_world` for that force-law branch.

Hardware-free checks:

```sh
python3 -m unittest discover -s tools/tricopter/sitl -p 'test_*.py' -v
make -C src/test CC=gcc CXX=g++ test_sitl_adc_unittest test_sitl_gyro_unittest
cmake -S tools/tricopter/sitl/gazebo \
  -B .local-artifacts/tricopter/gazebo-sitl/math-build -DTRICOPTER_BUILD_PLUGIN=OFF
cmake --build .local-artifacts/tricopter/gazebo-sitl/math-build
ctest --test-dir .local-artifacts/tricopter/gazebo-sitl/math-build --output-on-failure
```

The runner also builds/tests the actual Gazebo plugin before launching. New session directories under `.local-artifacts/tricopter/gazebo-sitl/runs/` preserve `run.json`, applied CLI settings/EEPROM, firmware/Gazebo logs, `flight.jsonl` and the plant CSV. The CSV includes actual basket angle, servo command, raw ADC, all motor efforts, rear axis, thrust/reaction torque and vehicle state for comparing tracking, saturation and yaw behavior. No physical flight validation is implied by a successful simulation.

The real-process transport/estimator/sensor check runs without Gazebo:

```sh
python3 tools/tricopter/sitl/check_protocol.py
```

It starts each actual binary with its own synthetic EEPROM, feeds FDM/RC, captures the raw actuator packets, checks roll/pitch signs, arms/disarms in software, and checks modified feedback loss/recovery. It does not simulate vehicle dynamics. It uses the same exclusive port lock as the runner.

Actual Gazebo checks:

```sh
python3 tools/tricopter/sitl/check_plant.py
python3 tools/tricopter/sitl/check_flight.py
```

The anchored plant check exercises full servo travel, actual-joint ADC, changing world force/reaction axes, spin reversal and actuator-loss shutdown. The flight check takes off, holds about 1 m, yaws both ways, lands and disarms with **both** firmware variants. It uses the identical external XYZ RC pilot reading Gazebo ground truth, with bounded ANGLE-mode inputs; it does not enable native altitude/position/autopilot modes or write motor/servo outputs directly. Every transmitted RC frame is retained. These matched synthetic trials show working integration, not physical-airframe accuracy or a performance advantage.

The shared bridge keeps one normal FDM state outstanding until raw PWM replies arrive, with bounded startup/stall retries. This prevents queued sensor bursts from driving the unchanged native SITL's instantaneous clock-rate estimator into false RX failsafe. Sensor ADC still follows each physics step. Pausing/resetting discards queued old actuator packets before a new state/reply boundary.

For a repeated yaw/throttle comparison with fixed measurement inputs:

```sh
python3 tools/tricopter/sitl/check_comparison_fresh.py --hover-throttle 1345
python3 tools/tricopter/sitl/plot_comparison.py /absolute/path/to/printed/run/directory
```

The comparison runs three counterbalanced stock/modified groups, with seven fresh-world/EEPROM flights per group and matched neutral/turning controls. The external pilot only supplies takeoff, setup and landing commands; all four flight channels are atomically overridden during each measurement. Fresh starts avoid the pre-event PID/maneuver-history confound observed in sequential trials. Actual transmitted commands and timestamps are retained, because real-time UDP does not give bit-identical timing. See the [protocol and measurement definitions](../../../docs/TRICOPTER_SITL_COMPARISON.md).

The updated DC/prop model uses 1345 µs as the shared measurement hover input; the helper's historical default is 1350 µs. The external setup pilot uses the declared DC steady-force inverse and actual tilted-tail force trim to reach the same setup gates. These calculations do not reach the receiver during fixed-input measurements and do not tune the firmware. The retained [updated-model characterization](../../../docs/TRICOPTER_UPDATED_MODEL_CHARACTERIZATION.md) predates active braking and records the coasting model; its frozen traces and response numbers remain historical results rather than measurements of the current braking-enabled plant. The subsequent [default active-braking verification](../../../docs/TRICOPTER_ACTIVE_BRAKING.md) records its model equations, settings and fresh stock/modified checks.

To repeat the independent yaw search and its zero-feedforward reference/evaluation, first complete a fresh original-gain baseline, then pass its printed directory:

```sh
python3 tools/tricopter/sitl/autotune_yaw.py --baseline /absolute/path/to/completed/baseline
```

The bounded search uses fresh CW/CCW flights, equal budgets, native gain readbacks and retained failures. It freezes the selected profiles before the separate yaw/collective evaluation; see [the full protocol](../../../docs/TRICOPTER_YAW_AUTOTUNE.md).

To compare those frozen hover gains in sustained climb and accelerating forward flight:

```sh
python3 tools/tricopter/sitl/check_envelope.py --dry-run
python3 tools/tricopter/sitl/check_envelope.py
python3 tools/tricopter/sitl/check_envelope.py --climb-throttle 1450 \
  --cases climb_control,climb_cw,climb_ccw
python3 tools/tricopter/sitl/check_envelope.py --forward-lead 6 \
  --cases forward_control,forward_cw,forward_ccw
python3 tools/tricopter/sitl/plot_envelope.py /absolute/path/to/printed/run/directory
```

This uses the completed hover campaign as gain provenance (`--reference` selects another compatible campaign), verifies the identical approved common SITL patch on both controllers, and preserves every other historical critical source/model/config input. The primary comparison schedules 54 fresh worlds; the two additional presets each schedule 18 rapid-climb or higher-speed forward cases. Every interrupted flight remains in its ledger; a runner exit of 1 means at least one program failed or an execution error occurred. The separate analysis reports complete outcome-ledger processing independently of flight success. See the [coordinate correction and 90 completed comparison flights](../../../docs/TRICOPTER_SITL_FRAME_CORRECTION.md). The [earlier uncorrected frame study](../../../docs/TRICOPTER_FLIGHT_ENVELOPE_COMPARISON.md) remains archived with its failed forward-turn traces.

This bridge uses real-time UDP, rather than a deterministic lockstep scheduler. For a fair restart or A/B comparison, stop the session and launch a fresh one; resetting only the Gazebo world does not reset Betaflight's PID/filter/controller history.

The [2026-11-08 validation record](../../../docs/validation/tricopter-sitl-2026-11-08/README.md) preserves build/source hashes, actual flight/plant traces, failed attempts and GUI verification.
