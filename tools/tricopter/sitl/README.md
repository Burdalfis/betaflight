# Tricopter Gazebo SITL: stock and feedback Betaflight

Run the same articulated synthetic tricopter with either genuine unmodified Betaflight **2026.6.2**, pinned at `e0b7bb01b17b21351057e9ead2d1ab39dd44fa16`, or this checkout's measured-position feedback controller. The stock source is exported from that commit and checked against every tracked Git blob before and after building. No quad-to-tricopter patch or feedback shim is applied to stock. The modified simulator adds a virtual ADC backend; STM32 ADC/servo/motor hardware paths are unchanged.

The default airframe approximates a **500 g tricopter with 1800 KV 2207 motors, GF51466 props and nominal 6S voltage**. Static thrust is estimated from nearby manufacturer test data. Geometry, inertia, drag, reaction ratio and loaded servo behavior still contain explicit estimates; this is not an identified aircraft model. Both variants use the same world and common receiver/PID settings. Every run records its exact firmware, plugin, world and configuration hashes.

## Install and build

This host uses Ubuntu 26.04. Gazebo **Jetty / gz-sim10** supports it; the installer follows the [official binary installation](https://gazebosim.org/docs/jetty/install_ubuntu/). It adds the signed OSRF repository and installs Gazebo, development headers and input/build dependencies. It backs up existing files at the two Gazebo repository/key paths before replacing them. It does not install ROS.

From the repository root:

```sh
sudo bash tools/tricopter/sitl/install_gazebo_jetty.sh
python3 tools/tricopter/sitl/build.py
```

The stock source, independent stock/modified object directories, binaries and build manifests live in `.local-artifacts/tricopter/gazebo-sitl/`. `build.py --variant stock` and `--variant modified` build separately. Both builds use the identical `EXTRA_FLAGS=-DUSE_SERVOS` compile option: bare SITL otherwise omits servo support. This is a build option on unmodified stock source. Existing FC firmware build artifacts and physical FC configuration are not used. The runner refuses changed source/binary hashes until that variant is rebuilt.

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

[The standalone SDF world](gazebo/worlds/tricopter.sdf) contains only local primitive geometry; it requires no Fuel downloads. Logical motors are **0 rear, 1 front-right, 2 front-left**. Front motors are fixed child links. The rear motor is mounted on an actual bounded revolute joint. A finite-torque servo controller follows the PWM/angle map with configured lag/speed limits, rather than teleporting the joint.

Each rotor's thrust and aerodynamic reaction torque are applied to its own link using the **current link orientation**. Thus the rear force and reaction axis move together, and physics transmits the joint loads to the frame. Optional rotor-inertia reaction defaults to zero; these initial coefficients do not identify gyroscopic servo loading. See [physics, coordinates and protocol details](gazebo/README.md).

The actual joint angle is mapped through the three ADC/angle points. The modified SITL receives raw **PB0** from that angle and **PC0 = 0**, then uses the existing canonical PB0−PC0 measurement path. PWM target is never substituted for actual basket position.

| Point | Servo PWM | Basket angle | PB0 counts | PC0 counts |
|---|---:|---:|---:|---:|
| low | 1100 µs | −47.8° | 1936 | 0 |
| center | 1509 µs | 0° | 1490 | 0 |
| high | 1700 µs | +21.6° | 1281 | 0 |

The synthetic hinge uses −X in the model's FLU coordinates so positive tilt produces Betaflight-positive yaw. This makes the configured simulator feedback polarity explicit; it does not validate the real aircraft's stabilizing yaw sign.

The bridge consumes stock SITL's **68-byte raw output** on UDP 9001: `<H2x16f`, with motor count 3, motor PWM in floats 0–2, and physical servo0 PWM in float 3. It does not mistake servo PWM for a fourth motor or use the fixed quad output on 9002. FDM data go to 9003; RC to 9004; virtual ADC to 9005. ADC packets are versioned/coherent, with sequence/time/epoch checks; invalid or old packets do not refresh the feedback age. Loss of actuator packets causes motor spin-down through configured lag and releases servo torque.

## Change the airframe or compare rotation

Edit a copy of the world and pass `--world /absolute/path/world.sdf` to **both** variants. Rotor link poses define lever arms; inertials define mass/inertia. Plugin fields define maximum thrust, reaction ratio, spin signs, motor time constant, servo map/dynamics and ADC map. Keep the modified firmware's three-point settings consistent with the plant's sensor map. The default total mass is 0.500 kg and each rotor's full-command thrust is **13.31255 N**, a fixed nominal-22.2 V estimate; no dynamic battery/voltage model is implied. See the [thrust source and motor-response assumptions](gazebo/README.md#default-aircraft-approximation). `--config extra.cli` appends deliberate simulator firmware changes; its resulting configuration is saved with each run.

To reverse all prop/motor rotation reaction signs while preserving thrust and basket calibration:

```sh
python3 tools/tricopter/sitl/run.py --variant stock --reverse-spin --device 'Xbox'
python3 tools/tricopter/sitl/run.py --variant modified --reverse-spin --device 'Xbox'
```

The runner negates the three plant `spin_sign` values. It does not reverse the geometric servo yaw direction. The firmware's existing `tri_yaw_direction` and neutral trim remain independently configurable.

## Checks and recordings

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

This bridge uses real-time UDP, rather than a deterministic lockstep scheduler. For a fair restart or A/B comparison, stop the session and launch a fresh one; resetting only the Gazebo world does not reset Betaflight's PID/filter/controller history.

The [2026-11-08 validation record](../../../docs/validation/tricopter-sitl-2026-11-08/README.md) preserves build/source hashes, actual flight/plant traces, failed attempts and GUI verification.
