# Gazebo tricopter SITL validation — 2026-11-08

This is the **initial historical snapshot**, using the earlier 1.09 kg synthetic world. The archive and its verification describe the state at capture time, including the then-unavailable controllers and blocked driver processes. Subsequent reboot, Xbox mapping, dependency isolation, centered camera and 500 g model results are recorded in the [current update](camera-500g-update.md). The original archive remains unchanged.

Gazebo Jetty **10.6.0**, the articulated tricopter plugin and independent stock/modified Betaflight SITL builds are installed and working. Follow [the run guide](../../../tools/tricopter/sitl/README.md). Both variants passed actual software-only takeoff, approximately 1 m hover, yaw in both directions, landing and disarming. Physical Bluetooth mapping is untested because the controllers are unavailable; pairing is left to the operator.

The [summary](validation-summary.json), [source/build manifest](manifest.json), [independent flight audit](independent-flight-audit.json) and [evidence archive inventory](evidence-manifest.json) bind the results to exact source, binaries, plugin, world, configuration and raw records. `evidence.tar.gz` preserves those records and the failed attempts. [Archive verification](archive-verification.json) checks every member and its hash.

## Stock source and common setup

Stock is unchanged Betaflight `2026.6.2`, pinned at **`e0b7bb01b17b21351057e9ead2d1ab39dd44fa16`**. All **4,750 Git blobs** match before/after building and in an independent audit. Both variants use the same `EXTRA_FLAGS=-DUSE_SERVOS` compile option: bare SITL otherwise excludes servos. No stock output shim or flight-source patch is applied. The modified source snapshot is based on `c9d57f5ad7aeccfff670b65ba6bc15dcf6fda5bf`, with simulator-only ADC additions.

The same SDF and common PID/receiver settings are used for both. Only the modified configuration adds the new feedback parameters/diagnostics. Logical motor order is rear/right/left; the native 68-byte raw packet contains motor count 3, motor floats 0–2 and physical servo0 in float 3. The fixed quad-only packet is not used. UART1/TCP 5761 remains available for Configurator while telemetry uses UART2/5762.

## Focused checks

- **30 Python tests pass:** HID mapping/interlocks/disconnect behavior, startup-health guards, rotation configuration and orphan-child cleanup.
- **9 ADC and 3 existing gyro tests pass**, plus all 9 ADC tests under fast-math.
- Actual Gazebo plugin compilation and both CTest targets pass, including **209 math/protocol checks** and the SDF articulation/topology check.
- Real native UDP/MSP checks pass for both binaries: 8,037 stock and 8,497 modified actuator packets all have the expected 68-byte/3+1 format; simultaneous MSP ports work; ±10° roll/pitch inputs give ±9.9° Mahony estimates with zero cross-axis response. Modified tests cover all ADC knots, true PB0−PC0 subtraction, withdrawal, stale/replayed sequence rejection and epoch/time-reset recovery.
- Actual anchored Gazebo checks pass at 1100/1509/1700 µs, including real joint movement, angle-derived PB0, PC0=0, changing rear world axis, reaction torque, spin reversal and motor/servo actuator-loss behavior. Reversing spin changes reaction signs while preserving thrust and the physical measurement map.

## Actual airborne comparison

The common external XYZ test pilot reads Gazebo ground truth and writes **only normal RC channels**, with bounded ANGLE-mode commands. Native altitude/position/autopilot modes are explicitly verified inactive. All transmitted RC packets, actual plant data and firmware telemetry are retained. This is the same pilot policy for both variants, not a claim of identical RC waveforms after closed-loop corrections.

| Measured quantity | Stock | Modified |
|---|---:|---:|
| Maximum hover height error about 1 m target | 0.0448 m | 0.0489 m |
| Maximum horizontal hover error | 0.2370 m | 0.2362 m |
| Maximum hover roll/pitch magnitude | 1.229° | 1.234° |
| World-CCW yaw change for high/low RC pulse | +12.79° / −11.63° | +13.02° / −12.95° |
| Final state | landed, disarmed | landed, disarmed |

Both land back near the initial 0.125 m body height with motor PWM 1000. During sampled armed operation modified feedback is valid throughout; hover firmware angle versus actual joint is 0.064° RMS / 0.090° maximum in the ideal synthetic three-point sensor model. ADC quantization stays below 0.5 counts and actual rear-axis reconstruction error below `2.9e−6`. The independent audit verifies the sensor follows actual joint position rather than the command, including during actuator lag; large command/position differences at disarm are not hover tracking errors.

The geometry, inertia, thrust and loaded-servo coefficients are **synthetic placeholders**. These trials establish a functioning shared simulation and measurement/force paths, not physical-airframe accuracy or a controller performance advantage. The HID profiles invert yaw so a nominal rightward stick produces clockwise/right motion under the verified native simulator convention; actual hardware axis codes remain to be inspected.

## Timing and GUI findings

An initial unpaced stock run reached hover but falsely entered native RX failsafe despite healthy RC send intervals. The shared bridge now permits one normal FDM state outstanding until raw PWM replies, with minimum spacing and bounded startup/stall retries. This avoids sensor-queue bursts driving the unmodified native SITL clock-rate estimator. ADC remains tied to every physics step. Both final flight trials pass with the paced plugin. Pause/reset tests also prevent old queued actuator packets from reviving motion.

Initial Ogre2 GUI probes hung during hybrid-GPU EGL initialization. Their windows were removed, but host-level inspection still finds the two old processes blocked in driver calls with SIGKILL pending. An earlier namespaced process check incorrectly reported exit; the archived host-level cleanup record supersedes it. Clearing those old waits may require graphics-driver recovery or reboot. The **Intel/Ogre1** viewer renders normally, and the runner frames/follows the model. [The rendered camera PNG](rendered-scene.png) verifies an actual scene/render event. Subsequent complete viewer process groups close normally. No graphics-driver reset or power-mode change was performed.

## Replay and evidence

Raw sessions remain under `.local-artifacts/tricopter/gazebo-sitl/` and `.local-artifacts/tricopter/gazebo-sitl-2026-11-08/`. The archive excludes rebuildable object directories, caches and the exported stock-source tree; the immutable Git revision recreates that tree. It includes final binaries/plugin, build manifests/logs, exact RC packets, real FDM/ADC/PWM observations, plant/pilot CSVs, configurations, EEPROMs, GUI exports and earlier failures.

From the repository root, compare hashes with the inventory and restore without replacing existing files:

```sh
tar --keep-old-files -xzf docs/validation/tricopter-sitl-2026-11-08/evidence.tar.gz
python3 tools/tricopter/sitl/check_protocol.py
python3 tools/tricopter/sitl/check_plant.py
python3 tools/tricopter/sitl/check_flight.py
```

The checks create fresh software sessions. No physical FC serial port, motor or servo was accessed for this work.
