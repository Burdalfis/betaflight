# Gazebo tricopter SITL

The [setup and run guide](../tools/tricopter/sitl/README.md) provides independent stock/modified Betaflight builds, a Gazebo Jetty articulated tricopter plant, synthetic differential-ADC input and a Bluetooth HID RC bridge. The Xbox profile is available now; the TX16S/ExpressLRS profile remains provisional until the transmitter is available.

The stock source is the unchanged `2026.6.2` baseline `e0b7bb01b17b21351057e9ead2d1ab39dd44fa16`. Both builds use the same servo-support compile option and common receiver/PID configuration. Existing raw SITL output already carries three motors plus physical servo0, so no stock transport or flight-control patch is required. The modified platform adds a virtual PB0/PC0 backend feeding the existing canonical position path.

The rear rotor is a child on a physical servo joint. Both its thrust and reaction torque follow its actual orientation. Actual basket angle produces PB0 from the three calibrated counts; PC0 stays zero. The current default approximates the operator's 500 g aircraft and 1800 KV/6S/GF51466 setup with 13.31255 N per rotor at a fixed nominal 22.2 V. This is scaled from nearby manufacturer bench data, not an identified motor/prop curve. Geometry, inertia, drag and loaded servo behavior remain estimates; the [model guide](../tools/tricopter/sitl/gazebo/README.md#default-aircraft-approximation) records sources and limitations.

Gazebo Jetty **10.6.0** is installed and the actual plugin builds. Both SITL binaries build, 4,750 stock Git blobs remain unchanged, two simultaneous MSP ports work, and real UDP checks pass for three motor outputs, servo PWM, roll/pitch signs, software arming/disarming and differential-sensor fault/recovery behavior. The input/launcher Python checks, native ADC/gyro checks and 209 Gazebo math/protocol checks plus SDF topology checks pass.

Both firmware variants have passed actual Gazebo takeoff, approximately 1 m hover, bidirectional yaw, landing and disarming with the same world and external XYZ RC pilot. Maximum hover altitude error is **0.0448 m stock / 0.0489 m modified**, with maximum horizontal error **0.2370 / 0.2362 m**. The pilot uses ground truth and sends only RC channels; native altitude/position/autopilot modes remain disabled. These synthetic matched runs establish working integration, not real-airframe calibration or superiority.

Anchored actual-physics checks cover full basket travel, changing rear thrust/reaction axes, ADC from actual joint position, spin reversal and actuator-loss behavior. The modified firmware's sampled armed feedback remains valid throughout its flight, with hover feedback angle versus actual joint RMS **0.064°** in this ideal synthetic sensor model. The shared FDM pacer resolves an observed false RX-failsafe issue without changing stock source. Failed attempts remain preserved with the passing records.

The Intel/Ogre1 GUI uses a centered third-person camera following position and yaw, with a level horizon and no spring interpolation. A separate invisible world target removes roll/pitch from the camera offset without affecting aircraft physics. The shared configuration preserves the aircraft's PID gains and rate curves, with a 30° ANGLE cap for the gamepad. The Bluetooth Xbox is now paired, its reported mapping checked, and actual input-bridge startup verified; TX16S mapping remains provisional. No physical FC access is part of these tools.

The numerical flight results above and the original validation package describe the earlier 1.09 kg synthetic world. They are historical integration evidence, not results for the current 500 g approximation.

See the [validation package](validation/tricopter-sitl-2026-11-08/README.md) for hashes, raw records and the independent audit.
