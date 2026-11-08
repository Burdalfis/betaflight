# Camera, Bluetooth and 500 g model update

This supplements the initial historical SITL snapshot. Its original archive remains unchanged. The [current summary](camera-500g-summary.json), [new archive inventory](camera-500g-evidence-manifest.json), [verification](camera-500g-archive-verification.json) and `camera-500g-evidence.tar.gz` preserve the updated sources, plugin, exact worlds/configuration, successful stock/modified sessions, camera measurements, test logs and failed attempts.

The default model totals **0.500 kg** including the three motor links. Each rotor supplies an estimated **13.31255 N** at full command for nominal 22.2 V, scaled from nearby manufacturer data for the requested 1800 KV 2207/GF51466 setup. Geometry, inertia, reaction ratio and loaded dynamics remain estimates. The 30 ms speed time constant is retained as a comparable-platform prior; the unloaded 7 V historical RPM results do not identify loaded 6S response. [Sources and assumptions](../../../tools/tricopter/sitl/gazebo/README.md#default-aircraft-approximation) distinguish these estimates from measurements.

The original aircraft PID gains and rate curves remain unchanged. Both variants receive a 30° ANGLE limit. They passed actual software takeoff, approximately 1 m hover, both yaw directions, landing and disarming with the same external ground-truth RC pilot. This validates integration, not physical-aircraft accuracy or a performance advantage.

| Current measured quantity | Stock | Modified |
|---|---:|---:|
| Maximum hover altitude error | 0.02550 m | 0.02582 m |
| Maximum horizontal hover error | 0.23584 m | 0.23517 m |
| Maximum hover roll/pitch magnitude | 1.218° | 1.240° |
| Final state | landed, disarmed | landed, disarmed |

The centered camera follows a separate collisionless, gravity-free kinematic target at offset (−2.4, 0, 1.0) m. The target copies aircraft position and yaw without inheriting bank/pitch. Actual stock GUI flight supplied **1,095 camera samples**, with approximately **2.49 mm RMS offset error** against asynchronously received target poses and effectively zero camera roll. Camera height changed by 1.135 m, and heading spanned roughly 24°, confirming movement rather than merely a successful setup request. [The rendered view](camera-centered.png) shows the centered third-person perspective.

Earlier static-target attempts did not follow aircraft movement in the GUI; both failed recordings remain archived. A separate failed local test stalled its RC thread during a synchronous Python transport screenshot request; the final test uses a subprocess for the screenshot and records healthy RC timing. These test failures are not counted as successful camera validation.

The paired Xbox reports Bluetooth codes different from the prepared USB layout. Read-only input recording verified the trigger and A/B/Y codes; the corrected Bluetooth profile passes actual bridge startup, and the previous profile is preserved as `xbox-usb.json`. Live startup reached zero arming-disable flags while remaining disarmed. Simulator children use native dependencies and Python, avoiding the observed Anaconda Protobuf mismatch. The earlier driver-stalled processes were cleared by the operator's reboot; final host inspection found no remaining test simulator processes.

Current focused checks pass **34 Python tests, 9 ADC tests, 3 gyro tests, 209 Gazebo math/protocol checks and both CTest targets**. The archive contains their logs and the current plugin build manifest. No physical flight controller or actuator was accessed. Manufacturer PDFs are linked and hashed, rather than redistributed in the new archive.
