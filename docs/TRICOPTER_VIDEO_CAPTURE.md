# Flight video capture at 60 fps

The recorder now opens a normal **2560×1440** Gazebo window on the physical display.
Nvidia PRIME GLX renders both Ogre1 and Qt's OpenGL scene; the RTX 5060 NVENC
engine encodes H.264 at 60 fps. Software rendering/encoding are rejected. The
recorder keeps its own window viewable, verifies its PID/process group,
geometry and chase-camera target/position, and captures that drawable only.
Focus is recorded as diagnostic metadata and activation is best effort;
another application's active window is never selected as the capture source.

The previous Xvfb recordings were encoded at 30 fps but rendered only 8–10 scene
updates/s. They remain historical artifacts and are superseded for flight
review. No interpolation or duplicate-frame upconversion is used to claim a
higher simulation cadence.

## Ground and scene changes

The stress-test derivative world replaces 202 overlapping thin line boxes with
one textured ground quad. Grid and directional markings are baked into a
repeating texture, so their intersections are not coplanar geometry. The GUI
grid is disabled, and camera clipping is 0.1–2000 m. SceneBroadcaster state and
dynamic-pose publication rise from 60 to 240 Hz to decouple visual updates from
the 60 Hz capture clock.

Aircraft geometry, inertia, collisions, forces, servos, PB0−PC0 feedback and
firmware are unchanged. The existing 20 km ground collision remains identical.
The canonical physics SDF is unchanged; modifications are applied to each
generated recording world. The focused world test restores only declared
visual/transport differences and compares every remaining world field.

Ground assets are `tools/tricopter/sitl/gazebo/meshes/review_ground.dae` and
`review_ground.png`. `generate_review_ground.py` reproduces them. The installed
Gazebo mesh loader verifies 4 vertices, 2 triangles, repeating UVs and the image
binding. They add one visual surface and no collision geometry.

## Recording

Run from the repository root:

```sh
python3 tools/tricopter/sitl/check_yaw_stress.py --profiles factory_yaw --cases hold_95_control --video
```

For an already running, correctly configured Gazebo partition:

```sh
python3 tools/tricopter/sitl/record_flight_video.py /absolute/path/to/trial --partition PARTITION --fps 60
```

The default is **2560×1440 at 60 fps** and the normal physical display.
`--width 1920 --height 1080` deliberately selects Full HD; failure to obtain the
requested drawable dimensions is an error rather than a silent size fallback.
Camera, renderer and
encoder evidence are saved beside `flight.mp4` in `video-camera.json` and
`video-record.json`. The child environment selects Nvidia GLX and Qt OpenGL
without changing desktop/global environment settings. Display/probe settings
are scoped to the child Gazebo window; the window and encoder close at the end.

## Verification

The QHD continuous-motion probe passed with 300 distinct geometric states over
5 s at 60.0 updates/s. A separate unfocused-window test decoded 180 distinct
moving-bar states in 3 s, with no repeats and maximum state hold 16.7 ms.
During that test all 16 attention samples showed Gazebo unfocused while a
separate owned blank control window remained active. Nvidia/Qt OpenGL continues
rendering without keyboard focus; a strict global-focus check had unnecessarily
blocked four preflight capture attempts. Those failed attempts remain in the
allocator study ledger and do not count as flight tests. Actual flight clips
receive separate resolution, scene-motion and camera-visibility checks.

The following 720p recordings document the earlier capture correction; the
new allocator/quadratic-drag study uses the QHD default.

An initial continuous-physics motion probe produced 300 distinct geometric
states in 300 frames over 5 s. An independent PCA-based audit measured motion of
the actual rotating bar outside the HUD:60.0 states/s, no repeated states,
minimum 2.51° motion per frame. A negative control with 60 encoded fps but 10
scene updates/s was rejected. The final integrated probe repeated the test
with the new ground and eight row overlay, again producing 300 distinct scene frames.
Actual Ogre context logs identify the RTX 5060, independently of `glxinfo`.

Probe evidence is under
`.local-artifacts/tricopter/gazebo-sitl/video-review/nvidia60-final-ground-hud`
and `video-review/independent-cadence`.

The first replacement stock/modified 95% pair is retained under
`.local-artifacts/tricopter/gazebo-sitl/runs/20261109T044105-both-yaw-stress-95-nvidia60-587788`.
Both encode 1280×720, 60/1 fps, 962 frames with `h264_nvenc`. These are fresh
illustrative retakes of the same RC/PID protocol; the earlier N=3 comparison
cohort and its results remain unchanged. Actual flight-scene cadence and
visibility checks are retained as separate review artifacts.

The longer stock/modified forward diagnostic was also recorded with these
settings at `runs/20261109T044324-both-forward70-nvidia60-video-590929` under
the same artifact root. Both recording programs completed. These remain
explicitly labelled gyro-only AHRS diagnostics; capture improvements do not
establish real-flight validity or a qualified steady-flight comparison.

For the actual 95% flights, an independent ground-only ROI audit over seconds 9–11
decoded 120 frames per clip. All 119 adjacent pairs had substantial scene motion
(minimum mean luma differences 3.37/255 stock and 3.80/255 modified), with no
repeated scene states. The ROI excludes both HUD and aircraft; this is visual
scene motion rather than a container-rate or text-update check.

The same independent audit over seconds 35–37 of both forward clips also finds
120 distinct states in 120 frames and no repeats. Their minimum luma changes
are 1.73/255 stock and 1.97/255 modified, with thousands of strongly changing
ground pixels per pair. All four retakes therefore demonstrate 60 real scene
updates/s in audited motion intervals, rather than merely carrying a 60 fps tag.

All four complete clips pass the separate camera-visibility audit:302/302
samples across their full lengths retain the aircraft. They contain9069frames
over151.15s in total. The forward views follow the aircraft for approximately
6.01km stock and5.48km modified. Their actual Ogre context logs are bound to
the corresponding startup intervals and identify the Nvidia renderer. The
aggregate visibility/codec/cleanup audit is retained at
`video-review/nvidia60-final-delivery/quality-review.json` under the artifact root.

## QHD allocator-study videos

The current quadratic-drag/allocator study records 2560×1440 at 60 fps. These
are fresh flights on the new shared plant, with the translucent prop discs;
they are not re-encodes of the historical linear-drag clips. Per-trial
`live-gpu-renderer-proof.json` and `actual-ogre-renderer.log` bind the exact owned
GUI PID to Nvidia hardware rendering, separately from the NVENC codec check.

| Scenario | Stock | Corrected allocator |
|---|---|---|
|95% hold, factory yaw gains|[QHD MP4](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261109T063249-both-authority-fix-95-qhd-confirmation-659013/01-factory_yaw-hold_95_control-stock/flight.mp4)|[QHD MP4](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261109T063249-both-authority-fix-95-qhd-confirmation-659013/01-factory_yaw-hold_95_control-modified/flight.mp4)|
|95% hold, hover-tuned gains|[QHD MP4](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261109T063249-both-authority-fix-95-qhd-confirmation-659013/01-hover_tuned-hold_95_control-stock/flight.mp4)|[QHD MP4](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261109T063249-both-authority-fix-95-qhd-confirmation-659013/01-hover_tuned-hold_95_control-modified/flight.mp4)|
|Fast forward CCW, hover-tuned gains|[QHD MP4](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261109T064040-both-authority-fix-forward-qhd-668043/01-hover_tuned-forward_70_ccw-stock/flight.mp4)|[QHD MP4](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261109T064040-both-authority-fix-forward-qhd-668043/01-hover_tuned-forward_70_ccw-modified/flight.mp4)|

The forward videos retain the normal native AHRS fusion and the measured
curved-entry/recovery limitations described in the [allocator comparison](TRICOPTER_YAW_AUTHORITY_FIX.md#fast-forward-flight-with-yaw-pulses).
They are illustrative retakes, not additional statistically independent
repetitions folded into the primary forward table.

All six QHD clips passed final quality review: **10,299/10,299 decoded frames**
retain the centered aircraft. Each clip's two-second moving-scene interval has
**120/120 distinct ground states at 60 Hz**, with no repeats; all six synthetic
10 Hz repeat controls are rejected. Ground-only regions exclude captions and
the aircraft. Manual native-frame review confirms three visible prop discs,
the tail disc following basket tilt, and no observed ground z-fighting in the
reviewed frames. Exact owned-window/camera checks, live Nvidia graphics
contexts, encoder selection and process cleanup pass for all six clips.

See the [hash-bound quality report](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/video-review/qhd-discs-review/final-six-qhd-capture-quality.json)
and [six-frame review sheet](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/video-review/qhd-discs-review/final-six-qhd-disc-review.png).
The motion checks establish cadence in the audited intervals; representative
image review is not a claim that every pixel in every frame was manually
inspected. Three audit-reader seek-boundary errors were retained and corrected
by aligning the reader to encoded frame boundaries; original clips were not
modified or re-encoded.

## Historical 720p replacement videos

| Scenario | Stock | Modified |
|---|---|---|
|95% hold|[MP4](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261109T044105-both-yaw-stress-95-nvidia60-587788/01-factory_yaw-hold_95_control-stock/flight.mp4)|[MP4](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261109T044105-both-yaw-stress-95-nvidia60-587788/01-factory_yaw-hold_95_control-modified/flight.mp4)|
|Forward diagnostic|[MP4](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261109T044324-both-forward70-nvidia60-video-590929/01-hover_tuned-forward_70_control-stock/flight.mp4)|[MP4](/home/alex/betaflight/.local-artifacts/tricopter/gazebo-sitl/runs/20261109T044324-both-forward70-nvidia60-video-590929/01-hover_tuned-forward_70_control-modified/flight.mp4)|
