# Bluetooth transmitter/gamepad input

`rc_input.py` reads a Linux input device through `python3-evdev` and sends Betaflight's existing SITL RC packets: **40 bytes, little-endian `<d16H`**, containing a timestamp and 16 PWM channels, at 50 Hz to `127.0.0.1:9004`. It serves either firmware variant with the same channel mapping. Set `map AETR1234` in both SITL configurations; channels 0–3 are roll, pitch, throttle and yaw. AUX1 (index 4) is ARM and AUX2 (index 5) is ANGLE. This program does not open a flight controller or pair Bluetooth devices.

Connect the Xbox controller through the operating system's Bluetooth settings. Discover its Linux event device, then inspect it before sending RC:

```sh
python3 tools/tricopter/sitl/rc_input.py --list
python3 tools/tricopter/sitl/rc_input.py --inspect 'Xbox'
```

Both commands send **no UDP**. `--inspect` prints axis ranges/current values and subsequent input changes; stop it with Ctrl-C. Select a unique controller name or an explicit `/dev/input/eventN` path. If there are multiple matching devices, the bridge requires a more specific choice. It does not grab the controller exclusively. The Linux account needs read permission for that input device.

Start RC delivery after SITL and the Gazebo bridge are running:

```sh
python3 tools/tricopter/sitl/rc_input.py \
  --device 'Xbox' \
  --profile tools/tricopter/sitl/profiles/xbox.json
```

`--host`, `--port`, `--rate` and optional `--duration SECONDS` support another local endpoint or a bounded run. Discovery/inspection do not load a profile or start RC delivery. No device is automatically selected for flying.

## Xbox profile

| Control | evdev default | SITL channel/action |
|---|---|---|
| Right stick left/right | `ABS_Z` | Roll |
| Right stick up/down | `ABS_RZ`, inverted | Pitch |
| Right trigger | `ABS_GAS` | Throttle; released = 1000, fully pressed = 2000 |
| Left stick left/right | `ABS_X`, inverted | Yaw |
| A | `BTN_SOUTH` | Toggle ARM |
| B | `BTN_EAST` | Disarm and clear the arm toggle; also requests low throttle while held |
| Y | `BTN_WEST` | Toggle ANGLE; initially enabled |
| Left bumper | `BTN_TL` | Spare AUX3, high while held |

The default `profiles/xbox.json` matches the paired Xbox Bluetooth controller observed on this PC: stick ranges are 0–65535, the right trigger is 0–1023, and the physical Y button reports code 308 (`BTN_WEST`). Xbox Linux drivers and connection modes differ. The previous mapping is retained in `profiles/xbox-usb.json` as an explicit alternate for controllers reporting right-stick `ABS_RX`/`ABS_RY`, right-trigger `ABS_RZ`, and Y as `BTN_NORTH`. Select it with `--profile tools/tricopter/sitl/profiles/xbox-usb.json` if inspection matches those codes.

The bridge rejects missing axes/buttons rather than guessing. Axis min/max values come from the kernel by default, so signed/unsigned ranges need no hardcoded conversion. Move each control separately and verify direction in the simulated receiver display before flight.

Yaw is deliberately inverted in all supplied HID profiles. Native Betaflight's `updateRcCommands` negates RC yaw when the shared `yaw_control_reversed=OFF` setting is used. In this synthetic airframe, positive gyro yaw is clockwise/right. A positive HID yaw axis (stick right) therefore sends RC below 1500, producing a clockwise/right yaw request. This matches the measured Gazebo/SITL direction checks; firmware yaw processing, the basket hinge and motor spin conventions are unchanged. The provisional ELRS profile assumes its yaw axis likewise increases when moved right; verify that assumption when the transmitter is available.

On startup, release A and the right trigger. The console reports `Arm interlock ready` after observing arm LOW and throttle ≤1050. Press A once to arm, then use the trigger for throttle. A second press disarms. Pressing A while throttle is high inhibits arming until the arm request is LOW and throttle is low again; use A to clear its toggle, release the trigger, then press A deliberately. B also clears the toggle. Button holds/repeats do not repeatedly toggle channels, and connecting with A held does not count as a press.

## Mapping and disconnect behavior

Each profile entry has a `channel` index, `kind`, and input `code`. Supported kinds are `centered`, `throttle`, `button`, `toggle`, `switch` and `constant`. Centered axes support `min`, `center`, `max`, `invert` and normalized `deadzone`; throttle/switch axes support endpoints/inversion, and switches define a `levels` array such as `[1000, 1500, 2000]`. Button/toggle entries can set `low`/`high` PWM, and toggles can set `initial`. Unmapped AUX channels remain 1000. All transmitted PWM is bounded to 1000–2000.

Quiet sticks remain valid: HID normally reports changes, so a lack of movement is not treated as a timeout. Device removal, read failure or failed periodic device IOCTL stops RC datagrams. Betaflight's native RX-loss path then sees the lost stream after approximately 150 ms of simulation time and applies its configured failsafe. The bridge waits for reconnection and resets the arming interlock/toggles. A scheduling stall longer than 250 ms or dropped input-event synchronization also resets the arm interlock. Reconnection cannot restore an old armed toggle automatically. Ctrl-C closes the stream and likewise leaves native RX-loss handling active.

`profiles/elrs.json` is a **provisional** TX16S/ExpressLRS BLE joystick mapping. The transmitter is not available for verification. Once it is available, enable its Bluetooth joystick mode, connect it as a Linux HID device, inspect each stick/switch, then adjust axis codes/inversion/endpoints in that profile. The RC bridge and both firmware variants do not otherwise change.

Hardware-free checks:

```sh
python3 -m unittest discover -s tools/tricopter/sitl -p 'test_rc_input.py' -v
```
