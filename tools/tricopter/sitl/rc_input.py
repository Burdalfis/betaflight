#!/usr/bin/env python3
"""Linux HID joystick to Betaflight's existing 16-channel SITL UDP receiver.

Device discovery/inspection never create a network socket. Mapping and arming
logic below are independent of evdev so they can be checked without a joystick.
"""
from __future__ import annotations

import argparse
from contextlib import closing
from dataclasses import dataclass, field
import errno
import json
import math
from pathlib import Path
import select
import socket
import struct
import sys
import time

PACKET = struct.Struct("<d16H")
DEFAULT_CHANNELS = [1500, 1500, 1000, 1500] + [1000] * 12  # AETR1234
KINDS = {"centered", "throttle", "switch", "button", "toggle", "constant"}


@dataclass
class InputState:
    axes: dict[str, float] = field(default_factory=dict)
    ranges: dict[str, tuple[float, float]] = field(default_factory=dict)
    buttons: dict[str, bool] = field(default_factory=dict)


def number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be finite numeric data")
    return value


def pwm(value):
    value = number(value, "PWM")
    if not 1000 <= value <= 2000:
        raise ValueError("PWM must be within 1000..2000")
    return int(round(value))


def validate_profile(profile):
    if not isinstance(profile, dict) or not isinstance(profile.get("channels"), list):
        raise ValueError("Profile needs an object containing a channels array")
    if profile.get("channel_order") != "AETR1234":
        raise ValueError("SITL RC profile must explicitly use AETR1234")
    used = set()
    for channel in profile.get("channels", []):
        if not isinstance(channel, dict):
            raise ValueError("Each channel mapping must be an object")
        index, kind = channel.get("channel"), channel.get("kind")
        if type(index) is not int or not 0 <= index < 16 or index in used:
            raise ValueError("Channel indexes must be unique integers in 0..15")
        used.add(index)
        if kind not in KINDS:
            raise ValueError(f"Unknown mapping kind: {kind}")
        if kind == "constant":
            pwm(channel["value"])
        elif not isinstance(channel.get("code"), str) or not channel["code"]:
            raise ValueError("Each input mapping needs an evdev code name")
        deadzone = number(channel.get("deadzone", 0), "deadzone")
        if not 0 <= deadzone < 1:
            raise ValueError("deadzone must be within 0..1 (exclusive upper bound)")
        for key in ("min", "center", "max"):
            if key in channel:
                number(channel[key], key)
        for key in ("low", "high"):
            if key in channel:
                pwm(channel[key])
        if kind == "switch":
            levels = channel.get("levels", [1000, 1500, 2000])
            if len(levels) < 2:
                raise ValueError("switch requires at least two levels")
            for level in levels:
                pwm(level)
    if not {0, 1, 2, 3}.issubset(used):
        raise ValueError("All four flight channels must be explicitly mapped")
    arm = profile.get("arm_channel", 4)
    if type(arm) is not int or arm not in used or arm < 4:
        raise ValueError("arm_channel must name a configured AUX channel")
    if not 1000 <= number(profile.get("low_throttle_max", 1050), "low_throttle_max") <= 1100:
        raise ValueError("low_throttle_max must be within 1000..1100")
    return profile


def axis_fraction(mapping, state):
    code = mapping["code"]
    if code not in state.axes or code not in state.ranges:
        raise ValueError(f"Missing axis/range: {code}; inspect the controller mapping")
    minimum = number(mapping.get("min", state.ranges[code][0]), "axis minimum")
    maximum = number(mapping.get("max", state.ranges[code][1]), "axis maximum")
    value = number(state.axes[code], "axis value")
    if minimum >= maximum:
        raise ValueError(f"Invalid axis range for {code}")
    if mapping["kind"] == "centered":
        center = number(mapping.get("center", (minimum + maximum) / 2), "axis center")
        if not minimum < center < maximum:
            raise ValueError(f"Axis center must be between endpoints for {code}")
        fraction = (value - center) / (maximum - center if value >= center else center - minimum)
        fraction = max(-1., min(1., fraction))
        deadzone = mapping.get("deadzone", 0)
        fraction = 0. if abs(fraction) <= deadzone else math.copysign((abs(fraction) - deadzone) / (1 - deadzone), fraction)
        return -fraction if mapping.get("invert", False) else fraction
    fraction = max(0., min(1., (value - minimum) / (maximum - minimum)))
    if mapping.get("invert", False):
        fraction = 1 - fraction
    deadzone = mapping.get("deadzone", 0)
    return 0. if fraction <= deadzone else (fraction - deadzone) / (1 - deadzone)


class Mapper:
    def __init__(self, profile):
        self.profile = validate_profile(profile)
        self.previous_buttons = {}
        self.toggles = {}

    def map(self, state):
        channels = DEFAULT_CHANNELS.copy()
        arm_low = False
        kill_code = self.profile.get("disarm_button")
        kill = bool(state.buttons.get(kill_code, False)) if kill_code else False
        for item in self.profile["channels"]:
            index, kind = item["channel"], item["kind"]
            if kind == "constant":
                value, released = item["value"], True
            elif kind in {"button", "toggle"}:
                code = item["code"]
                if code not in state.buttons:
                    raise ValueError(f"Missing button: {code}; inspect the controller mapping")
                held = bool(state.buttons[code])
                if kind == "toggle":
                    # Seed with the observed state: connecting while held is
                    # not a press and cannot toggle the arming channel on.
                    before = self.previous_buttons.get(index, held)
                    active = self.toggles.get(index, bool(item.get("initial", False)))
                    if held and not before:
                        active = not active
                    if kill and index == self.profile.get("arm_channel", 4):
                        active = False
                    self.previous_buttons[index], self.toggles[index] = held, active
                else:
                    active = held != bool(item.get("invert", False))
                value = item.get("high", 2000) if active else item.get("low", 1000)
                released = not held if kind == "toggle" else not active
            else:
                fraction = axis_fraction(item, state)
                if kind == "centered":
                    value = 1500 + 500 * fraction
                elif kind == "throttle":
                    value = 1000 + 1000 * fraction
                else:
                    levels = item.get("levels", [1000, 1500, 2000])
                    value = levels[min(len(levels) - 1, int(fraction * (len(levels) - 1) + .5))]
                released = value <= 1250
            channels[index] = pwm(value)
            if index == self.profile.get("arm_channel", 4):
                arm_low = released and channels[index] <= 1250
        return channels, arm_low, kill


class ArmGate:
    """Low-throttle/released-arm interlock; disconnect never sends stale RC."""
    def __init__(self, arm_channel=4, low_throttle_max=1050):
        self.arm_channel, self.low_throttle_max = arm_channel, low_throttle_max
        self.reset()

    def reset(self):
        self.ready, self.previous_request = False, False

    def apply(self, channels, arm_control_low, *, connected=True, kill=False):
        if not connected:
            self.reset()
            return None  # Stop datagrams so native Betaflight RXLOSS runs.
        out = list(channels)
        requested = out[self.arm_channel] >= 1750
        low_throttle = out[2] <= self.low_throttle_max
        if kill:
            self.reset()
            out[self.arm_channel], out[2] = 1000, 1000
            return out
        if not self.ready and arm_control_low and not requested and low_throttle:
            self.ready = True
        if self.ready and requested and not self.previous_request and not low_throttle:
            self.ready = False
        if not self.ready:
            out[self.arm_channel] = 1000
        self.previous_request = requested
        return out


def encode_packet(timestamp, channels):
    if number(timestamp, "timestamp") < 0 or len(channels) != 16:
        raise ValueError("Packet needs a nonnegative timestamp and 16 channels")
    if any(type(value) is not int or not 1000 <= value <= 2000 for value in channels):
        raise ValueError("Packet channels must be integer PWM values within 1000..2000")
    return PACKET.pack(timestamp, *channels)


def import_evdev():
    try:
        import evdev
    except ImportError as exc:
        raise RuntimeError("Install python3-evdev to read Linux joystick devices") from exc
    return evdev


def list_devices(evdev):
    devices = []
    for path in evdev.list_devices():
        try:
            device = evdev.InputDevice(path)
            caps = device.capabilities(absinfo=False)
            # A gamepad needs flight axes. Do not suggest keyboards/touchpads.
            if len(caps.get(evdev.ecodes.EV_ABS, [])) >= 4:
                devices.append((path, device.name))
            device.close()
        except OSError:
            continue
    return sorted(devices)


def open_device(evdev, selector):
    if selector.startswith("/"):
        return evdev.InputDevice(selector)
    matches = [(path, name) for path, name in list_devices(evdev) if selector.casefold() in name.casefold()]
    if len(matches) > 1:
        raise ValueError(f"Device name is ambiguous: {matches}; select a /dev/input/event path")
    if not matches:
        raise FileNotFoundError(errno.ENODEV, f"No accessible controller matching {selector!r}")
    return evdev.InputDevice(matches[0][0])


def read_state(device, evdev, profile):
    state = InputState()
    capabilities = device.capabilities(absinfo=False)
    active = set(device.active_keys())
    for item in profile["channels"]:
        kind = item["kind"]
        if kind == "constant":
            continue
        code = item["code"]
        numeric = evdev.ecodes.ecodes.get(code)
        event_type = evdev.ecodes.EV_KEY if kind in {"button", "toggle"} else evdev.ecodes.EV_ABS
        if numeric is None or numeric not in capabilities.get(event_type, []):
            raise ValueError(f"Controller lacks {code}; run --inspect and adjust the profile")
        if event_type == evdev.ecodes.EV_ABS:
            info = device.absinfo(numeric)
            state.axes[code], state.ranges[code] = info.value, (info.min, info.max)
        else:
            state.buttons[code] = numeric in active
    code = profile.get("disarm_button")
    if code:
        numeric = evdev.ecodes.ecodes.get(code)
        if numeric is None or numeric not in capabilities.get(evdev.ecodes.EV_KEY, []):
            raise ValueError(f"Controller lacks disarm button {code}")
        state.buttons[code] = numeric in active
    return state


def inspect_device(evdev, selector):
    with closing(open_device(evdev, selector)) as device:
        print(f"Inspecting {device.path}: {device.name}; no RC packets sent", flush=True)
        print(json.dumps(device.capabilities(verbose=True, absinfo=True), default=str, indent=2), flush=True)
        for event in device.read_loop():
            if event.type in (evdev.ecodes.EV_ABS, evdev.ecodes.EV_KEY):
                code = evdev.ecodes.bytype[event.type].get(event.code, str(event.code))
                print(f"{code}: {event.value}", flush=True)


def stream(evdev, selector, profile, host, port, rate, duration=None):
    period, started = 1 / rate, time.monotonic()
    gate = ArmGate(profile.get("arm_channel", 4), profile.get("low_throttle_max", 1050))
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as output:
        while duration is None or time.monotonic() - started < duration:
            try:
                with closing(open_device(evdev, selector)) as device:
                    mapper = Mapper(profile)
                    state = read_state(device, evdev, profile)
                    mapper.map(state)  # Seed toggle edges from current states.
                    gate.reset()
                    print(f"Connected {device.name}; release arm and throttle before arming", flush=True)
                    next_send = last_loop = time.monotonic()
                    next_snapshot = next_send + .1
                    last_ready = False
                    while duration is None or time.monotonic() - started < duration:
                        now = time.monotonic()
                        if now - last_loop > .25:
                            # Process stalls also require a new deliberate arm.
                            mapper, gate = Mapper(profile), ArmGate(gate.arm_channel, gate.low_throttle_max)
                            state = read_state(device, evdev, profile)
                            mapper.map(state)
                        last_loop = now
                        readable, _, _ = select.select([device.fd], [], [], max(0, min(period, next_send - now)))
                        if readable:
                            try:
                                events = list(device.read())
                            except BlockingIOError:
                                events = None
                            if events == []:
                                raise OSError(errno.ENODEV, "Controller disconnected")
                            for event in events or []:
                                if event.type == evdev.ecodes.EV_SYN and event.code == evdev.ecodes.SYN_DROPPED:
                                    state, mapper = read_state(device, evdev, profile), Mapper(profile)
                                    gate.reset()
                                    mapper.map(state)
                                elif event.type in (evdev.ecodes.EV_KEY, evdev.ecodes.EV_ABS):
                                    for code in state.axes if event.type == evdev.ecodes.EV_ABS else state.buttons:
                                        if evdev.ecodes.ecodes[code] == event.code:
                                            if event.type == evdev.ecodes.EV_ABS:
                                                state.axes[code] = event.value
                                            else:
                                                state.buttons[code] = bool(event.value)
                                    mapper.map(state)  # Preserve short button edges between send ticks.
                        now = time.monotonic()
                        if now >= next_snapshot:
                            # IOCTLs detect removal even with completely quiet
                            # sticks. Lack of change events itself is not loss.
                            state = read_state(device, evdev, profile)
                            next_snapshot = now + .1
                        if now >= next_send:
                            values, arm_low, kill = mapper.map(state)
                            channels = gate.apply(values, arm_low, kill=kill)
                            output.sendto(encode_packet(now - started, channels), (host, port))
                            if gate.ready != last_ready:
                                print("Arm interlock ready" if gate.ready else "Arm inhibited: release arm and lower throttle", flush=True)
                                last_ready = gate.ready
                            next_send = now + period
            except (OSError, EOFError) as exc:
                gate.apply(DEFAULT_CHANNELS, False, connected=False)
                print(f"Controller unavailable: {exc}; RC datagrams stopped, waiting to reconnect", file=sys.stderr, flush=True)
                time.sleep(.5)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--list", action="store_true", help="List accessible gamepads; no UDP")
    mode.add_argument("--inspect", metavar="DEVICE", help="Print capabilities and changes; no UDP")
    mode.add_argument("--device", help="Explicit /dev/input/event path or unique controller name")
    parser.add_argument("--profile", type=Path, default=Path(__file__).with_name("profiles") / "xbox.json")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9004)
    parser.add_argument("--rate", type=float, default=50)
    parser.add_argument("--duration", type=float, help="Optional bounded sending duration, seconds")
    args = parser.parse_args(argv)
    if not (args.list or args.inspect or args.device):
        parser.error("Sending requires an explicit --device; use --list or --inspect first")
    if not math.isfinite(args.rate) or not 10 <= args.rate <= 250 or not 1 <= args.port <= 65535:
        parser.error("Rate must be 10..250 Hz and port 1..65535")
    if args.duration is not None and (not math.isfinite(args.duration) or args.duration <= 0):
        parser.error("duration must be positive and finite")
    try:
        evdev = import_evdev()
        if args.list:
            devices = list_devices(evdev)
            for path, name in devices:
                print(f"{path}\t{name}")
            if not devices:
                print("No accessible gamepads; connect Bluetooth HID and check input-device permissions")
        elif args.inspect:
            inspect_device(evdev, args.inspect)
        else:
            profile = validate_profile(json.loads(args.profile.read_text()))
            stream(evdev, args.device, profile, args.host, args.port, args.rate, args.duration)
    except KeyboardInterrupt:
        return 0
    except (OSError, RuntimeError, ValueError, KeyError, TypeError) as exc:
        print(f"RC bridge: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
