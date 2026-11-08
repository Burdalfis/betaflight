"""RC mapping, wire format and input-loss interlocks; no joystick required."""
import copy
from contextlib import redirect_stdout, redirect_stderr
import io
import json
import math
import errno
from pathlib import Path
import struct
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, Mock, patch

import rc_input as rc


def profile():
    return json.loads((Path(__file__).with_name("profiles") / "xbox-usb.json").read_text())


def state():
    axes = {"ABS_X": 0, "ABS_RX": 0, "ABS_RY": 0, "ABS_RZ": 0}
    ranges = {name: (-100, 100) for name in axes}
    ranges["ABS_RZ"] = (0, 100)
    return rc.InputState(axes, ranges, {name: False for name in ["BTN_SOUTH", "BTN_NORTH", "BTN_TL", "BTN_EAST"]})


class MappingTests(unittest.TestCase):
    def test_all_shipped_profiles_validate_and_elrs_remains_provisional(self):
        for path in Path(__file__).with_name("profiles").glob("*.json"):
            self.assertIsNotNone(rc.validate_profile(json.loads(path.read_text())))
        elrs = json.loads((Path(__file__).with_name("profiles") / "elrs.json").read_text())
        self.assertTrue(elrs["mapping_provisional"])

    def test_xbox_neutral_has_low_trigger_arm_low_and_angle_on(self):
        values, low, kill = rc.Mapper(profile()).map(state())
        self.assertEqual(values, [1500, 1500, 1000, 1500, 1000, 2000] + [1000] * 10)
        self.assertTrue(low)
        self.assertFalse(kill)

    def test_centered_endpoints_clamp_deadzone_and_yaw_mapping(self):
        s, mapper = state(), rc.Mapper(profile())
        for raw, wanted in [(-200, 1000), (-100, 1000), (-5, 1500), (0, 1500), (5, 1500), (100, 2000), (200, 2000)]:
            s.axes["ABS_RX"] = raw
            self.assertEqual(mapper.map(s)[0][0], wanted)
        s.axes["ABS_X"] = -100
        self.assertEqual(mapper.map(s)[0][3], 2000)

    def test_all_profiles_map_right_yaw_to_native_sitl_clockwise_command(self):
        # The actual Gazebo/SITL direction check establishes gyro+yaw = CW.
        # Native updateRcCommands negates RC yaw with yaw_control_reversed=OFF:
        # rcCommand[YAW] *= -GET_DIRECTION(false). Keep firmware/plant intact;
        # raw HID right must therefore map below1500 for a positive CW request.
        for path in Path(__file__).with_name("profiles").glob("*.json"):
            p = json.loads(path.read_text())
            axes, ranges, buttons = {}, {}, {}
            for item in p["channels"]:
                if item["kind"] in {"centered", "throttle", "switch"}:
                    axes[item["code"]] = 0
                    ranges[item["code"]] = (-100, 100)
                elif item["kind"] in {"button", "toggle"}:
                    buttons[item["code"]] = False
            if p.get("disarm_button"):
                buttons[p["disarm_button"]] = False
            yaw = next(item for item in p["channels"] if item["channel"] == 3)
            self.assertTrue(yaw["invert"], path)
            s, mapper = rc.InputState(axes, ranges, buttons), rc.Mapper(p)
            for raw, expected_pwm, expected_clockwise_sign in [(100, 1000, 1), (-100, 2000, -1)]:
                s.axes[yaw["code"]] = raw
                pwm = mapper.map(s)[0][3]
                self.assertEqual(pwm, expected_pwm, path)
                native_yaw_request = -(pwm - 1500)
                self.assertGreater(native_yaw_request * expected_clockwise_sign, 0, path)

    def test_pitch_is_inverted_and_center_may_be_asymmetric(self):
        s, p = state(), profile()
        s.axes["ABS_RY"] = -100
        self.assertEqual(rc.Mapper(p).map(s)[0][1], 2000)
        p["channels"][0].update(min=-20, center=10, max=100, deadzone=0)
        for raw, wanted in [(-20, 1000), (-5, 1250), (10, 1500), (55, 1750), (100, 2000)]:
            s.axes["ABS_RX"] = raw
            self.assertEqual(rc.Mapper(p).map(s)[0][0], wanted)

    def test_right_trigger_has_low_rest_full_range_and_optional_inversion(self):
        s, p = state(), profile()
        mapper = rc.Mapper(p)
        for raw, wanted in [(-10, 1000), (0, 1000), (2, 1000), (51, 1500), (100, 2000), (110, 2000)]:
            s.axes["ABS_RZ"] = raw
            self.assertEqual(mapper.map(s)[0][2], wanted)
        p["channels"][2]["invert"] = True
        s.axes["ABS_RZ"] = 100
        self.assertEqual(rc.Mapper(p).map(s)[0][2], 1000)

    def test_switch_axis_and_constant_spare_channels(self):
        p, s = profile(), state()
        p["channels"][4] = {"channel": 4, "kind": "switch", "code": "ABS_X", "levels": [1000, 1500, 2000]}
        p["channels"].append({"channel": 15, "kind": "constant", "value": 1750})
        for value, expected, released in [(-100, 1000, True), (0, 1500, False), (100, 2000, False)]:
            s.axes["ABS_X"] = value
            channels, low, _ = rc.Mapper(p).map(s)
            self.assertEqual(channels[4], expected)
            self.assertEqual(channels[15], 1750)
            self.assertEqual(low, released)

    def test_button_repeated_press_does_not_retoggle(self):
        s, mapper = state(), rc.Mapper(profile())
        mapper.map(s)
        s.buttons["BTN_SOUTH"] = True
        for _ in range(10):
            values, low, _ = mapper.map(s)
            self.assertEqual(values[4], 2000)
            self.assertFalse(low)
        s.buttons["BTN_SOUTH"] = False
        self.assertEqual(mapper.map(s)[0][4], 2000)
        s.buttons["BTN_SOUTH"] = True
        self.assertEqual(mapper.map(s)[0][4], 1000)

    def test_connecting_with_a_held_does_not_toggle_or_release_arm_interlock(self):
        s = state()
        s.buttons["BTN_SOUTH"] = True
        mapper = rc.Mapper(profile())
        values, low, _ = mapper.map(s)
        self.assertEqual(values[4], 1000)
        self.assertFalse(low)
        s.buttons["BTN_SOUTH"] = False
        self.assertTrue(mapper.map(s)[1])
        s.buttons["BTN_SOUTH"] = True
        self.assertEqual(mapper.map(s)[0][4], 2000)

    def test_b_disarm_clears_arm_toggle_without_rearming_after_release(self):
        s, mapper = state(), rc.Mapper(profile())
        mapper.map(s)
        s.buttons["BTN_SOUTH"] = True
        self.assertEqual(mapper.map(s)[0][4], 2000)
        s.buttons["BTN_EAST"] = True
        values, _, kill = mapper.map(s)
        self.assertTrue(kill)
        self.assertEqual(values[4], 1000)
        s.buttons["BTN_EAST"] = False
        self.assertEqual(mapper.map(s)[0][4], 1000)
        s.buttons["BTN_SOUTH"] = False
        self.assertEqual(mapper.map(s)[0][4], 1000)

    def test_missing_axis_button_or_invalid_runtime_range_rejects_frame(self):
        for kind in ["axis", "button", "range", "center"]:
            s, p = state(), profile()
            if kind == "axis":
                del s.axes["ABS_RX"]
            elif kind == "button":
                del s.buttons["BTN_SOUTH"]
            elif kind == "range":
                s.ranges["ABS_RX"] = (1, 1)
            else:
                p["channels"][0]["center"] = 100
            with self.assertRaises(ValueError):
                rc.Mapper(p).map(s)

    def test_invalid_profile_values_reject_before_streaming(self):
        for name in ["order", "duplicate", "missing", "nan", "kind", "deadzone", "arm", "throttle_limit"]:
            p = profile()
            if name == "order": p["channel_order"] = "TAER1234"
            elif name == "duplicate": p["channels"].append(copy.deepcopy(p["channels"][0]))
            elif name == "missing": p["channels"].pop(0)
            elif name == "nan": p["channels"][0]["min"] = float("nan")
            elif name == "kind": p["channels"][0]["kind"] = "unrecognized"
            elif name == "deadzone": p["channels"][0]["deadzone"] = 1
            elif name == "arm": p["arm_channel"] = 2
            else: p["low_throttle_max"] = 1500
            with self.subTest(name=name), self.assertRaises(ValueError):
                rc.validate_profile(p)


class BluetoothXboxTests(unittest.TestCase):
    """Replay the axis ranges and button codes observed on the paired Xbox."""
    def setUp(self):
        self.profile = json.loads((Path(__file__).with_name("profiles") / "xbox.json").read_text())
        self.mapper = rc.Mapper(self.profile)
        self.state = rc.InputState(
            {"ABS_X": 32768, "ABS_Z": 32768, "ABS_RZ": 32768, "ABS_GAS": 0},
            {"ABS_X": (0, 65535), "ABS_Z": (0, 65535), "ABS_RZ": (0, 65535), "ABS_GAS": (0, 1023)},
            {"BTN_SOUTH": False, "BTN_EAST": False, "BTN_WEST": False, "BTN_TL": False})

    def test_observed_unsigned_sticks_and_trigger_are_independent(self):
        channels, low, kill = self.mapper.map(self.state)
        self.assertEqual(channels[:6], [1500, 1500, 1000, 1500, 1000, 2000])
        self.assertTrue(low)
        self.assertFalse(kill)
        for code, channel, low_pwm, high_pwm in [
            ("ABS_Z", 0, 1000, 2000),
            ("ABS_RZ", 1, 2000, 1000),
            ("ABS_X", 3, 2000, 1000),
        ]:
            for raw, expected in [(0, low_pwm), (65535, high_pwm)]:
                with self.subTest(code=code, raw=raw):
                    self.state.axes[code] = raw
                    channels = self.mapper.map(self.state)[0]
                    self.assertEqual(channels[channel], expected)
                    self.assertEqual(channels[2], 1000)
            self.state.axes[code] = 32768
        for raw, expected in [(0, 1000), (20, 1000), (1023, 2000)]:
            self.state.axes["ABS_GAS"] = raw
            channels = self.mapper.map(self.state)[0]
            self.assertEqual(channels[2], expected)
            self.assertEqual([channels[i] for i in (0, 1, 3)], [1500] * 3)

    def test_observed_numeric_a_b_y_buttons_map_to_arm_disarm_angle(self):
        # Captured EV_KEY codes: A=304, B=305, Y=308 on this Bluetooth HID.
        codes = {"ABS_X": 0, "ABS_Z": 2, "ABS_RZ": 5, "ABS_GAS": 9,
                 "BTN_SOUTH": 304, "BTN_EAST": 305, "BTN_WEST": 308, "BTN_TL": 310}
        evdev = SimpleNamespace(ecodes=SimpleNamespace(EV_ABS=3, EV_KEY=1, ecodes=codes))
        device = Mock()
        device.capabilities.return_value = {3: [0, 2, 5, 9], 1: [304, 305, 308, 310]}
        device.absinfo.side_effect = lambda code: SimpleNamespace(
            value=0 if code == 9 else 32768, min=0, max=1023 if code == 9 else 65535)
        def frame(*pressed):
            device.active_keys.return_value = list(pressed)
            return self.mapper.map(rc.read_state(device, evdev, self.profile))
        self.assertEqual(frame()[0][4:6], [1000, 2000])
        self.assertEqual(frame(304)[0][4], 2000)
        channels, _, kill = frame(305)
        self.assertTrue(kill)
        self.assertEqual(channels[4], 1000)
        frame()
        self.assertEqual(frame(308)[0][5], 1000)


class InterlockTests(unittest.TestCase):
    def test_startup_requires_arm_low_and_low_throttle(self):
        for arm, throttle, physically_low, wanted_ready in [(2000, 1000, False, False), (1000, 1700, True, False), (1000, 1000, False, False), (1000, 1000, True, True)]:
            gate, values = rc.ArmGate(), rc.DEFAULT_CHANNELS.copy()
            values[4], values[2] = arm, throttle
            self.assertEqual(gate.apply(values, physically_low)[4], 1000)
            self.assertEqual(gate.ready, wanted_ready)

    def test_ready_arm_then_raising_throttle_is_allowed(self):
        gate, values = rc.ArmGate(), rc.DEFAULT_CHANNELS.copy()
        gate.apply(values, True)
        values[4] = 2000
        self.assertEqual(gate.apply(values, False)[4], 2000)
        values[2] = 1750
        self.assertEqual(gate.apply(values, False)[4], 2000)

    def test_arming_with_high_throttle_inhibits_until_low_arm_and_throttle(self):
        gate, values = rc.ArmGate(), rc.DEFAULT_CHANNELS.copy()
        gate.apply(values, True)
        values[2], values[4] = 1700, 2000
        self.assertEqual(gate.apply(values, False)[4], 1000)
        values[2] = 1000
        self.assertEqual(gate.apply(values, False)[4], 1000)
        values[4] = 1000
        gate.apply(values, True)
        values[4] = 2000
        self.assertEqual(gate.apply(values, False)[4], 2000)

    def test_disconnect_returns_no_frame_and_reconnection_cannot_rearm_high(self):
        gate, values = rc.ArmGate(), rc.DEFAULT_CHANNELS.copy()
        gate.apply(values, True)
        values[4] = 2000
        self.assertEqual(gate.apply(values, False)[4], 2000)
        self.assertIsNone(gate.apply(values, False, connected=False))
        self.assertEqual(gate.apply(values, False)[4], 1000)

    def test_quiet_sticks_do_not_expire_and_kill_requires_new_deliberate_arm(self):
        gate, values = rc.ArmGate(), rc.DEFAULT_CHANNELS.copy()
        gate.apply(values, True)
        values[4] = 2000
        for _ in range(1000):
            self.assertEqual(gate.apply(values, False)[4], 2000)
        values[2] = 1600
        result = gate.apply(values, False, kill=True)
        self.assertEqual((result[4], result[2]), (1000, 1000))
        self.assertEqual(gate.apply(values, False)[4], 1000)


class PacketAndCliTests(unittest.TestCase):
    def test_packet_has_native_sitl_40byte_little_endian_layout(self):
        values = [1000 + i * 10 for i in range(16)]
        packet = rc.encode_packet(1.25, values)
        self.assertEqual(len(packet), 40)
        self.assertEqual(packet, struct.pack("<d16H", 1.25, *values))
        self.assertEqual(struct.unpack_from("<H", packet, 8 + 4 * 2)[0], values[4])

    def test_packet_rejects_bad_count_nonfinite_time_or_invalid_pwm(self):
        for timestamp, values in [(math.nan, rc.DEFAULT_CHANNELS), (math.inf, rc.DEFAULT_CHANNELS), (-1, rc.DEFAULT_CHANNELS), (0, [1000] * 15), (0, [999] * 16), (0, [2001] * 16), (0, [True] * 16), (0, [1500.] * 16)]:
            with self.assertRaises(ValueError): rc.encode_packet(timestamp, values)

    def test_list_and_inspect_do_not_open_udp_socket(self):
        with patch.object(rc, "import_evdev", return_value=Mock()), patch.object(rc, "list_devices", return_value=[]), patch.object(rc, "inspect_device") as inspect, patch.object(rc.socket, "socket") as sock, redirect_stdout(io.StringIO()):
            self.assertEqual(rc.main(["--list"]), 0)
            self.assertEqual(rc.main(["--inspect", "Xbox"]), 0)
            inspect.assert_called_once()
            sock.assert_not_called()

    def test_sending_without_explicit_device_rejects_and_never_opens_socket(self):
        with patch.object(rc.socket, "socket") as sock, redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            rc.main([])
        sock.assert_not_called()

    def test_device_read_uses_numeric_abs_capabilities_and_kernel_ranges(self):
        p = profile()
        names = [item["code"] for item in p["channels"]] + [p["disarm_button"]]
        codes = dict(zip(names, range(len(names))))
        ecodes = SimpleNamespace(EV_ABS=3, EV_KEY=1, ecodes=codes)
        device = Mock()
        device.capabilities.return_value = {3: [codes[n] for n in names if n.startswith("ABS")], 1: [codes[n] for n in names if n.startswith("BTN")]}
        device.active_keys.return_value = []
        device.absinfo.return_value = SimpleNamespace(value=0, min=-100, max=100)
        result = rc.read_state(device, SimpleNamespace(ecodes=ecodes), p)
        device.capabilities.assert_called_once_with(absinfo=False)
        self.assertEqual(result.ranges["ABS_RX"], (-100, 100))
        self.assertFalse(result.buttons["BTN_SOUTH"])


class RuntimeTests(unittest.TestCase):
    def run_fake_device(self, disconnect_at=None):
        clock = SimpleNamespace(now=0.)
        names = [item["code"] for item in profile()["channels"]] + ["BTN_EAST"]
        codes = dict(zip(names, range(len(names))))
        evdev = SimpleNamespace(ecodes=SimpleNamespace(EV_ABS=3, EV_KEY=1, EV_SYN=0, SYN_DROPPED=3, ecodes=codes))
        device = Mock(name="mock_hid")
        device.name, device.fd = "Synthetic Xbox", 123
        device.capabilities.return_value = {3: [codes[n] for n in names if n.startswith("ABS")], 1: [codes[n] for n in names if n.startswith("BTN")]}
        device.active_keys.return_value = []

        def absinfo(code):
            if disconnect_at is not None and clock.now >= disconnect_at:
                raise OSError(errno.ENODEV, "Synthetic disconnect")
            return SimpleNamespace(value=0, min=0 if code == codes["ABS_RZ"] else -100, max=100)

        def select_tick(read, write, error, timeout):
            clock.now += max(timeout, .001)
            return [], [], []  # Completely quiet sticks throughout.

        device.absinfo.side_effect = absinfo
        sock = MagicMock()
        output = sock.__enter__.return_value
        with patch.object(rc, "open_device", return_value=device), patch.object(rc.socket, "socket", return_value=sock), patch.object(rc.select, "select", side_effect=select_tick), patch.object(rc.time, "monotonic", side_effect=lambda: clock.now), patch.object(rc.time, "sleep", side_effect=lambda seconds: setattr(clock, "now", clock.now + seconds)), redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            rc.stream(evdev, "Synthetic", profile(), "127.0.0.1", 9004, 50, duration=.25)
        return [rc.PACKET.unpack(call.args[0]) for call in output.sendto.call_args_list], device

    def test_quiet_device_keeps_streaming_and_uses_compatible_evdev_lifetime(self):
        packets, device = self.run_fake_device()
        self.assertGreaterEqual(len(packets), 10)
        self.assertGreater(packets[-1][0], .2)
        self.assertTrue(all(packet[3] == 1000 and packet[5] == 1000 for packet in packets))
        self.assertGreater(device.active_keys.call_count, 1)  # Periodic health query.
        device.close.assert_called_once()  # evdev has no context-manager API.

    def test_device_ioctl_failure_stops_datagrams_instead_of_replaying_cached_sticks(self):
        packets, device = self.run_fake_device(disconnect_at=.1)
        self.assertGreater(len(packets), 1)
        self.assertLess(packets[-1][0], .12)
        device.close.assert_called_once()


if __name__ == "__main__":
    unittest.main()
