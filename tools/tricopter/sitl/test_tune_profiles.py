"""Native-offset, CLI provenance and narrowly bounded comparison-mask checks."""
import copy
from pathlib import Path
import struct
import tempfile
import unittest

from tune_profiles import overlay_text, shared_msp_readback, verify_profile


def record(gains=(50, 10, 20), feedforward=47):
    # Retained 2026.6.2 common readbacks, without requiring ignored run data.
    pid = bytearray.fromhex('23100f23110f320a14324b4b280000')
    advanced = bytearray.fromhex(
        '00000000000000000000000000000000001e00000050000000000001010005002f00'
        '31002f0000282e00002300c80f64000001410f5a07000001414605')
    pid[6:9] = bytes(gains)
    struct.pack_into('<H', advanced, 36, feedforward)
    return {'common_msp_readback': {'112': pid.hex(), '94': advanced.hex(),
                                   '90': '01010100e001f401', '92': '01c800fa000000'}}


def change_byte(value, command, offset):
    changed = copy.deepcopy(value)
    data = bytearray.fromhex(changed['common_msp_readback'][command])
    data[offset] = (data[offset]+1)%256
    changed['common_msp_readback'][command] = data.hex()
    return changed


class TuneProfiles(unittest.TestCase):
    def log(self, root, value=0, suffix=''):
        text = ('# set feedforward_yaw_hold_gain = 0\nfeedforward_yaw_hold_gain set to 0\n'
                '# get feedforward_yaw_hold_gain\n'
                f'feedforward_yaw_hold_gain = {value}\nprofile 0\nAllowed range: 0 - 100\n'+suffix)
        (root/'provision.log').write_text(text)

    def test_overlay_only_sets_requested_yaw_fields_without_save(self):
        text = overlay_text({'p': 80, 'i': 16, 'd': 30})
        self.assertEqual(text.splitlines(), ['profile 0', 'set p_yaw = 80', 'set i_yaw = 16',
            'set d_yaw = 30', 'set f_yaw = 0', 'set feedforward_yaw_hold_gain = 0',
            'get p_yaw', 'get i_yaw', 'get d_yaw', 'get f_yaw', 'get feedforward_yaw_hold_gain'])
        self.assertTrue(text.endswith('\n'))
        self.assertNotIn('save', text)

    def test_overlay_checks_native_setting_ranges_and_types(self):
        for gains in ({'p': -1, 'i': 10, 'd': 20}, {'p': 251, 'i': 10, 'd': 20},
                      {'p': True, 'i': 10, 'd': 20}, {'p': 50., 'i': 10, 'd': 20}, {'p': 50}):
            with self.subTest(gains=gains), self.assertRaises(ValueError):overlay_text(gains)
        for kwargs in ({'feedforward':1001}, {'hold_gain':101}, {'hold_gain':-1}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):overlay_text({'p':50,'i':10,'d':20}, **kwargs)

    def test_verify_native_offsets_and_uint16_feedforward(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);self.log(root, 12)
            r=record((70,14,32), 513)
            audit=verify_profile(r, {'p':70,'i':14,'d':32}, feedforward=513,hold_gain=12,directory=root)
            self.assertTrue(audit['pass']);self.assertEqual(audit['feedforward'],513)
            self.assertEqual(audit['hold_gain'],12)
            self.assertEqual(audit['yaw_pid_readback']['byte_range'],[6,9])
            self.assertEqual(audit['yaw_feedforward_readback']['byte_range'],[36,38])

    def test_masks_exactly_declared_yaw_bytes_without_mutation(self):
        original=record();before=copy.deepcopy(original)
        tuned=record((90,18,40), 0)
        canonical=shared_msp_readback(tuned)
        self.assertEqual(canonical,shared_msp_readback(original))
        self.assertEqual(canonical['masked_byte_ranges'],{'112':[[6,9]],'94':[[36,38]]})
        self.assertEqual(original,before)
        self.assertEqual(bytes.fromhex(canonical['readback']['112'])[:6],bytes.fromhex(original['common_msp_readback']['112'])[:6])
        self.assertEqual(bytes.fromhex(canonical['readback']['94'])[32:36],struct.pack('<2H',47,49))

    def test_masks_do_not_hide_roll_pitch_filters_or_other_yaw_settings(self):
        base=record()
        for command,offset in [('112',0),('112',4),('112',9),('94',32),('94',34),
                               ('94',41),('94',44),('94',52),('92',1),('90',6)]:
            with self.subTest(command=command,offset=offset):
                self.assertNotEqual(shared_msp_readback(base),shared_msp_readback(change_byte(base,command,offset)))

    def test_reference_allows_only_tuned_pid_and_f_difference(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);self.log(root)
            base=record();tuned=record((90,18,40),0)
            self.assertTrue(verify_profile(tuned,{'p':90,'i':18,'d':40},directory=root,reference=base)['reference_checked'])
            for command,offset in [('112',0),('94',32),('92',1)]:
                with self.subTest(command=command),self.assertRaisesRegex(AssertionError,'non-tuned'):
                    verify_profile(change_byte(tuned,command,offset),{'p':90,'i':18,'d':40},directory=root,reference=base)

    def test_pid_and_feedforward_mismatches_fail(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);self.log(root)
            with self.assertRaisesRegex(AssertionError,'Yaw PID'):
                verify_profile(record(),{'p':51,'i':10,'d':20},directory=root)
            with self.assertRaisesRegex(AssertionError,'Yaw F'):
                verify_profile(record(),{'p':50,'i':10,'d':20},directory=root)

    def test_cli_set_echo_is_not_a_get_readback(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary)
            for text in ('# set feedforward_yaw_hold_gain = 0\nfeedforward_yaw_hold_gain set to 0\n',
                         '# get feedforward_yaw_hold_gain\nfeedforward_yaw_hold_gain set to 0\n'):
                (root/'provision.log').write_text(text)
                with self.assertRaises(AssertionError):verify_profile(record(feedforward=0),{'p':50,'i':10,'d':20},directory=root)

    def test_cli_wrong_hold_or_later_set_fails(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary)
            self.log(root,15)
            with self.assertRaisesRegex(AssertionError,'hold gain'):
                verify_profile(record(feedforward=0),{'p':50,'i':10,'d':20},directory=root)
            self.log(root,0,'# set feedforward_yaw_hold_gain = 15\nfeedforward_yaw_hold_gain set to 15\n')
            with self.assertRaisesRegex(AssertionError,'after'):
                verify_profile(record(feedforward=0),{'p':50,'i':10,'d':20},directory=root)

    def test_truncated_or_malformed_pinned_replies_fail(self):
        for command,value in [('112','00'*9),('94','00'*38),('112','not hex')]:
            r=record();r['common_msp_readback'][command]=value
            with self.subTest(command=command,value=value),self.assertRaises(ValueError):shared_msp_readback(r)


if __name__=='__main__':unittest.main()
