"""Integrity, timing-qualification and loss-accounting tests; no MCU required."""
import csv
import hashlib
import io
import json
from pathlib import Path
import shutil
import struct
import subprocess
import tempfile
import unittest
from unittest.mock import patch
import zlib

import capture


def frame(seq=0, stamp=1000, flags=capture.SENSOR_CONFIGURED | capture.DRDY_QUALIFIED,
          kind=capture.SAMPLE, version=1, xyz=(-123, 45, 256), aux=0x80,
          latency=17, pwm_width=1420, pwm_seq=7, pwm_time=900, overrun=0, gaps=0, drops=0):
    payload = capture.PAYLOAD.pack(b"ADXL", version, kind, flags, seq, stamp,
                                   *xyz, aux, latency, pwm_width, pwm_seq, pwm_time,
                                   overrun, gaps, drops)
    return payload + struct.pack("<I", zlib.crc32(payload))


def decoded(**kwargs):
    return capture.Decoder().feed(frame(**kwargs))[0]


class DecoderTests(unittest.TestCase):
    def test_fragmentation_at_every_byte_preserves_signed_and_64_bit_fields(self):
        wire = frame(seq=0xFEDCBA98, stamp=0x123456789ABCDEF0,
                     xyz=(-32768, 32767, -1), aux=0xABCD, latency=0xDEADBEEF,
                     pwm_time=0xFEDCBA9876543210, overrun=17, gaps=19, drops=23)
        parser = capture.Decoder()
        rows = []
        for byte in wire:
            rows.extend(parser.feed(bytes([byte])))
        self.assertEqual(len(rows), 1)
        row = rows[0]
        self.assertEqual((row["wire_x"], row["wire_y"], row["wire_z"]), (-32768, 32767, -1))
        self.assertEqual(row["time_us"], 0x123456789ABCDEF0)
        self.assertEqual(row["pwm_rise_time_us"], 0xFEDCBA9876543210)
        self.assertEqual(row["record_seq"], 0xFEDCBA98)
        self.assertEqual(row["aux"], 0xABCD)
        self.assertEqual(row["read_latency_us"], 0xDEADBEEF)
        self.assertEqual((row["sensor_overrun_events"], row["service_gap_events"], row["queue_dropped_frames"]), (17, 19, 23))
        self.assertEqual(parser.finish()["truncated_tail_bytes"], 0)

    def test_noise_crc_failure_and_chunk_boundary_resynchronize(self):
        damaged = bytearray(frame(seq=2))
        damaged[22] ^= 0x80
        wire = b"noiseAD" + frame(seq=1) + damaged + b"garbage" + frame(seq=3)
        parser = capture.Decoder()
        rows = []
        for start in range(0, len(wire), 13):
            rows.extend(parser.feed(wire[start:start + 13]))
        self.assertEqual([r["record_seq"] for r in rows], [1, 3])
        stats = parser.finish()
        self.assertEqual(stats["crc_errors"], 1)
        self.assertEqual(stats["discarded_bytes"], len(wire) - 128)

    def test_supported_framing_does_not_decode_unknown_version_or_type(self):
        parser = capture.Decoder()
        rows = parser.feed(frame(version=2) + frame(kind=99) + frame(seq=4))
        self.assertEqual([r["record_seq"] for r in rows], [4])
        self.assertEqual(parser.stats["unsupported_versions"], 1)
        self.assertEqual(parser.stats["unsupported_types"], 1)
        self.assertEqual(parser.stats["discarded_bytes"], 0)

    def test_embedded_magic_in_payload_is_not_an_extra_record(self):
        wire = bytearray(frame())
        wire[48:52] = b"ADXL"
        wire[60:] = struct.pack("<I", zlib.crc32(wire[:60]))
        parser = capture.Decoder()
        self.assertEqual(len(parser.feed(wire + frame(seq=1))), 2)
        self.assertEqual(parser.finish()["crc_errors"], 0)

    def test_truncated_tail_is_reported_and_never_a_sample(self):
        parser = capture.Decoder()
        self.assertEqual(len(parser.feed(frame() + frame(seq=1)[:37])), 1)
        self.assertEqual(parser.finish()["truncated_tail_bytes"], 37)
        self.assertEqual(parser.finish()["truncated_tail_bytes"], 37)
        with self.assertRaises(ValueError):
            parser.feed(b"")

    def test_trailing_garbage_is_distinct_from_truncated_magic_prefix(self):
        parser = capture.Decoder()
        parser.feed(b"nonsense")
        self.assertEqual(parser.finish()["truncated_tail_bytes"], 0)
        self.assertEqual(parser.stats["discarded_bytes"], 8)
        parser = capture.Decoder()
        parser.feed(b"nonsenseADX")
        self.assertEqual(parser.finish()["truncated_tail_bytes"], 3)


class TrackingTests(unittest.TestCase):
    def test_fifo_xyz_coherence_does_not_establish_a_drdy_timestamp(self):
        tracker = capture.Tracker()
        for seq, depth in enumerate((1, 31)):
            row = tracker.row(decoded(seq=seq, stamp=1000 + seq * 10,
                                      flags=capture.SENSOR_CONFIGURED | capture.FIFO_COHERENT | capture.PWM_FRESH,
                                      aux=(depth << 8) | 0x80))
            self.assertEqual(row["fifo_coherent"], 1)
            self.assertEqual(row["fifo_depth_before_read"], depth)
            self.assertEqual(row["int_source"], "0x0080")
            self.assertEqual(row["time_source"], "fifo_spi_read_start")
            self.assertEqual(row["sample_time_source"], "fifo_spi_read_start")
            self.assertEqual(row["drdy_timestamp_qualified"], 0)
            self.assertEqual((row["raw_x"], row["raw_y"], row["raw_z"]), (-123, 45, 256))
            self.assertEqual(row["pwm_fresh"], 1)
        quality = tracker.summary()
        self.assertEqual(quality["fifo_sample_rows"], 2)
        self.assertEqual(quality["fifo_coherent_sample_rows"], 2)
        self.assertEqual(quality["qualified_sample_timestamps"], 0)
        self.assertEqual(quality["rows_with_unknown_flag_bits"], 0)

    def test_fifo_full_empty_and_invalid_depth_are_retained_but_not_coherent(self):
        tracker = capture.Tracker()
        for seq, depth in enumerate((0, 32, 255)):
            row = tracker.row(decoded(seq=seq, flags=capture.FIFO_COHERENT, aux=(depth << 8) | 1))
            self.assertEqual(row["fifo_coherent"], 0)
            self.assertEqual(row["fifo_depth_before_read"], depth)
            self.assertEqual(row["raw_x"], -123)
            self.assertEqual(row["time_source"], "fifo_spi_read_start")
        quality = tracker.summary()
        self.assertEqual(quality["sample_rows"], 3)
        self.assertEqual(quality["fifo_sample_rows"], 3)
        self.assertEqual(quality["fifo_coherent_sample_rows"], 0)
        self.assertEqual(quality["fifo_invalid_depth_rows"], 3)
        self.assertEqual(quality["qualified_sample_timestamps"], 0)

    def test_conflicting_fifo_and_drdy_flags_never_imply_precise_time(self):
        tracker = capture.Tracker()
        row = tracker.row(decoded(flags=capture.SENSOR_CONFIGURED | capture.FIFO_COHERENT | capture.DRDY_QUALIFIED,
                                  aux=0x0480))
        self.assertEqual(row["flags"], "0x0083")
        self.assertEqual(row["fifo_depth_before_read"], 4)
        self.assertEqual(row["fifo_coherent"], 0)
        self.assertEqual(row["drdy_timestamp_qualified"], 0)
        self.assertEqual(row["sample_time_source"], "fifo_spi_read_start")
        self.assertEqual(tracker.summary()["qualified_sample_timestamps"], 0)
        self.assertEqual(tracker.summary()["fifo_timestamp_conflict_rows"], 1)
        self.assertEqual(tracker.summary()["fifo_coherent_sample_rows"], 0)

    def test_fifo_flag_does_not_reinterpret_status_or_diagnostic_slots(self):
        tracker = capture.Tracker()
        status = tracker.row(decoded(seq=10, kind=capture.STATUS, flags=capture.FIFO_COHERENT,
                                     xyz=(0xE5, 0x0D, 8), aux=0))
        diagnostic = tracker.row(decoded(seq=11, kind=capture.DIAGNOSTIC, flags=capture.FIFO_COHERENT,
                                         xyz=(0x180, 0x80, 0), aux=0x0480))
        for row in (status, diagnostic):
            for name in ("fifo_coherent", "fifo_depth_before_read", "sample_time_source", "raw_x", "int_source"):
                self.assertEqual(row[name], "", name)
        self.assertEqual(status["fault_code"], 0)
        self.assertEqual(diagnostic["diagnostic_post_int_source"], "0x04")
        quality = tracker.summary()
        self.assertEqual(quality["fifo_sample_rows"], 0)
        self.assertEqual(quality["fifo_coherent_sample_rows"], 0)
        self.assertEqual(quality["sample_rows"], 0)
        self.assertEqual(quality["status_rows"], 1)
        self.assertEqual(quality["diagnostic_rows"], 1)

    def test_legacy_sample_sources_keep_prior_meaning(self):
        tracker = capture.Tracker()
        row = tracker.row(decoded())
        self.assertEqual(row["time_source"], "drdy_edge")
        self.assertEqual(row["sample_time_source"], "drdy_edge")
        self.assertEqual(row["drdy_timestamp_qualified"], 1)
        self.assertEqual(row["int_source"], "0x0080")
        self.assertEqual(row["fifo_coherent"], 0)
        self.assertEqual(row["fifo_depth_before_read"], "")
        row = tracker.row(decoded(seq=1, flags=capture.SENSOR_CONFIGURED | capture.POLL_FALLBACK))
        self.assertEqual(row["time_source"], "spi_read_start")
        self.assertEqual(row["sample_time_source"], "spi_read_start")
        self.assertEqual(tracker.summary()["qualified_sample_timestamps"], 1)
        self.assertEqual(tracker.summary()["fifo_sample_rows"], 0)

    def test_diagnostic_aliases_are_not_acceleration_status_or_pwm(self):
        tracker = capture.Tracker()
        # Even unexpected measurement/session flags on a diagnostic cannot
        # promote the aliased register slots to PWM or gravity observations.
        row = tracker.row(decoded(kind=capture.DIAGNOSTIC, seq=10, stamp=5000, flags=0x7F,
                                  xyz=(0x1FF, 0x80, 0), aux=0x0180, latency=0x80000001,
                                  pwm_width=0xA50A0F03, pwm_seq=0x80000004,
                                  pwm_time=0xFFFFFFFE, overrun=11, gaps=13, drops=17))
        self.assertEqual(row["type"], "diagnostic")
        self.assertEqual(row["time_source"], "diagnostic_clock")
        self.assertEqual(row["diagnostic_bits"], "0x01ff")
        for name in ("pin2_level", "pre_xyz_pin2_level", "post_xyz_pin2_level", "drdy_pending",
                     "lc_nvic_enabled", "lc_nvic_pending", "primask", "lc_raw_state_available",
                     "post_int_source_valid"):
            self.assertEqual(row["diagnostic_" + name], 1, name)
        self.assertEqual(row["diagnostic_int_enable"], "0x80")
        self.assertEqual(row["diagnostic_int_map"], "0x00")
        self.assertEqual(row["diagnostic_pre_int_source"], "0x80")
        self.assertEqual(row["diagnostic_post_int_source"], "0x01")
        self.assertEqual(row["diagnostic_drdy_isr_sequence"], 0x80000001)
        self.assertEqual(row["diagnostic_pin2_pcr"], "0xa50a0f03")
        self.assertEqual(row["diagnostic_portd_isfr"], "0x80000004")
        self.assertEqual(row["diagnostic_last_isr_micros_low32"], 0xFFFFFFFE)
        for name in ("raw_x", "raw_y", "raw_z", "int_source", "devid", "bw_rate", "data_format",
                     "fault_code", "fault_name", "read_latency_us", "pwm_width_us", "pwm_seq",
                     "pwm_rise_time_us", "pwm_fresh", "drdy_timestamp_qualified", "usb_session_start"):
            self.assertEqual(row[name], "", name)
        quality = tracker.summary()
        self.assertEqual(quality["diagnostic_rows"], 1)
        for name in ("sample_rows", "status_rows", "qualified_sample_timestamps", "fresh_pwm_sample_rows",
                     "usb_session_start_rows"):
            self.assertEqual(quality[name], 0, name)
        self.assertEqual(quality["status_fault_code_counts"], {})
        self.assertEqual(row["queue_dropped_frames"], 17)

    def test_diagnostic_unavailable_raw_and_post_source_remain_unknown(self):
        row = capture.Tracker().row(decoded(kind=capture.DIAGNOSTIC, flags=capture.SENSOR_CONFIGURED,
                                           xyz=(0x70, 0x80, 0), aux=0xFF80,
                                           pwm_width=123, pwm_seq=456))
        self.assertEqual(row["diagnostic_lc_raw_state_available"], 0)
        self.assertEqual(row["diagnostic_post_int_source_valid"], 0)
        for name in ("lc_nvic_enabled", "lc_nvic_pending", "pin2_pcr", "portd_isfr",
                     "post_int_source"):
            self.assertEqual(row["diagnostic_" + name], "", name)
        self.assertEqual(row["diagnostic_primask"], 1)
        self.assertEqual(row["diagnostic_pre_int_source"], "0x80")

    def test_diagnostic_is_in_normal_record_sequence_without_new_epoch(self):
        tracker = capture.Tracker()
        tracker.row(decoded(kind=capture.STATUS, flags=capture.USB_SESSION_START,
                            seq=100, stamp=1000, aux=0))
        tracker.row(decoded(kind=capture.DIAGNOSTIC, flags=capture.SENSOR_CONFIGURED,
                            seq=101, stamp=1001, xyz=(0, 0x80, 0)))
        row = tracker.row(decoded(seq=102, stamp=2000))
        self.assertEqual(row["seq_gap_before"], 0)
        self.assertEqual(row["stream_epoch"], 0)
        quality = tracker.summary()
        self.assertEqual(quality["records"], 3)
        self.assertEqual(quality["sample_rows"], 1)
        self.assertEqual(quality["status_rows"], 1)
        self.assertEqual(quality["diagnostic_rows"], 1)
        self.assertEqual(quality["status_fault_code_counts"], {"0": 1})

    def test_sequence_wrap_is_contiguous_and_forward_gap_counts_records(self):
        tracker = capture.Tracker()
        tracker.row(decoded(seq=0xFFFFFFFF, stamp=1))
        row = tracker.row(decoded(seq=0, stamp=2))
        self.assertEqual(row["seq_gap_before"], 0)
        row = tracker.row(decoded(seq=3, stamp=3))
        self.assertEqual(row["seq_gap_before"], 2)
        self.assertEqual(len(tracker.epochs), 1)
        self.assertEqual(tracker.summary()["missing_record_sequences"], 2)

    def test_explicit_usb_session_separates_epochs_without_inventing_gap(self):
        tracker = capture.Tracker()
        tracker.row(decoded(seq=100, stamp=10000))
        row = tracker.row(decoded(seq=500, stamp=20000, kind=capture.STATUS,
                                  flags=capture.USB_SESSION_START | capture.SENSOR_CONFIGURED,
                                  xyz=(0xE5, 0x0D, 0x08), aux=0))
        self.assertEqual(row["stream_epoch"], 1)
        self.assertEqual(row["epoch_reason"], "usb_session_start")
        self.assertEqual(row["seq_gap_before"], 0)
        self.assertEqual(row["time_regression"], 0)
        self.assertEqual(row["raw_x"], "")
        self.assertEqual((row["devid"], row["bw_rate"], row["data_format"]), (0xE5, 0x0D, 0x08))

    def test_reset_and_clock_regression_are_separately_visible(self):
        tracker = capture.Tracker()
        tracker.row(decoded(seq=100, stamp=50000))
        row = tracker.row(decoded(seq=0, stamp=20))
        self.assertEqual(row["seq_discontinuity"], 1)
        self.assertEqual(row["time_regression"], 1)
        self.assertEqual(row["epoch_reason"], "sequence_reset_or_duplicate")
        self.assertEqual(tracker.summary()["missing_record_sequences"], 0)
        row = tracker.row(decoded(seq=1, stamp=10))
        self.assertEqual(row["seq_discontinuity"], 0)
        self.assertEqual(row["time_regression"], 1)
        self.assertEqual(row["stream_epoch"], 1)

    def test_unqualified_time_and_stale_pwm_are_not_promoted_to_measurements(self):
        tracker = capture.Tracker()
        row = tracker.row(decoded(flags=capture.SENSOR_CONFIGURED | capture.POLL_FALLBACK,
                                  pwm_width=1420, pwm_time=12))
        self.assertEqual(row["time_source"], "spi_read_start")
        self.assertEqual(row["drdy_timestamp_qualified"], 0)
        self.assertEqual(row["pwm_fresh"], 0)
        self.assertEqual(row["pwm_width_us"], 1420)
        self.assertEqual(row["pwm_rise_time_us"], 12)
        self.assertEqual(row["raw_x"], -123)
        quality = tracker.summary()
        self.assertEqual(quality["qualified_sample_timestamps"], 0)
        self.assertEqual(quality["fresh_pwm_sample_rows"], 0)
        self.assertEqual(quality["poll_fallback_sample_rows"], 1)

    def test_counters_are_not_summed_or_presented_as_exact_lost_samples(self):
        tracker = capture.Tracker()
        for seq in range(3):
            tracker.row(decoded(seq=seq, stamp=seq * 1250,
                                flags=capture.SENSOR_CONFIGURED | capture.SENSOR_OVERRUN | capture.SERVICE_GAP,
                                overrun=10, gaps=20, drops=30))
        quality = tracker.summary()
        self.assertEqual(quality["epochs"][0]["last_counters"]["queue_dropped_frames"], 30)
        self.assertEqual(quality["sensor_overrun_sample_rows"], 3)
        self.assertEqual(quality["missing_record_sequences"], 0)
        self.assertIn("not exact lost sensor sample counts", quality["loss_interpretation"])

    def test_fault_rows_remain_visible_and_unknown_faults_are_retained(self):
        tracker = capture.Tracker()
        for seq, code in enumerate((1, 2, 3, 4, 5, 999)):
            row = tracker.row(decoded(seq=seq, kind=capture.STATUS, flags=0, aux=code))
            self.assertEqual(row["fault_code"], code)
            if code == 5:
                self.assertEqual(row["fault_name"], "FIFO_DISCARD")
        self.assertEqual(row["fault_name"], "UNKNOWN")
        self.assertEqual(tracker.summary()["status_rows"], 6)
        self.assertEqual(tracker.summary()["sample_rows"], 0)


class ArtifactTests(unittest.TestCase):
    def test_raw_chunks_are_visible_to_a_live_monitor_before_more_input(self):
        wire = frame()
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)

            def chunks():
                yield wire[:17], 1, 1
                self.assertEqual((output / "capture.bin").read_bytes(), wire[:17])
                yield wire[17:], 2, 2
                self.assertEqual((output / "capture.bin").read_bytes(), wire)

            report = capture.record_stream(chunks(), output)
            self.assertEqual(report["quality"]["sample_rows"], 1)
            self.assertEqual(report["parser"]["crc_errors"], 0)

    @unittest.skipUnless(shutil.which("g++"), "C++ compiler unavailable for firmware wire-format check")
    def test_actual_firmware_diagnostic_encoder_uses_existing_layout(self):
        header = Path(__file__).resolve().parent / "include"
        program = r'''
#include <cstdio>
#include "logger_protocol.h"
int main() {
    adxl_logger::Fields fields = {};
    fields.type = adxl_logger::kDiagnosticFrame;
    fields.flags = adxl_logger::SensorConfigured;
    fields.sequence = 101;
    fields.timeUs = UINT64_C(0x123456789abcdef0);
    fields.x = 0x1ff; fields.y = 0x80; fields.z = 0;
    fields.auxiliary = 0x0180;
    fields.readLatencyUs = 0xfedcba98U;
    fields.pwmWidthUs = 0xa50a0f03U;
    fields.pwmSequence = 0x80000004U;
    fields.pwmRiseTimeUs = 0xfffffffeU;
    fields.sensorOverruns = 11; fields.serviceGaps = 13; fields.queueDroppedFrames = 17;
    uint8_t output[adxl_logger::kFrameSize];
    adxl_logger::encode(fields, output);
    return std::fwrite(output, 1, sizeof output, stdout) == sizeof output ? 0 : 2;
}
'''
        with tempfile.TemporaryDirectory() as temporary:
            source, binary = Path(temporary) / "encode.cpp", Path(temporary) / "encode"
            source.write_text(program)
            subprocess.run(["g++", "-std=c++11", "-Wall", "-Wextra", "-Werror", "-I" + str(header),
                            str(source), "-o", str(binary)], check=True, capture_output=True)
            wire = subprocess.run([str(binary)], check=True, capture_output=True).stdout
        self.assertEqual(wire, frame(kind=capture.DIAGNOSTIC, seq=101, stamp=0x123456789ABCDEF0,
                                     flags=capture.SENSOR_CONFIGURED, xyz=(0x1FF, 0x80, 0), aux=0x0180,
                                     latency=0xFEDCBA98, pwm_width=0xA50A0F03, pwm_seq=0x80000004,
                                     pwm_time=0xFFFFFFFE, overrun=11, gaps=13, drops=17))
        record = capture.Decoder().feed(wire)[0]
        self.assertEqual(record["record_type"], capture.DIAGNOSTIC)
        row = capture.Tracker().row(record)
        self.assertEqual(row["diagnostic_drdy_isr_sequence"], 0xFEDCBA98)
        self.assertEqual(row["diagnostic_last_isr_micros_low32"], 0xFFFFFFFE)
        self.assertEqual(row["pwm_width_us"], "")

    @unittest.skipUnless(shutil.which("g++"), "C++ compiler unavailable for firmware wire-format check")
    def test_actual_firmware_encoder_is_decoded_with_exact_fields(self):
        # Exercise the actual shared firmware header, not a second Python
        # encoder.  This catches CRC/offset/endianness drift between the sides.
        header = Path(__file__).resolve().parent / "include"
        program = r'''
#include <cstdio>
#include "logger_protocol.h"
int main() {
    const uint8_t vector[] = "123456789";
    if (adxl_logger::crc32(vector, 9) != 0xcbf43926U) return 1;
    adxl_logger::Fields fields = {};
    fields.type = adxl_logger::kSampleFrame;
    fields.flags = adxl_logger::SensorConfigured | adxl_logger::QualifiedDrdyTime;
    fields.sequence = 0xfedcba98U;
    fields.timeUs = UINT64_C(0x123456789abcdef0);
    fields.x = -32768; fields.y = 32767; fields.z = -1;
    fields.auxiliary = 0x80; fields.readLatencyUs = 17;
    fields.pwmWidthUs = 1420; fields.pwmSequence = 7;
    fields.pwmRiseTimeUs = UINT64_C(0xfedcba9876543210);
    fields.sensorOverruns = 17; fields.serviceGaps = 19; fields.queueDroppedFrames = 23;
    uint8_t output[adxl_logger::kFrameSize];
    adxl_logger::encode(fields, output);
    return std::fwrite(output, 1, sizeof output, stdout) == sizeof output ? 0 : 2;
}
'''
        with tempfile.TemporaryDirectory() as temporary:
            source, binary = Path(temporary) / "encode.cpp", Path(temporary) / "encode"
            source.write_text(program)
            subprocess.run(["g++", "-std=c++11", "-Wall", "-Wextra", "-Werror", "-I" + str(header),
                            str(source), "-o", str(binary)], check=True, capture_output=True)
            wire = subprocess.run([str(binary)], check=True, capture_output=True).stdout
        self.assertEqual(wire, frame(seq=0xFEDCBA98, stamp=0x123456789ABCDEF0,
                                     xyz=(-32768, 32767, -1), pwm_time=0xFEDCBA9876543210,
                                     overrun=17, gaps=19, drops=23))
        self.assertEqual(capture.Decoder().feed(wire)[0]["wire_x"], -32768)

    def test_offline_decode_retains_every_byte_and_valid_row_with_hashes(self):
        data = b"junk" + frame(kind=capture.STATUS, seq=1, flags=capture.USB_SESSION_START,
                                 xyz=(0xE5, 0x0D, 0x08), aux=0) + frame(seq=2) + frame(seq=3)[:20]
        with tempfile.TemporaryDirectory() as temporary:
            source, output = Path(temporary) / "original.bin", Path(temporary) / "decoded"
            source.write_bytes(data)
            report = capture.record_stream(capture.binary_chunks(source), output, {"mode": "offline_decode"})
            self.assertEqual((output / "capture.bin").read_bytes(), data)
            self.assertEqual(report["parser"]["valid_frames"], 2)
            self.assertEqual(report["parser"]["truncated_tail_bytes"], 20)
            self.assertEqual(report["files"]["capture.bin"]["sha256"], hashlib.sha256(data).hexdigest())
            saved = json.loads((output / "summary.json").read_text())
            self.assertEqual(saved["quality"]["records"], 2)
            with (output / "records.csv").open() as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual([r["type"] for r in rows], ["status", "sample"])
            self.assertEqual(rows[1]["host_rx_monotonic_ns"], "")
            self.assertEqual(rows[1]["raw_x"], "-123")
            with self.assertRaises(FileExistsError):
                capture.record_stream([], output)
            self.assertEqual((output / "capture.bin").read_bytes(), data)

    def test_read_failure_preserves_partial_capture_and_reports_failure(self):
        def broken():
            yield frame(), 123, 456
            raise OSError("disconnected")
        with tempfile.TemporaryDirectory() as temporary:
            with self.assertRaises(OSError):
                capture.record_stream(broken(), temporary)
            report = json.loads((Path(temporary) / "summary.json").read_text())
            self.assertEqual(report["quality"]["sample_rows"], 1)
            self.assertEqual(report["end_reason"], "error: OSError: disconnected")
            self.assertEqual((Path(temporary) / "capture.bin").read_bytes(), frame())

    def test_cli_rejects_invalid_duration_and_input_overwrite(self):
        bad_args = [
            ["--port", "dummy", "--output", "out", "--seconds", "nan"],
            ["--port", "dummy", "--output", "out", "--seconds", "0"],
            ["--input", "in.bin", "--output", "out", "--seconds", "1"],
            ["--input", "out/capture.bin", "--output", "out"],
        ]
        for argv in bad_args:
            with self.subTest(argv=argv), patch("sys.stderr", new=io.StringIO()), self.assertRaises(SystemExit):
                capture.arguments(argv)


if __name__ == "__main__":
    unittest.main()
