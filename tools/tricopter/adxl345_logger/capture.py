#!/usr/bin/env python3
"""Capture or decode the Teensy ADXL345 logger's version-1 binary stream.

Sample time_us is a Teensy DRDY ISR timestamp only when flag 1 is set.  Otherwise
it is the SPI read-start time, not an independently known measurement time.
USB arrival times are host diagnostics, not sensor capture timestamps.  PWM
fields always retain the wire values; pwm_fresh must qualify their use.
Type-3 diagnostics reuse wire slots for interrupt/register state.  They are
decoded into diagnostic_* columns, never into acceleration or PWM columns.
FIFO-coherent samples have a coherent XYZ tuple but no qualified DRDY time:
their time_us is SPI read start and the sensor sample's queue age is unknown.

Only the standard library is needed for --input.  --port additionally needs
pyserial.  This program sends no commands, never resets the board, and does not
control the servo or flight controller.
"""

import argparse
from collections import Counter
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import struct
import sys
import time
import zlib


MAGIC = b"ADXL"
VERSION = 1
FRAME_SIZE = 64
PAYLOAD = struct.Struct("<4sBBHIQhhhHIIIQIII")
CRC = struct.Struct("<I")
assert PAYLOAD.size == 60

SAMPLE = 1
STATUS = 2
DIAGNOSTIC = 3
SENSOR_CONFIGURED = 1 << 0
DRDY_QUALIFIED = 1 << 1
PWM_FRESH = 1 << 2
SENSOR_OVERRUN = 1 << 3
SERVICE_GAP = 1 << 4
POLL_FALLBACK = 1 << 5
USB_SESSION_START = 1 << 6
FIFO_COHERENT = 1 << 7
FAULT_CODES = {
    0: "OK",
    1: "BAD_DEVICE_ID",
    2: "CONFIG_READBACK_MISMATCH",
    3: "NO_SAMPLE_TIMEOUT",
    4: "DRDY_FALLBACK",
    5: "FIFO_DISCARD",
}

FIELDS = [
    "host_rx_monotonic_ns", "host_rx_unix_ns", "stream_epoch", "epoch_reason",
    "type", "record_seq", "seq_gap_before", "seq_discontinuity", "time_regression",
    "time_us", "time_source", "raw_x", "raw_y", "raw_z", "int_source",
    "devid", "bw_rate", "data_format", "fault_code", "fault_name", "flags",
    "sensor_configured", "drdy_timestamp_qualified", "pwm_fresh",
    "sensor_overrun", "service_gap", "poll_fallback", "usb_session_start",
    "read_latency_us", "pwm_width_us", "pwm_seq", "pwm_rise_time_us",
    "sensor_overrun_events", "service_gap_events", "queue_dropped_frames",
    "diagnostic_bits", "diagnostic_pin2_level", "diagnostic_pre_xyz_pin2_level",
    "diagnostic_post_xyz_pin2_level", "diagnostic_drdy_pending",
    "diagnostic_lc_nvic_enabled", "diagnostic_lc_nvic_pending", "diagnostic_primask",
    "diagnostic_lc_raw_state_available", "diagnostic_post_int_source_valid",
    "diagnostic_int_enable", "diagnostic_int_map", "diagnostic_pre_int_source",
    "diagnostic_post_int_source", "diagnostic_drdy_isr_sequence",
    "diagnostic_pin2_pcr", "diagnostic_portd_isfr", "diagnostic_last_isr_micros_low32",
    "fifo_coherent", "fifo_depth_before_read", "sample_time_source",
]
COUNTER_FIELDS = ("sensor_overrun_events", "service_gap_events", "queue_dropped_frames")


class Decoder:
    """Incremental bounded framing; CRC guards every accepted frame.

    Unknown versions/types with a correct CRC are counted and skipped as entire
    frames.  Damaged frames resynchronize one byte at a time.  A sensor-overrun
    event is not an exact count of lost samples.
    """

    def __init__(self):
        self.buffer = bytearray()
        self.stats = dict(valid_frames=0, crc_errors=0, unsupported_versions=0,
                          unsupported_types=0, discarded_bytes=0, truncated_tail_bytes=0)
        self.finished = False

    def feed(self, data):
        if self.finished:
            raise ValueError("Decoder already finished")
        self.buffer.extend(data)
        records = []
        while self.buffer:
            offset = self.buffer.find(MAGIC)
            if offset < 0:
                # Retain only an actual magic prefix across chunk boundaries.
                keep = 0
                for length in range(1, min(len(MAGIC), len(self.buffer) + 1)):
                    if self.buffer[-length:] == MAGIC[:length]:
                        keep = length
                discard = len(self.buffer) - keep
                self.stats["discarded_bytes"] += discard
                del self.buffer[:discard]
                break
            if offset:
                self.stats["discarded_bytes"] += offset
                del self.buffer[:offset]
            if len(self.buffer) < FRAME_SIZE:
                break
            frame = self.buffer[:FRAME_SIZE]
            if zlib.crc32(frame[:60]) != CRC.unpack_from(frame, 60)[0]:
                self.stats["crc_errors"] += 1
                self.stats["discarded_bytes"] += 1
                del self.buffer[0]
                continue
            values = PAYLOAD.unpack_from(frame)
            del self.buffer[:FRAME_SIZE]
            if values[1] != VERSION:
                self.stats["unsupported_versions"] += 1
                continue
            if values[2] not in (SAMPLE, STATUS, DIAGNOSTIC):
                self.stats["unsupported_types"] += 1
                continue
            keys = ("magic", "version", "record_type", "flags", "record_seq", "time_us",
                    "wire_x", "wire_y", "wire_z", "aux", "read_latency_us", "pwm_width_us",
                    "pwm_seq", "pwm_rise_time_us", *COUNTER_FIELDS)
            records.append(dict(zip(keys, values)))
            self.stats["valid_frames"] += 1
        return records

    def finish(self):
        if not self.finished:
            self.stats["truncated_tail_bytes"] = len(self.buffer)
            self.buffer.clear()
            self.finished = True
        return dict(self.stats)


class Tracker:
    """Preserve every decoded row and describe sequence/clock discontinuities.

    Epochs separate explicit USB-session starts or backward sequence resets.
    Clock regression is recorded independently: a backward timestamp alone is
    not asserted to prove a reboot.  Forward sequence gaps count missing logger
    records, not missing sensor conversions, and do not identify their cause.
    """

    def __init__(self):
        self.previous = None
        self.epochs = []
        self.counts = Counter()
        self.faults = Counter()

    def row(self, record, host_monotonic_ns=None, host_unix_ns=None):
        flags = record["flags"]
        sample = record["record_type"] == SAMPLE
        status = record["record_type"] == STATUS
        diagnostic = record["record_type"] == DIAGNOSTIC
        fifo = sample and bool(flags & FIFO_COHERENT)
        fifo_depth = (record["aux"] >> 8) & 0xFF
        fifo_bad_depth = fifo and not 1 <= fifo_depth <= 31
        fifo_time_conflict = fifo and bool(flags & DRDY_QUALIFIED)
        fifo_coherent = fifo and not fifo_bad_depth and not fifo_time_conflict
        drdy_qualified = bool(flags & DRDY_QUALIFIED) and not fifo
        session_start = not diagnostic and bool(flags & USB_SESSION_START)
        gap, discontinuity, regression = 0, False, False
        reason = ""
        if self.previous is None:
            reason = "first_frame"
        else:
            regression = record["time_us"] < self.previous["time_us"]
            delta = (record["record_seq"] - self.previous["record_seq"]) & 0xFFFFFFFF
            if session_start:
                reason = "usb_session_start"
            elif delta == 0 or delta >= 0x80000000:
                discontinuity = True
                reason = "sequence_reset_or_duplicate"
            else:
                gap = delta - 1
        if reason:
            self.epochs.append({
                "epoch": len(self.epochs), "reason": reason,
                "first_record_seq": record["record_seq"], "first_time_us": record["time_us"],
                "first_counters": {k: record[k] for k in COUNTER_FIELDS}, "records": 0,
            })
        epoch = self.epochs[-1]
        epoch["records"] += 1
        epoch["last_record_seq"] = record["record_seq"]
        epoch["last_time_us"] = record["time_us"]
        epoch["last_counters"] = {k: record[k] for k in COUNTER_FIELDS}
        self.counts["records"] += 1
        self.counts[{SAMPLE: "sample_rows", STATUS: "status_rows", DIAGNOSTIC: "diagnostic_rows"}[record["record_type"]]] += 1
        self.counts["missing_record_sequences"] += gap
        self.counts["sequence_discontinuities"] += int(discontinuity)
        self.counts["timestamp_regressions"] += int(regression)
        self.counts["usb_session_start_rows"] += int(session_start)
        if flags & ~0xFF:
            self.counts["rows_with_unknown_flag_bits"] += 1
        if sample:
            self.counts["qualified_sample_timestamps"] += int(drdy_qualified)
            self.counts["fifo_sample_rows"] += int(fifo)
            self.counts["fifo_coherent_sample_rows"] += int(fifo_coherent)
            self.counts["fifo_invalid_depth_rows"] += int(fifo_bad_depth)
            self.counts["fifo_timestamp_conflict_rows"] += int(fifo_time_conflict)
            for name, mask in [("configured_sample_rows", SENSOR_CONFIGURED),
                               ("fresh_pwm_sample_rows", PWM_FRESH),
                               ("sensor_overrun_sample_rows", SENSOR_OVERRUN),
                               ("service_gap_sample_rows", SERVICE_GAP),
                               ("poll_fallback_sample_rows", POLL_FALLBACK)]:
                self.counts[name] += int(bool(flags & mask))
        elif status:
            self.faults[str(record["aux"])] += 1
        row = {key: "" for key in FIELDS}
        row.update(
            host_rx_monotonic_ns="" if host_monotonic_ns is None else host_monotonic_ns,
            host_rx_unix_ns="" if host_unix_ns is None else host_unix_ns,
            stream_epoch=epoch["epoch"], epoch_reason=reason,
            type={SAMPLE: "sample", STATUS: "status", DIAGNOSTIC: "diagnostic"}[record["record_type"]],
            record_seq=record["record_seq"],
            seq_gap_before=gap, seq_discontinuity=int(discontinuity), time_regression=int(regression),
            time_us=record["time_us"],
            time_source=("fifo_spi_read_start" if fifo else "drdy_edge" if drdy_qualified else "spi_read_start") if sample
                        else ("status_clock" if status else "diagnostic_clock"),
            flags=f"0x{flags:04x}",
            **{key: record[key] for key in COUNTER_FIELDS},
        )
        for key, mask in [("sensor_configured", SENSOR_CONFIGURED), ("drdy_timestamp_qualified", DRDY_QUALIFIED),
                          ("pwm_fresh", PWM_FRESH), ("sensor_overrun", SENSOR_OVERRUN),
                          ("service_gap", SERVICE_GAP), ("poll_fallback", POLL_FALLBACK),
                          ("usb_session_start", USB_SESSION_START)]:
            if not diagnostic or key == "sensor_configured":
                row[key] = int(bool(flags & mask))
        if not diagnostic:
            row.update(read_latency_us=record["read_latency_us"], pwm_width_us=record["pwm_width_us"],
                       pwm_seq=record["pwm_seq"], pwm_rise_time_us=record["pwm_rise_time_us"])
        if sample:
            row.update(raw_x=record["wire_x"], raw_y=record["wire_y"], raw_z=record["wire_z"],
                       int_source=f"0x{record['aux'] & (0xFF if fifo else 0xFFFF):04x}",
                       drdy_timestamp_qualified=int(drdy_qualified), fifo_coherent=int(fifo_coherent),
                       fifo_depth_before_read=fifo_depth if fifo else "",
                       sample_time_source=row["time_source"])
        elif status:
            row.update(devid=record["wire_x"], bw_rate=record["wire_y"], data_format=record["wire_z"],
                       fault_code=record["aux"], fault_name=FAULT_CODES.get(record["aux"], "UNKNOWN"))
        else:
            bits = record["wire_x"] & 0xFFFF
            row.update(diagnostic_bits=f"0x{bits:04x}",
                       diagnostic_lc_raw_state_available=int(bool(bits & (1 << 7))),
                       diagnostic_post_int_source_valid=int(bool(bits & (1 << 8))),
                       diagnostic_int_enable=f"0x{record['wire_y']:02x}",
                       diagnostic_int_map=f"0x{record['wire_z']:02x}",
                       diagnostic_pre_int_source=f"0x{record['aux'] & 0xFF:02x}",
                       diagnostic_drdy_isr_sequence=record["read_latency_us"],
                       diagnostic_last_isr_micros_low32=record["pwm_rise_time_us"])
            for key, bit in [("diagnostic_pin2_level", 0), ("diagnostic_pre_xyz_pin2_level", 1),
                             ("diagnostic_post_xyz_pin2_level", 2), ("diagnostic_drdy_pending", 3),
                             ("diagnostic_primask", 6)]:
                row[key] = int(bool(bits & (1 << bit)))
            # Raw LC registers and NVIC state are unavailable on other MCUs;
            # zero in an unqualified wire slot must not become a known value.
            if bits & (1 << 7):
                row.update(diagnostic_lc_nvic_enabled=int(bool(bits & (1 << 4))),
                           diagnostic_lc_nvic_pending=int(bool(bits & (1 << 5))),
                           diagnostic_pin2_pcr=f"0x{record['pwm_width_us']:08x}",
                           diagnostic_portd_isfr=f"0x{record['pwm_seq']:08x}")
            if bits & (1 << 8):
                row["diagnostic_post_int_source"] = f"0x{(record['aux'] >> 8) & 0xFF:02x}"
        self.previous = record
        return row

    def summary(self):
        keys = ["records", "sample_rows", "status_rows", "diagnostic_rows", "missing_record_sequences",
                "sequence_discontinuities", "timestamp_regressions", "usb_session_start_rows",
                "rows_with_unknown_flag_bits", "configured_sample_rows", "qualified_sample_timestamps",
                "fresh_pwm_sample_rows", "sensor_overrun_sample_rows", "service_gap_sample_rows",
                "poll_fallback_sample_rows", "fifo_sample_rows", "fifo_coherent_sample_rows",
                "fifo_invalid_depth_rows", "fifo_timestamp_conflict_rows"]
        return {**{key: self.counts[key] for key in keys}, "status_fault_code_counts": dict(self.faults),
                "epochs": self.epochs,
                "loss_interpretation": "Sequence gaps are missing logger records; overrun/service-gap counters are events, not exact lost sensor sample counts. Counter snapshots are cumulative within an MCU boot, not summed across rows."}


def file_sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        while chunk := handle.read(65536):
            digest.update(chunk)
    return digest.hexdigest()


def source_hashes():
    here = Path(__file__).resolve().parent
    names = ["capture.py", "platformio.ini", "src/main.cpp", "include/logger_protocol.h"]
    return {name: file_sha256(here / name) for name in names if (here / name).is_file()}


def record_stream(chunks, output, metadata=None):
    """Write raw bytes, all valid frames and summary; chunks carry host times.

    Each item is (bytes, optional host monotonic ns, optional host Unix ns).
    Memory use is bounded by chunk size plus parser state, regardless of run
    length.  Existing capture artifacts are never overwritten.
    """
    folder = Path(output)
    folder.mkdir(parents=True, exist_ok=True)
    paths = {name: folder / name for name in ("capture.bin", "records.csv", "summary.json")}
    if any(path.exists() for path in paths.values()):
        raise FileExistsError("Output already contains capture.bin, records.csv or summary.json")
    started = datetime.now(timezone.utc).isoformat()
    sources = source_hashes()
    decoder, tracker = Decoder(), Tracker()
    raw_bytes, ended = 0, "input_exhausted"
    try:
        with paths["capture.bin"].open("xb") as raw, paths["records.csv"].open("x", newline="") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=FIELDS)
            writer.writeheader()
            try:
                for data, mono, unix in chunks:
                    raw.write(data)
                    # The basket health monitor reads this file while the
                    # capture is active. Expose each received chunk rather
                    # than waiting for Python's potentially large file buffer.
                    # This is visibility to readers, not an fsync guarantee.
                    raw.flush()
                    raw_bytes += len(data)
                    for record in decoder.feed(data):
                        writer.writerow(tracker.row(record, mono, unix))
            except KeyboardInterrupt:
                ended = "keyboard_interrupt"
            except Exception as error:
                ended = f"error: {type(error).__name__}: {error}"
                raise
            finally:
                raw.flush()
                csv_file.flush()
    finally:
        # Preserve a diagnostic manifest even on unplug/read failure.  It does
        # not turn an exception into a successful acquisition.
        report = {
            "schema_version": 1, "protocol_version": VERSION, "frame_size": FRAME_SIZE,
            "started_utc": started, "finished_utc": datetime.now(timezone.utc).isoformat(),
            "end_reason": ended, "raw_bytes": raw_bytes,
            "parser": decoder.finish(), "quality": tracker.summary(),
            "source_sha256_at_start": sources,
            "source_files_unchanged_during_capture": sources == source_hashes(),
            "metadata": metadata or {},
            "files": {name: {"sha256": file_sha256(path), "bytes": path.stat().st_size}
                      for name, path in paths.items() if name != "summary.json" and path.exists()},
            "measurement_contract": {
                "raw_axes": "Signed uncalibrated ADXL345 counts, no angle or gravity correction",
                "sample_time": "DRDY ISR time only with drdy_timestamp_qualified=1; otherwise SPI read-start time. FIFO samples use fifo_spi_read_start and have unmeasured queue/sample age. Coherent XYZ does not establish a precise measurement timestamp. Sensor internal delay and ISR latency remain uncalibrated.",
                "fifo_coherence": "fifo_coherent requires sample flag 7, depth 1..31 and no qualified-DRDY flag. Depth 0/full32/out-of-range or simultaneous FIFO/DRDY claims are retained but rejected as coherent; their quality counters are explicit. Other quality flags remain separate. FIFO aux low byte is INT_SOURCE and high byte is FIFO depth before the pop.",
                "host_time": "USB receive-chunk arrival only; absent during offline decoding",
                "pwm": "Completed input pulse rise timestamp/width; use only with pwm_fresh=1",
                "position_source": "FC continues logging PB0-PC0; this logger does not measure or replace that source",
                "source_fingerprints": "Host source files at capture time; not proof that this firmware binary was flashed",
                "diagnostics": "Type 3 carries interrupt/register snapshots in diagnostic_* columns only; these are not acceleration or PWM observations. LC raw register/NVIC columns require diagnostic_lc_raw_state_available=1; post-XYZ INT_SOURCE requires diagnostic_post_int_source_valid=1.",
            },
        }
        with paths["summary.json"].open("x") as handle:
            json.dump(report, handle, indent=2)
            handle.write("\n")
    return report


def binary_chunks(path):
    with Path(path).open("rb") as handle:
        while chunk := handle.read(65536):
            yield chunk, None, None


def serial_chunks(port, seconds=None, baud=115200):
    try:
        import serial
    except ImportError as error:
        raise RuntimeError("USB capture requires pyserial: python3 -m pip install pyserial") from error
    # The baud setting is a USB-CDC convention here.  No 1200-baud reset, DTR
    # pulse or application command is sent.  An open native-USB session asserts
    # DTR so the firmware knows it has a reader.
    with serial.Serial(port=port, baudrate=baud, timeout=0.05) as connection:
        started = time.monotonic()
        while seconds is None or time.monotonic() - started < seconds:
            data = connection.read(min(max(connection.in_waiting, 1), 65536))
            if data:
                yield data, time.monotonic_ns(), time.time_ns()


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--port", help="Explicit Teensy native USB serial port, e.g. /dev/ttyACM1")
    source.add_argument("--input", type=Path, help="Decode an existing raw capture.bin without hardware")
    parser.add_argument("--output", type=Path, required=True, help="Capture directory; existing artifacts are never overwritten")
    parser.add_argument("--seconds", type=float, help="USB acquisition duration; omit to stop with Ctrl-C")
    args = parser.parse_args(argv)
    if args.seconds is not None and (not math.isfinite(args.seconds) or args.seconds <= 0):
        parser.error("--seconds must be positive and finite")
    if args.input is not None and args.seconds is not None:
        parser.error("--seconds is only valid with --port")
    if args.input is not None and args.input.resolve() in {
            (args.output / name).resolve() for name in ("capture.bin", "records.csv", "summary.json")}:
        parser.error("Input must be outside the output artifact paths")
    return args


def main(argv=None):
    args = arguments(argv)
    metadata = {"argv": list(sys.argv[1:] if argv is None else argv), "python": sys.version,
                "mode": "usb" if args.port else "offline_decode", "port": args.port,
                "input": str(args.input.resolve()) if args.input else None, "seconds": args.seconds}
    chunks = serial_chunks(args.port, args.seconds) if args.port else binary_chunks(args.input)
    try:
        report = record_stream(chunks, args.output, metadata)
    except (OSError, RuntimeError) as error:
        print(f"Capture failed: {error}", file=sys.stderr)
        return 1
    print(json.dumps({"output": str(args.output.resolve()), "end_reason": report["end_reason"],
                      "parser": report["parser"], "quality": {key: value for key, value in report["quality"].items()
                                                              if key not in ("epochs", "loss_interpretation")}}, indent=2))
    return 0 if report["quality"]["sample_rows"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
