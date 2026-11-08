"""SITL utilities connecting over loopback. No serial/physical FC access."""
from __future__ import annotations
import argparse
import fcntl
import hashlib
import json
import os
import signal
from pathlib import Path
import socket
import struct
import subprocess
import threading
import time

ROOT = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parent
ARTIFACTS = ROOT / '.local-artifacts/tricopter/gazebo-sitl'
RC_PACKET = struct.Struct('<d16H')
RAW_PWM_PACKET = struct.Struct('<H2x16f')
FDM_PACKET = struct.Struct('<18d')
ADC_PACKET = struct.Struct('<IHHdIIHHI')


def start_child(command, **kwargs):
    """Keep launcher, Gazebo GUI/server and their descendants in one owned group."""
    process = subprocess.Popen(command, start_new_session=True, **kwargs)
    process._tricopter_process_group = process.pid
    return process


def stop_process(process: subprocess.Popen | None) -> None:
    if process is None:
        return
    group = getattr(process, '_tricopter_process_group', None)
    if group is None and process.poll() is not None:
        return
    try:
        if group: os.killpg(group, signal.SIGTERM)
        else: process.terminate()
    except ProcessLookupError:
        return
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        process.poll()  # Reap the launcher even when it exits before its GUI.
        try:
            if group: os.killpg(group, 0)
            elif process.poll() is not None: return
        except ProcessLookupError:
            return
        time.sleep(.05)
    try:
        if group: os.killpg(group, signal.SIGKILL)
        else: process.kill()
    except ProcessLookupError:
        pass
    try: process.wait(timeout=1)
    except subprocess.TimeoutExpired: pass


class SessionLock:
    """One shared port set; prevent accidental simultaneous stock/modified runs."""
    def __init__(self):
        ARTIFACTS.mkdir(parents=True, exist_ok=True)
        self.stream = (ARTIFACTS / 'session.lock').open('a+')
        try:
            fcntl.flock(self.stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            self.stream.close()
            raise RuntimeError('Another tricopter SITL session holds the shared ports') from None
    def close(self):
        if not self.stream.closed:
            fcntl.flock(self.stream, fcntl.LOCK_UN)
            self.stream.close()
    def __enter__(self): return self
    def __exit__(self, *_): self.close()


class Msp:
    def __init__(self, sock: socket.socket):
        self.sock = sock
        self.buffer = bytearray()
        self.lock = threading.Lock()
    @classmethod
    def connect(cls, timeout=12, port=5761):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                return cls(socket.create_connection(('127.0.0.1', port), timeout=.25))
            except OSError:
                time.sleep(.1)
        raise TimeoutError(f'SITL MSP TCP 127.0.0.1:{port} did not open')
    def close(self): self.sock.close()
    def request(self, command: int, payload: bytes = b'', timeout=2) -> bytes:
        if not 0 <= command <= 255 or len(payload) > 255:
            raise ValueError('MSP v1 command/payload too large')
        body = bytes([len(payload), command]) + payload
        checksum = 0
        for byte in body: checksum ^= byte
        with self.lock:
            self.sock.sendall(b'$M<' + body + bytes([checksum]))
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                while len(self.buffer) >= 6:
                    marker = self.buffer.find(b'$M')
                    if marker < 0:
                        self.buffer[:] = self.buffer[-2:]
                        break
                    if marker: del self.buffer[:marker]
                    if len(self.buffer) < 6: break
                    if self.buffer[2] not in (ord('>'), ord('!')):
                        del self.buffer[0]; continue
                    size, cmd = self.buffer[3:5]
                    if len(self.buffer) < size + 6: break
                    frame = bytes(self.buffer[:size+6]); del self.buffer[:size+6]
                    check = 0
                    for byte in frame[3:]: check ^= byte
                    if check: raise RuntimeError('Bad MSP reply checksum')
                    if cmd != command: continue
                    if frame[2] == ord('!'): raise RuntimeError(f'MSP rejected command {cmd}')
                    return frame[5:-1]
                self.sock.settimeout(max(.01, deadline - time.monotonic()))
                try: data = self.sock.recv(4096)
                except socket.timeout: break
                if not data: raise ConnectionError('SITL closed MSP TCP')
                self.buffer.extend(data)
        raise TimeoutError(f'MSP command {command} timed out')
    def status(self):
        boxes = self.request(119)
        payload = self.request(150)
        modes = struct.unpack_from('<I', payload, 6)[0]
        offset = 16 + payload[15]
        flags = struct.unpack_from('<I', payload, offset + 1)[0]
        armed = bool(modes & (1 << boxes.index(0))) if 0 in boxes else False
        return {'armed': armed, 'arming_disable_flags': flags, 'mode_mask': modes}


class RcFeed:
    """Deterministic input for automated tests/replay; interactive runs use evdev."""
    def __init__(self, rate=50):
        self.channels = [1500, 1500, 1000, 1500] + [1000]*12
        self.rate = rate
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.thread = threading.Thread(target=self._run, daemon=True)
    def start(self): self.thread.start()
    def set(self, channel, value):
        if not 0 <= channel < 16 or not 1000 <= value <= 2000: raise ValueError('RC channel/value')
        with self.lock: self.channels[channel] = int(value)
    def _run(self):
        origin = time.monotonic()
        while not self.stop.is_set():
            with self.lock: channels = self.channels.copy()
            self.sock.sendto(RC_PACKET.pack(time.monotonic()-origin, *channels), ('127.0.0.1',9004))
            self.stop.wait(1/self.rate)
    def close(self):
        self.stop.set()
        if self.thread.is_alive(): self.thread.join(timeout=2)
        self.sock.close()


def provision(binary: Path, directory: Path, variant: str, extra_config: Path | None = None) -> Path:
    directory.mkdir(parents=True, exist_ok=True)
    eeprom = directory / 'eeprom.bin'
    if eeprom.exists(): raise FileExistsError(f'Refusing to replace existing session EEPROM: {eeprom}')
    text = (HERE/'configs/common.cli').read_text()
    if variant == 'modified': text += '\n' + (HERE/'configs/modified.cli').read_text()
    if extra_config: text += '\n' + extra_config.read_text()
    config = directory/'firmware.cli'; config.write_text(text+'\n')
    result = subprocess.run([str(binary.resolve()), '--config', str(config.resolve())], cwd=directory,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=45)
    (directory/'provision.log').write_text(result.stdout)
    if result.returncode or not eeprom.exists() or any(mark in result.stdout for mark in ['###ERROR', 'Unknown command', 'Invalid name', 'Invalid value', 'Parse error']):
        raise RuntimeError(f'SITL configuration failed; inspect {directory/"provision.log"}')
    return config


def start_firmware(binary: Path, directory: Path):
    stream = (directory/'firmware.log').open('w')
    process = start_child(['stdbuf','-oL',str(binary.resolve())], cwd=directory, stdout=stream, stderr=subprocess.STDOUT)
    stream.close()
    return process


def fresh_run_directory(artifacts: Path, variant: str, label='interactive') -> Path:
    stamp = time.strftime('%Y%m%dT%H%M%S', time.gmtime())
    directory = artifacts/'runs'/f'{stamp}-{variant}-{label}-{os.getpid()}'
    directory.mkdir(parents=True, exist_ok=False)
    return directory
