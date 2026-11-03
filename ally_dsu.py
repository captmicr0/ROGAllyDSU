#!/usr/bin/env python3
"""ROG Ally DSU (Cemuhook protocol v1001) motion server.

Python 3.10+; Windows requires `pip install winsdk`.
Use --demo to exercise networking without a Windows motion sensor.
"""
from __future__ import annotations

import argparse
import logging
import math
import multiprocessing as mp
import os
import select
import socket
import struct
import threading
import time
import zlib
from dataclasses import dataclass

LOG = logging.getLogger('ally_dsu')
VERSION = 1001
PORT = 26760
VERSION_MSG = 0x100000
INFO_MSG = 0x100001
DATA_MSG = 0x100002
MAC = bytes.fromhex('02 52 4f 47 41 00')  # stable, locally administered pseudo-MAC


@dataclass(frozen=True)
class Motion:
    ax: float = 0.0
    ay: float = 0.0
    az: float = 1.0
    pitch: float = 0.0
    yaw: float = 0.0
    roll: float = 0.0


def safe_float(value: float) -> float:
    value = float(value)
    return value if math.isfinite(value) else 0.0


def packet(magic: bytes, sender_id: int, message: int, payload: bytes = b'') -> bytes:
    body = struct.pack('<I', message) + payload
    data = bytearray(struct.pack('<4sHHII', magic, VERSION, len(body), 0, sender_id) + body)
    struct.pack_into('<I', data, 8, zlib.crc32(data) & 0xffffffff)
    return bytes(data)


def parse_packet(data: bytes, *, expect_magic: bytes = b'DSUC'):
    if len(data) < 20 or data[:4] != expect_magic:
        return None
    version, length, checksum, sender_id = struct.unpack_from('<HHII', data, 4)
    if version != VERSION or length < 4 or len(data) < 16 + length:
        return None
    data = bytearray(data[:16 + length])
    struct.pack_into('<I', data, 8, 0)
    if zlib.crc32(data) & 0xffffffff != checksum:
        return None
    message = struct.unpack_from('<I', data, 16)[0]
    return sender_id, message, bytes(data[20:])


def controller_prefix(slot: int = 0) -> bytes:
    if slot == 0:
        return struct.pack('<BBBB6sB', 0, 2, 2, 1, MAC, 0x05)
    return bytes((slot,)) + bytes(10)


def info_packet(server_id: int, slot: int) -> bytes:
    return packet(b'DSUS', server_id, INFO_MSG, controller_prefix(slot) + b'\x00')


def motion_packet(server_id: int, sequence: int, timestamp_us: int, motion: Motion) -> bytes:
    # 11 controller info + 1 connected + 4 seq + 4 buttons + 4 sticks +
    # 12 analog buttons + 12 touch + 8 timestamp + 24 motion = 80 payload bytes.
    payload = bytearray(controller_prefix())
    payload += struct.pack('<BI', 1, sequence & 0xffffffff)
    payload += b'\x00' * 4                         # digital masks, PS/home, touch click
    payload += bytes([128] * 4)                    # sticks at rest
    payload += b'\x00' * 12                       # analog D-pad/face/shoulders
    payload += b'\x00' * 12                       # two 6-byte touch records
    payload += struct.pack('<Q', timestamp_us & 0xffffffffffffffff)
    payload += struct.pack('<6f', *(safe_float(x) for x in (
        motion.ax, motion.ay, motion.az, motion.pitch, motion.yaw, motion.roll
    )))
    assert len(payload) == 80, len(payload)
    result = packet(b'DSUS', server_id, DATA_MSG, payload)
    assert len(result) == 100
    return result


def _sensor_worker(shared, stop, hz, accel, gyro, sensitivity, accel_sensitivity, axes):
    """Separate OS PROCESS: blocked WinRT calls cannot stall the UDP server."""
    try:
        from winsdk.windows.devices.sensors import Gyrometer, Accelerometer
        g = Gyrometer.get_default() if gyro else None
        a = Accelerometer.get_default() if accel else None
        if gyro and g is None:
            raise RuntimeError('Windows does not expose a gyrometer')
        if accel and a is None:
            raise RuntimeError('Windows does not expose an accelerometer')
        interval = 1.0 / hz
        for name, sensor in (('gyrometer', g), ('accelerometer', a)):
            if sensor is not None:
                desired = max(1, round(1000 / hz))
                minimum = int(sensor.minimum_report_interval)
                sensor.report_interval = max(desired, minimum)
                LOG.info('%s: min interval %d ms, requested %d ms, actual %d ms',
                         name, minimum, desired, sensor.report_interval)
        raw_a = (0.0, 0.0, 0.0)
        raw_g = (0.0, 0.0, 0.0)
        next_tick = time.perf_counter()
        while not stop.is_set():
            try:
                fresh = False
                if a is not None:
                    r = a.get_current_reading()
                    if r is not None:
                        raw_a = (r.acceleration_x, r.acceleration_y, r.acceleration_z)
                        fresh = True
                if g is not None:
                    r = g.get_current_reading()
                    if r is not None:
                        raw_g = (r.angular_velocity_x, r.angular_velocity_y, r.angular_velocity_z)
                        fresh = True
                if fresh:
                    ax, ay, az = raw_a
                    gx, gy, gz = raw_g
                    if axes == 'original':
                        ax, ay, az = ay, az, ax
                    values = [safe_float(v) * accel_sensitivity for v in (ax, ay, az)]
                    values += [safe_float(v) * sensitivity for v in (gx, gy, gz)]
                    # One short critical section, never held across a WinRT call.
                    with shared.get_lock():
                        for i, v in enumerate(values):
                            shared[i] = v
                        shared[6] += 1  # successful samples
                        shared[7] = time.monotonic()  # most recent read
            except Exception:
                LOG.exception('Windows sensor read error')
                stop.wait(0.2)
            next_tick += interval
            delay = next_tick - time.perf_counter()
            if delay < -interval:
                next_tick = time.perf_counter()
                delay = interval
            stop.wait(max(0, delay))
    except Exception:
        LOG.exception('Sensor worker could not start')
        raise
    finally:
        for sensor in (locals().get('g'), locals().get('a')):
            if sensor is not None:
                try:
                    sensor.report_interval = 0
                except Exception:
                    pass


class WindowsSensors:
    """WinRT runs in its own PROCESS so a blocking sensor poll cannot stall UDP."""

    def __init__(self, hz=125, *, accel=True, gyro=True,
                 sensitivity=1.0, accel_sensitivity=1.0, axes='original'):
        self.ctx = mp.get_context('spawn')
        # Six floats, successful reading count, last reading monotonic time.
        self.shared = self.ctx.Array('d', [0.0] * 8, lock=True)
        self.stop_event = self.ctx.Event()
        self.options = (hz, accel, gyro, sensitivity, accel_sensitivity, axes)
        self.process = None

    def start(self):
        self.process = self.ctx.Process(
            target=_sensor_worker,
            args=(self.shared, self.stop_event, *self.options),
            name='AllyWinRTSensorWorker', daemon=True)
        self.process.start()
        LOG.info('WinRT isolated in separate process PID=%s', self.process.pid)

    def get_motion(self):
        with self.shared.get_lock():
            vals = tuple(self.shared[:6])
        return Motion(*vals)

    def status(self):
        with self.shared.get_lock():
            samples, timestamp = self.shared[6], self.shared[7]
        if self.process is not None and not self.process.is_alive():
            LOG.error('WinRT sensor process exited (exit code %s); network is still running',
                      self.process.exitcode)
        return int(samples), (time.monotonic() - timestamp if timestamp else None)

    def stop(self):
        self.stop_event.set()
        if self.process is not None:
            self.process.join(timeout=1.5)
            if self.process.is_alive():
                LOG.warning('WinRT sensor worker hung; terminating it')
                self.process.terminate()
                self.process.join(timeout=1)


class DemoSensors:
    def start(self):
        pass

    def get_motion(self):
        elapsed = time.monotonic()
        return Motion(0, 0, 1, math.sin(elapsed) * 30, math.cos(elapsed * .7) * 20, 0)

    def status(self):
        return 1, 0.0

    def stop(self):
        pass


class DSUServer:
    def __init__(self, sensors, *, host='127.0.0.1', port=PORT, send_hz=125,
                 timeout=15.0, status_interval=10.0):
        self.sensors = sensors
        self.send_interval = 1 / send_hz
        self.timeout = timeout
        self.status_interval = status_interval
        self.server_id = int.from_bytes(os.urandom(4), 'little')
        self.clients = {}  # (IP, port) -> (last_seen_monotonic, client_id)
        self._stop = threading.Event()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, port))
        self.sock.setblocking(False)
        self.address = self.sock.getsockname()
        self._sequence = 0
        self._boot_us = time.monotonic_ns() // 1000
        self._last_status = time.monotonic()
        self._send_count = 0
        self._last_report_send = 0
        self._last_report_time = time.monotonic()

    def stop(self):
        self._stop.set()

    def _send(self, addr, data):
        try:
            self.sock.sendto(data, addr)
        except OSError as exc:
            LOG.warning('UDP send to %s failed: %s', addr, exc)

    def _handle(self, data, addr):
        parsed = parse_packet(data)
        if parsed is None:
            LOG.debug('Invalid DSU request from %s, %d bytes', addr, len(data))
            return
        sender_id, kind, payload = parsed
        if kind == VERSION_MSG:
            self._send(addr, packet(b'DSUS', self.server_id, VERSION_MSG, struct.pack('<H', VERSION)))
        elif kind == INFO_MSG:
            if len(payload) < 4:
                return
            count, = struct.unpack_from('<i', payload)
            if not 1 <= count <= 4 or len(payload) < 4 + count:
                return
            for slot in payload[4:4 + count]:
                if slot < 4:
                    self._send(addr, info_packet(self.server_id, slot))
        elif kind == DATA_MSG:
            if len(payload) < 8:
                return
            flags, slot = payload[0], payload[1]
            requested_mac = payload[2:8]
            # All, slot-specific or MAC-specific registration, including both flags.
            if flags & ~3:
                return
            valid = (flags == 0 or
                     ((not flags & 1 or slot == 0) and
                      (not flags & 2 or requested_mac == MAC)))
            if not valid:
                return
            if addr not in self.clients:
                LOG.info('Subscribed %s client_id=%08x flags=%d slot=%d', addr, sender_id, flags, slot)
            self.clients[addr] = (time.monotonic(), sender_id)
        # Other protocol extensions are not supported, intentionally.

    def serve(self):
        LOG.info('DSU v%d listening on %s:%d (server ID %08x)', VERSION,
                 self.address[0], self.address[1], self.server_id)
        next_send = time.perf_counter()
        try:
            while not self._stop.is_set():
                now = time.monotonic()
                for addr, (seen, _) in list(self.clients.items()):
                    if now - seen > self.timeout:
                        del self.clients[addr]
                        LOG.info('Subscription expired: %s', addr)

                delay = max(0, min(.05, next_send - time.perf_counter()))
                try:
                    ready, _, _ = select.select([self.sock], [], [], delay)
                    if ready:
                        # Drain a bounded number of incoming requests so transmitting
                        # doesn't starve under heavy request traffic.
                        for _ in range(64):
                            try:
                                data, addr = self.sock.recvfrom(2048)
                            except BlockingIOError:
                                break
                            self._handle(data, addr)
                except OSError as exc:
                    if not self._stop.is_set():
                        LOG.error('Socket error: %s', exc)
                    break

                now_perf = time.perf_counter()
                if now_perf >= next_send:
                    if self.clients:
                        # No motion-change check: transmit even if sensors are stationary.
                        sample = self.sensors.get_motion()
                        timestamp_us = time.monotonic_ns() // 1000 - self._boot_us
                        outgoing = motion_packet(self.server_id, self._sequence, timestamp_us, sample)
                        for addr in tuple(self.clients):
                            self._send(addr, outgoing)
                            self._send_count += 1
                    self._sequence = (self._sequence + 1) & 0xffffffff
                    next_send += self.send_interval
                    if now_perf - next_send > self.send_interval:
                        next_send = now_perf + self.send_interval

                if now - self._last_status >= self.status_interval:
                    samples, age = self.sensors.status()
                    report_elapsed = max(0.001, now - self._last_report_time)
                    observed_rate = (self._send_count - self._last_report_send) / report_elapsed
                    LOG.info('Clients=%d UDP packets=%d actual UDP rate=%.1f packets/s sensor reads=%d sensor age=%s',
                             len(self.clients), self._send_count, observed_rate, samples,
                             'n/a' if age is None else f'{age:.2f}s')
                    self._last_report_send = self._send_count
                    self._last_report_time = now
                    self._last_status = now
        finally:
            self.sock.close()


def main():
    ap = argparse.ArgumentParser(description='ROG Ally DSU gyro server (Cemuhook v1001)')
    ap.add_argument('--host', default='127.0.0.1', help='Bind address (default loopback; 0.0.0.0 for LAN)')
    ap.add_argument('--port', type=int, default=PORT)
    ap.add_argument('--send-rate', type=int, default=125, help='Motion packets per second (default 125)')
    ap.add_argument('--poll-rate', type=int, default=125, help='Windows sensor polls per second (default 125)')
    ap.add_argument('--timeout', type=float, default=15, help='Subscription timeout seconds (default 15)')
    ap.add_argument('--status-interval', type=float, default=10)
    ap.add_argument('--no-accel', action='store_true')
    ap.add_argument('--no-gyro', action='store_true')
    ap.add_argument('--sensitivity', '-s', type=float, default=1.0, help='Gyro scale')
    ap.add_argument('--accel-sensitivity', type=float, default=1.0)
    ap.add_argument('--axes', choices=('original', 'direct'), default='direct')
    ap.add_argument('--demo', action='store_true', help='Generate synthetic gyro readings; no WinRT needed')
    ap.add_argument('--verbose', '-v', action='store_true')
    args = ap.parse_args()
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format='%(asctime)s %(levelname)s %(message)s', datefmt='%H:%M:%S')
    if not 1 <= args.send_rate <= 500 or not 1 <= args.poll_rate <= 500:
        ap.error('Rates must be 1 through 500 Hz')
    if args.timeout <= 0 or args.status_interval <= 0:
        ap.error('Timeout and status interval must be positive')
    if not 0 < args.port <= 65535:
        ap.error('Invalid UDP port')
    if not all(math.isfinite(x) for x in (args.sensitivity, args.accel_sensitivity)):
        ap.error('Sensitivity must be finite')
    if args.no_gyro and args.no_accel and not args.demo:
        ap.error('Both sensors are disabled')
    mp.freeze_support()
    try:
        sensors = (DemoSensors() if args.demo else WindowsSensors(
            args.poll_rate, accel=not args.no_accel, gyro=not args.no_gyro,
            sensitivity=args.sensitivity, accel_sensitivity=args.accel_sensitivity,
            axes=args.axes))
        server = DSUServer(sensors, host=args.host, port=args.port,
                           send_hz=args.send_rate, timeout=args.timeout,
                           status_interval=args.status_interval)
        sensors.start()
        try:
            server.serve()
        except KeyboardInterrupt:
            LOG.info('Stopping')
        finally:
            server.stop()
            sensors.stop()
    except (RuntimeError, OSError) as exc:
        LOG.error('%s', exc)
        raise SystemExit(1) from exc


if __name__ == '__main__':
    main()
