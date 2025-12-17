import argparse
import socket
import struct
import threading
import time

from winsdk.windows.devices.sensors import Gyrometer, Accelerometer


# ---------- Sensor reader (accel + gyro via WinRT) ----------

class WindowsImuReader:
    """
    Polls the default accelerometer and gyrometer via Windows.Devices.Sensors.
    Exposes get_motion() -> (accel_x, accel_y, accel_z, gyro_pitch, gyro_yaw, gyro_roll)
    with axes mapped for DSU/cemuhook.
    """

    def __init__(self, poll_hz=250):
        self.poll_interval = 1.0 / poll_hz
        self._lock = threading.Lock()

        # Raw values from Windows before mapping
        self._accel_x = 0.0  # AccelerationX -> roll
        self._accel_y = 0.0  # AccelerationY -> pitch
        self._accel_z = 0.0  # AccelerationZ -> yaw

        self._gyro_x = 0.0   # AngularVelocityX -> pitch
        self._gyro_y = 0.0   # AngularVelocityY -> roll
        self._gyro_z = 0.0   # AngularVelocityZ -> yaw

        self._stop = False

        # Get default WinRT sensors.[web:35][web:125]
        self._gyro = Gyrometer.get_default()
        self._accel = Accelerometer.get_default()

        if self._accel is None:
            raise RuntimeError("No accelerometer sensor found")
        if self._gyro is None:
            raise RuntimeError("No gyrometer sensor found")

        # Try to set report interval close to poll interval (in ms)
        desired_ms = int(self.poll_interval * 1000)
        self._gyro.report_interval = max(self._gyro.minimum_report_interval, desired_ms)
        self._accel.report_interval = max(self._accel.minimum_report_interval, desired_ms)

        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

    def _poll_loop(self):
        while not self._stop:
            try:
                # Accelerometer reading (may be None).[web:125]
                a_read = self._accel.get_current_reading()
                if a_read is not None:
                    ax = float(a_read.acceleration_x)  # roll
                    ay = float(a_read.acceleration_y)  # pitch
                    az = float(a_read.acceleration_z)  # yaw

                    with self._lock:
                        self._accel_x = ax
                        self._accel_y = ay
                        self._accel_z = az

                # Gyro reading (may be None).[web:35]
                g_read = self._gyro.get_current_reading()
                if g_read is not None:
                    gx = float(g_read.angular_velocity_x)  # pitch (deg/s)
                    gy = float(g_read.angular_velocity_y)  # roll
                    gz = float(g_read.angular_velocity_z)  # yaw

                    with self._lock:
                        self._gyro_x = gx
                        self._gyro_y = gy
                        self._gyro_z = gz

            except Exception:
                # On any failure, keep last values
                pass

            time.sleep(self.poll_interval)

    def get_motion(self):
        """
        Return mapped DSU axes:
        accel_x, accel_y, accel_z, gyro_pitch, gyro_yaw, gyro_roll
        accel in g's, gyro in deg/s (matches cemuhook spec).[web:60]
        """
        with self._lock:
            ax = self._accel_x
            ay = self._accel_y
            az = self._accel_z
            gx = self._gyro_x
            gy = self._gyro_y
            gz = self._gyro_z

        # Map accelerometer:
        #   AccelX -> roll
        #   AccelY -> pitch
        #   AccelZ -> yaw
        # DSU accel X,Y,Z loosely aligned with pitch,yaw,roll.
        accel_x = ay   # pitch
        accel_y = az   # yaw
        accel_z = ax   # roll

        # Map gyro:
        #   AngularVelocityX -> pitch
        #   AngularVelocityY -> roll
        #   AngularVelocityZ -> yaw
        gyro_pitch = gx
        gyro_yaw   = gz
        gyro_roll  = gy

        return accel_x, accel_y, accel_z, gyro_pitch, gyro_yaw, gyro_roll

    def stop(self):
        self._stop = True
        self._thread.join(timeout=1.0)


# ---------- DSU / Cemuhook UDP server ----------

class DSUServer:
    DSU_PORT = 26760

    def __init__(self, imu_reader, send_hz=250, send_accel=True, send_gyro=True,
                 sensitivity=1.0):
        self.imu_reader = imu_reader
        self.send_interval = 1.0 / send_hz
        self.send_accel = send_accel
        self.send_gyro = send_gyro
        self.sensitivity = float(sensitivity)

        # Determine local IP (best-effort)
        hostname = socket.gethostname()
        self.host_ip = "0.0.0.0"

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.host_ip, self.DSU_PORT))
        self.sock.settimeout(0.1)

        self._clients = set()
        self._stop = False

        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._send_thread = threading.Thread(target=self._send_loop, daemon=True)

    def start(self):
        self._recv_thread.start()
        self._send_thread.start()

    def stop(self):
        self._stop = True
        self._recv_thread.join(timeout=1.0)
        self._send_thread.join(timeout=1.0)
        self.sock.close()

    def _recv_loop(self):
        while not self._stop:
            try:
                data, addr = self.sock.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                break

            if len(data) < 4:
                continue

            magic = data[:4]
            # DSUC = client → server registration; record client endpoint.
            if magic == b"DSUC":
                if addr not in self._clients:
                    ip, port = addr
                    print(f"Client connected from {ip}:{port}")
                self._clients.add(addr)

    def _send_loop(self):
        packet_counter = 0
        while not self._stop:
            now = time.time()
            timestamp_us = int(now * 1_000_000)

            accel_x, accel_y, accel_z, gyro_pitch, gyro_yaw, gyro_roll = \
                self.imu_reader.get_motion()

            # Apply accel/gyro masks according to CLI flags.
            if not self.send_accel:
                accel_x = accel_y = accel_z = 0.0
            if not self.send_gyro:
                gyro_pitch = gyro_yaw = gyro_roll = 0.0
            
            # Apply global sensitivity (both accel and gyro).
            s = self.sensitivity
            accel_x *= s
            accel_y *= s
            accel_z *= s
            gyro_pitch *= s
            gyro_yaw *= s
            gyro_roll *= s

            packet = self._build_motion_packet(
                slot=0,
                connected=True,
                packet_counter=packet_counter,
                timestamp_us=timestamp_us,
                accel_x=accel_x,
                accel_y=accel_y,
                accel_z=accel_z,
                gyro_pitch=gyro_pitch,
                gyro_yaw=gyro_yaw,
                gyro_roll=gyro_roll,
            )
            packet_counter = (packet_counter + 1) & 0xFFFFFFFF

            for addr in list(self._clients):
                try:
                    self.sock.sendto(packet, addr)
                    ip, port = addr
                    print(f"sent packet to {ip}:{port}")
                except OSError:
                    self._clients.discard(addr)

            time.sleep(self.send_interval)

    def _build_motion_packet(
        self,
        slot,
        connected,
        packet_counter,
        timestamp_us,
        accel_x,
        accel_y,
        accel_z,
        gyro_pitch,
        gyro_yaw,
        gyro_roll,
    ):
        # Header for server→client packet.[web:60]
        magic = b"DSUS"
        version = 1
        packet_type = 0x1001  # “Actual controller data”
        crc32 = 0

        # Identification header (11 bytes):
        #   u8 flags (1 = slot-based registration)
        #   u8 slot
        #   6-byte MAC (dummy)
        #   u16 reserved
        ident_flags = 1
        ident_slot = slot & 0xFF
        mac_bytes = b"\x00\x11\x22\x33\x44\x55"
        ident_reserved = 0

        ident = struct.pack(
            ">BB6sH",
            ident_flags,
            ident_slot,
            mac_bytes,
            ident_reserved,
        )

        is_connected = 1 if connected else 0
        buttons_1 = 0
        buttons_2 = 0
        home_btn = 0
        touch_btn = 0
        left_x = 128
        left_y = 128
        right_x = 128
        right_y = 128
        adpad_l = adpad_d = adpad_r = adpad_u = 0
        analog_y = analog_b = analog_a = analog_x = 0
        analog_r1 = analog_l1 = analog_r2 = analog_l2 = 0

        # Two empty touches (inactive)
        touch_struct = struct.pack(">BBHH", 0, 0, 0, 0)
        touch1 = touch_struct
        touch2 = touch_struct

        payload = b"".join(
            [
                ident,
                struct.pack(">B", is_connected),
                struct.pack(">I", packet_counter),
                struct.pack(">B", buttons_1),
                struct.pack(">B", buttons_2),
                struct.pack(">B", home_btn),
                struct.pack(">B", touch_btn),
                struct.pack(">B", left_x),
                struct.pack(">B", left_y),
                struct.pack(">B", right_x),
                struct.pack(">B", right_y),
                struct.pack(">B", adpad_l),
                struct.pack(">B", adpad_d),
                struct.pack(">B", adpad_r),
                struct.pack(">B", adpad_u),
                struct.pack(">B", analog_y),
                struct.pack(">B", analog_b),
                struct.pack(">B", analog_a),
                struct.pack(">B", analog_x),
                struct.pack(">B", analog_r1),
                struct.pack(">B", analog_l1),
                struct.pack(">B", analog_r2),
                struct.pack(">B", analog_l2),
                touch1,
                touch2,
                struct.pack(">Q", timestamp_us),
                struct.pack(">f", float(accel_x)),
                struct.pack(">f", float(accel_y)),
                struct.pack(">f", float(accel_z)),
                struct.pack(">f", float(gyro_pitch)),
                struct.pack(">f", float(gyro_yaw)),
                struct.pack(">f", float(gyro_roll)),
            ]
        )

        length = len(payload)
        header = struct.pack(">4sHHHI", magic, version, packet_type, length, crc32)
        return header + payload


# ---------- Main ----------

def main():
    parser = argparse.ArgumentParser(
        description="ROG Ally DSU motion server (accel+gyro)"
    )
    parser.add_argument(
        "--no-accel",
        action="store_true",
        help="Do not send accelerometer data in DSU packets.",
    )
    parser.add_argument(
        "--no-gyro",
        action="store_true",
        help="Do not send gyroscope data in DSU packets.",
    )
    parser.add_argument(
        "--rate",
        type=int,
        default=250,
        help="Sampling and send rate in Hz (default: 250).",
    )
    parser.add_argument(
        "-s",
        "--sensitivity",
        type=float,
        default=1.0,
        help="Global motion sensitivity multiplier (default: 1.0).",
    )
    args = parser.parse_args()

    imu_reader = WindowsImuReader(poll_hz=args.rate)
    server = DSUServer(
        imu_reader,
        send_hz=args.rate,
        send_accel=not args.no_accel,
        send_gyro=not args.no_gyro,
        sensitivity=args.sensitivity,
    )

    print(
        f"ROG Ally DSU motion server running at {server.host_ip}:{server.DSU_PORT}"
    )
    if args.no_accel:
        print("NOTE: accelerometer data disabled (sending gyro only).")
    if args.no_gyro:
        print("NOTE: gyroscope data disabled (sending accel only).")
    print("Configure your emulator to use this IP/port as a cemuhook/DSU server.")

    server.start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        server.stop()
        imu_reader.stop()


if __name__ == "__main__":
    main()
