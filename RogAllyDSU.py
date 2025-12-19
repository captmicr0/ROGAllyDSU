import argparse
import socket
import struct
import threading
import time
import zlib

from winsdk.windows.devices.sensors import Gyrometer, Accelerometer


# ---------- Sensor reader (accel + gyro via WinRT) ----------

class WindowsImuReader:
    """
    Polls the default accelerometer and gyrometer via Windows.Devices.Sensors.
    Exposes get_motion() -> (accel_x, accel_y, accel_z, gyro_pitch, gyro_yaw, gyro_roll)
    with axes mapped for DSU/cemuhook.
    """
    
    def __init__(self, poll_hz=250, clamp_accel=False, clamp_accel_limit=5.0, 
             clamp_gyro=False, clamp_gyro_limit=150.0):
        self.poll_interval = 1.0 / poll_hz
        self._clamp_accel = clamp_accel
        self._clamp_accel_limit = clamp_accel_limit
        self._clamp_gyro = clamp_gyro
        self._clamp_gyro_limit = clamp_gyro_limit

        self._lock = threading.Lock()

        # Raw values from Windows before mapping
        self._accel_x = 0.0  # AccelerationX -> roll
        self._accel_y = 0.0  # AccelerationY -> pitch
        self._accel_z = 0.0  # AccelerationZ -> yaw

        self._gyro_x = 0.0   # AngularVelocityX -> pitch
        self._gyro_y = 0.0   # AngularVelocityY -> roll
        self._gyro_z = 0.0   # AngularVelocityZ -> yaw

        self._stop = False

        # Get default WinRT sensors.
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
                # Accelerometer reading (may be None).
                a_read = self._accel.get_current_reading()
                if a_read is not None:
                    ax = float(a_read.acceleration_x)  # roll
                    ay = float(a_read.acceleration_y)  # pitch
                    az = float(a_read.acceleration_z)  # yaw

                    # Clamp accel if enabled
                    if self._clamp_accel:
                        limit = self._clamp_accel_limit
                        ax = max(min(ax, limit), -limit)
                        ay = max(min(ay, limit), -limit)
                        az = max(min(az, limit), -limit)
                    
                    with self._lock:
                        self._accel_x = ax
                        self._accel_y = ay
                        self._accel_z = az

                # Gyro reading (may be None).
                g_read = self._gyro.get_current_reading()
                if g_read is not None:
                    gx = float(g_read.angular_velocity_x)  # pitch (deg/s)
                    gy = float(g_read.angular_velocity_y)  # roll
                    gz = float(g_read.angular_velocity_z)  # yaw

                    # Clamp gyro if enabled
                    if self._clamp_gyro:
                        limit = self._clamp_gyro_limit
                        gx = max(min(gx, limit), -limit)
                        gy = max(min(gy, limit), -limit)
                        gz = max(min(gz, limit), -limit)

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
        accel in g's, gyro in deg/s (matches cemuhook spec).
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
        gyro_pitch = -gx
        gyro_yaw   = gz
        gyro_roll  = -gy

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
                event_type = int.from_bytes(data[16:16+4], 'little', signed=False)
                if event_type == 0x100001:
                    packet = self._build_info_packet()
                    self.sock.sendto(packet, addr)
                if event_type == 0x100002: # Actual controllers data
                    if addr not in self._clients:
                        ip, port = addr
                        print(f"Client connected from {ip}:{port}")
                    self._clients.add(addr)

    def _send_loop(self):
        packet_counter = 0
        while not self._stop:
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
                packet_counter=packet_counter,
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
                except OSError:
                    self._clients.discard(addr)

            time.sleep(self.send_interval)

    def _build_info_packet(self):
        packet = b"".join(
            [
                # HEADER
                struct.pack(">4s", b'DSUS'),            # MAGIC STRING (DSUS)
                struct.pack("<H", 1001),                # VERSION (1001)
                struct.pack("<H", 84),                  # LENGTH
                struct.pack("<I", 0),                   # CRC32
                struct.pack("<I", 0x66778899),          # CLIENT/SERVER ID

                # EVENT TYPE
                struct.pack("<I", 0x100001),            # EVENT TYPE (Not actually part of header so it counts as length)
                                                        # (0x100002 = Actual controllers data)

                # BEGINNING
                struct.pack("<B", 0),                   # SLOT
                struct.pack("<B", 2),                   # SLOT STATE (2 = CONNECTED)
                struct.pack("<B", 2),                   # DEVICE MODEL (2 = FULL GYRO)
                struct.pack("<B", 2),                   # CONNECTION TYPE (2 = BLUETOOTH)
                struct.pack(">6s", b'\x00\x00\x00\x00'),# MAC ADDRESS OF DEVICE
                struct.pack("<B", 0x05),                # BATTERY STATUS (0x05 = FULL)

                # ZERO BYTE
                struct.pack("<B", 0),                   # ZERo BYTE
            ]
        )
        
        crc = zlib.crc32(packet) & 0xFFFFFFFF

        # Splice CRC into bytes 8–11 (little‑endian)
        packet = bytearray(packet)
        packet[8:12] = struct.pack("<I", crc)

        return bytes(packet)

    def _build_motion_packet(
        self,
        packet_counter,
        accel_x,
        accel_y,
        accel_z,
        gyro_pitch,
        gyro_yaw,
        gyro_roll,
    ):
        packet = b"".join(
            [
                # HEADER
                struct.pack(">4s", b'DSUS'),            # MAGIC STRING (DSUS)
                struct.pack("<H", 1001),                # VERSION (1001)
                struct.pack("<H", 84),                  # LENGTH
                struct.pack("<I", 0),                   # CRC32
                struct.pack("<I", 0x66778899),          # CLIENT/SERVER ID

                # EVENT TYPE
                struct.pack("<I", 0x100002),            # EVENT TYPE (Not actually part of header so it counts as length)
                                                        # (0x100002 = Actual controllers data)

                # BEGINNING
                struct.pack("<B", 0),                   # SLOT
                struct.pack("<B", 2),                   # SLOT STATE (2 = CONNECTED)
                struct.pack("<B", 2),                   # DEVICE MODEL (2 = FULL GYRO)
                struct.pack("<B", 2),                   # CONNECTION TYPE (2 = BLUETOOTH)
                struct.pack(">6s", b'\x00\x00\x00\x00'),# MAC ADDRESS OF DEVICE
                struct.pack("<B", 0x05),                # BATTERY STATUS (0x05 = FULL)

                # CONTROLLER DATA
                struct.pack("<B", 1),                   # IS CONNECTED (1 = CONNECTED)
                struct.pack("<I", packet_counter),      # PACKET NUMBER
                struct.pack("<B", 0),                   # BUTTONS BITMASK
                struct.pack("<B", 0),                   # BUTTONS BITMASK
                struct.pack("<B", 0),                   # HOME BUTTON
                struct.pack("<B", 0),                   # TOUCH BUTTON
                struct.pack("<B", 128),                 # LEFT STICK X (+R, -L)
                struct.pack("<B", 128),                 # LEFT STICK Y (+U, -D)
                struct.pack("<B", 128),                 # RIGHT STICK X (+R, -L)
                struct.pack("<B", 128),                 # RIGHT STICK Y (+U, -D)
                struct.pack("<B", 0),                   # ANALOG D-PAD LEFT
                struct.pack("<B", 0),                   # ANALOG D-PAD DOWN
                struct.pack("<B", 0),                   # ANALOG D-PAD RIGHT
                struct.pack("<B", 0),                   # ANALOG D-PAD UP
                struct.pack("<B", 0),                   # ANALOG Y
                struct.pack("<B", 0),                   # ANALOG B
                struct.pack("<B", 0),                   # ANALOG A
                struct.pack("<B", 0),                   # ANALOG X
                struct.pack("<B", 0),                   # ANALOG R1
                struct.pack("<B", 0),                   # ANALOG L1
                struct.pack("<B", 0),                   # ANALOG R2
                struct.pack("<B", 0),                   # ANALOG L2
                struct.pack("<BBHH", 0, 0, 0, 0),       # FIRST TOUCH
                struct.pack("<BBHH", 0, 0, 0, 0),       # SECOND TOUCH
                struct.pack("<Q", int(time.time() * 1000000)),   # MOTION TIMESTAMP IN uS
                struct.pack("<f", float(accel_x)),      # ACCELEROMETER X AXIS
                struct.pack("<f", float(accel_y)),      # ACCELEROMETER Y AXIS
                struct.pack("<f", float(accel_z)),      # ACCELEROMETER Z AXIS
                struct.pack("<f", float(gyro_pitch)),   # GYROSCOPE PITCH
                struct.pack("<f", float(gyro_yaw)),     # GYROSCOPE YAW
                struct.pack("<f", float(gyro_roll)),    # GYROSCOPE ROLL
            ]
        )
        
        crc = zlib.crc32(packet) & 0xFFFFFFFF

        # Splice CRC into bytes 8–11 (little‑endian)
        packet = bytearray(packet)
        packet[8:12] = struct.pack("<I", crc)

        return bytes(packet)

# ---------- Main ----------

def main():
    parser = argparse.ArgumentParser(
        description="ROG Ally DSU motion server (accel+gyro)"
    )

    parser.add_argument("--accel", action="store_true", help="Send accelerometer data in DSU packets.")
    parser.add_argument("--no-gyro", action="store_true", help="Do not send gyroscope data in DSU packets.")
    parser.add_argument("--rate", type=int, default=250, help="Sampling and send rate in Hz (default: 250).",)
    parser.add_argument("-s", "--sensitivity", type=float, default=1.0, help="Global motion sensitivity multiplier (default: 1.0).")

    parser.add_argument('--clamp-accel', action='store_true', help="Enable accelerometer clamping")
    parser.add_argument('--clamp-gyro', action='store_true', help="Enable gyroscope clamping")
    parser.add_argument('--clamp-accel-limit', type=float, default=5.0, help="Accel clamp limit in g (default: 5.0)")
    parser.add_argument('--clamp-gyro-limit', type=float, default=150.0, help="Gyro clamp limit in deg/s (default: 150.0)")

    args = parser.parse_args()

    imu_reader = WindowsImuReader(poll_hz=args.rate)
    server = DSUServer(
        imu_reader,
        send_hz=args.rate,
        send_accel=args.accel,
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
