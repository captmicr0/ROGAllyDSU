import time
import threading

import win32com.client  # pip install pywin32


class GyroReader:
    """Polls the default 3D gyrometer and exposes raw X/Y/Z (deg/s)."""

    def __init__(self, poll_hz=10):
        self.poll_interval = 1.0 / poll_hz
        self._lock = threading.Lock()
        self._gx = 0.0
        self._gy = 0.0
        self._gz = 0.0
        self._stop = False

        # Create SensorManager COM object.
        self._sensor_manager = win32com.client.Dispatch(
            "SensorManager.SensorManager"
        )

        # Windows universal sensor type GUID for 3D gyrometer.
        self._gyro_type = "{09485F5A-759E-42C2-BD4B-A349B75C8643}"

        self._sensor = self._get_default_gyro()
        if self._sensor is None:
            raise RuntimeError("No gyrometer sensor found")

        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

    def _get_default_gyro(self):
        sensors = self._sensor_manager.GetSensorsByType(self._gyro_type)
        if sensors is None or sensors.GetCount() == 0:
            return None
        return sensors.GetAt(0)

    def _poll_loop(self):
        while not self._stop:
            try:
                report = self._sensor.GetData()
                # Gyroscope data fields: degrees per second around X, Y, Z.
                gx = float(report.GetSensorValue(
                    "{09485F5A-759E-42C2-BD4B-A349B75C8643}, 3"
                ))  # AngularVelocityX
                gy = float(report.GetSensorValue(
                    "{09485F5A-759E-42C2-BD4B-A349B75C8643}, 4"
                ))  # AngularVelocityY
                gz = float(report.GetSensorValue(
                    "{09485F5A-759E-42C2-BD4B-A349B75C8643}, 5"
                ))  # AngularVelocityZ

                with self._lock:
                    self._gx, self._gy, self._gz = gx, gy, gz
            except Exception:
                # Keep last values on error.
                pass

            time.sleep(self.poll_interval)

    def get(self):
        with self._lock:
            return self._gx, self._gy, self._gz

    def stop(self):
        self._stop = True
        self._thread.join(timeout=1.0)


def main():
    gyro = GyroReader(poll_hz=2)  # internal polling at 2 Hz (0.5 s)
    print("Reading gyroscope every 0.5 s (deg/s). Press Ctrl+C to stop.")
    try:
        while True:
            gx, gy, gz = gyro.get()
            print(f"Gyro X={gx:.3f}  Y={gy:.3f}  Z={gz:.3f}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        gyro.stop()


if __name__ == "__main__":
    main()
