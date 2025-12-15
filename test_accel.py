import time
import threading

import win32com.client  # pip install pywin32


class AccelReader:
    """Polls the default 3D accelerometer and exposes raw X/Y/Z (in g)."""

    def __init__(self, poll_hz=10):
        self.poll_interval = 1.0 / poll_hz
        self._lock = threading.Lock()
        self._ax = 0.0
        self._ay = 0.0
        self._az = 0.0
        self._stop = False

        # Create SensorManager COM object.
        self._sensor_manager = win32com.client.Dispatch(
            "SensorManager.SensorManager"
        )

        # Windows universal sensor type GUID for 3D accelerometer.
        self._accel_type = "{C2FB0F5F-2B3A-4C02-9870-9CCFD2E3F9E5}"

        self._sensor = self._get_default_accel()
        if self._sensor is None:
            raise RuntimeError("No accelerometer sensor found")

        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

    def _get_default_accel(self):
        sensors = self._sensor_manager.GetSensorsByType(self._accel_type)
        if sensors is None or sensors.GetCount() == 0:
            return None
        return sensors.GetAt(0)

    def _poll_loop(self):
        while not self._stop:
            try:
                report = self._sensor.GetData()
                # Accelerometer data fields: X, Y, Z, in g.
                ax = float(report.GetSensorValue(
                    "{C2FB0F5F-2B3A-4C02-9870-9CCFD2E3F9E5}, 3"
                ))  # AccelerationX
                ay = float(report.GetSensorValue(
                    "{C2FB0F5F-2B3A-4C02-9870-9CCFD2E3F9E5}, 4"
                ))  # AccelerationY
                az = float(report.GetSensorValue(
                    "{C2FB0F5F-2B3A-4C02-9870-9CCFD2E3F9E5}, 5"
                ))  # AccelerationZ

                with self._lock:
                    self._ax, self._ay, self._az = ax, ay, az
            except Exception:
                # Keep last values on error.
                pass

            time.sleep(self.poll_interval)

    def get(self):
        with self._lock:
            return self._ax, self._ay, self._az

    def stop(self):
        self._stop = True
        self._thread.join(timeout=1.0)


def main():
    accel = AccelReader(poll_hz=2)  # internal polling at 2 Hz (0.5 s)
    print("Reading accelerometer every 0.5 s (g). Press Ctrl+C to stop.")
    try:
        while True:
            ax, ay, az = accel.get()
            print(f"Accel X={ax:.3f}  Y={ay:.3f}  Z={az:.3f}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        accel.stop()


if __name__ == "__main__":
    main()
