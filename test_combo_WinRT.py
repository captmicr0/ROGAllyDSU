import time

from winsdk.windows.devices.sensors import Gyrometer, Accelerometer


def main():
    gyro = Gyrometer.get_default()
    accel = Accelerometer.get_default()

    if gyro is None:
        print("No gyrometer found")
    if accel is None:
        print("No accelerometer found")
    if gyro is None and accel is None:
        return

    # Try to set report interval around 500 ms if supported
    desired_ms = 500
    if gyro is not None:
        min_gyro = gyro.minimum_report_interval
        gyro.report_interval = max(min_gyro, desired_ms)
    if accel is not None:
        min_accel = accel.minimum_report_interval
        accel.report_interval = max(min_accel, desired_ms)

    print("Reading sensors every ~0.5 s. Press Ctrl+C to stop.")

    try:
        while True:
            if gyro is not None:
                g_read = gyro.get_current_reading()
                gx = g_read.angular_velocity_x
                gy = g_read.angular_velocity_y
                gz = g_read.angular_velocity_z
                print(f"GYRO  dps  X={gx:8.3f}  Y={gy:8.3f}  Z={gz:8.3f}")
            else:
                print("GYRO  dps  (not available)")

            if accel is not None:
                a_read = accel.get_current_reading()
                ax = a_read.acceleration_x
                ay = a_read.acceleration_y
                az = a_read.acceleration_z
                print(f"ACCEL g    X={ax:8.3f}  Y={ay:8.3f}  Z={az:8.3f}")
            else:
                print("ACCEL g    (not available)")

            print("-" * 60)
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
