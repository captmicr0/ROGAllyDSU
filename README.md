# ROG Ally DSU Motion Server

Standalone Python implementation of the DSU/Cemuhook UDP v1001 protocol. **Version 2: sensor polling now executes in a dedicated OS process to avoid WinRT-induced UDP stalls.** Shares the Windows accelerometer and gyrometer with emulators and DSU-compatible applications. It does not emulate a virtual gamepad; controller buttons still come from the normal Windows input system.

## Setup on ROG Ally (Windows)

1. Install Python 3.10+ (Python 3.12 is fine).
2. In PowerShell or Command Prompt: `py -m pip install winsdk`
3. Run `Start_AllyDSU.bat` or `py ally_dsu.py` from this folder.
4. In your emulator or PadTest, configure DSU / Cemuhook / UDP input at **127.0.0.1**, port **26760**, controller **slot 0**. Only one DSU server can bind the port at a time.
5. To verify UDP and actual streaming, open a second command prompt in the folder and run `py probe_dsu.py --seconds 5`.

**Default:** gyroscope + accelerometer enabled; 125 Hz sensor polling and UDP send rate; 15-second subscription expiry. The server sends *every frame* while subscribed, even when the gyro is stationary. The sender's clock is monotonic. Controller discovery replies are immediate.

**Network clients:** add `--host 0.0.0.0` and allow UDP port 26760 in Windows Firewall on your trusted private network. Keep the default localhost binding for same-device emulators.

## Useful options

- `--send-rate 250` : 250 DSU messages per second instead of 125
- `--poll-rate 250` : request 250 WinRT sensor polls per second (actual sensor sampling is driver-dependent)
- `--sensitivity 0.5` : half gyro angular velocity
- `--accel-sensitivity 0.5` : half accelerometer values
- `--axes direct` : use direct WinRT X,Y,Z accelerometer mapping instead of the previous script's Y,Z,X remapping; gyro remains X,Y,Z -> pitch,yaw,roll
- `--no-accel` : zero accelerometer output if the accelerometer isn't available
- `--no-gyro` : zero gyroscope output
- `--timeout 30` : retain inactive subscriptions 30 seconds instead of 15
- `--verbose` : log rejected DSU packets
- `--demo` : synthetic gyro input to test network functionality, without requiring Windows/winsdk
- `--host 0.0.0.0` : listen on all IPv4 interfaces

For example: `py ally_dsu.py --send-rate 250 --poll-rate 250 --verbose`

## Troubleshooting

- If version and info replies appear but motion frames stay at zero in the probe, check the DSU subscription request and whether another server is running.
- If `probe_dsu.py` reports ~125 Hz but PadTest disconnects, check its endpoint (127.0.0.1:26760), conflicting servers, Windows Firewall/security software, and PadTest compatibility.
- The periodic log reports actual UDP packets/second, total sensor reads, and the age of the last available reading. **With one active client, UDP rate should stay around 125 packets/s while the Ally is perfectly still, even if sensor reads stop.** If the age is N/A, the Windows driver is not delivering readings. If the age steadily grows, investigate sensor/driver health.
- The Windows sensor worker runs as a separate OS process; if a WinRT call blocks while stationary, the network sender continues using its last sensor values.
- WinRT drivers may return the same cached sensor sample while stationary. This is normal: the UDP server still sends valid packets at the configured rate.
- Some applications use different signs/axis conventions. This package preserves the **gyro mapping** from the original script by default. Axis calibration may still be needed per emulator.
- This is not an Xbox controller input bridge. It intentionally advertises a motion-capable controller with neutral buttons and sticks. Use the Ally's normal controls separately.

## Test without Windows

Run `python -m unittest -v` to verify protocol serialization, lengths, CRCs, discovery, subscriptions, continuous packets and expiry. Run `python ally_dsu.py --demo`, and in a second terminal run `python probe_dsu.py --seconds 5` to exercise the UDP flow.

## Implementation references

- Protocol specification: https://github.com/v1993/cemuhook-protocol/blob/master/README.md
- Microsoft WinRT gyrometer: https://learn.microsoft.com/en-us/windows/uwp/devices-sensors/use-the-gyrometer
- Windows sensor interval requirements: https://learn.microsoft.com/en-us/uwp/api/windows.devices.sensors.gyrometer.reportinterval

## Limits

This was tested with a simulated sensor and a real UDP socket in the development environment, **not physically on an ROG Ally** or directly with PadTest. Actual Ally gyro access and PadTest interoperability require testing on your device. No background process or Windows service installation is performed.

## Verifying the original stationary problem

1. Launch `Start_AllyDSU.bat`, then start PadTest.
2. Move the Ally once and then leave it completely still for 30 seconds.
3. Look for `actual UDP rate=... packets/s`. With one PadTest subscriber this should stay near **125.0**, not collapse to 0.4 or 1.0.
4. `sensor age` may grow during this period. This is fine for transport: old readings are deliberately repeated. If PadTest still shows no gyro despite a stable transmission rate, its interpretation of samples/timestamps should be investigated separately.
5. If the child sensor process exits, its exit code appears in the periodic log and networking continues.

The old process used two Python *threads* for WinRT and UDP. A blocking native WinRT call can prevent other Python threads from running. The new version uses separate *processes*, each with its own Python interpreter/GIL.
