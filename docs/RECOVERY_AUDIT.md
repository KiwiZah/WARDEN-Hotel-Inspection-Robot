# WARDEN recovered-source audit

This repository is a reviewed reconstruction from the team's final shared code, not a claim of identical final flashed binaries. Originals are preserved in the independent SD-card/Telegram backups.

## Findings from recovered files

- Final shared host: original `warden host.py` matches `warden_host.py` (SHA-256 `2c3bff27a2a562dcf90b396b6a4a27f7140206bfbf17d74366379a497831357c`).
- Recovered final ESP32 candidate: `warden sensor hub (2).ino` supports `EVENT`; earlier versions did not.
- The host originally had `FAKE_THERMAL=True` (randomly generated room temperatures), `USE_LED=False`, and `USE_YOLO=False`.
- The older enabled systemd stack used standard TurtleBot bringup (potential LiDAR/ESP32 serial clash), ran a separate teleop and host publisher to `/cmd_vel`, and did not ensure ROS was sourced for host.
- The June 18 historical host log shows `ModuleNotFoundError: No module named cv2` from a different standalone launcher. The final demonstration was reported as working, so that log is not evidence of a final failure.
- Servo control was PCA9685 in an earlier sketch, but direct GPIO13 `ESP32Servo` in the final shared sketch.
- Line-following code was a separate experiment, not part of enabled four-service boot configuration.

## Changes made here

- Removed simulated thermal telemetry entirely. Firmware returns JSON null if AMG8833 absent/read fails; the host displays unavailable.
- Added bounded command replies and filtering of asynchronous serial debug messages.
- Separated motor control from scan/dashboard: **teleop is sole publisher** to `/cmd_vel`, host publishes `/warden/scan_active`; teleop stops and fails closed if interlock or joystick goes stale.
- Remote AI failure, missing camera, missing thermal/Wi-Fi telemetry, or scan exception -> `INCONCLUSIVE`, never a false negative `CLEAR`.
- Protected long-running scan against incorrectly resetting status while thread still works.
- Switched ROS driver to motors-only launch; boot services start host before teleop to make scan interlock available.
- Explicit opt-in for LED, celebratory spins, **and remote AI image uploads**; hardware validation and privacy consent required.
- Corrected touchscreen copy: the TFT queues an alert for the host to poll, **not an actual hotel staff notification**.
- No secrets, system images, caches, captures, or private logs are committed.

## Known limitations

- RF scanner detects nearby **Wi-Fi AP signals**, not generic RF bug transmitters.
- MQ135 is a **raw ADC signal**, not calibrated ppm or air-quality concentration. An unplugged analog input can read noise; inspect wiring before claiming gas monitoring.
- The visual-model score is a heuristic, not a statistically calibrated likelihood. No production security assurance.
- Offline tests do not prove real-world compatibility or detection accuracy.
- The optional line follower requires GPIO wiring/calibration and has not been verified on the robot; it is not deployed by default.
