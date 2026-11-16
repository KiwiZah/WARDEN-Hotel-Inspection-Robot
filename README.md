# WARDEN — Hotel Privacy Inspection Robot

**Inha University capstone · Team SPECTRA · 2026 · reviewed source snapshot**

WARDEN is a **research prototype** combining TurtleBot3 Burger, Raspberry Pi 4, ESP32 sensor hub, Logitech C920, tilt servo, ROS 2 Humble, and an inspection dashboard. It was demonstrated as a team engineering project; this repository represents a **cleaned, safety-reviewed reconstruction** from the recovered final shared source files. **The cleaned build has not yet been validated end-to-end on the original hardware.**

## Components

```text
  Android gamepad -> ROS2 /joy -> warden_teleop -> /cmd_vel -> TurtleBot3 OpenCR
                                   ^
                      /warden/scan_active (pause interlock)
                                   |
  C920 webcam -> WARDEN Python host / Flask dashboard (:8000)
                                   |
                      ESP32 serial PING/ENV/THERMAL/RF/EVENT/SERVO/LED/STATUS
                                   |
          BME280 · AMG8833 · MQ135 ADC · Wi-Fi scan · TFT · servo · status LED
```

## What is real, what is unavailable

- **No generated or random telemetry.** ESP32 reads actual sensor APIs; missing/failed readings appear unavailable (`null`/blank), not substituted values.
- **Thermal:** requires functioning AMG8833; host now reads the firmware's `THERMAL` response. Unplugged -> unavailable.
- **Environment:** BME280 readings are direct; MQ135 value is raw ADC (not ppm). Connectivity of the analog probe cannot be inferred from an ADC number alone.
- **RF:** nearby Wi-Fi access-point RSSI readings, not a dedicated hidden-camera RF detector.
- **Camera inspection:** captures real C920 frames at actual servo angles. Optional LED differential optical layer is **off until power verified**. AI vision needs an explicit API key and external connection; if unavailable, classification is `INCONCLUSIVE`.
- **Driving:** exactly one ROS velocity publisher (teleop); host triggers scans and interlocks motion. Celebratory spins are disabled by default for safety.
- **Line follower:** retained as an optional experimental ROS package, not enabled by the normal startup.

## Layout

- `firmware/warden_sensor_hub/` — ESP32 Arduino sketch, `EVENT` support and sensor fault handling
- `src/warden_host.py` — Flask dashboard, serial client, webcam and inspection
- `src/warden_teleop.py` — ROS joystick teleoperation and scan safety interlock
- `ros/drive_only.launch.py` — motor driver without LiDAR
- `scripts/` and `deploy/systemd/` — explicit ROS Humble service deployment
- `experimental/warden_line_follow/` — separate, unverified line-following experiment
- `docs/RECOVERY_AUDIT.md` — reconstructed changes and original engineering limitations
- `docs/HARDWARE_VERIFICATION.md` — required preflight/hardware tests

## Reproducibility (Raspberry Pi Ubuntu + ROS Humble)

1. Verify the ESP32 firmware wiring and hardware against `docs/HARDWARE_VERIFICATION.md`. **Never flash blindly.**
2. Install ROS 2 Humble, TurtleBot3 packages, joystick driver and platform system packages (including `python3-opencv` and `python3-rclpy`).
3. Create a virtual environment with ROS system packages visible: `python3 -m venv --system-site-packages .venv` and `.venv/bin/pip install -r requirements.txt`.
4. Create a local private environment file from `config/warden.env.example` at `~/.config/warden/warden.env`. Set correct serial port. Never commit secrets. The default dashboard binds **localhost only**; use a firewall/SSH tunnel for remote viewing.
5. After hardware checks, on the Pi run `bash scripts/install_services.sh` to **install but not start** four services. Start/test in stages, observe logs, and validate the emergency stop.
6. Use `python3 -m unittest discover -s tests -v` for offline logic checks.

## Limitations and responsible claims

This is a capstone demonstrator, **not a validated surveillance-detection or emergency-response product**. Touchscreen requests are shown on a local dashboard, not sent to hotel staff. Vision scores are heuristic, and historical quantitative test results have not been reproduced. Source audit and static checks are not substitutes for hardware integration tests.

Project was collaborative; this source package integrates team artifacts and preserves their provenance. No private keys, original SD images, or visitor camera captures are included.
