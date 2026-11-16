# Hardware verification checklist (not a claim of completed testing)

1. **Back up** your existing SD card and ESP32 flash first. Do not flash a board with unknown wiring.
2. Confirm ESP32 DevKitC V4 and **GPIO13 direct servo**, not the older PCA9685 configuration. Verify 3.3 V logic and common ground.
3. Confirm BME280 addresses 0x76/0x77, AMG8833 at 0x68/0x69, MQ135 analog output into GPIO34 (never >3.3 V), LED gate GPIO23, RGB 25/26/27, and display SPI pinout in the sketch.
4. Check the physical ESP32–Pi serial link. Configure `WARDEN_SERIAL_PORT`; do not assume `/dev/serial0` is correct and do not probe OpenCR while motors operate.
5. In isolation, send `PING` and confirm `PONG`; read `ENV`, `THERMAL`, `RF`, and `EVENT` and validate JSON fields against a real sensor change.
6. Disconnect one sensor at a time safely and confirm missing readings are null/unavailable, **not plausible defaults**. MQ135 ADC is uncalibrated and an unconnected analog pin can still return electrical noise.
7. Start ROS motor driver with wheels lifted. Verify `/cmd_vel` has only one publisher (`warden_teleop`) and scan pause stops wheels.
8. Validate webcam availability and captured images before pressing Scan; compare known camera-like and non-camera objects. Don't present this test as a validated security detection rate.
9. Only enable LED after power/brownout issues are resolved. Only enable spin in a cleared test area with emergency stop and supervised operator.
10. If AI is offline/has no key, a scan must show `INCONCLUSIVE` rather than `CLEAR`.

**Unverified in this delivery:** Full Arduino compilation, physical wiring, ESP32 flash content, camera performance, robot boot, actual detection accuracy. All source-level checks and offline tests are separate from hardware verification.

11. **Privacy before any external AI call:** default `WARDEN_ENABLE_AI=0` prevents remote image uploads. Only set it to `1` after explicit consent, a camera privacy review, and controlled test conditions. Verify the remote service policy and make sure no bystanders or sensitive room details are in frames.
