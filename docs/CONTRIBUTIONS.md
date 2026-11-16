# Engineering contributions and responsibility

WARDEN was developed by **Team SPECTRA** as an Inha University capstone project. The roles below reflect Zahrokhon Kodirova's own account of the team's division of work and the recovered source/project materials. This is an attribution statement, **not a claim that one person built every part of the robot**.

## Zahrokhon Kodirova — hardware integration and sensing software

**Hardware and integration (primary responsibility)**

- Assembled and wired the project hardware, including the Raspberry Pi/ESP32 connection, camera, sensing modules, servo/LED elements, and TFT interface.
- Integrated the **BME280** environmental sensor (temperature/humidity/pressure), **MQ-135** analog gas sensor, **AMG8833** thermal-array sensor, and ESP32 Wi-Fi/RSSI scanning functions.
- Performed physical wiring, integration, troubleshooting, and hands-on prototype testing.

**Sensing, inspection, and user-interface software (primary responsibility)**

- Developed the ESP32 sensor-hub logic and Raspberry Pi-side sensing/application software.
- Implemented the camera/servo-based inspection and **optical glint-detection algorithm**, including the LED on/off differential-imaging design.
- Developed integration of thermal, Wi-Fi/RF, environmental, and image/AI evidence into the inspection and dashboard workflow.
- Built the Raspberry Pi monitoring/dashboard code to display sensor states and inspection outcomes.
- Developed the **TFT touchscreen guest-assistance prototype**: Navigation Help, Call Staff, and Emergency Alert selections. These are *local UI/event flows* forwarded to the dashboard, **not a deployed conversational assistant, navigation system, emergency dispatcher, or live hotel-staff messaging service**.
- Tested and debugged the sensing hardware, detection logic, communications, and prototype integration.

**Technical limitations that affect how these contributions are presented**

- Original hardware and optical routines were implemented, but the recovered final host configuration had `USE_LED=False` and `FAKE_THERMAL=True`. Therefore **implementation** must not be equated with evidence of a fully enabled, validated four-layer detection pipeline in the final saved runtime configuration.
- The current reviewed code removes simulated thermal readings and handles missing data as unavailable/inconclusive. It is a **later source cleanup**, not a claim that the original physical demonstration ran this exact updated build.
- The ESP32 `RF` routine scans nearby Wi-Fi networks and signal strengths; it is not an all-band RF-detector circuit. The MQ-135 output is uncalibrated raw ADC, not gas concentration in ppm.
- The physical prototype and user-reported testing are genuine project work; detection performance, full sensor calibration, and regenerated build compatibility are not independently verified in this repository.

## Other team contributions

Another teammate developed **TurtleBot3 movement/drive control**, joystick navigation, the **automatic program startup/bring-up**, and the celebratory/ready-rotation behavior. The **3D model/render** was also created by someone other than Zahrokhon Kodirova. These are not claimed as her work.

The wider project, presentations, and visual materials are credited to **Team SPECTRA**. For the complete team roster and academic supervision, see the original capstone poster; avoid assigning additional individual roles without confirmation.

## Relationship to the cleaned repository

This repository contains original project designs and **reviewed/reconstructed code**. Later cleanup improved error handling, removed deliberate simulated thermal values, separated teleop from host ROS velocity commands, and updated deployment procedures. These code-review changes should not be misrepresented as features exercised during the original demonstration. For provenance and test limits, see [Recovery audit](RECOVERY_AUDIT.md), [Presentation evidence](PRESENTATION_EVIDENCE.md), and [Hardware verification](HARDWARE_VERIFICATION.md).
