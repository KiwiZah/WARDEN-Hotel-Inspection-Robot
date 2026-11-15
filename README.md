WARDEN — Multi-Modal Hotel Privacy Inspection Robot
Engineering capstone project | Inha University | 2026 | Team SPECTRA

WARDEN is a working research/engineering prototype that combines a TurtleBot3 mobile robot, a Raspberry Pi host, an ESP32 sensor interface, camera-based inspection, and a live web dashboard to investigate potential hidden-camera risks in hotel-room-like environments.

The aim was to connect physical sensing, robot control, computer vision, and digital reporting in one inspection workflow—not to claim a production-ready security system.
My contribution
As a member of Team SPECTRA, I led the hands-on development of the WARDEN prototype, including:
- Selecting hardware components and planning their integration.
- Assembling and configuring the working TurtleBot3-based prototype and attached sensors.
- Developing and integrating Python-based host functionality, with AI-assisted coding support.
- Connecting sensor readings, image-inspection logic, and a live monitoring dashboard.
- Troubleshooting the hardware/software setup and conducting controlled demonstrations.
This was a team capstone. This page describes my own technical contribution; credit for other team members' work is retained.
Prototype architecture
Camera + inspection head ───────┐
                                │
ESP32 sensor hub ── serial ── Raspberry Pi 4 ── Flask web dashboard
                                │
                            ROS 2 /cmd_vel
                                │
                           TurtleBot3 base
The recovered Python host script implements or configures:
- Robot control: ROS 2 joystick messages and velocity commands to the TurtleBot3 base.
- Image capture: camera frames from multiple servo angles.
- Inspection: optional LED-on/LED-off image differences, AI-assisted vision checks, and supporting sensor signals.
- Decision logic: weighted confidence scoring from available inspection signals.
- Monitoring: a Flask dashboard showing inspection status, telemetry, alerts, and visual results.
The prototype includes configurable features. Not all features were enabled at the same time in the recovered code snapshot: LED-based optical scanning and continuous YOLO detection were disabled in that saved configuration, and thermal readings were simulated. Final demonstration settings are being reconciled with project records.
Technologies
Python · ROS 2 · TurtleBot3 · Raspberry Pi 4 · ESP32 · OpenCV · Flask · Serial communication · Sensor integration · Computer vision
Testing and results
WARDEN was demonstrated in a controlled, room-scale setup using planted camera-like targets and reflective non-camera objects. The project documented inspection outputs, confidence scores, and dashboard alerts.

Different presentation materials contain different target counts, so final quantitative results are being checked against the original test records before publication. These tests do not establish real-world hidden-camera detection reliability.
Current status
- The working TurtleBot3 platform was returned to the university after the capstone.
- Project posters, presentations, and demonstration evidence have been preserved.
- A historical Python host file has been recovered; supporting ESP32 firmware, ROS 2 setup files, and final configurations are being collected.
- The repository is being assembled and reviewed before public release.
Documentation and demo
Project poster, system diagrams, prototype photographs, and demonstration links will be organized here after checking their accuracy and sharing permissions.
Future improvements
- Reproduce the final tested configuration and document dependencies.
- Validate camera, thermal, and RF sensor behavior with larger and more varied datasets.
- Improve calibration and reduce false positives across different environments.
- Evaluate more autonomous navigation and integration with hotel operations.
Project context: Inha University, Engineering and Business Capstone Design (2026). Team SPECTRA.# WARDEN-Hotel-Inspection-Robot
Multi-modal robotic inspection system integrating computer vision, sensors, ROS 2, and real-time monitoring.
