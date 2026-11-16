#!/usr/bin/env python3
"""
WARDEN host - integrated inspection + monitoring + dashboard.

Runs on the Raspberry Pi (the robot's brain). It:
  * talks to the ESP32 sensor hub over serial (ENV / THERMAL / RF / SERVO / LED / STATUS / EVENT)
  * reads the C920 camera
  * drives the robot base from the joystick (left stick -> /cmd_vel)
  * ON DEMAND (X button / dashboard): runs a hidden-camera inspection sweep
      - servo pans the head, LED flashes, differential glint detection (image layer)
      - an AI vision model confirms whether a lens is actually present (AI layer)
      - optical + AI + thermal + RF fuse into one confidence score + verdict
      - reports WHERE the suspected device is (annotated frame + region label)
  * mirrors the guest TFT buttons onto the dashboard (EVENT -> alert banner):
      navigation request / call staff / EMERGENCY
  * serves a live web dashboard at http://<pi-ip>:8000

Run order (no lidar driver, to avoid the CP2102 port clash with the ESP32):
    ros2 launch ~/drive_only.launch.py     # motors (turtlebot3_node on OpenCR/ttyACM0)
    ros2 run joy joy_node                  # gamepad -> /joy
    python3 warden_host.py                 # ESP32 + camera + driving + dashboard
"""

import os
import time
import json
import base64
import threading
import urllib.request
from dataclasses import dataclass, field, asdict

import serial                 # pip install pyserial
import cv2                     # sudo apt install python3-opencv   (easier on a Pi)
import numpy as np
from flask import Flask, jsonify, render_template_string, send_from_directory

# ============================== CONFIG ==============================
class CFG:
    # --- ESP32 serial ---
    SERIAL_PORT = os.environ.get("WARDEN_SERIAL_PORT", "/dev/serial0")  # confirm physical wiring
    BAUD = 115200
    SERIAL_TIMEOUT = 2             # default read timeout (s) for fast commands
    RF_TIMEOUT = 6                 # Wi-Fi scan is slow - give RF a longer read window

    # --- camera ---
    CAMERA_INDEX = 0               # C920 on the Pi - try 0, then 1, 2...
    FRAME_W = 1280
    FRAME_H = 720

    # --- web dashboard ---
    SERVER_HOST = os.environ.get("WARDEN_SERVER_HOST", "127.0.0.1")  # explicitly opt into LAN access
    SERVER_PORT = 8000

    # captured inspection photos are written here and served at /captures/<name>
    CAPTURE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "captures")

    # --- always-on background sensors ---
    SENSOR_PERIOD = 2.0            # seconds between ENV/THERMAL/RF/EVENT polls


    # --- joystick: DRIVE + SCAN (single ROS node off the /joy topic) ---
    USE_ROS_JOY = True
    JOY_TOPIC = "/joy"
    DRIVE_TOPIC = "/cmd_vel"
    JOY_SCAN_BUTTON = 0            # X on your pad (verified via ros2 topic echo /joy)
    AXIS_LINEAR = 1               # stick up/down  -> forward/back
    AXIS_ANGULAR = 0              # stick left/right -> turn
    DRIVE_DEADZONE = 0.10
    BASE_LINEAR = 0.18            # m/s at full stick (Burger max ~0.22)
    BASE_ANGULAR = 1.20           # rad/s at full stick (Burger max ~2.84)

    # --- optical inspection sweep (servo pans the head while the LED pulses) ---
    SERVO_ANGLES = [75, 90, 110]            # down (75), center (90), up (110) - measured sweet spots
    SERVO_CENTER = 90                       # park angle when the scan finishes
    SERVO_WAKE = True                       # gentle nod when a scan starts (obvious it's moving)
    SERVO_SETTLE = 0.4                      # s to let the servo + camera settle before the shot
    SERVO_DWELL = 1.0                       # s to hold at each angle AFTER the shot (smooth pacing)
    SERVO_GLIDE = True                      # ease between angles in small steps instead of snapping
    SERVO_STEP = 4                          # deg per glide step (smaller = smoother, more serial traffic)
    SERVO_STEP_DT = 0.035                   # s between glide steps
    USE_LED = os.environ.get("WARDEN_ENABLE_LED", "0") == "1"  # off until power supply verified
    LED_WARMUP = 0.15
    LOCK_EXPOSURE = True                    # MUST lock or the frame brightens when the LED fires
    MANUAL_EXPOSURE_MODE = 0.25            # OpenCV/V4L2: 0.25 = manual, 0.75 = auto
    MANUAL_EXPOSURE_VALUE = 70             # fixed exposure (backend-dependent unit; tune 20-150)
    LOCK_WHITE_BALANCE = True
    MANUAL_WB_TEMP = 4600                  # fixed white-balance colour temp (K)
    DIFF_THRESH = 60                       # min brightening (0-255) to count as a glint
    STRONG_BONUS = 20                      # strong glint = brightening >= DIFF_THRESH + this
    MIN_AREA = 4                           # px - reject single-pixel sensor noise
    MAX_AREA = 600                         # px - reject large bright blobs

    # --- layer fusion:  C = wo*So + wa*Sa + wt*St + wr*Sr ;  suspicious if C >= tau ---
    #   So = min(1, Nstrong / OPTICAL_SAT)        Sa in [0,1] (AI vision)
    #   St = measured thermal hotspot indicator; Sr = nearby strong Wi-Fi networks
    # The model score is not a validated detection probability;
    # thermal and nearby Wi-Fi are contextual signals, not proof of a hidden camera.
    OPTICAL_SAT = 5                        # strong glints (peak per view) that saturate So
    THERMAL_DELTA = 3.0                    # degC hot-spot threshold (Tmax - Tmean)
    # Weights depend on whether the LED (optical layer) is alive.
    if USE_LED:                            # optical + AI co-primary (they must agree)
        W_OPTICAL = 0.40
        W_AI = 0.40
        W_THERMAL = 0.10
        W_RF = 0.10
    else:                                  # LED off -> optical is dead, AI is the decider
        W_OPTICAL = 0.00
        W_AI = 0.80
        W_THERMAL = 0.05
        W_RF = 0.15
    SUSPICIOUS_AT = 0.55                   # tau

    # --- AI vision (OpenAI) - confirms a camera lens in the best frame ---
    USE_AI = True
    AI_MODEL = "gpt-4o-mini"               # cheap vision model; "gpt-4o" for stronger
    AI_TIMEOUT = 15                        # seconds; fails soft if no internet / no key

    # NEVER simulate telemetry. Unplugged/failed sensors yield unavailable values.

    # --- safety: force-recover if a scan wedges (e.g. a frozen camera read) ---
    MAX_SCAN_SECONDS = 90  # includes up to three remote AI requests


def _load_env_file():
    """Load KEY=VALUE lines from a .env next to this script into os.environ.
    Dependency-free (no python-dotenv). A real environment variable always wins;
    only keys not already set get filled in. Tolerates blanks, # comments,
    an optional leading 'export', and quoted values."""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".env")
    try:
        with open(path, "r", encoding="utf-8") as f:
            for raw in f:
                line = raw.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                if line.startswith("export "):
                    line = line[len("export "):]
                key, _, val = line.partition("=")
                key = key.strip()
                val = val.strip().strip('"').strip("'")
                if key and key not in os.environ:
                    os.environ[key] = val
    except FileNotFoundError:
        print("[env] no .env file next to warden_host.py")
    except Exception as ex:
        print("[env] .env load skipped:", ex)


_load_env_file()
OPENAI_API_KEY = os.environ.get("OPENAI_API_KEY", "")
if not OPENAI_API_KEY:
    print("[env] OPENAI_API_KEY not set - AI vision will be OFF (verdict capped at RF weight)")


# ============================== SHARED STATE ==============================
@dataclass
class State:
    status: str = "idle"                         # idle | scanning
    driving: bool = False                        # base currently commanded to move
    scan_started: float = 0.0                    # epoch when current scan began (watchdog)
    env: dict = field(default_factory=dict)
    thermal: dict = field(default_factory=dict)
    rf: dict = field(default_factory=dict)
    last_inspection: dict = field(default_factory=dict)
    alert: str = "none"                          # guest TFT request: none|navigation|staff|emergency
    updated: float = 0.0
    sensor_errors: dict = field(default_factory=dict)


STATE = State()
STATE_LOCK = threading.Lock()
SERIAL_LOCK = threading.Lock()
CAMERA_LOCK = threading.Lock()
SCAN_REQUEST = threading.Event()
SCAN_ACTIVE = threading.Event()      # set during a sweep so the sensor loop yields the serial line


# ============================== ESP32 ==============================
class ESP32:
    """Thin wrapper over the serial link to the sensor hub."""

    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=CFG.SERIAL_TIMEOUT)
        time.sleep(2.0)                # ESP32 reboots when the port opens - wait it out
        self.ser.reset_input_buffer()

    def cmd(self, text, expect_json=False, read_timeout=None):
        """Bounded request/response; ignore asynchronous [touch] debug messages."""
        timeout = CFG.SERIAL_TIMEOUT if read_timeout is None else read_timeout
        with SERIAL_LOCK:
            self.ser.timeout = min(timeout, 0.6)
            self.ser.reset_input_buffer()
            self.ser.write((text + "\n").encode("ascii"))
            deadline = time.monotonic() + timeout
            try:
                while time.monotonic() < deadline:
                    line = self.ser.readline().decode("utf-8", errors="replace").strip()
                    if not line or line.startswith("["):
                        continue
                    if expect_json:
                        try:
                            reply = json.loads(line)
                        except json.JSONDecodeError:
                            continue
                        if isinstance(reply, dict):
                            return reply
                    else:
                        if line == "ERR":
                            raise RuntimeError(f"ESP32 rejected {text!r}")
                        if line in ("OK", "PONG"):
                            return line
                raise TimeoutError(f"ESP32 did not respond to {text!r} in {timeout}s")
            finally:
                self.ser.timeout = CFG.SERIAL_TIMEOUT


# ============================== ALWAYS-ON SENSORS ==============================
def sensor_loop(esp):
    """Read only actual ESP32 replies; report missing data instead of fabricated values."""
    polls = (("env", "ENV", CFG.SERIAL_TIMEOUT),
             ("thermal", "THERMAL", CFG.SERIAL_TIMEOUT),
             ("rf", "RF", CFG.RF_TIMEOUT),
             ("event", "EVENT", CFG.SERIAL_TIMEOUT))
    while True:
        if SCAN_ACTIVE.is_set():
            time.sleep(0.1)
            continue
        results = {}
        errors = {}
        for name, command, timeout in polls:
            if SCAN_ACTIVE.is_set():
                break
            try:
                reply = esp.cmd(command, expect_json=True, read_timeout=timeout)
                if not isinstance(reply, dict):
                    raise ValueError("expected JSON object")
                results[name] = reply
            except Exception as exc:
                results[name] = {}
                errors[name] = str(exc)
                print(f"[sensor] {command}: {exc}")
        else:
            with STATE_LOCK:
                STATE.env = results.get("env", {})
                thermal = results.get("thermal", {})
                # The firmware uses JSON null when the AMG8833 is absent.
                STATE.thermal = thermal if isinstance(thermal.get("mean"), (int, float)) else {}
                rf = results.get("rf", {})
                STATE.rf = rf if isinstance(rf.get("n"), int) and rf["n"] >= 0 else {}
                evt = results.get("event", {})
                if evt.get("alert") in ("none", "navigation", "staff", "emergency"):
                    STATE.alert = evt["alert"]
                STATE.sensor_errors = errors
                STATE.updated = time.time()
        time.sleep(CFG.SENSOR_PERIOD)


# ============================== YOLO (room audit) ==============================
# ============================== OPTICAL DETECTION ==============================
def find_glints(off, on):
    """Differential retroreflection: pixels that brighten a lot when the LED turns on."""
    g_off = cv2.cvtColor(off, cv2.COLOR_BGR2GRAY)
    g_on = cv2.cvtColor(on, cv2.COLOR_BGR2GRAY)
    diff = cv2.subtract(g_on, g_off)
    _, mask = cv2.threshold(diff, CFG.DIFF_THRESH, 255, cv2.THRESH_BINARY)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    glints = []
    for c in cnts:
        area = cv2.contourArea(c)
        if CFG.MIN_AREA <= area <= CFG.MAX_AREA:
            m = cv2.moments(c)
            if m["m00"] > 0:
                cx, cy = int(m["m10"] / m["m00"]), int(m["m01"] / m["m00"])
                glints.append({"x": cx, "y": cy, "area": float(area),
                               "bright": int(diff[cy, cx])})
    return glints


def annotate_and_save(frame, glints, angle):
    """Draw detected glints on the frame and save it for the dashboard. Returns filename."""
    img = frame.copy()
    for g in glints:
        strong = g["bright"] >= CFG.DIFF_THRESH + CFG.STRONG_BONUS
        color = (0, 0, 255) if strong else (0, 200, 255)   # red = strong, amber = weak
        cv2.circle(img, (g["x"], g["y"]), 12, color, 2)
    cv2.putText(img, f"tilt {angle} deg", (16, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA)
    fname = f"angle_{angle}.jpg"
    try:
        cv2.imwrite(os.path.join(CFG.CAPTURE_DIR, fname), img,
                    [int(cv2.IMWRITE_JPEG_QUALITY), 70])
    except Exception as ex:
        print("[inspect] save error:", ex)
        return ""
    return fname


def detection_region(frame_shape, strong_glints, ai_box):
    """Where is the suspected device? Bounding box (px) + a human-readable location,
    fusing the strong-glint cluster with the AI lens box (if any)."""
    h, w = frame_shape[:2]
    xs, ys = [], []
    for g in strong_glints:
        xs.append(g["x"]); ys.append(g["y"])
    if ai_box and len(ai_box) == 4:
        bx, by, bw, bh = ai_box
        xs += [int(bx * w), int((bx + bw) * w)]
        ys += [int(by * h), int((by + bh) * h)]
    if not xs:
        return None
    x0, x1 = max(0, min(xs)), min(w - 1, max(xs))
    y0, y1 = max(0, min(ys)), min(h - 1, max(ys))
    cx, cy = (x0 + x1) // 2, (y0 + y1) // 2
    col = ["left", "center", "right"][min(2, cx * 3 // w)]
    row = ["upper", "middle", "lower"][min(2, cy * 3 // h)]
    where = "center" if (col == "center" and row == "middle") else f"{row}-{col}"
    return {"box": [x0, y0, x1 - x0, y1 - y0], "cx": cx, "cy": cy, "where": where}


def lock_camera(cap):
    """Pin exposure + white balance so the LED-off / LED-on pair is directly comparable."""
    if CFG.LOCK_EXPOSURE:
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, CFG.MANUAL_EXPOSURE_MODE)
        cap.set(cv2.CAP_PROP_EXPOSURE, CFG.MANUAL_EXPOSURE_VALUE)
    if CFG.LOCK_WHITE_BALANCE:
        try:
            cap.set(cv2.CAP_PROP_AUTO_WB, 0)
            cap.set(cv2.CAP_PROP_WB_TEMPERATURE, CFG.MANUAL_WB_TEMP)
        except Exception:
            pass


def capture_pair(esp, cap):
    """Grab one LED-OFF and one LED-ON frame with matched, locked exposure.
    If USE_LED is False the optical layer is skipped: grab a single frame,
    return it as 'on' (AI + annotation still use it) and 'off'=None so
    find_glints yields nothing and the optical score stays 0."""
    lock_camera(cap)

    if not CFG.USE_LED:
        for _ in range(3):
            cap.read()
        ok, frame = cap.read()
        return None, (frame if ok else None)

    esp.cmd("LED OFF")
    time.sleep(0.2)
    for _ in range(3):
        cap.read()
    ok1, off = cap.read()

    esp.cmd("LED ON")
    time.sleep(CFG.LED_WARMUP)
    for _ in range(2):
        cap.read()
    ok2, on = cap.read()

    esp.cmd("LED OFF")
    return (off if ok1 else None), (on if ok2 else None)


# ============================== AI VISION (image detection) ==============================
def ai_vision_check(frame_bgr):
    """Ask an OpenAI vision model whether a camera lens is visible in the frame.
    Fails soft: any error (no key, no internet, bad response) returns 'not present'."""
    if not CFG.USE_AI or not OPENAI_API_KEY or frame_bgr is None:
        return {"available": False, "present": False, "confidence": 0.0, "box": None, "note": "ai off"}
    try:
        ok, buf = cv2.imencode(".jpg", frame_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        if not ok:
            return {"available": False, "present": False, "confidence": 0.0, "box": None, "note": "encode failed"}
        b64 = base64.b64encode(buf.tobytes()).decode()
        prompt = ("Inspect this photo for any camera lens - phone cameras, webcams, or "
                  "concealed/spy/pinhole cameras (a small dark circle or glassy dot). "
                  "If you find one, give its location as a bounding box in fractions of the "
                  "image (0..1). Respond ONLY with compact JSON, no markdown: "
                  '{"camera_present": true|false, "confidence": 0.0-1.0, '
                  '"box": [x, y, w, h], "note": "<short what/where>"}. '
                  "Use the box of the most likely lens; if none, set box to null.")
        payload = {
            "model": CFG.AI_MODEL,
            "messages": [{
                "role": "user",
                "content": [
                    {"type": "text", "text": prompt},
                    {"type": "image_url",
                     "image_url": {"url": f"data:image/jpeg;base64,{b64}"}},
                ],
            }],
            "max_tokens": 150,
            "temperature": 0,
        }
        req = urllib.request.Request(
            "https://api.openai.com/v1/chat/completions",
            data=json.dumps(payload).encode(),
            headers={"Content-Type": "application/json",
                     "Authorization": f"Bearer {OPENAI_API_KEY}"},
        )
        with urllib.request.urlopen(req, timeout=CFG.AI_TIMEOUT) as r:
            resp = json.loads(r.read().decode())
        text = resp["choices"][0]["message"]["content"].strip()
        text = text.replace("```json", "").replace("```", "").strip()
        data = json.loads(text)
        box = data.get("box")
        if not (isinstance(box, list) and len(box) == 4):
            box = None
        if not isinstance(data.get("camera_present"), bool):
            raise ValueError("AI response missing boolean camera_present")
        score = float(data.get("confidence"))
        if not 0 <= score <= 1:
            raise ValueError("AI confidence outside [0,1]")
        return {"available": True, "present": data["camera_present"],
                "confidence": score, "box": box,
                "note": str(data.get("note", ""))[:120]}
    except Exception as ex:
        print("[ai] vision check failed:", ex)
        return {"available": False, "present": False, "confidence": 0.0, "box": None, "note": "ai unavailable"}


def servo_glide(esp, frm, to):
    """Ease the head from frm->to in small steps so it glides instead of snapping.
    Returns the angle now held. The sensor loop is paused during a scan, so the
    serial line is free and these small per-step commands stay responsive."""
    if (not CFG.SERVO_GLIDE) or frm is None or to == frm:
        esp.cmd(f"SERVO {to}")
        return to
    step = CFG.SERVO_STEP if to > frm else -CFG.SERVO_STEP
    a = frm
    while a != to:
        a += step
        if (step > 0 and a > to) or (step < 0 and a < to):
            a = to
        esp.cmd(f"SERVO {a}")
        time.sleep(CFG.SERVO_STEP_DT)
    return to


# ============================== INSPECTION ==============================
def run_inspection(esp, cap):
    """One full sweep: servo pans, capture per angle, glint detection,
    AI lens check on every frame (max confidence), fuse optical+AI+thermal+RF, locate the device."""
    angles = CFG.SERVO_ANGLES
    STRONG = CFG.DIFF_THRESH + CFG.STRONG_BONUS
    SCAN_ACTIVE.set()            # pause background sensor polling - the sweep owns the serial line

    with STATE_LOCK:
        STATE.status = "scanning"
        STATE.scan_started = time.time()
    time.sleep(0.15)  # let teleop receive /warden/scan_active before the head moves
    try:
        esp.cmd("STATUS 0 0 255")     # blue = scanning
        per_angle = []
        shots = []                 # one entry per capture: angle, frame, glints, strong-count
        best_on = None
        best_angle = None
        best_glints = []

        with CAMERA_LOCK:
            cur = CFG.SERVO_CENTER            # head parks at center between scans
            # gentle nod so it's obvious the scan just started (no snappy double-move)
            if CFG.SERVO_WAKE:
                cur = servo_glide(esp, cur, max(0, CFG.SERVO_CENTER - 12))
                time.sleep(0.15)
                cur = servo_glide(esp, cur, CFG.SERVO_CENTER)
                time.sleep(0.15)
            lock_camera(cap)

            for ang in angles:
                cur = servo_glide(esp, cur, ang)     # ease the head to this angle
                time.sleep(CFG.SERVO_SETTLE)         # let it settle before the shot
                off, on = capture_pair(esp, cap)     # take the picture
                glints = find_glints(off, on) if (off is not None and on is not None) else []
                strong = [g for g in glints if g["bright"] >= STRONG]
                img = annotate_and_save(on, glints, ang) if on is not None else ""
                per_angle.append({
                    "angle": ang,
                    "glints": len(glints),
                    "strong": len(strong),
                    "max_bright": max((g["bright"] for g in glints), default=0),
                    "img": img,
                })
                if on is not None:
                    shots.append({"angle": ang, "frame": on,
                                  "glints": glints, "strong": len(strong)})
                time.sleep(CFG.SERVO_DWELL)          # hold a beat before moving on
            cur = servo_glide(esp, cur, CFG.SERVO_CENTER)   # ease back to center

        peak_strong = max((s["strong"] for s in shots), default=0)

        # ---- AI vision on EVERY captured frame; keep the most confident result ----
        ai = None
        ai_angle = None
        for s in shots:
            r = ai_vision_check(s["frame"])
            if ai is None or (r.get("available") and not ai.get("available")) or (r.get("available") == ai.get("available") and r.get("confidence", 0.0) > ai.get("confidence", 0.0)):
                ai, ai_angle = r, s["angle"]
        if ai is None:
            ai = {"available": False, "present": False, "confidence": 0.0, "box": None, "note": "no frames"}
        ai_box = ai.get("box")

        # frame to localise/annotate: AI's frame if it hit, else the strongest-glint frame
        chosen = None
        if ai.get("present") and ai_angle is not None:
            chosen = next((s for s in shots if s["angle"] == ai_angle), None)
        if chosen is None and shots:
            chosen = max(shots, key=lambda s: s["strong"])
        if chosen is not None:
            best_on, best_angle, best_glints = chosen["frame"], chosen["angle"], chosen["glints"]

        # ---- four-layer fusion ----
        with STATE_LOCK:
            thermal, rf = STATE.thermal, STATE.rf
        s_optical = min(1.0, peak_strong / float(CFG.OPTICAL_SAT))
        s_ai = ai["confidence"] if ai.get("present") else 0.0
        thermal_valid = all(isinstance(thermal.get(k), (int, float)) for k in ("min", "max", "mean"))
        rf_valid = isinstance(rf.get("n"), int) and isinstance(rf.get("strong"), int)
        s_thermal = (1.0 if thermal["max"] - thermal["mean"] > CFG.THERMAL_DELTA else 0.0) if thermal_valid else None
        s_rf = (1.0 if rf["strong"] > 0 else 0.0) if rf_valid else None
        # Fusion score is a heuristic, not a calibrated probability.
        confidence = (CFG.W_OPTICAL * s_optical + CFG.W_AI * s_ai
                      + CFG.W_THERMAL * (s_thermal or 0) + CFG.W_RF * (s_rf or 0))
        verdict = ("INCONCLUSIVE" if not ai.get("available") or not shots else
                   "SUSPICIOUS" if confidence >= CFG.SUSPICIOUS_AT else "clear")

        # ---- where is it? region from the strong-glint cluster (+ AI box) ----
        area = None
        angles_with_strong = sum(1 for a in per_angle if a["strong"] > 0)
        if best_on is not None:
            strong_best = [g for g in best_glints if g["bright"] >= STRONG]
            area = detection_region(best_on.shape, strong_best, ai_box)
            try:
                img = best_on.copy()
                h, w = img.shape[:2]
                for g in best_glints:
                    cv2.circle(img, (g["x"], g["y"]), 12,
                               (0, 0, 255) if g["bright"] >= STRONG else (0, 200, 255), 2)
                if ai_box and len(ai_box) == 4:
                    bx, by, bw, bh = ai_box
                    cv2.rectangle(img, (int(bx * w), int(by * h)),
                                  (int((bx + bw) * w), int((by + bh) * h)), (0, 255, 0), 2)
                    cv2.putText(img, "AI lens", (int(bx * w), max(18, int(by * h) - 6)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)
                if area:
                    x, y, bw, bh = area["box"]; pad = 16
                    cv2.rectangle(img, (max(0, x - pad), max(0, y - pad)),
                                  (min(w - 1, x + bw + pad), min(h - 1, y + bh + pad)),
                                  (0, 90, 255), 3)
                    if verdict == "SUSPICIOUS":
                        cv2.putText(img, f"SUSPECT: {area['where']}",
                                    (max(0, x - pad), max(24, y - pad - 8)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 90, 255), 2, cv2.LINE_AA)
                cv2.putText(img, f"tilt {best_angle} deg", (16, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA)
                if best_angle is not None:
                    cv2.imwrite(os.path.join(CFG.CAPTURE_DIR, f"angle_{best_angle}.jpg"), img,
                                [int(cv2.IMWRITE_JPEG_QUALITY), 72])
            except Exception as ex:
                print("[inspect] best-frame annotate error:", ex)

        result = {
            "time": time.strftime("%H:%M:%S"),
            "verdict": verdict,
            "confidence": round(confidence, 2),
            "threshold": CFG.SUSPICIOUS_AT,
            "optical_score": round(s_optical, 2),
            "ai_present": ai.get("present", False),
            "ai_available": ai.get("available", False),
            "ai_confidence": round(ai.get("confidence", 0.0), 2),
            "ai_note": ai.get("note", ""),
            "thermal_hot": (bool(s_thermal) if s_thermal is not None else None),
            "rf_present": (bool(s_rf) if s_rf is not None else None),
            "angles_with_strong": angles_with_strong,
            "peak_strong": peak_strong,
            "area": area,                 # {box:[x,y,w,h], cx, cy, where} or None
            "best_angle": best_angle,
            "per_angle": per_angle,
            "scores": {"optical": round(s_optical, 2), "ai": round(s_ai, 2),
                       "thermal": s_thermal, "rf": s_rf},
            "weights": {"optical": CFG.W_OPTICAL, "ai": CFG.W_AI,
                        "thermal": CFG.W_THERMAL, "rf": CFG.W_RF},
        }
        with STATE_LOCK:
            STATE.last_inspection = result
            STATE.updated = time.time()
        esp.cmd("STATUS 255 0 0" if verdict == "SUSPICIOUS" else "STATUS 0 0 255" if verdict == "INCONCLUSIVE" else "STATUS 0 255 0")
        loc = area["where"] if area else "n/a"
        print(f"[inspect] {verdict} C={result['confidence']} "
              f"(opt {s_optical:.2f} ai {s_ai:.2f} th {s_thermal} rf {s_rf}) "
              f"peak_strong={peak_strong} area={loc}")
        return result

    finally:
        SCAN_ACTIVE.clear()          # resume background sensor polling
        with STATE_LOCK:
            STATE.status = "idle"
            STATE.scan_started = 0.0


def inspection_worker(esp, cap):
    """Waits for a scan request (X button / dashboard), then runs one inspection."""
    while True:
        SCAN_REQUEST.wait()
        SCAN_REQUEST.clear()
        if STATE.status == "scanning":
            continue
        try:
            run_inspection(esp, cap)
        except Exception as ex:
            print("[inspect] error:", ex)
            with STATE_LOCK:
                STATE.status = "idle"
                STATE.scan_started = 0.0


# ============================== ROS /joy: DRIVE + SCAN ==============================
def ros_joy_loop():
    """Receive scan requests, observe real velocity and publish scan pause state.

    IMPORTANT: Teleop is the *only* publisher to /cmd_vel. This host never drives
    the robot, preventing competing velocity commands.
    """
    if not CFG.USE_ROS_JOY:
        return
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Joy
        from geometry_msgs.msg import Twist
        from std_msgs.msg import Bool
    except Exception as exc:
        print("[joy] ROS modules unavailable; scan interlock unavailable:", exc)
        return

    class JoyScanMonitor(Node):
        def __init__(self):
            super().__init__("warden_scan_monitor")
            self.create_subscription(Joy, CFG.JOY_TOPIC, self.joy_cb, 10)
            self.create_subscription(Twist, CFG.DRIVE_TOPIC, self.drive_cb, 10)
            self.pause_pub = self.create_publisher(Bool, "/warden/scan_active", 10)
            self.prev_scan = 0
            self.last_drive_at = 0.0
            self.create_timer(0.05, self.tick)

        def joy_cb(self, msg):
            b = CFG.JOY_SCAN_BUTTON
            if b < len(msg.buttons):
                cur = msg.buttons[b]
                if cur == 1 and self.prev_scan == 0:
                    print("[joy] scan button pressed")
                    SCAN_REQUEST.set()
                self.prev_scan = cur

        def drive_cb(self, msg):
            self.last_drive_at = time.monotonic()
            with STATE_LOCK:
                STATE.driving = abs(msg.linear.x) > 0.001 or abs(msg.angular.z) > 0.001

        def tick(self):
            with STATE_LOCK:
                scanning = STATE.status == "scanning"
                if time.monotonic() - self.last_drive_at > 0.5:
                    STATE.driving = False
            self.pause_pub.publish(Bool(data=scanning))

    rclpy.init()
    node = JoyScanMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


# ============================== WATCHDOG ==============================
def scan_watchdog():
    warned = False
    while True:
        time.sleep(1.0)
        with STATE_LOCK:
            scanning = STATE.status == "scanning" and STATE.scan_started > 0
            stuck = scanning and time.time() - STATE.scan_started > CFG.MAX_SCAN_SECONDS
        if stuck and not warned:
            print(f"[watchdog] scan exceeded {CFG.MAX_SCAN_SECONDS}s; keeping base paused until worker finishes")
        warned = stuck


# ============================== DASHBOARD ==============================
app = Flask(__name__)

DASHBOARD = """<!doctype html>
<html lang="en"><head><meta charset="utf-8"><title>WARDEN</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
  :root{
    --bg:#0a0c10; --panel:#12161d; --panel2:#161b24; --line:#232a35;
    --ink:#e8edf4; --muted:#7d8896; --dim:#586473;
    --amber:#ffb020; --green:#34d27b; --red:#ff4d5e; --blue:#3d9bff;
    --mono:"SF Mono","JetBrains Mono","Fira Code",ui-monospace,"Cascadia Code",Menlo,Consolas,monospace;
    --sans:ui-sans-serif,-apple-system,"Segoe UI",Roboto,sans-serif;
  }
  *{box-sizing:border-box}
  body{margin:0;background:
      radial-gradient(1200px 600px at 80% -10%, #14202b 0%, transparent 60%),
      radial-gradient(900px 500px at -10% 110%, #1a1320 0%, transparent 55%),
      var(--bg);
    color:var(--ink);font-family:var(--sans);
    background-attachment:fixed;min-height:100vh;
    -webkit-font-smoothing:antialiased}
  .wrap{max-width:1180px;margin:0 auto;padding:22px 20px 60px}
  .topbar{display:flex;align-items:center;justify-content:space-between;
    border-bottom:1px solid var(--line);padding-bottom:14px;margin-bottom:22px}
  .brand{font-family:var(--mono);font-weight:700;font-size:20px;letter-spacing:6px}
  .brand .dot{color:var(--amber)}
  .brand small{display:block;font-size:10px;letter-spacing:3px;color:var(--dim);
    font-weight:500;margin-top:3px}
  .pills{display:flex;gap:8px;align-items:center}
  .pill{font-family:var(--mono);font-size:11px;letter-spacing:1.5px;text-transform:uppercase;
    padding:6px 12px;border-radius:6px;border:1px solid var(--line);color:var(--muted);
    background:var(--panel)}
  .pill.on{color:var(--bg);border-color:transparent}
  .pill.idle.on{background:var(--blue)} .pill.scanning.on{background:var(--amber)}
  .pill.drive.on{background:var(--green)}

  /* guest assistant alert banner (TFT -> dashboard) */
  .alertbar{display:none;align-items:center;gap:14px;border-radius:12px;padding:16px 20px;
    margin-bottom:18px;font-family:var(--mono);letter-spacing:.5px;border:1px solid var(--line)}
  .alertbar.show{display:flex}
  .alertbar .ai{font-size:24px;line-height:1}
  .alertbar .at{font-size:15px;font-weight:700}
  .alertbar .asub{font-size:12px;color:var(--muted);margin-top:3px}
  .alertbar.navigation{background:rgba(61,155,255,.12);border-color:var(--blue);color:var(--blue)}
  .alertbar.staff{background:rgba(255,176,32,.12);border-color:var(--amber);color:var(--amber)}
  .alertbar.emergency{background:rgba(255,77,94,.16);border-color:var(--red);color:var(--red);
    padding:20px 24px;animation:wpulse 1s ease-in-out infinite}
  .alertbar.emergency .at{font-size:19px;letter-spacing:1px}
  @keyframes wpulse{0%,100%{box-shadow:0 0 0 0 rgba(255,77,94,0)}50%{box-shadow:0 0 26px 3px rgba(255,77,94,.5)}}

  /* verdict banner */
  .verdict{position:relative;overflow:hidden;border:1px solid var(--line);
    border-radius:14px;background:linear-gradient(180deg,var(--panel2),var(--panel));
    padding:22px 24px;margin-bottom:20px;display:flex;align-items:center;
    justify-content:space-between;gap:18px}
  .verdict::before{content:"";position:absolute;inset:0;width:6px;background:var(--dim)}
  .verdict.clear::before{background:var(--green)}
  .verdict.SUSPICIOUS::before{background:var(--red)}
  .vmain{display:flex;flex-direction:column;gap:4px;padding-left:8px}
  .vlabel{font-family:var(--mono);font-size:11px;letter-spacing:2px;color:var(--muted);
    text-transform:uppercase}
  .vtext{font-family:var(--mono);font-size:34px;font-weight:700;letter-spacing:1px}
  .vtext.clear{color:var(--green)} .vtext.SUSPICIOUS{color:var(--red)} .vtext.INCONCLUSIVE{color:var(--amber)} .vtext.none{color:var(--dim)}
  .vmeta{font-size:12px;color:var(--muted);font-family:var(--mono)}
  .varea{font-size:12.5px;color:var(--red);font-family:var(--mono);letter-spacing:.3px;margin-top:2px}
  .conf{text-align:right;font-family:var(--mono)}
  .conf b{font-size:40px;font-weight:700;display:block;line-height:1}
  .conf span{font-size:11px;letter-spacing:2px;color:var(--muted);text-transform:uppercase}

  .grid{display:grid;grid-template-columns:1.4fr 1fr;gap:16px}
  @media(max-width:820px){.grid{grid-template-columns:1fr}}
  .card{background:var(--panel);border:1px solid var(--line);border-radius:12px;padding:16px}
  .card h2{font-family:var(--mono);font-size:11px;letter-spacing:2px;color:var(--muted);
    text-transform:uppercase;margin:0 0 14px;display:flex;justify-content:space-between}
  .card h2 .hint{color:var(--dim);letter-spacing:1px}

  /* tilt gauge */
  .gauge{display:flex;align-items:center;gap:18px;flex-wrap:wrap}
  .gauge svg{flex:0 0 auto}
  .legend{font-family:var(--mono);font-size:11px;color:var(--muted);line-height:1.9}
  .legend .sw{display:inline-block;width:10px;height:10px;border-radius:2px;margin-right:6px;
    vertical-align:middle}
  #aisummary{font-family:var(--mono);font-size:12px;color:var(--muted);margin-top:12px;
    padding-top:11px;border-top:1px solid var(--line)}

  /* fusion bars */
  .fz{display:grid;grid-template-columns:64px 1fr 66px;align-items:center;gap:10px;
    font-family:var(--mono);font-size:11.5px;color:var(--muted);padding:6px 0}
  .fz .bar{height:6px;border-radius:4px;background:#1b2230;overflow:hidden}
  .fz .bar i{display:block;height:100%;width:0;border-radius:4px;transition:width .35s ease}
  .fz b{font-size:11.5px;color:var(--ink);text-align:right}
  .fz.opt .bar i{background:var(--red)} .fz.ai .bar i{background:var(--green)}
  .fz.th .bar i{background:var(--amber)} .fz.rf .bar i{background:var(--blue)}

  /* photo gallery */
  .shots{display:grid;grid-template-columns:repeat(auto-fill,minmax(150px,1fr));gap:10px}
  .shot{border:1px solid var(--line);border-radius:8px;overflow:hidden;background:#000;position:relative}
  .shot img{display:block;width:100%;height:auto;aspect-ratio:16/9;object-fit:cover}
  .shot .tag{position:absolute;top:6px;left:6px;font-family:var(--mono);font-size:10px;
    padding:2px 7px;border-radius:4px;background:rgba(0,0,0,.65);color:var(--ink);letter-spacing:1px}
  .shot .hit{position:absolute;bottom:6px;right:6px;font-family:var(--mono);font-size:10px;
    padding:2px 7px;border-radius:4px;letter-spacing:.5px}
  .shot .hit.s{background:var(--red);color:#fff} .shot .hit.n{background:var(--panel2);color:var(--muted)}
  .empty{color:var(--dim);font-family:var(--mono);font-size:12px;padding:14px 2px}

  /* telemetry */
  .kv{display:flex;justify-content:space-between;font-size:13px;padding:5px 0;
    border-bottom:1px dashed var(--line)}
  .kv:last-child{border-bottom:0}
  .kv span{color:var(--muted)} .kv b{font-family:var(--mono);font-weight:600}
  .kv b.off{color:var(--dim)}
  .stack{display:flex;flex-direction:column;gap:16px}
  .foot{margin-top:22px;font-family:var(--mono);font-size:11px;color:var(--dim);
    display:flex;justify-content:space-between;letter-spacing:1px}
</style></head><body>
<div class="wrap">
  <div class="topbar">
    <div class="brand">WARDEN<span class="dot">.</span><small>AI // OPTICAL // THERMAL // RF</small></div>
    <div class="pills">
      <div id="p-status" class="pill idle on">idle</div>
      <div id="p-drive" class="pill drive">drive</div>
      <button onclick="scan()" class="pill" style="cursor:pointer;color:var(--amber);border-color:var(--amber)">▶ run scan</button>
    </div>
  </div>

  <div id="alert" class="alertbar"></div>

  <div id="verdict" class="verdict">
    <div class="vmain">
      <div class="vlabel">last inspection</div>
      <div id="vtext" class="vtext none">— — —</div>
      <div id="vmeta" class="vmeta">no scan yet</div>
      <div id="varea" class="varea"></div>
    </div>
    <div class="conf"><b id="conf">--</b><span>confidence</span></div>
  </div>

  <div class="grid">
    <div class="stack">
      <div class="card">
        <h2>camera sweep <span class="hint" id="sweep-hint">head tilt</span></h2>
        <div class="gauge">
          <svg id="arc" width="220" height="130" viewBox="0 0 220 130"></svg>
          <div class="legend">
            <div><span class="sw" style="background:var(--red)"></span>strong glint (lens-like)</div>
            <div><span class="sw" style="background:var(--amber)"></span>weak / single reflection</div>
            <div><span class="sw" style="background:var(--green)"></span>AI lens region</div>
            <div><span class="sw" style="background:var(--dim)"></span>angle swept, clear</div>
          </div>
        </div>
        <div id="aisummary">AI: no scan yet</div>
      </div>
      <div class="card">
        <h2>captured frames <span class="hint">orange = suspect area · green = AI lens · rings = glints</span></h2>
        <div id="shots" class="shots"></div>
      </div>
    </div>

    <div class="stack">
      <div class="card"><h2>signal fusion <span class="hint">C ≥ 0.55 → suspicious</span></h2>
        <div class="fz opt"><span>optical</span><div class="bar"><i id="fz-opt"></i></div><b id="fzv-opt">--</b></div>
        <div class="fz ai"><span>ai vision</span><div class="bar"><i id="fz-ai"></i></div><b id="fzv-ai">--</b></div>
        <div class="fz th"><span>thermal</span><div class="bar"><i id="fz-th"></i></div><b id="fzv-th">--</b></div>
        <div class="fz rf"><span>rf</span><div class="bar"><i id="fz-rf"></i></div><b id="fzv-rf">--</b></div>
        <div class="kv" style="margin-top:10px"><span>weighted C</span><b id="fz-total">--</b></div>
      </div>
      <div class="card"><h2>environment</h2>
        <div class="kv"><span>temperature</span><b id="t">--</b></div>
        <div class="kv"><span>humidity</span><b id="h">--</b></div>
        <div class="kv"><span>pressure</span><b id="p">--</b></div>
        <div class="kv"><span>gas sensor (raw ADC, not ppm)</span><b id="gas">--</b></div>
      </div>
      <div class="card"><h2>thermal</h2>
        <div class="kv"><span>min</span><b id="tmin">--</b></div>
        <div class="kv"><span>max</span><b id="tmax">--</b></div>
        <div class="kv"><span>mean</span><b id="tmean">--</b></div>
        <div class="kv"><span>hot spot</span><b id="thot">--</b></div>
      </div>
      <div class="card"><h2>rf (wi-fi)</h2>
        <div class="kv"><span>devices</span><b id="rfn">--</b></div>
        <div class="kv"><span>strong</span><b id="rfs">--</b></div>
      </div>
    </div>
  </div>

  <div class="foot"><span id="upd">awaiting telemetry…</span><span>WARDEN host · :8000</span></div>
</div>

<script>
const ANGLES = [75,90,110];
async function scan(){ try{ await fetch('/scan',{method:'POST'}); }catch(e){} }
function set(id,v){ const el=document.getElementById(id); if(el) el.textContent=v; }
function fz(id, score, wt){
  const bar=document.getElementById('fz-'+id), val=document.getElementById('fzv-'+id);
  const s=(typeof score==='number')? score : null;
  if(bar) bar.style.width=((s!=null? Math.max(0,Math.min(1,s)) : 0)*100)+'%';
  if(val) val.textContent=(s!=null? s.toFixed(2):'--')+(wt!=null? '  ×'+wt : '');
}

function drawArc(perAngle){
  const svg=document.getElementById('arc'); svg.innerHTML='';
  const cx=110, cy=120, r=92;
  const NS='http://www.w3.org/2000/svg';
  const base=document.createElementNS(NS,'path');
  const p=a=>{const rad=Math.PI*(1-a/180);return [cx+r*Math.cos(rad), cy-r*Math.sin(rad)];};
  const [x0,y0]=p(0), [x1,y1]=p(180);
  base.setAttribute('d',`M ${x0} ${y0} A ${r} ${r} 0 0 1 ${x1} ${y1}`);
  base.setAttribute('fill','none'); base.setAttribute('stroke','#232a35'); base.setAttribute('stroke-width','3');
  svg.appendChild(base);
  const byAngle={}; (perAngle||[]).forEach(a=>byAngle[a.angle]=a);
  ANGLES.forEach(ang=>{
    const [x,y]=p(ang); const d=byAngle[ang];
    const dot=document.createElementNS(NS,'circle');
    dot.setAttribute('cx',x); dot.setAttribute('cy',y); dot.setAttribute('r','7');
    let c='#586473';
    if(d){ if(d.strong>0) c='#ff4d5e'; else if(d.glints>0) c='#ffb020'; }
    dot.setAttribute('fill',c); dot.setAttribute('stroke','#0a0c10'); dot.setAttribute('stroke-width','2');
    svg.appendChild(dot);
    const lbl=document.createElementNS(NS,'text');
    const [lx,ly]=p(ang);
    lbl.setAttribute('x',lx); lbl.setAttribute('y',ang===90?ly-14:ly-12);
    lbl.setAttribute('fill','#7d8896'); lbl.setAttribute('font-size','10');
    lbl.setAttribute('font-family','monospace'); lbl.setAttribute('text-anchor','middle');
    lbl.textContent=ang+'°'; svg.appendChild(lbl);
  });
}

function drawShots(perAngle, t, bestAngle){
  const box=document.getElementById('shots');
  if(!perAngle || !perAngle.length){ box.innerHTML='<div class="empty">no captures yet — run a scan</div>'; return; }
  box.innerHTML='';
  perAngle.forEach(a=>{
    const cell=document.createElement('div'); cell.className='shot';
    if(a.img){
      const im=document.createElement('img');
      im.src='/captures/'+a.img+'?t='+t; im.alt='tilt '+a.angle;
      cell.appendChild(im);
    }
    const tag=document.createElement('div'); tag.className='tag';
    tag.textContent=(a.angle===bestAngle?'★ ':'')+a.angle+'°';
    const hit=document.createElement('div');
    hit.className='hit '+(a.strong>0?'s':'n');
    hit.textContent=(a.strong>0? a.strong+' strong' : a.glints+' glint');
    cell.appendChild(tag); cell.appendChild(hit); box.appendChild(cell);
  });
}

async function tick(){
  try{
    const d=await (await fetch('/data')).json();
    const ps=document.getElementById('p-status');
    ps.textContent=d.status; ps.className='pill '+d.status+' on';
    document.getElementById('p-drive').className='pill drive'+(d.driving?' on':'');

    const e=d.env||{};
    set('t', e.t!=null? e.t+' °C':'--'); set('h', e.h!=null? e.h+' %':'--');
    set('p', e.p!=null? e.p+' hPa':'--'); set('gas', e.gas!=null? e.gas:'--');
    const th=d.thermal||{};
    set('tmin', th.min!=null? th.min+' °C':'--'); set('tmax', th.max!=null? th.max+' °C':'--');
    set('tmean', th.mean!=null? th.mean+' °C':'--');
    const rf=d.rf||{};
    set('rfn', rf.n!=null? rf.n:'--'); set('rfs', rf.strong!=null? rf.strong:'--');

    const ins=d.last_inspection||{};
    const v=ins.verdict||null;
    const sc=ins.scores||{}; const wt=ins.weights||{};
    const vt=document.getElementById('vtext');
    vt.textContent = v? (v==='SUSPICIOUS'?'SUSPICIOUS':(v==='INCONCLUSIVE'?'INCONCLUSIVE':'CLEAR')) : '— — —';
    vt.className='vtext '+(v||'none');
    document.getElementById('verdict').className='verdict '+(v||'');
    set('conf', ins.confidence!=null? ins.confidence:'--');
    set('thot', ins.thermal_hot===null || ins.thermal_hot===undefined? '--' : (ins.thermal_hot?'yes':'no'));
    const thotEl=document.getElementById('thot'); if(thotEl) thotEl.className=ins.thermal_hot?'':'off';

    // fusion bars
    fz('opt', sc.optical, wt.optical); fz('ai', sc.ai, wt.ai);
    fz('th', sc.thermal, wt.thermal); fz('rf', sc.rf, wt.rf);
    set('fz-total', ins.confidence!=null? (ins.confidence+'  /  '+(ins.threshold!=null?ins.threshold:0.55)) : '--');

    document.getElementById('vmeta').textContent = ins.time?
      ('C '+ins.confidence+' · '+ins.angles_with_strong+'/'+ANGLES.length+' angles strong · AI '+(ins.ai_confidence!=null?ins.ai_confidence:'–')+' · '+ins.time)
      : 'no scan yet';
    document.getElementById('aisummary').textContent = ins.time?
      (ins.ai_present? ('AI: ⚠ lens ('+ins.ai_confidence+')'+(ins.ai_note? ' — '+ins.ai_note : ''))
                     : ((ins.ai_available ? 'AI: no lens detected' : 'AI: unavailable / not evaluated')+(ins.ai_note? ' — '+ins.ai_note : '')))
      : 'AI: no scan yet';
    const ar=ins.area; const va=document.getElementById('varea');
    if(va){
      if(v==='SUSPICIOUS' && ar) va.textContent='▣ suspected device: '+ar.where+'  ·  box ['+ar.box.join(', ')+'] px';
      else va.textContent='';
    }

    drawArc(ins.per_angle);
    drawShots(ins.per_angle, Math.floor((d.updated||0)), ins.best_angle);

    // guest TFT alert -> dashboard banner
    const al=d.alert||'none';
    const ab=document.getElementById('alert');
    if(al==='none'){ ab.className='alertbar'; ab.innerHTML=''; }
    else{
      const m={navigation:['🧭','Navigation request','Request shown on this dashboard'],
               staff:['🛎','Guest requested staff','Request shown on this dashboard'],
               emergency:['⚠','EMERGENCY ALERT','Guest request received — human response required']}[al]||['','',''];
      ab.className='alertbar show '+al;
      ab.innerHTML='<div class="ai">'+m[0]+'</div><div><div class="at">'+m[1]+'</div><div class="asub">'+m[2]+'</div></div>';
    }
    set('upd', d.updated? 'updated '+new Date(d.updated*1000).toLocaleTimeString() : 'awaiting telemetry…');
  }catch(err){ /* transient */ }
}
drawArc([]); drawShots([],0);
setInterval(tick,1000); tick();
</script>
</body></html>"""


@app.route("/")
def index():
    return render_template_string(DASHBOARD)


@app.route("/data")
def data():
    with STATE_LOCK:
        return jsonify(asdict(STATE))


@app.route("/captures/<path:fname>")
def captures(fname):
    return send_from_directory(CFG.CAPTURE_DIR, fname)


@app.route("/scan", methods=["POST"])
def scan():
    SCAN_REQUEST.set()
    return jsonify({"ok": True})


# ============================== MAIN ==============================
def main():
    print("WARDEN host starting...")
    os.makedirs(CFG.CAPTURE_DIR, exist_ok=True)

    esp = None
    for attempt in range(1, 11):
        try:
            esp = ESP32(CFG.SERIAL_PORT, CFG.BAUD)
            break
        except Exception as ex:
            print(f"[esp] open {CFG.SERIAL_PORT} failed ({ex}); retry {attempt}/10 in 2s")
            time.sleep(2)
    if esp is None:
        print(f"[esp] {CFG.SERIAL_PORT} not reachable after retries - exiting")
        return
    if esp.cmd("PING") != "PONG":
        raise RuntimeError("ESP32 did not respond PONG; wrong serial device or firmware")
    print("ESP32 handshake: PONG")

    cap = cv2.VideoCapture(CFG.CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CFG.FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CFG.FRAME_H)
    lock_camera(cap)                  # pin exposure/WB up front (re-asserted each scan)
    if not cap.isOpened():
        print("[camera] unavailable; scans will be INCONCLUSIVE")

    threading.Thread(target=sensor_loop, args=(esp,), daemon=True).start()
    threading.Thread(target=scan_watchdog, daemon=True).start()
    threading.Thread(target=ros_joy_loop, daemon=True).start()          # drive + scan
    threading.Thread(target=inspection_worker, args=(esp, cap), daemon=True).start()

    print(f"Dashboard listening on {CFG.SERVER_HOST}:{CFG.SERVER_PORT}")
    app.run(host=CFG.SERVER_HOST, port=CFG.SERVER_PORT, threaded=True)


if __name__ == "__main__":
    main()
