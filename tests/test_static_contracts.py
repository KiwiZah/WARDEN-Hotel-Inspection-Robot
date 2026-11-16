"""Offline source contract tests; not a hardware benchmark."""
from pathlib import Path
import ast
import unittest

ROOT = Path(__file__).resolve().parents[1]


class SourceContractTests(unittest.TestCase):
    def test_python_modules_parse(self):
        for rel in ("src/warden_host.py", "src/warden_teleop.py", "ros/drive_only.launch.py"):
            with self.subTest(module=rel):
                ast.parse((ROOT / rel).read_text(encoding="utf-8"))

    def test_no_fake_thermal_or_random_telemetry(self):
        src = (ROOT / "src/warden_host.py").read_text()
        self.assertNotIn("FAKE_THERMAL", src)
        self.assertNotIn("fake_thermal", src)
        self.assertNotIn("import random", src)
        self.assertIn('esp.cmd(command, expect_json=True', src)

    def test_teleop_is_only_motor_publisher(self):
        host = (ROOT / "src/warden_host.py").read_text()
        teleop = (ROOT / "src/warden_teleop.py").read_text()
        self.assertNotIn('create_publisher(Twist, CFG.DRIVE_TOPIC', host)
        self.assertIn('create_publisher(Twist, "/cmd_vel"', teleop)
        self.assertIn('"/warden/scan_active"', host)
        self.assertIn('"/warden/scan_active"', teleop)

    def test_firmware_null_on_thermal_failures(self):
        firmware = (ROOT / "firmware/warden_sensor_hub/warden_sensor_hub.ino").read_text()
        self.assertIn('!amgReadGrid(px)', firmware)
        self.assertIn('min\\\":null', firmware)
        self.assertIn('line == "EVENT"', firmware)

    def test_failures_do_not_report_clear(self):
        host = (ROOT / "src/warden_host.py").read_text()
        self.assertIn('"verdict": "INCONCLUSIVE", "confidence": None', host)
        self.assertIn('or not thermal_valid or not rf_valid', host)

    def test_no_embedded_credentials(self):
        for path in ROOT.rglob('*'):
            if path.is_file() and path.suffix not in ('.pyc',) and path.name != 'test_static_contracts.py':
                self.assertNotIn('sk-' + 'proj-', path.read_text(errors='ignore'))


if __name__ == "__main__":
    unittest.main()
