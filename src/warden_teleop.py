#!/usr/bin/env python3
"""WARDEN ROS 2 manual drive; only node allowed to publish /cmd_vel.

Scans pause the robot. Spins are OPTIONAL and must be enabled explicitly.
"""
import math
import os
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


class WardenTeleop(Node):
    AXIS_LINEAR = 3
    AXIS_ANGULAR = 2
    BTN_CELEBRATE = 3

    def __init__(self):
        super().__init__("warden_teleop")
        self.create_subscription(Joy, "/joy", self.on_joy, 10)
        self.create_subscription(Bool, "/warden/scan_active", self.on_scan, 10)
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.last_axes = []
        self.last_buttons = []
        self.last_joy_at = 0.0
        self.last_interlock_at = 0.0
        self.scanning = False
        self.spin_seconds = 0.0
        self.hz = 20
        self.spin_enabled = os.environ.get("WARDEN_ENABLE_SPIN", "0") == "1"
        self.base_linear = 0.15
        self.base_angular = 1.0
        self.deadzone = 0.10
        self.spin_speed = 1.5
        self.create_timer(1.0 / self.hz, self.tick)
        self.get_logger().info("teleop ready; spin feature enabled=%s" % self.spin_enabled)

    def on_scan(self, msg):
        self.last_interlock_at = time.monotonic()
        self.scanning = bool(msg.data)
        if self.scanning:
            self.spin_seconds = 0.0

    def on_joy(self, msg):
        self.last_joy_at = time.monotonic()
        self.last_axes = list(msg.axes)
        if len(self.last_buttons) != len(msg.buttons):
            self.last_buttons = [0] * len(msg.buttons)
        if self.spin_enabled and len(msg.buttons) > self.BTN_CELEBRATE:
            i = self.BTN_CELEBRATE
            if msg.buttons[i] and not self.last_buttons[i] and not self.scanning:
                self.spin_seconds = 3 * 2 * math.pi / self.spin_speed
        self.last_buttons = list(msg.buttons)

    def tick(self):
        t = Twist()
        now = time.monotonic()
        # Fail closed: neither joystick nor scan interlock may silently disappear.
        interlock_valid = now - self.last_interlock_at < 0.5
        joystick_valid = now - self.last_joy_at < 0.5
        if interlock_valid and not self.scanning and joystick_valid:
            if self.spin_seconds > 0 and self.spin_enabled:
                t.angular.z = self.spin_speed
                self.spin_seconds = max(0.0, self.spin_seconds - 1.0 / self.hz)
            elif len(self.last_axes) > max(self.AXIS_LINEAR, self.AXIS_ANGULAR):
                lin = self.last_axes[self.AXIS_LINEAR]
                ang = self.last_axes[self.AXIS_ANGULAR]
                t.linear.x = self.base_linear * (lin if abs(lin) >= self.deadzone else 0.0)
                t.angular.z = self.base_angular * (ang if abs(ang) >= self.deadzone else 0.0)
        else:
            self.spin_seconds = 0.0
        self.publisher.publish(t)

    def destroy_node(self):
        self.publisher.publish(Twist())
        super().destroy_node()


def main():
    rclpy.init()
    node = WardenTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
