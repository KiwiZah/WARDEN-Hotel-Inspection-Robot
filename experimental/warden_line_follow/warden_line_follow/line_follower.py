#!/usr/bin/env python3
"""
WARDEN line follower (ROS 2 Humble).

Reads a 5-channel TCRT5000 digital array wired to Raspberry Pi GPIO,
runs a weighted-position PD loop, and publishes geometry_msgs/Twist to
'cmd_vel_auto'. twist_mux arbitrates this against the joystick.

All gains + pins + the line-detect level are ROS parameters, so you can
tune live without restarting:
    ros2 param set /line_follower kp 1.1
    ros2 param set /line_follower base_speed 0.10

BENCH-VERIFY before driving (wheels off the ground):
  1. line_detected_level: most boards read 0 (LOW) over black tape.
     Confirm with:  gpioget gpiochip0 5 6 13 19 26   (slide tape under each)
  2. sensor_pins order must be physical LEFT -> RIGHT.
  3. gpiochip: Pi 4 = gpiochip0 -> param 0. (Pi 5 = gpiochip4 -> param 4.)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import lgpio


class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')

        # --- Parameters (live-tunable) ---
        self.declare_parameter('sensor_pins', [5, 6, 13, 19, 26])   # BCM, left->right
        self.declare_parameter('weights', [-2.0, -1.0, 0.0, 1.0, 2.0])
        self.declare_parameter('line_detected_level', 0)            # 0=LOW, 1=HIGH (VERIFY)
        self.declare_parameter('gpiochip', 0)                       # Pi4=0, Pi5=4
        self.declare_parameter('kp', 0.9)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.5)
        self.declare_parameter('base_speed', 0.08)                 # m/s (Burger max ~0.22)
        self.declare_parameter('max_angular', 1.5)                 # rad/s
        self.declare_parameter('lost_timeout', 0.4)                # s before stopping
        self.declare_parameter('rate_hz', 40.0)

        self.pins = list(self.get_parameter('sensor_pins').value)
        chip = self.get_parameter('gpiochip').value

        if len(self.pins) != 5:
            raise ValueError("line follower requires exactly five sensor pins")
        # --- GPIO setup ---
        self.h = lgpio.gpiochip_open(chip)
        for p in self.pins:
            lgpio.gpio_claim_input(self.h, p)

        # --- State ---
        self.last_error = 0.0
        self.last_control_at = None
        self.integral = 0.0
        self.lost_since = None

        self.pub = self.create_publisher(Twist, 'cmd_vel_auto', 10)
        rate = float(self.get_parameter('rate_hz').value)
        self.timer = self.create_timer(1.0 / rate, self.loop)
        self.get_logger().info(f'line_follower up: pins={self.pins} chip={chip}')

    def loop(self):
        # Read tunables fresh each tick so `ros2 param set` takes effect live
        weights = list(self.get_parameter('weights').value)
        detected = self.get_parameter('line_detected_level').value
        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value
        base = self.get_parameter('base_speed').value
        max_w = self.get_parameter('max_angular').value
        lost_timeout = self.get_parameter('lost_timeout').value

        raw = [lgpio.gpio_read(self.h, p) for p in self.pins]
        active = [r == detected for r in raw]
        n = sum(1 for a in active if a)

        twist = Twist()
        now = self.get_clock().now().nanoseconds * 1e-9

        # --- Line lost ---
        if n == 0:
            if self.lost_since is None:
                self.lost_since = now
            if now - self.lost_since > lost_timeout:
                self.pub.publish(Twist())          # full stop
                return
            # pivot toward the side the line was last seen on.
            # +error => line was to the RIGHT => turn right => negative angular.z
            twist.linear.x = 0.0
            twist.angular.z = -max_w if self.last_error >= 0 else max_w
            self.pub.publish(twist)
            return
        self.lost_since = None

        # --- PD control ---
        position = sum(w for w, a in zip(weights, active) if a) / n   # ~ -2..+2
        error = position
        dt = max(1e-3, now - self.last_control_at) if self.last_control_at is not None else 1.0 / max(1.0, self.get_parameter("rate_hz").value)
        self.last_control_at = now
        self.integral += error * dt
        self.integral = max(-10.0, min(10.0, self.integral))          # anti-windup
        derivative = (error - self.last_error) / dt
        correction = kp * error + ki * self.integral + kd * derivative
        self.last_error = error

        # REP-103: +angular.z = left (CCW). Line on the right (+error) => turn right => negate.
        twist.linear.x = float(base)
        twist.angular.z = max(-max_w, min(max_w, -correction))
        self.pub.publish(twist)

    def destroy_node(self):
        try:
            self.pub.publish(Twist())              # stop on shutdown
            lgpio.gpiochip_close(self.h)
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = LineFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
