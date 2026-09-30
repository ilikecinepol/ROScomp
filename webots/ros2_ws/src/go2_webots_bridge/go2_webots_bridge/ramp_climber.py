"""Conservative A-frame command source for the central safety mux."""

from math import asin, atan2, copysign, cos, sin

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def euler_from_quaternion(q):
    roll = atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y))
    pitch = asin(clamp(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0))
    yaw = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return roll, pitch, yaw


class RampClimber(Node):
    def __init__(self):
        super().__init__("go2_ramp_climber")
        self.declare_parameter("approach_speed", 0.16)
        self.declare_parameter("climb_speed", 0.20)
        self.declare_parameter("descent_speed", 0.12)
        self.declare_parameter("exit_speed", 0.14)
        self.declare_parameter("exit_distance", 0.35)
        self.declare_parameter("telemetry_timeout", 0.5)
        self.declare_parameter("fault_on_stale", True)
        self.declare_parameter("command_topic", "/cmd_vel_skill")
        self._cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter("command_topic").value), 10
        )
        self._state_pub = self.create_publisher(String, "/ramp_climber/state", 10)
        self.create_subscription(Imu, "/imu", self._on_imu, 20)
        self.create_subscription(Odometry, "/odom", self._on_odom, 20)
        self.create_timer(0.05, self._tick)
        self._phase = "WAITING"
        self._phase_since = self.get_clock().now()
        self._roll = self._pitch = self._yaw = None
        self._x = self._y = self._z = None
        self._last_imu = self._last_odom = None
        self._baseline_z = self._initial_yaw = None
        self._slope_sign = 0.0
        self._peak_rise = 0.0
        self._best_climb_rise = 0.0
        self._last_climb_progress = None
        self._exit_start = None
        self.get_logger().info("Ramp controller waiting for /imu and /odom; align Go2 with the ramp.")

    def _on_imu(self, message):
        self._roll, self._pitch, _ = euler_from_quaternion(message.orientation)
        self._last_imu = self.get_clock().now()

    def _on_odom(self, message):
        p = message.pose.pose.position
        self._x, self._y, self._z = p.x, p.y, p.z
        _, _, self._yaw = euler_from_quaternion(message.pose.pose.orientation)
        self._last_odom = self.get_clock().now()

    def _set_phase(self, phase):
        if phase == self._phase:
            return
        self._phase = phase
        self._phase_since = self.get_clock().now()
        self._state_pub.publish(String(data=phase))
        self.get_logger().info(
            f"phase={phase}, pitch={self._pitch or 0.0:+.3f}, rise={self._current_rise():+.3f} m"
        )

    def _current_rise(self):
        if self._z is None or self._baseline_z is None:
            return 0.0
        return self._z - self._baseline_z

    def _elapsed(self):
        return (self.get_clock().now() - self._phase_since).nanoseconds / 1e9

    def _tick(self):
        now = self.get_clock().now()
        self._state_pub.publish(String(data=self._phase))
        if self._last_imu is None or self._last_odom is None:
            self._publish_stop()
            return
        stale = max(
            (now - self._last_imu).nanoseconds / 1e9,
            (now - self._last_odom).nanoseconds / 1e9,
        )
        if stale > float(self.get_parameter("telemetry_timeout").value):
            if bool(self.get_parameter("fault_on_stale").value):
                self._set_phase("FAULT_STALE_TELEMETRY")
            self._publish_stop()
            return
        if abs(self._roll) > 0.50 or abs(self._pitch) > 0.78:
            self._set_phase("FAULT_ATTITUDE")

        if self._phase == "WAITING":
            self._baseline_z = self._z
            self._initial_yaw = self._yaw
            self._set_phase("APPROACH")

        rise = self._current_rise()
        self._peak_rise = max(self._peak_rise, rise)
        if self._phase == "APPROACH" and abs(self._pitch) > 0.10:
            self._slope_sign = copysign(1.0, self._pitch)
            self._best_climb_rise = rise
            self._last_climb_progress = now
            self._set_phase("CLIMB")
        elif self._phase == "CLIMB":
            if rise >= self._best_climb_rise + 0.01:
                self._best_climb_rise = rise
                self._last_climb_progress = now
            if rise > 0.15 and self._pitch * self._slope_sign < 0.02:
                self._set_phase("CREST")
            elif (
                self._last_climb_progress is not None
                and (now - self._last_climb_progress).nanoseconds / 1e9 > 20.0
            ):
                self._set_phase("FAULT_CLIMB_STALLED")
        elif self._phase == "CREST":
            if self._pitch * self._slope_sign < -0.08:
                self._set_phase("DESCENT")
            elif self._elapsed() > 2.0:
                self._set_phase("DESCENT")
        elif self._phase == "DESCENT":
            if abs(self._pitch) < 0.07 and rise < 0.10 and self._peak_rise > 0.20:
                self._exit_start = (self._x, self._y)
                self._set_phase("EXIT")
        elif self._phase == "EXIT":
            distance = ((self._x - self._exit_start[0]) ** 2 + (self._y - self._exit_start[1]) ** 2) ** 0.5
            if distance >= float(self.get_parameter("exit_distance").value):
                self._set_phase("DONE")

        if self._phase.startswith("FAULT") or self._phase == "DONE":
            self._publish_stop()
            return

        speed = {
            "APPROACH": float(self.get_parameter("approach_speed").value),
            "CLIMB": float(self.get_parameter("climb_speed").value),
            "CREST": 0.10,
            "DESCENT": float(self.get_parameter("descent_speed").value),
            "EXIT": float(self.get_parameter("exit_speed").value),
        }.get(self._phase, 0.0)
        cmd = Twist()
        cmd.linear.x = speed
        yaw_error = atan2(sin(self._initial_yaw - self._yaw), cos(self._initial_yaw - self._yaw))
        cmd.angular.z = clamp(0.8 * yaw_error, -0.35, 0.35)
        self._cmd_pub.publish(cmd)

    def _publish_stop(self):
        self._cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = RampClimber()
    try:
        rclpy.spin(node)
    finally:
        node._publish_stop()
        node.destroy_node()
        rclpy.shutdown()
