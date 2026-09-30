"""Single guarded publisher for the final /cmd_vel output."""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from .safety_policy import SafetyPolicy, Velocity


class SafetyCmdMux(Node):
    def __init__(self):
        super().__init__("go2_safety_cmd_mux")
        self.declare_parameter("observe_only", True)
        self.declare_parameter("dry_run", True)
        self.declare_parameter("command_timeout", 0.20)
        self.declare_parameter("max_vx", 0.35)
        # The 2026-09-29 lateral trial did not establish controlled motion.
        self.declare_parameter("max_vy", 0.0)
        self.declare_parameter("max_wz", 0.35)
        self.declare_parameter("max_linear_accel", 0.25)
        self.declare_parameter("max_yaw_accel", 0.60)
        self.declare_parameter("nav_topic", "/cmd_vel_nav")
        self.declare_parameter("skill_topic", "/cmd_vel_skill")
        self.declare_parameter("teleop_topic", "/cmd_vel_teleop_test")
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("preview_topic", "/cmd_vel_safe_preview")

        self._observe_only = bool(self.get_parameter("observe_only").value)
        self._dry_run = bool(self.get_parameter("dry_run").value)
        self._policy = SafetyPolicy(
            timeout_s=float(self.get_parameter("command_timeout").value),
            max_vx=float(self.get_parameter("max_vx").value),
            max_vy=float(self.get_parameter("max_vy").value),
            max_wz=float(self.get_parameter("max_wz").value),
            max_linear_accel=float(self.get_parameter("max_linear_accel").value),
            max_yaw_accel=float(self.get_parameter("max_yaw_accel").value),
        )
        self._output = self.create_publisher(
            Twist, str(self.get_parameter("output_topic").value), 10
        )
        self._preview = self.create_publisher(
            Twist, str(self.get_parameter("preview_topic").value), 10
        )
        self._status = self.create_publisher(String, "/safety_cmd_mux/status", 10)
        for source, parameter in (
            ("nav", "nav_topic"),
            ("skill", "skill_topic"),
            ("teleop", "teleop_topic"),
        ):
            self.create_subscription(
                Twist,
                str(self.get_parameter(parameter).value),
                lambda message, source=source: self._on_command(source, message),
                10,
            )
        self.create_timer(0.05, self._tick)
        self.get_logger().warning(
            f"Safety mux started: observe_only={self._observe_only}, dry_run={self._dry_run}"
        )

    def _on_command(self, source, message):
        try:
            self._policy.submit(
                source,
                Velocity(message.linear.x, message.linear.y, message.angular.z),
                time.monotonic(),
            )
        except ValueError as exc:
            self._policy.stop(time.monotonic())
            self.get_logger().error(str(exc))

    @staticmethod
    def _message(velocity):
        message = Twist()
        message.linear.x = velocity.vx
        message.linear.y = velocity.vy
        message.angular.z = velocity.wz
        return message

    def _tick(self):
        enabled = not self._observe_only
        velocity, source = self._policy.command(time.monotonic(), enabled=enabled)
        message = self._message(velocity)
        self._preview.publish(message)
        if not self._dry_run:
            self._output.publish(message)
        mode = "observe_only" if self._observe_only else ("dry_run" if self._dry_run else "armed")
        self._status.publish(String(data=f"{mode}:{source or 'stop'}"))

    def publish_stop(self):
        message = self._message(self._policy.stop(time.monotonic()))
        self._preview.publish(message)
        if not self._dry_run:
            self._output.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyCmdMux()
    try:
        rclpy.spin(node)
    finally:
        node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
