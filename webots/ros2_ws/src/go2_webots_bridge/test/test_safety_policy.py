import unittest

from go2_webots_bridge.safety_policy import SafetyPolicy, Velocity


class SafetyPolicyTests(unittest.TestCase):
    def test_rejects_timeout_above_contract(self):
        with self.assertRaises(ValueError):
            SafetyPolicy(timeout_s=0.251)

    def test_rejects_negative_lateral_limit(self):
        with self.assertRaises(ValueError):
            SafetyPolicy(max_vy=-0.01)

    def test_stale_command_stops_immediately(self):
        policy = SafetyPolicy(timeout_s=0.20)
        policy.submit("nav", Velocity(0.35, 0.0, 0.0), 1.0)
        initial, source = policy.command(1.0)
        self.assertEqual("nav", source)
        self.assertEqual(Velocity(), initial)
        moving, source = policy.command(1.1)
        self.assertGreater(moving.vx, 0.0)
        stopped, source = policy.command(1.201)
        self.assertIsNone(source)
        self.assertEqual(Velocity(), stopped)

    def test_priority_is_teleop_then_skill_then_nav(self):
        policy = SafetyPolicy()
        policy.submit("nav", Velocity(0.10, 0.0, 0.0), 1.0)
        policy.submit("skill", Velocity(0.20, 0.0, 0.0), 1.0)
        policy.submit("teleop", Velocity(0.30, 0.0, 0.0), 1.0)
        command, source = policy.command(1.0)
        self.assertEqual("teleop", source)
        self.assertEqual(0.0, command.vx)

    def test_first_command_cannot_bypass_acceleration_limit(self):
        policy = SafetyPolicy()
        policy.submit("nav", Velocity(0.35, 0.0, 0.0), 1.0)
        first, _ = policy.command(1.0)
        second, _ = policy.command(1.05)
        self.assertEqual(Velocity(), first)
        self.assertAlmostEqual(0.0125, second.vx)

    def test_limits_velocity_and_acceleration(self):
        policy = SafetyPolicy(max_linear_accel=0.25, max_yaw_accel=0.60)
        policy.command(1.0)
        policy.submit("nav", Velocity(4.0, -4.0, 4.0), 1.0)
        command, _ = policy.command(1.1)
        self.assertAlmostEqual(0.025, command.vx)
        self.assertEqual(0.0, command.vy)
        self.assertAlmostEqual(0.060, command.wz)

    def test_lateral_motion_requires_explicit_nonzero_limit(self):
        policy = SafetyPolicy()
        policy.submit("teleop", Velocity(0.0, 0.15, 0.0), 1.0)
        command, _ = policy.command(1.0)
        self.assertEqual(0.0, command.vy)

    def test_observe_only_never_forwards_input(self):
        policy = SafetyPolicy()
        policy.submit("nav", Velocity(0.35, 0.0, 0.0), 1.0)
        command, source = policy.command(1.0, enabled=False)
        self.assertIsNone(source)
        self.assertEqual(Velocity(), command)

    def test_explicit_stop_clears_all_sources_and_latches_zero(self):
        policy = SafetyPolicy(max_vy=0.1)
        policy.command(1.0)
        for source in ("nav", "skill", "teleop"):
            policy.submit(source, Velocity(0.2, 0.1, 0.2), 1.0)
        policy.command(1.1)
        self.assertEqual(Velocity(), policy.stop(1.11))
        command, source = policy.command(1.12)
        self.assertIsNone(source)
        self.assertEqual(Velocity(), command)

    def test_future_dated_command_is_never_selected(self):
        policy = SafetyPolicy()
        policy.submit("nav", Velocity(0.2, 0.0, 0.0), 2.0)
        command, source = policy.command(1.0)
        self.assertIsNone(source)
        self.assertEqual(Velocity(), command)

    def test_stale_high_priority_source_does_not_hide_fresh_nav(self):
        policy = SafetyPolicy(timeout_s=0.2)
        policy.command(1.0)
        policy.submit("teleop", Velocity(0.3, 0.0, 0.0), 1.0)
        policy.submit("nav", Velocity(0.1, 0.0, 0.0), 1.21)
        command, source = policy.command(1.21)
        self.assertEqual("nav", source)
        self.assertGreater(command.vx, 0.0)

    def test_non_finite_command_forces_rejection(self):
        policy = SafetyPolicy()
        for velocity in (
            Velocity(float("nan"), 0.0, 0.0),
            Velocity(0.0, float("inf"), 0.0),
            Velocity(0.0, 0.0, float("-inf")),
        ):
            with self.assertRaises(ValueError):
                policy.submit("nav", velocity, 1.0)


if __name__ == "__main__":
    unittest.main()
