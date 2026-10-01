import unittest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from response_profile import load_response_profile


PROFILE = Path(__file__).resolve().parents[2] / "config" / "go2_r6_measured_response.json"


class ResponseProfileTests(unittest.TestCase):
    def setUp(self):
        self.profile = load_response_profile(PROFILE)

    def test_recorded_forward_and_asymmetric_yaw_gains(self):
        forward, lateral, left = self.profile.realize(0.35, 0.0, 0.35)
        self.assertAlmostEqual(forward, 0.29995)
        self.assertAlmostEqual(lateral, 0.0)
        self.assertAlmostEqual(left, 0.143675)
        forward, lateral, right = self.profile.realize(0.35, 0.0, -0.35)
        self.assertAlmostEqual(forward, 0.29995)
        self.assertAlmostEqual(lateral, 0.0)
        self.assertAlmostEqual(right, -0.08015)

    def test_unverified_axes_fail_closed(self):
        self.assertEqual(self.profile.realize(-0.35, 0.3, 0.0), (0.0, 0.0, 0.0))

    def test_commands_are_limited_before_gain(self):
        self.assertEqual(self.profile.realize(2.0, 2.0, -2.0), (0.29995, 0.0, -0.08015))


if __name__ == "__main__":
    unittest.main()
