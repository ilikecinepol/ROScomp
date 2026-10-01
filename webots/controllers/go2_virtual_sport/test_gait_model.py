import unittest

from gait_model import foot_target


class GaitModelTest(unittest.TestCase):
    def test_standing_has_no_horizontal_motion(self):
        for leg in ("FL", "FR", "RL", "RR"):
            target = foot_target(leg, 0.0, 0.0, 0.0, 0.0)
            self.assertAlmostEqual(target.x, 0.0)
            self.assertAlmostEqual(target.y, 0.0)

    def test_forward_stride_is_equal_on_all_legs(self):
        frequency = foot_target("FL", 0.0, 0.3, 0.0, 0.0).frequency
        values = [
            foot_target(
                leg,
                0.0 if leg in ("FL", "RR") else 0.5 / frequency,
                0.3,
                0.0,
                0.0,
            ).x
            for leg in ("FL", "FR", "RL", "RR")
        ]
        self.assertAlmostEqual(values[0], values[1])
        self.assertAlmostEqual(values[0], values[2])
        self.assertAlmostEqual(values[0], values[3])

    def test_left_turn_uses_opposite_left_and_right_strides(self):
        fl = foot_target("FL", 0.0, 0.0, 0.0, 0.35)
        fr = foot_target("FR", 0.5 / fr_frequency(0.35), 0.0, 0.0, 0.35)
        self.assertLess(fl.x, 0.0)
        self.assertGreater(fr.x, 0.0)
        self.assertAlmostEqual(abs(fl.x), abs(fr.x), places=6)

    def test_turn_lateral_motion_reverses_front_to_rear(self):
        fl = foot_target("FL", 0.0, 0.0, 0.0, 0.35)
        rl = foot_target("RL", 0.5 / fl.frequency, 0.0, 0.0, 0.35)
        self.assertGreater(fl.y, 0.0)
        self.assertLess(rl.y, 0.0)
        self.assertAlmostEqual(abs(fl.y), abs(rl.y), places=6)

    def test_right_turn_reverses_left_turn(self):
        left = foot_target("FL", 0.0, 0.0, 0.0, 0.35)
        right = foot_target("FL", 0.0, 0.0, 0.0, -0.35)
        self.assertAlmostEqual(left.x, -right.x, places=6)
        self.assertAlmostEqual(left.y, -right.y, places=6)


def fr_frequency(wz):
    return foot_target("FR", 0.0, 0.0, 0.0, wz).frequency


if __name__ == "__main__":
    unittest.main()
