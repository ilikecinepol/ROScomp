"""Проверка перехода от поворота к проходу по последовательности поз."""
import unittest
from dataclasses import replace
from test_mission import SensorFixture,surface


class AlignmentSettleTests(unittest.TestCase):
    def fixture(self):
        f=SensorFixture([surface()])
        f.until_traverse()
        f.controller._phase('ALIGN',f.t)
        return f

    def test_single_aligned_frame_is_not_completion(self):
        f=self.fixture()
        for _ in range(2):
            result=f.tick()
            self.assertEqual(result.phase,'ALIGN')
            self.assertFalse(result.moving)
        self.assertEqual(f.tick().phase,'TRAVERSE')

    def test_crossing_target_while_rotating_does_not_start_traversal(self):
        f=self.fixture()
        for yaw in [.06,0.,-.06,0.,.06,0.]:
            f.pose=replace(f.pose,yaw=yaw)
            result=f.tick()
            self.assertEqual(result.phase,'ALIGN')
            self.assertFalse(result.moving)
        for _ in range(3):result=f.tick()
        self.assertEqual(result.phase,'TRAVERSE')

    def test_gap_resets_settle_window(self):
        f=self.fixture();f.tick();f.tick()
        f.t+=1.
        self.assertEqual(f.tick().phase,'ALIGN')
        self.assertEqual(f.tick().phase,'ALIGN')
        self.assertEqual(f.tick().phase,'ALIGN')
        self.assertEqual(f.tick().phase,'TRAVERSE')

    def test_missing_support_resets_settle_window(self):
        f=self.fixture();f.tick();f.tick()
        self.assertEqual(f.tick(support_verified=False).phase,'ALIGN')
        self.assertEqual(f.tick().phase,'ALIGN')
        self.assertEqual(f.tick().phase,'ALIGN')
        self.assertEqual(f.tick().phase,'TRAVERSE')

    def test_duplicate_observation_cannot_complete_settle(self):
        f=self.fixture();f.tick();f.tick()
        old=f.t
        self.assertEqual(f.tick(t=old).phase,'ALIGN')
        self.assertEqual(f.tick().phase,'ALIGN')
        self.assertEqual(f.tick().phase,'ALIGN')
        self.assertEqual(f.tick().phase,'TRAVERSE')
