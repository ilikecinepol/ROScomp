"""Отдельная горка: синтетические наблюдения, без сети и физического робота."""
import unittest
from dataclasses import replace
from test_mission import SensorFixture, surface
from wolf_go2.mission import MissionController
from wolf_go2.models import Policy


class TrialTests(unittest.TestCase):
    def fixture(self, items=None, policy=None):
        f = SensorFixture([surface('aframe')] if items is None else items, policy)
        f.controller = MissionController(f.policy, trial_kind='aframe')
        return f

    def test_trial_start_needs_support_not_finish_line(self):
        f = self.fixture()
        self.assertEqual(f.tick(start=True, start_line_visible=False, support_verified=False).phase, 'WAIT_START')
        self.assertEqual(f.tick(start=True, start_line_visible=False).phase, 'DISCOVER')
        self.assertFalse(f.last.moving)

    def test_no_target_never_explores(self):
        for items in ([], [surface('bridge')], [replace(surface('aframe'), geometry_verified=False)]):
            f = self.fixture(items)
            f.tick(start=True)
            for _ in range(10):
                self.assertFalse(f.tick().moving)
                self.assertIsNone(f.controller.current)

    def test_no_unvalidated_skill(self):
        f = self.fixture(policy=Policy())
        f.tick(start=True)
        self.assertFalse(f.tick().moving)
        self.assertIsNone(f.controller.current)

    def test_full_observed_path_stops_without_return(self):
        f = self.fixture()
        f.until_traverse()
        phases = set()
        for _ in range(300):
            c = f.controller
            phases.add(c.phase)
            if c.phase == 'TRAVERSE':
                f.advance_to(c.current.path[c.waypoint_index])
            elif c.phase == 'VERIFY':
                f.advance_to((*c._exit_goal, c.current.path[-1][2]))
            if c.phase in ('FINISHED', 'FAULT'):
                break
            f.tick()
        self.assertEqual(c.phase, 'FINISHED', f.last)
        self.assertEqual(c.completed_kinds, {'aframe'})
        self.assertNotIn('RETURN', phases)
        for _ in range(3): self.assertFalse(f.tick().moving)

    def test_kind_validation(self):
        with self.assertRaises(ValueError): MissionController(Policy(), trial_kind='unknown')


class PlatformTrialTests(TrialTests):
    """Платформы выбираются отдельно от горки, все проверки опоры сохраняются."""
    def fixture(self, items=None, policy=None):
        f = SensorFixture([surface('platforms')] if items is None else items, policy)
        f.controller = MissionController(f.policy, trial_kind='platforms')
        return f

    def test_full_observed_path_stops_without_return(self):
        f = self.fixture()
        f.until_traverse()
        phases = set()
        for _ in range(400):
            c = f.controller
            phases.add(c.phase)
            if c.phase == 'TRAVERSE':
                f.advance_to(c.current.path[c.waypoint_index])
            elif c.phase == 'VERIFY':
                f.advance_to((*c._exit_goal, c.current.path[-1][2]))
            if c.phase in ('FINISHED', 'FAULT'):
                break
            f.tick()
        self.assertEqual(c.phase, 'FINISHED', f.last)
        self.assertEqual(c.completed_kinds, {'platforms'})
        self.assertNotIn('RETURN', phases)
        self.assertFalse(f.tick().moving)

    def test_wrong_kind_and_ambiguous_target_stop(self):
        for items in ([surface('aframe')], [surface('platforms'), surface('platforms', name='second')]):
            f = self.fixture(items)
            f.tick(start=True)
            for _ in range(5):
                self.assertFalse(f.tick().moving)
                self.assertIsNone(f.controller.current)
