"""Занятый проход: ограниченное ожидание без отключения контроля движения."""
import unittest
from unittest.mock import patch
from wolf_go2.models import Grid, Observation, Pose, Policy
from wolf_go2.mission import MissionController


class RouteWaitTests(unittest.TestCase):
    def make_controller(self):
        c = MissionController(Policy(progress_timeout=2., route_wait_timeout=5.))
        c._phase('APPROACH', 1.)
        c._dt = .1
        return c

    def obs(self, t):
        return Observation(t, Pose(0, 0, 0, .31),
            Grid((-2., -2.), .1, [[0]*70 for _ in range(70)], verified=True),
            localized=True, localization_error_m=.02)

    def navigate(self, c, t, blocked):
        # Планировщик отдельно тестируется на настоящих картах; здесь меняем
        # только результат поиска, чтобы проверить время ожидания и остановку.
        c._nav_path = []
        with patch('wolf_go2.mission.astar', return_value=[] if blocked else [(0.,0.),(2.,0.)]):
            return c._navigate(self.obs(t), (2.,0.), 'вход')

    def test_blocked_longer_than_progress_timeout_then_resumes(self):
        c = self.make_controller()
        for t in (1.,2.,3.,4.):
            d = self.navigate(c,t,True)
            self.assertFalse(d.moving)
            self.assertEqual(c.phase,'APPROACH')
        self.assertTrue(self.navigate(c,4.5,False).moving)

    def test_blocked_timeout_stops(self):
        c = self.make_controller()
        self.navigate(c,1.,True)
        d = self.navigate(c,6.,True)
        self.assertEqual(c.phase,'FAULT')
        self.assertFalse(d.moving)

    def test_intermittent_clear_path_does_not_reset_wait_budget(self):
        c = self.make_controller()
        self.navigate(c,1.,True)
        self.navigate(c,4.,False)
        self.navigate(c,4.1,True)
        d = self.navigate(c,6.2,False)
        self.assertEqual(c.phase,'FAULT')
        self.assertFalse(d.moving)

    def test_stationary_on_clear_path_still_faults(self):
        c = self.make_controller()
        self.navigate(c,1.,False)
        d = self.navigate(c,3.1,False)
        self.assertEqual(c.phase,'FAULT')
        self.assertFalse(d.moving)
