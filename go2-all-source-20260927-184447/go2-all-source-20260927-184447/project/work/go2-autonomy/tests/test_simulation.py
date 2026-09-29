"""Проверки честности кинематической петли, не испытания реального робота."""
import json
import math
import unittest

from wolf_go2.models import Decision, KINDS, Pose
from wolf_go2.simulation import SCOPE, SyntheticWorld, integrate_pose, run_simulation


class KinematicSimulationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.runs = [run_simulation(seed, max_steps=4500) for seed in range(4)]

    def test_integrator_uses_body_velocity_and_rotation(self):
        pose = integrate_pose(Pose(1., 2., math.pi/2), Decision(vx=.2, vy=.1), 2.)
        self.assertAlmostEqual(pose.x, .8)
        self.assertAlmostEqual(pose.y, 2.4)
        circle = integrate_pose(Pose(0., 0., 0.), Decision(vx=.2, wz=.5), math.pi)
        self.assertAlmostEqual(circle.x, .4)
        self.assertAlmostEqual(circle.y, .4)

    def test_four_scenes_are_transformed_and_have_all_types(self):
        starts = set()
        for report in self.runs:
            self.assertEqual(report['scope'], SCOPE)
            self.assertEqual(set(item['kind'] for item in report['scene']['obstacles']), set(KINDS))
            starts.add(tuple(report['scene']['start'].values()))
            json.dumps(report, allow_nan=False)
        self.assertEqual(len(starts), 4)
        self.assertEqual({report['latency_steps'] for report in self.runs}, {0, 1, 2})

    def test_every_pose_is_integral_of_previous_command_without_teleportation(self):
        for report in self.runs:
            for before, after in zip(report['trajectory'], report['trajectory'][1:]):
                predicted = integrate_pose(Pose(before['x'], before['y'], before['yaw']),
                    Decision(vx=before['vx'], vy=before['vy'], wz=before['wz']), report['dt'])
                self.assertAlmostEqual(predicted.x, after['x'], places=10)
                self.assertAlmostEqual(predicted.y, after['y'], places=10)
                self.assertAlmostEqual(predicted.yaw, after['yaw'], places=10)

    def test_completion_needs_five_unique_types_and_actual_return_crossing(self):
        for report in self.runs:
            self.assertEqual(len(report['completed_kinds']), len(set(report['completed_kinds'])))
            if report['completed']:
                self.assertEqual(set(report['completed_kinds']), set(KINDS))
                returned = next(item['t'] for item in report['transitions'] if item['phase'] == 'RETURN')
                self.assertTrue(any(item['t'] >= returned for item in report['line_crossings']))
            else:
                self.assertNotEqual(report['final_phase'], 'FINISHED')
                self.assertTrue(report['reason'])

    def test_return_regression_completes_four_observed_synthetic_scenes(self):
        # До планирования измеренного отрезка seeds 0/1 останавливались у
        # start_pose и ждали несостоявшееся пересечение. Проверка не меняет мир.
        for report in self.runs:
            self.assertTrue(report['completed'], (report['seed'], report['reason']))
            self.assertEqual(report['final_phase'], 'FINISHED')

    def test_short_run_does_not_claim_completion(self):
        report = run_simulation(99, max_steps=3, latency_steps=0)
        self.assertFalse(report['completed'])
        self.assertEqual(report['completed_kinds'], [])

    def test_line_event_and_height_are_world_geometry_not_phase(self):
        world = SyntheticWorld(2)
        cx, cy = world.line_center
        nx, ny = world.line_normal
        a, b = Pose(cx-.1*nx, cy-.1*ny, 0.), Pose(cx+.1*nx, cy+.1*ny, 0.)
        self.assertTrue(world.line_crossed(a, b))
        self.assertTrue(world.line_crossed(b, a))
        self.assertFalse(world.line_crossed(a, a))
        near = world.observe(4., a, a, 0.)
        self.assertTrue(near.start_line_visible)
        self.assertEqual(near.start_line.endpoints, world.line_endpoints)
        self.assertEqual(near.start_line.observed_at, 4.)
        self.assertEqual(near.start_line.frame_epoch, near.frame_epoch)
        self.assertEqual(near.start_line.confidence, 1.)
        self.assertTrue(near.start_line.geometry_verified)
        self.assertEqual(near.start_line.confirmations, 4)
        far_pose = Pose(cx + 3*nx, cy + 3*ny, 0.)
        far = world.observe(5., far_pose, far_pose, 0.)
        self.assertFalse(far.start_line_visible)
        self.assertIsNone(far.start_line)
        for item in world.obstacles:
            for point in item.path:
                self.assertAlmostEqual(world.support_height(Pose(point[0], point[1], 0.)), point[2])


if __name__ == '__main__':
    unittest.main()
