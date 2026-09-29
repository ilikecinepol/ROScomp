"""Скорость навыка и непрерывность движения между наблюдёнными узлами."""
import unittest
from dataclasses import replace
from wolf_go2.models import Policy, Pose
from test_mission import SensorFixture, surface


class SkillSpeedTests(unittest.TestCase):
    def test_per_kind_limit_without_changing_other_skills(self):
        policy = Policy(max_vx=.45, skill_speeds={'slalom': .30, 'platforms': .45}).validate()
        self.assertEqual(policy.speed_for_skill('slalom'), .30)
        self.assertEqual(policy.speed_for_skill('platforms'), .45)
        self.assertEqual(policy.speed_for_skill('teeter'), .10)
        self.assertEqual(policy.validated_skills, ())

    def test_reject_bad_skill_limits(self):
        for limits in ({'slalom': .7}, {'unknown': .2}, {'slalom': True},
                       {'slalom': float('nan')}, {'slalom': 0}, []):
            with self.subTest(limits=limits), self.assertRaises(ValueError):
                Policy(skill_speeds=limits).validate()

    def test_reached_internal_node_does_not_force_stop(self):
        item = surface('slalom', origin=(1.2, .125))
        item = replace(item, path=((1.2,.125,0.),(1.25,.125,0.),(1.5,.125,0.),(1.8,.125,0.)))
        fixture = SensorFixture([item])
        fixture.until_traverse()
        controller = fixture.controller
        fixture.pose = Pose(1.2,.125,0.,fixture.pose.z)
        controller._last_pose = fixture.pose
        controller._last_command = (.1,0.)
        decision = fixture.tick()
        self.assertEqual(controller.waypoint_index, 2)
        self.assertGreater(decision.vx, 0.)
        self.assertEqual(len(controller.visited_waypoints), 2)
