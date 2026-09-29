"""Испытания автомата на синтетике: не проверка физической проходимости."""
import unittest
from dataclasses import replace
from test_mission import SensorFixture, surface
from wolf_go2.training import TrainingController
from wolf_go2.mission import MissionController


class TrainingTests(unittest.TestCase):
    def test_first_trial_does_not_need_prior_success_or_certify_policy(self):
        f=SensorFixture([surface('slalom')])
        f.policy=replace(f.policy,validated_skills=())
        f.controller=TrainingController(f.policy,'slalom',commissioning=True)
        result=self.finish(f)
        self.assertEqual(result.phase,'FINISHED',result.reason)
        self.assertEqual(f.policy.validated_skills,())

    def test_commissioning_cannot_enable_full_loop(self):
        f=SensorFixture([])
        with self.assertRaises(ValueError):
            MissionController(f.policy,commissioning=True)

    def test_first_trial_still_rejects_unknown_geometry(self):
        item=replace(surface('slalom'),geometry_verified=False)
        f=SensorFixture([item])
        f.policy=replace(f.policy,validated_skills=())
        f.controller=TrainingController(f.policy,'slalom',commissioning=True)
        for i in range(10):self.assertFalse(f.tick(start=i==0).moving)

    def finish(self,f):
        for _ in range(1000):
            c=f.controller.controller
            if c.phase=='APPROACH':
                f.advance_to(c._nav_path[c._nav_index] if c._nav_path else c._approach_goal)
            elif c.phase=='ALIGN':
                import math
                a,b=c.current.path[:2]
                f.advance_to((f.pose.x,f.pose.y),math.atan2(b[1]-a[1],b[0]-a[0]))
            elif c.phase=='TRAVERSE':f.advance_to(c.current.path[c.waypoint_index])
            elif c.phase=='VERIFY':f.advance_to((*c._exit_goal,c.current.path[-1][2]))
            decision=f.tick(start=c.phase=='WAIT_START')
            if decision.phase in ('FAULT','FINISHED'):return decision
        self.fail('Истёк лимит синтетических шагов')

    def test_four_independent_skills(self):
        for kind in ('slalom','aframe','teeter','platforms'):
            with self.subTest(kind=kind):
                f=SensorFixture([surface(kind)])
                f.controller=TrainingController(f.policy,kind)
                result=self.finish(f)
                self.assertEqual(result.phase,'FINISHED',result.reason)
                self.assertEqual(result.completed,(kind,))
                self.assertFalse(result.moving)

    def test_slalom_then_only_align(self):
        f=SensorFixture([surface('slalom'),surface('aframe',origin=(4.,2.))])
        f.controller=TrainingController(f.policy,'slalom','aframe')
        result=self.finish(f)
        self.assertEqual(result.phase,'FINISHED',result.reason)
        self.assertEqual(result.completed,('slalom',))
        self.assertEqual(f.controller.controller.visited_waypoints,[])
        self.assertFalse(result.moving)

    def test_missing_target_never_moves(self):
        f=SensorFixture([]);f.controller=TrainingController(f.policy,'teeter')
        f.tick(start=True)
        for _ in range(5):self.assertFalse(f.tick().moving)

    def test_occupied_start_stops(self):
        f=SensorFixture([surface('slalom')]);f.controller=TrainingController(f.policy,'slalom')
        f.tick(start=True)
        f.grid.cells=[[1]*len(row) for row in f.grid.cells]
        self.assertFalse(f.tick().moving)

    def test_epoch_change_rejected(self):
        f=SensorFixture([]);f.controller=TrainingController(f.policy,'slalom','aframe')
        f.tick(start=True)
        result=f.tick(frame_epoch='changed')
        self.assertEqual(result.phase,'FAULT')
        self.assertFalse(result.moving)
