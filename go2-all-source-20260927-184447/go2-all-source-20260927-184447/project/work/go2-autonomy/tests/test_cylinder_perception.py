"""Основной алгоритм видит реальные цилиндры, но не засчитывает неполную змейку."""
import unittest
import numpy as np
from wolf_go2.perception import PerceptionPipeline
from wolf_go2.models import Pose
from test_perception import scene, contract


class CylinderPerceptionTests(unittest.TestCase):
    def cloud(self, count):
        columns = [[[1.+i*1.4+dx,dy,z] for dx in (0.,.05,.1)
                    for dy in (0.,.05,.1) for z in np.arange(.2,1.25,.05)] for i in range(count)]
        return np.concatenate([scene(), *[np.asarray(c) for c in columns]])

    def test_cylinders_reach_main_pipeline_without_false_completion(self):
        engine = PerceptionPipeline()
        for t in (1.,1.1,1.2):
            obs = engine.build_observation(t,Pose(0,0,0,.3),self.cloud(3),geometry_contract=contract())
        rows = [c for c in engine.candidates if c.get('candidate_type')=='cylindrical_pole_row']
        self.assertEqual(len(rows),1)
        self.assertEqual(rows[0]['observed_poles'],3)
        self.assertFalse(rows[0]['row_complete_verified'])
        self.assertFalse(rows[0]['geometry_verified'])
        self.assertFalse(any(o.geometry_verified for o in obs.obstacles if o.kind=='slalom'))

    def test_two_poles_do_not_get_invented_third(self):
        engine = PerceptionPipeline()
        engine.build_observation(1.,Pose(0,0,0,.3),self.cloud(2),geometry_contract=contract())
        self.assertFalse(any(c.get('candidate_type')=='cylindrical_pole_row' for c in engine.candidates))
