import unittest
import numpy as np
from wolf_go2.cloud_memory import CloudMemory


class CloudMemoryTests(unittest.TestCase):
    def test_invalid_frame_breaks_support_history(self):
        from wolf_go2.perception import PerceptionPipeline
        from wolf_go2.models import Pose
        from test_perception import scene,contract
        p=PerceptionPipeline();c=contract(temporal_window_s=.3)
        self.assertTrue(p.build_observation(1.,Pose(0,0,0,.3),scene(),geometry_contract=c).support_verified)
        invalid=p.build_observation(1.05,Pose(0,0,0,.3),[[float('nan'),0,0]],geometry_contract=c)
        self.assertIsNone(invalid.grid)
        after=p.build_observation(1.1,Pose(0,0,0,.3),scene(holes=True),geometry_contract=c)
        self.assertFalse(after.support_verified)

    def test_recent_surface_retained_and_then_expires(self):
        m=CloudMemory();a=np.array([[0,0,0.],[1,0,1.]])
        m.merge(a,1.,1.,.3,.6,('odom',0))
        b=np.array([[0,0,0.]])
        self.assertEqual(len(m.merge(b,1.1,1.1,.3,.6,('odom',0))),2)
        self.assertEqual(len(m.merge(b,1.4,1.4,.3,.6,('odom',0))),1)

    def test_epoch_clears_history(self):
        m=CloudMemory();m.merge([[1,1,1]],1.,1.,.3,.6,0)
        result=m.merge([[2,2,2]],1.1,1.1,.3,.6,1)
        self.assertEqual(result.tolist(),[[2.,2.,2.]])

    def test_input_copied_and_duplicate_not_refreshed(self):
        m=CloudMemory();a=np.array([[1.,1.,1.]])
        m.merge(a,1.,1.,.3,.6,0);a[:]=9
        m.merge([[1,1,1]],1.,1.1,.3,.6,0)
        result=m.merge([[2,2,2]],1.31,1.31,.3,.6,0)
        self.assertEqual(result.tolist(),[[2.,2.,2.]])

    def test_backward_or_stale_rejected(self):
        m=CloudMemory();m.merge([[1,1,1]],1.,1.,.3,.6,0)
        with self.assertRaises(ValueError):m.merge([[2,2,2]],.9,1.1,.3,.6,0)
        with self.assertRaises(ValueError):m.merge([[2,2,2]],1.,2.,.3,.6,0)

    def test_occupied_point_not_replaced_by_floor(self):
        m=CloudMemory();m.merge([[0,0,1]],1.,1.,.3,.6,0)
        self.assertEqual(m.merge([[0,0,0]],1.1,1.1,.3,.6,0).tolist(),[[0.,0.,0.],[0.,0.,1.]])

    def test_pipeline_remembers_measured_floor_but_not_forever(self):
        from wolf_go2.perception import PerceptionPipeline
        from wolf_go2.models import Pose
        from test_perception import scene,contract
        p=PerceptionPipeline();c=contract(temporal_window_s=.3)
        full=p.build_observation(1.,Pose(0,0,0,.3),scene(),geometry_contract=c)
        self.assertTrue(full.support_verified)
        brief=p.build_observation(1.1,Pose(0,0,0,.3),scene(holes=True),geometry_contract=c)
        self.assertTrue(brief.support_verified)
        expired=p.build_observation(1.5,Pose(0,0,0,.3),scene(holes=True),geometry_contract=c)
        self.assertFalse(expired.support_verified)
