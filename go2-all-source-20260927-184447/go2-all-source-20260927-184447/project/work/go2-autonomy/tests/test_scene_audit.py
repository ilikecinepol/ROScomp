import unittest
import numpy as np
from wolf_go2.models import Pose
from wolf_go2.scene_audit import inspect_scene
from wolf_go2.scene_audit import inspect_monitor
from wolf_go2.sensors import SensorMonitor, Stream


class SceneAuditTests(unittest.TestCase):
    def test_missing_data_never_confirms_route(self):
        for points,pose,meta in [(None,None,{}),(np.zeros((40,3)),Pose(0,0,0),{'resolution':True})]:
            result=inspect_scene(points,pose,meta)
            self.assertFalse(result['route_verified'])
            self.assertEqual(result['pole_candidates'],[])

    def test_observed_column_is_only_a_candidate(self):
        floor=[[x,y,0] for x in np.arange(-1,1,.05) for y in np.arange(-1,1,.05)]
        column=[[.6+x,.6+y,z] for x in [0,.05,.1] for y in [0,.05,.1] for z in np.arange(.05,1.2,.05)]
        result=inspect_scene(np.array(floor+column),Pose(0,0,0,.3),{'resolution':.05,'frame_id':'odom'})
        self.assertEqual(len(result['pole_candidates']),1)
        self.assertFalse(result['route_verified'])
        self.assertFalse(result['pole_candidates'][0]['verified'])
        monitor=SensorMonitor()
        monitor.last_pose=(10.,Pose(0,0,0,.3))
        monitor.streams['ULIDAR_ARRAY']=Stream(payload={'resolution':.05,'frame_id':'odom'})
        self.assertEqual(inspect_monitor(np.array(floor+column),monitor),result)
