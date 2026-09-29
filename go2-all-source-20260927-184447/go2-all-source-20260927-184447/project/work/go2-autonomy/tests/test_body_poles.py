import math
import unittest
from wolf_go2.models import Policy,Pose
from wolf_go2.poles import exclude_body_conflicts,pole_rows


class BodyPoleTests(unittest.TestCase):
    def test_false_fourth_pole_not_counted_and_input_retained(self):
        poles=[{'center_xy':[x,0.],'width_xy_m':[.15,.15]} for x in (0,1,2,3)]
        filtered=exclude_body_conflicts(poles,Pose(0,0,0),Policy())
        self.assertEqual(len(filtered),4)
        self.assertNotIn('row_eligible',poles[0])
        self.assertTrue(filtered[0]['body_conflict'])
        self.assertEqual(set(pole_rows(filtered)[0]['pole_indices']),{1,2,3})

    def test_rotated_body_and_uncertainty(self):
        poles=[{'center_xy':[0.,.35],'width_xy_m':[.05,.05]}]
        self.assertFalse(exclude_body_conflicts(poles,Pose(0,0,math.pi/2),Policy())[0]['row_eligible'])
        self.assertNotIn('body_conflict',exclude_body_conflicts(poles,Pose(0,0,0),Policy())[0])
        self.assertTrue(exclude_body_conflicts(poles,Pose(0,0,0),Policy(),.2)[0]['body_conflict'])

    def test_preexisting_exclusion_not_overwritten(self):
        p={'center_xy':[2.,0.],'width_xy_m':[.1,.1],'row_eligible':False}
        self.assertFalse(exclude_body_conflicts([p],Pose(0,0,0),Policy())[0]['row_eligible'])

    def test_bad_geometry_rejected(self):
        with self.assertRaises(ValueError):
            exclude_body_conflicts([{'center_xy':[float('nan'),0.],'width_xy_m':[.1,.1]}],Pose(0,0,0),Policy())
