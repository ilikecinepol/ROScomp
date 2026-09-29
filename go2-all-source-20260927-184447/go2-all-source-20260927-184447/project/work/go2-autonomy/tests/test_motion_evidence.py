"""Проверки измерения устойчивой позы после импульса."""
import math
import unittest
from wolf_go2.motion_evidence import settled_pose


def row(t, x, yaw=0):
    return {'topic':'ROBOTODOM', 'phase':'after', 'elapsed_s':t,
            'message':{'data':{'pose':{'position':{'x':x,'y':0},
            'orientation':{'x':0,'y':0,'z':math.sin(yaw/2),'w':math.cos(yaw/2)}}}}}


class MotionEvidenceTests(unittest.TestCase):
    def test_lean_then_return_uses_settled_tail(self):
        rows=[row(i*.1, .1 if i<20 else .01) for i in range(41)]
        center,spread=settled_pose(rows,'after')
        self.assertAlmostEqual(center[0],.01)
        self.assertEqual(spread,0)

    def test_yaw_wrap_does_not_average_to_zero(self):
        center,_=settled_pose([row(i*.1,0,math.pi-.01 if i%2 else -math.pi+.01) for i in range(11)],'after')
        self.assertGreater(abs(center[2]),3.1)

    def test_insufficient_tail_rejected(self):
        with self.assertRaises(ValueError):settled_pose([row(0,0)],'after')
