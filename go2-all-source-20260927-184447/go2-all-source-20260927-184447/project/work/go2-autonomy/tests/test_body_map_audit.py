import unittest
import numpy as np
from wolf_go2.body_map_audit import body_map_audit
from wolf_go2.models import Grid, Policy, Pose


class BodyMapAuditTests(unittest.TestCase):
    def inspect(self, obstacle=None):
        cells=np.zeros((40,40),dtype=int)
        top=np.zeros((40,40))
        if obstacle is not None:
            x,y=obstacle;cells[y,x]=1;top[y,x]=.8
        grid=Grid((-1.,-1.),.05,cells.tolist(),0,True)
        before=[row[:] for row in grid.cells]
        result=body_map_audit(grid,top,np.zeros_like(top),Pose(0,0,0),Policy(),.08)
        self.assertEqual(before,grid.cells)
        self.assertFalse(result['motion_authorized'])
        return result

    def test_free_body_is_not_route_permission(self):
        self.assertEqual(self.inspect()['status'],'body_clear')

    def test_obstacle_inside_body_never_erased(self):
        result=self.inspect((20,20))
        self.assertEqual(result['status'],'occupied_inside_nominal_body')
        self.assertEqual(len(result['occupied_samples']),1)
        self.assertAlmostEqual(result['occupied_samples'][0]['height_above_floor_m'],.8)

    def test_margin_distinguished_from_body(self):
        self.assertEqual(self.inspect((20,26))['status'],'clearance_margin_blocked')
