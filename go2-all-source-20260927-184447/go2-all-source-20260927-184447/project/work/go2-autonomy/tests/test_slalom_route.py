import unittest
import numpy as np
from wolf_go2.models import Grid,Pose,Policy
from wolf_go2.slalom_route import plan_slalom

class SlalomRouteTests(unittest.TestCase):
    def test_near_flank_entry_avoids_wall_behind_first_pole(self):
        grid,centers=self.setup_scene()
        ix=int((-.6-grid.origin[0])/grid.resolution)
        for row in grid.cells:row[ix]=1
        plan=plan_slalom(grid,centers,.18,Pose(.2,1.,0),Policy(),.05)
        self.assertTrue(plan['path'],plan)
        self.assertEqual(plan['entry_mode'],'flank_near')
        self.assertEqual(len(plan['alternating_waypoints_xy']),len(centers))
        self.assertLess(plan['path'][0][0],centers[0][0])

    def setup_scene(self):
        cells=np.zeros((100,190),dtype=int)
        for cx in (50,90,130):
            yy,xx=np.indices(cells.shape);cells[(xx-cx)**2+(yy-50)**2<=9]=1
        return Grid((-2.025,-2.525),.05,cells.tolist(),verified=True),[(.5,0),(2.5,0),(4.5,0)]

    def test_auto_selects_supported_alternating_path(self):
        grid,centers=self.setup_scene()
        plan=plan_slalom(grid,centers,.18,Pose(-1,0,0),Policy(),.05)
        self.assertTrue(plan['path'],plan)
        sides=np.sign(np.asarray(plan['alternating_waypoints_xy'])[:,1])
        self.assertTrue(np.all(sides[1:]*sides[:-1]<0))
        self.assertFalse(plan['route_verified'])

    def test_unknown_strip_blocks_crossing(self):
        grid,centers=self.setup_scene()
        for row in grid.cells:row[70]=-1
        plan=plan_slalom(grid,centers,.18,Pose(-1,0,0),Policy(),.05)
        self.assertFalse(plan['path'])

    def test_unverified_grid_never_gives_path(self):
        grid,centers=self.setup_scene();grid.verified=False
        self.assertFalse(plan_slalom(grid,centers,.18,Pose(-1,0,0),Policy(),.05)['path'])

    def test_chooses_other_side_when_first_gate_is_blocked(self):
        grid,centers=self.setup_scene()
        x=int((.5-grid.origin[0])/grid.resolution)
        y=int((.855-grid.origin[1])/grid.resolution)
        grid.cells[y][x]=1
        plan=plan_slalom(grid,centers,.18,Pose(-1,0,0),Policy(),.05)
        self.assertTrue(plan['path'],plan)
        self.assertEqual(plan['first_side'],'right')
        fixed=plan_slalom(grid,centers,.18,Pose(-1,0,0),Policy(),.05,first_side='left')
        self.assertFalse(fixed['path'])

    def test_blocked_center_entry_can_use_free_flank(self):
        grid,centers=self.setup_scene()
        # Посторонний объект занимает только прежнюю центральную точку входа.
        x=int((-.355-grid.origin[0])/grid.resolution)
        y=int((0-grid.origin[1])/grid.resolution)
        grid.cells[y][x]=1
        plan=plan_slalom(grid,centers,.18,Pose(-1,1.2,0),Policy(),.05)
        self.assertTrue(plan['path'],plan)
        self.assertIn(plan['entry_mode'],('flank','flank_near'))
        sides=np.sign(np.asarray(plan['alternating_waypoints_xy'])[:,1])
        self.assertTrue(np.all(sides[1:]*sides[:-1]<0))
