"""Проверяем габариты, неизвестность и весь поворот, а не только концы."""
import math
import unittest
from wolf_go2.models import Grid,Pose,Policy
from wolf_go2.footprint import footprint_report,swept_clear


class FootprintTests(unittest.TestCase):
    def scene(self):
        grid=Grid((-2.025,-2.025),.05,[[0]*81 for _ in range(81)],verified=True)
        return grid,Policy(),.02

    def test_heading_changes_clearance_in_corridor(self):
        grid,p,u=self.scene()
        for y,row in enumerate(grid.cells):
            if abs(grid.origin[1]+(y+.5)*grid.resolution)>.4:
                row[:]=[1]*len(row)
        self.assertTrue(footprint_report(grid,Pose(0,0,0),p,u)['clear'])
        self.assertFalse(footprint_report(grid,Pose(0,0,math.pi/2),p,u)['clear'])
        self.assertTrue(swept_clear(grid,Pose(-.5,0,0),Pose(.5,0,0),p,u))
        self.assertFalse(swept_clear(grid,Pose(0,0,0),Pose(0,0,math.pi),p,u))

    def test_unknown_cell_is_not_floor(self):
        grid,p,u=self.scene();grid.cells[40][40]=-1
        report=footprint_report(grid,Pose(0,0,0),p,u)
        self.assertFalse(report['clear']);self.assertEqual(report['unknown_cells'],1)

    def test_free_endpoints_do_not_allow_crossing_obstacle(self):
        grid,p,u=self.scene();grid.cells[40][40]=1
        a,b=Pose(-1,0,0),Pose(1,0,0)
        self.assertTrue(footprint_report(grid,a,p,u)['clear'])
        self.assertTrue(footprint_report(grid,b,p,u)['clear'])
        self.assertFalse(swept_clear(grid,a,b,p,u))

    def test_outside_and_unverified_rejected(self):
        grid,p,u=self.scene()
        self.assertFalse(footprint_report(grid,Pose(2,0,0),p,u)['clear'])
        grid.verified=False
        self.assertFalse(footprint_report(grid,Pose(0,0,0),p,u)['clear'])
