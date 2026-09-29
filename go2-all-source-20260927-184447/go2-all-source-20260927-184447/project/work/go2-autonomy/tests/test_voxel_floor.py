"""Квантование пола не превращает наблюдённые соседние уровни в стену."""
import unittest
import numpy as np
from wolf_go2.perception import _support_grid
from wolf_go2.models import Policy


class VoxelFloorTests(unittest.TestCase):
    def test_adjacent_levels_only_with_voxel_contract(self):
        top=np.array([[-.05,0,.05,.10,np.nan]])
        grid,raised=_support_grid(top,0.,np.zeros(2),.05,0,Policy(),voxel_size=.05)
        self.assertEqual(grid.cells,[[0,0,0,1,-1]])
        self.assertTrue(raised[0,3])
        raw,_=_support_grid(top,0.,np.zeros(2),.05,0,Policy())
        self.assertEqual(raw.cells,[[1,0,1,1,-1]])

    def test_coarse_voxel_cannot_hide_excessive_step(self):
        grid,_=_support_grid(np.array([[.10]]),0.,np.zeros(2),.1,0,Policy(),voxel_size=.1)
        self.assertEqual(grid.cells,[[1]])
