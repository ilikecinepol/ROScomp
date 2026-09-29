"""Проверки геометрии ската; не испытания локомоции."""
import unittest
import numpy as np
from wolf_go2.ramp_plane import detect_ramp


class RampPlaneTests(unittest.TestCase):
    def test_ramp_separated_from_connected_flat_mat_and_outliers(self):
        x,y=np.meshgrid(np.linspace(.3,1.5,26),np.linspace(-.25,.25,12))
        ramp=np.column_stack((x.ravel(),y.ravel(),(.4*x+.05).ravel()))
        x,y=np.meshgrid(np.linspace(.1,2,35),np.linspace(-.9,.9,25))
        mat=np.column_stack((x.ravel(),y.ravel(),np.full(x.size,.15)))
        rng=np.random.default_rng(5)
        noise=rng.uniform([.05,-1,.1],[3,1,1],(100,3))
        result=detect_ramp(np.vstack((ramp,mat,noise)))
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result['coefficients_z_ax_by_c'][0],.4,delta=.04)
        self.assertFalse(result['motion_authorized'])
        self.assertFalse(result['metric_target'])

    def test_flat_surface_is_not_ramp(self):
        x,y=np.meshgrid(np.linspace(.1,2,30),np.linspace(-.8,.8,30))
        self.assertIsNone(detect_ramp(np.column_stack((x.ravel(),y.ravel(),np.full(x.size,.2)))))

    def test_sparse_and_invalid(self):
        self.assertIsNone(detect_ramp(np.zeros((3,3))))
        self.assertIsNone(detect_ramp(np.full((80,3),np.nan)))
        with self.assertRaises(ValueError):detect_ramp([1,2,3])
