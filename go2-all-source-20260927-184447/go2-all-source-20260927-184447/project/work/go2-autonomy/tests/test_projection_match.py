import unittest
from wolf_go2.projection_match import curve_cost


class ProjectionMatchTests(unittest.TestCase):
    def test_nearby_line_outside_silhouette_is_not_a_match(self):
        cost,residual=curve_cost([[18,15],[18,20]], [20,10,30,30])
        self.assertEqual(cost,1e6)
        self.assertIsNone(residual)

    def test_inside_point_gives_residual(self):
        cost,residual=curve_cost([[24,20],[24,40]], [20,10,30,30])
        self.assertEqual(cost,1.)
        self.assertEqual(residual,[1.,0.])

    def test_vertical_disjoint_projection_is_not_a_match(self):
        self.assertEqual(curve_cost([[25,40]], [20,10,30,30])[0],1e6)

    def test_invalid_data_rejected(self):
        for points,box in [([[float('nan'),20]],[20,10,30,30]),
                           ([[25,20]],[20,10,20,30])]:
            with self.assertRaises(ValueError):curve_cost(points,box)
