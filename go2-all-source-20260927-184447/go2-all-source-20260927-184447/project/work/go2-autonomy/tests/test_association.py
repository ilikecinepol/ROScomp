import unittest
import numpy as np
from wolf_go2.association import gated_assignment


class AssociationTests(unittest.TestCase):
    def test_distant_pair_is_not_forced(self):
        self.assertEqual(gated_assignment([[80.]]),[])

    def test_unique_pairs(self):
        pairs=gated_assignment([[10,100],[100,15]])
        self.assertEqual([(p['image_index'],p['projected_index']) for p in pairs],[(0,0),(1,1)])
        self.assertTrue(all(p['accepted'] for p in pairs))

    def test_ambiguous_neighbours_rejected(self):
        self.assertFalse(gated_assignment([[10,11]])[0]['accepted'])
        self.assertFalse(gated_assignment([[10],[11]])[0]['accepted'])

    def test_more_silhouettes_than_objects(self):
        pairs=gated_assignment([[1],[100],[100]])
        self.assertEqual(len(pairs),1)
        self.assertTrue(pairs[0]['accepted'])

    def test_global_assignment_resolves_local_competition(self):
        pairs=gated_assignment([[10,11],[11,100]])
        self.assertEqual([(p['image_index'],p['projected_index']) for p in pairs],[(0,1),(1,0)])
        self.assertTrue(all(p['accepted'] for p in pairs))

    def test_no_objects_and_invalid_costs(self):
        self.assertEqual(gated_assignment(np.empty((2,0))),[])
        for value in ([[float('nan')]], [[-1.]], [1,2]):
            with self.assertRaises(ValueError):gated_assignment(value)
