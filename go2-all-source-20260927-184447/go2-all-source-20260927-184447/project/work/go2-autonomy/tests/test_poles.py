import unittest
import numpy as np
from wolf_go2.poles import pole_candidates, slalom_pole_candidates, pole_rows


class PoleTests(unittest.TestCase):
    def test_partial_near_poles_extend_two_observed_tall_anchors(self):
        points=[]
        for x,height in ((0.,.85),(1.,.8),(2.,.5),(3.,.25)):
            points.extend([[x+dx,dy,z] for dx in (0.,.05,.1) for dy in (0.,.05,.1)
                           for z in np.arange(.1,height+.001,.05)])
        found=slalom_pole_candidates(np.asarray(points),[1e-18,0,1e-17])
        rows=pole_rows(found)
        self.assertTrue(any(len(r['pole_indices'])==4 for r in rows))
        self.assertTrue(any(p.get('partially_observed') for p in found))
        self.assertTrue(all(not r['route_verified'] for r in rows))

    def test_low_objects_without_two_tall_anchors_are_not_promoted(self):
        points=[]
        for x in (0.,1.,2.,3.):
            points.extend([[x+dx,dy,z] for dx in (0.,.05,.1) for dy in (0.,.05,.1)
                           for z in np.arange(.1,.251,.05)])
        self.assertEqual(pole_rows(slalom_pole_candidates(np.asarray(points),[0,0,0])),[])

    def test_short_columns_remain_unverified(self):
        found=slalom_pole_candidates(np.array(self.column(0,.65)+self.column(1,.65)+self.column(2,.65)),[0,0,0])
        self.assertEqual(len(found),3)
        rows=pole_rows(found)
        self.assertEqual(len(rows),1)
        self.assertFalse(rows[0]['route_verified'])

    def test_gap_is_not_bridged(self):
        rows=pole_rows([{'center_xy':[x,0]} for x in [0,1,2,5]])
        self.assertEqual(len(rows),1)
        self.assertEqual(set(rows[0]['pole_indices']),{0,1,2})

    def test_isolated_and_crowded_objects_are_not_row(self):
        self.assertEqual(pole_rows([{'center_xy':[x,0]} for x in [0,.1,.2]]),[])
        self.assertEqual(pole_rows([{'center_xy':[0,0]}]),[])

    def test_invalid_row_coordinates(self):
        with self.assertRaises(ValueError):
            pole_rows([{'center_xy':[float('nan'),0]}]*3)

    def test_short_wall_is_not_pole(self):
        cloud=np.array([[x,0,z] for x in np.arange(0,2,.05) for z in np.arange(.35,.7,.05)])
        self.assertEqual(slalom_pole_candidates(cloud,[0,0,0]),[])

    def test_slalom_keeps_tall_poles_with_connected_lower_parts(self):
        low=[[x,0,z] for x in np.arange(0,1.1,.05) for z in np.arange(.35,.8,.05)]
        found=slalom_pole_candidates(np.array(self.column(0)+self.column(1)+low),[0,0,0])
        self.assertEqual(len(found),2)
        self.assertTrue(all(p['row_eligible'] is False for p in found))

    def test_three_upper_fragments_of_wall_do_not_form_slalom(self):
        low=[[x,0,z] for x in np.arange(0,2.15,.05) for z in np.arange(.15,.8,.05)]
        found=slalom_pole_candidates(np.array(self.column(0)+self.column(1)+self.column(2)+low),[0,0,0])
        self.assertEqual(len(found),3)
        self.assertEqual(pole_rows(found),[])

    def column(self,x,height=1.25):
        return [[x+dx,dy,z] for dx in [0,.05,.1] for dy in [0,.05,.1] for z in np.arange(.35,height,.05)]

    def test_distinct_columns_and_low_object(self):
        cloud=np.array(self.column(0)+self.column(1)+self.column(2,.65))
        found=pole_candidates(cloud,[0,0,0])
        self.assertEqual(len(found),2)
        self.assertTrue(all(not p['verified'] for p in found))

    def test_wall_is_not_pole(self):
        cloud=np.array([[x,0,z] for x in np.arange(0,2,.05) for z in np.arange(.35,1.3,.05)])
        self.assertEqual(pole_candidates(cloud,[0,0,0]),[])

    def test_low_connection_does_not_merge_poles(self):
        low=[[x,0,z] for x in np.arange(0,1.1,.05) for z in np.arange(.35,.65,.05)]
        found=pole_candidates(np.array(self.column(0)+self.column(1)+low),[0,0,0])
        self.assertEqual(len(found),2)
        self.assertTrue(all(not p['verified'] for p in found))
