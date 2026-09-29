import unittest
from wolf_go2.preparation import timing_estimate,prepare_report

class PreparationTests(unittest.TestCase):
    def test_wire_rounding_is_reported_without_certifying_freshness(self):
        result=prepare_report({'wire_audit':{'packets':100,'distinct_exact_stamps':1,
            'first':{'stamp_form':'number','represented_decimal_quantum':'1000'}}})
        self.assertEqual(result['lidar_wire_time_status'],'coarse_constant_number_on_wire')
        self.assertFalse(result['ready_for_motion'])
        self.assertTrue(any('JSON' in text for text in result['blockers']))

    def test_clock_scale_not_absolute_latency(self):
        result=timing_estimate([(10+i*.1,100+i*.001) for i in range(100)])
        self.assertAlmostEqual(result['source_to_receive_scale'],100,places=6)
        self.assertIsNone(result['absolute_latency_s'])
        self.assertFalse(result['verified'])

    def test_constant_timestamp(self):
        self.assertEqual(timing_estimate([(i*.1,1790413000.) for i in range(100)])['status'],'constant_source_stamp')

    def test_backward_clock(self):
        rows=[(i,i) for i in range(20)];rows[-1]=(19,0)
        self.assertEqual(timing_estimate(rows)['status'],'non_monotonic')

    def test_sparse_and_nonfinite_are_not_calibrated(self):
        self.assertEqual(timing_estimate([(1,float('nan'))])['status'],'insufficient_samples')

    def test_scene_candidate_does_not_unlock_motion(self):
        result=prepare_report({'scene_audit':{'camera_striped_poles':[{}]*3,'slalom_row_candidates':[{}]},
                               'packet_audit':{'transport_status':'content_changes_source_time_unavailable'}})
        self.assertFalse(result['ready_for_motion']);self.assertFalse(result['profile_modified'])
        self.assertEqual(result['camera_poles'],3)
        self.assertTrue(any('метка' in text for text in result['blockers']))
