import unittest
from wolf_go2.crossmodal_tracks import PairPersistence


class PairPersistenceTests(unittest.TestCase):
    def test_counts_unique_frames_without_claiming_calibration(self):
        tracker=PairPersistence()
        for i in range(3):rows=tracker.update('a',i,i,[[1,2]])
        self.assertEqual(rows[0]['consecutive_frames'],3)
        self.assertFalse(rows[0]['calibration_verified'])

    def test_duplicates_do_not_inflate_count(self):
        tracker=PairPersistence()
        tracker.update('a',1,1,[[1,2]])
        self.assertEqual(tracker.update('a',1,1,[[1,2]]),[])
        self.assertEqual(tracker.update('a',2,1,[[1,2]]),[])
        self.assertEqual(tracker.update('a',3,3,[[1,2]])[0]['consecutive_frames'],2)

    def test_context_gap_backwards_and_missing_reset(self):
        for context,t,c,points in [('b',2,2,[[1,2]]),('a',5,5,[[1,2]]),
                                  ('a',0,0,[[1,2]]),('a',2,2,[])]:
            tracker=PairPersistence()
            tracker.update('a',1,1,[[1,2]])
            rows=tracker.update(context,t,c,points)
            if points:self.assertEqual(rows[0]['consecutive_frames'],1)
            else:self.assertEqual(tracker.update('a',3,3,[[1,2]])[0]['consecutive_frames'],1)

    def test_ambiguous_identity_not_confirmed(self):
        tracker=PairPersistence()
        tracker.update('a',0,0,[[0,0],[.1,0]])
        rows=tracker.update('a',1,1,[[.05,0]])
        self.assertEqual(rows[0]['consecutive_frames'],1)

    def test_slow_drift_cannot_chain_across_objects(self):
        tracker=PairPersistence()
        tracker.update('a',0,0,[[0,0]])
        tracker.update('a',1,1,[[.1,0]])
        rows=tracker.update('a',2,2,[[.2,0]])
        self.assertEqual(rows[0]['consecutive_frames'],1)
