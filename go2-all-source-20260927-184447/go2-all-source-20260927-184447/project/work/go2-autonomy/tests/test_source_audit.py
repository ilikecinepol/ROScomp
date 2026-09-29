import unittest
from wolf_go2.source_audit import SourceAudit

class SourceAuditTests(unittest.TestCase):
    def test_progress_and_no_fabricated_calibration(self):
        audit=SourceAudit(clock=lambda:10.)
        for second in (1,2,2,1):
            audit.receive('SLAM_ODOMETRY',{'data':{'header':{'stamp':{'sec':second,'nanosec':0},'frame_id':'odom'},'child_frame_id':'base_link','points':[1]*100}})
        state=audit.report()['SLAM_ODOMETRY']
        self.assertEqual((state['advances'],state['duplicates'],state['backwards']),(1,1,1))
        self.assertEqual(state['header']['child_frame_id'],'base_link')
        self.assertNotIn('points',str(state))
        self.assertFalse(state['acquisition_age_verified'])

    def test_missing_and_malformed_do_not_count_as_valid(self):
        audit=SourceAudit()
        for msg in ({},None,{'data':[]},{'data':{'header':None}}):audit.receive('ULIDAR',msg)
        self.assertEqual(audit.report()['ULIDAR']['valid_stamps'],0)
        self.assertIsNone(audit.report()['UWB_STATE']['receive_age_s'])
