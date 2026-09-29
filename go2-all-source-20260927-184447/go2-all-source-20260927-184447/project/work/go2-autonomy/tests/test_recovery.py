import math
import unittest
from wolf_go2.recovery import SensorRecovery


class RecoveryTests(unittest.TestCase):
    def test_grace_period_and_missing_stream(self):
        r=SensorRecovery(0.)
        self.assertEqual(r.actions(1.,{'ULIDAR_ARRAY':-math.inf}),[])
        self.assertEqual(r.actions(2.,{'ULIDAR_ARRAY':-math.inf}),['ULIDAR_ARRAY'])

    def test_camera_has_longer_grace(self):
        r=SensorRecovery(0.)
        self.assertEqual(r.actions(3.,{'CAMERA':-math.inf}),[])
        self.assertEqual(r.actions(5.,{'CAMERA':-math.inf}),['CAMERA'])

    def test_no_retry_loop_and_bounded_attempts(self):
        r=SensorRecovery(0.)
        self.assertEqual(r.actions(2.,{'ULIDAR_ARRAY':0.}),['ULIDAR_ARRAY'])
        self.assertEqual(r.actions(3.,{'ULIDAR_ARRAY':0.}),[])
        self.assertEqual(r.actions(8.,{'ULIDAR_ARRAY':0.}),['ULIDAR_ARRAY'])
        self.assertEqual(r.actions(50.,{'ULIDAR_ARRAY':0.}),[])

    def test_new_packet_confirms_transport_only(self):
        r=SensorRecovery(0.)
        r.actions(2.,{'ULIDAR_ARRAY':0.})
        r.actions(2.5,{'ULIDAR_ARRAY':2.4})
        self.assertTrue(r.report()['transport_resumed_after_attempt']['ULIDAR_ARRAY'])
        self.assertFalse(r.report()['measurement_freshness_verified'])
        self.assertFalse(r.report()['motion_authorized'])

    def test_fresh_transport_is_not_restarted_for_bad_source_clock(self):
        r=SensorRecovery(0.)
        self.assertEqual(r.actions(10.,{'ULIDAR_ARRAY':9.9}),[])

    def test_started_mission_never_restarts_streams(self):
        r=SensorRecovery(0.)
        self.assertEqual(r.actions(10.,{'ULIDAR_ARRAY':0.},True),[])
        self.assertEqual(r.report()['attempts'],{})

    def test_old_confirmation_does_not_confirm_second_attempt(self):
        r=SensorRecovery(0.)
        r.actions(2.,{'ULIDAR_ARRAY':0.})
        r.actions(3.,{'ULIDAR_ARRAY':2.5})
        r.actions(8.,{'ULIDAR_ARRAY':2.5})
        self.assertFalse(r.report()['transport_resumed_after_attempt']['ULIDAR_ARRAY'])
