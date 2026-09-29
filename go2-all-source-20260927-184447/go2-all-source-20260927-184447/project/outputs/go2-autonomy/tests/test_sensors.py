"""Локальные проверки качества потоков, без подключения к устройству."""
import math
import unittest

from wolf_go2.models import Policy
from wolf_go2.sensors import SensorMonitor, finite, pose_from_message, stamp_seconds


def odom(x=0., y=0., z=.30, yaw=0., source=100.):
    return {'data': {'header': {'stamp': source, 'frame_id': 'world'}, 'pose': {
        'position': {'x': x, 'y': y, 'z': z},
        'orientation': {'x': 0., 'y': 0., 'z': math.sin(yaw/2), 'w': math.cos(yaw/2)}}}}


def sport():
    return {'data': {'imu_state': {'rpy': [0., 0., 0.]}, 'body_height': .30,
                     'velocity': [0., 0., 0.], 'error_code': 100}}


def low():
    return {'data': {'motor_state': [{'temperature': 30.} for _ in range(12)],
                     'bms_state': {'soc': 80.}}}


def healthy_monitor(received=10., source=100.):
    monitor = SensorMonitor()
    for name, message in (('LF_SPORT_MOD_STATE', sport()), ('LOW_STATE', low()),
                          ('ROBOTODOM', odom(source=source)), ('CAMERA', {'data': {}})):
        monitor.ingest(name, message, received, source)
    return monitor


class SensorTests(unittest.TestCase):
    def setUp(self):
        self.policy = Policy()

    def test_finite_and_source_stamp_validation(self):
        for value in (True, False, None, '1', float('nan'), float('inf')):
            self.assertFalse(finite(value))
            self.assertIsNone(stamp_seconds(value))
        self.assertAlmostEqual(stamp_seconds({'sec': 10, 'nanosec': 250000000}), 10.25)
        for source in ({'sec': 10, 'nanosec': -1}, {'sec': 10, 'nanosec': 1e9},
                       {'sec': False, 'nanosec': 0}, {'sec': float('nan'), 'nanosec': 0}):
            self.assertIsNone(stamp_seconds(source))

    def test_named_quaternion_and_normalization(self):
        packet = odom(x=1., y=-2., z=.4, yaw=1.2)
        pose = pose_from_message(packet)
        self.assertEqual((pose.x, pose.y, pose.z), (1., -2., .4))
        self.assertAlmostEqual(pose.yaw, 1.2)
        q = packet['data']['pose']['orientation']
        for key in q:
            q[key] *= 1.005
        self.assertAlmostEqual(pose_from_message(packet).yaw, 1.2)

    def test_nan_and_nonunit_quaternion_rejected(self):
        for part, key, value in (('position', 'x', float('nan')), ('position', 'y', True),
                                 ('orientation', 'w', 2.), ('orientation', 'z', float('inf'))):
            packet = odom()
            packet['data']['pose'][part][key] = value
            with self.assertRaises(ValueError):
                pose_from_message(packet)

    def test_healthy_state_including_sport_mode_code_100(self):
        monitor = healthy_monitor()
        self.assertEqual(monitor.health(10.1, self.policy), [])
        self.assertEqual(monitor.pose_epoch, 0)

    def test_missing_and_stale_streams_block(self):
        self.assertTrue(SensorMonitor().health(10., self.policy))
        monitor = healthy_monitor()
        self.assertTrue(any('CAMERA' in reason for reason in monitor.health(10.7, self.policy)))
        self.assertTrue(any('ROBOTODOM' in reason for reason in monitor.health(10.7, self.policy)))

    def test_low_state_uses_two_second_limit(self):
        monitor = healthy_monitor()
        for name, message in (('LF_SPORT_MOD_STATE', sport()), ('ROBOTODOM', odom()), ('CAMERA', {'data': {}})):
            monitor.ingest(name, message, 11.5, 101.)
        self.assertEqual(monitor.health(11.6, self.policy), [])
        for name, message in (('LF_SPORT_MOD_STATE', sport()), ('ROBOTODOM', odom()), ('CAMERA', {'data': {}})):
            monitor.ingest(name, message, 12.2, 102.)
        self.assertTrue(any('LOW_STATE' in reason for reason in monitor.health(12.2, self.policy)))

    def test_duplicate_source_does_not_refresh_progress(self):
        monitor = healthy_monitor()
        monitor.ingest('CAMERA', {'data': {}}, 10.3, 100.)
        monitor.ingest('CAMERA', {'data': {}}, 10.7, 100.)
        stream = monitor.streams['CAMERA']
        self.assertEqual(stream.duplicates, 2)
        self.assertEqual(stream.progressed, 10.)
        self.assertTrue(any('продвижение времени CAMERA' in reason for reason in monitor.health(10.7, self.policy)))

    def test_missing_or_nan_stamp_does_not_inherit_previous_validity(self):
        for stamp in (None, float('nan'), float('inf')):
            monitor = healthy_monitor()
            monitor.ingest('ROBOTODOM', odom(x=.01, source=stamp), 10.1)
            self.assertTrue(monitor.health(10.1, self.policy), stamp)
            age = monitor.calibrated_age('ROBOTODOM', 10.1, {'verified': True, 'scale': 1., 'offset': -90.})
            self.assertTrue(math.isinf(age), stamp)

    def test_backward_source_is_latched(self):
        monitor = healthy_monitor()
        monitor.ingest('CAMERA', {'data': {}}, 10.1, 99.)
        monitor.ingest('CAMERA', {'data': {}}, 10.2, 101.)
        self.assertEqual(monitor.streams['CAMERA'].backwards, 1)
        self.assertTrue(any('продвижение времени CAMERA' in reason for reason in monitor.health(10.2, self.policy)))

    def test_backward_receive_is_fault_and_packet_not_applied(self):
        monitor = healthy_monitor()
        monitor.ingest('ROBOTODOM', odom(x=5.), 9.9, 101.)
        self.assertEqual(monitor.last_pose[1].x, 0.)
        self.assertTrue(any('часы пошли назад' in reason for reason in monitor.health(10.1, self.policy)))

    def test_invalid_receive_rejected(self):
        for received in (True, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                SensorMonitor().ingest('CAMERA', {'data': {}}, received, 1.)

    def test_nan_pose_fault_keeps_previous_valid_pose(self):
        monitor = healthy_monitor()
        monitor.ingest('ROBOTODOM', odom(x=float('nan')), 10.1, 100.1)
        self.assertEqual(monitor.last_pose[1].x, 0.)
        self.assertEqual(monitor.streams['ROBOTODOM'].invalid, 1)
        self.assertTrue(any('Некорректная поза' in reason for reason in monitor.health(10.1, self.policy)))

    def test_xy_and_yaw_jumps_start_new_epoch(self):
        for packet in (odom(x=1.), odom(y=1.), odom(yaw=1.)):
            monitor = healthy_monitor()
            monitor.ingest('ROBOTODOM', packet, 10.05, 100.05)
            self.assertEqual(monitor.pose_epoch, 1)
            self.assertTrue(any('Скачок координат' in reason for reason in monitor.health(10.05, self.policy)))

    def test_yaw_wrap_is_not_a_jump(self):
        monitor = healthy_monitor()
        monitor.last_pose = None
        monitor.ingest('ROBOTODOM', odom(yaw=math.pi-.02), 10.1, 100.1)
        monitor.ingest('ROBOTODOM', odom(yaw=-math.pi+.02), 10.15, 100.15)
        self.assertEqual(monitor.pose_epoch, 0)

    def test_z_jump_changes_frame_epoch(self):
        monitor = healthy_monitor()
        monitor.ingest('ROBOTODOM', odom(z=2.), 10.05, 100.05)
        self.assertEqual(monitor.pose_epoch, 1, 'Скачок высоты изменяет метрическую систему поверхностей')

    def test_jump_at_equal_receive_time_is_not_ignored(self):
        monitor = healthy_monitor()
        monitor.ingest('ROBOTODOM', odom(x=2.), 10., 100.01)
        self.assertEqual(monitor.pose_epoch, 1)

    def test_calibrated_age_requires_verified_contract(self):
        monitor = healthy_monitor(source=10000.)
        for calibration in (None, {}, {'verified': False, 'scale': .001, 'offset': 0.}):
            self.assertTrue(math.isinf(monitor.calibrated_age('CAMERA', 10.2, calibration)))
        self.assertAlmostEqual(monitor.calibrated_age('CAMERA', 10.2,
            {'verified': True, 'scale': .001, 'offset': 0.}), .2)
        self.assertTrue(math.isinf(monitor.calibrated_age('CAMERA', 9.,
            {'verified': True, 'scale': .001, 'offset': 0.})))

    def test_invalid_calibration_scale_cannot_create_fresh_data(self):
        monitor = healthy_monitor(source=100.)
        for scale in (0., -1., False, True, float('nan'), float('inf')):
            offset = 10.-float(scale)*100. if math.isfinite(float(scale)) else 0.
            age = monitor.calibrated_age('CAMERA', 10.1, {'verified': True, 'scale': scale, 'offset': offset})
            self.assertTrue(math.isinf(age), scale)

    def test_health_limits_and_incomplete_state(self):
        for mutate in (
            lambda d: d['imu_state']['rpy'].__setitem__(0, math.radians(20)),
            lambda d: d.__setitem__('body_height', .1),
            lambda d: d.__setitem__('velocity', [1., 0., 0.]),
            lambda d: d.__setitem__('velocity', [float('nan'), 0., 0.])):
            monitor = healthy_monitor()
            mutate(monitor.streams['LF_SPORT_MOD_STATE'].payload)
            self.assertTrue(monitor.health(10.1, self.policy))
        monitor = healthy_monitor()
        monitor.streams['LOW_STATE'].payload['bms_state']['soc'] = 10.
        self.assertTrue(any('Заряд' in reason for reason in monitor.health(10.1, self.policy)))
        monitor = healthy_monitor()
        monitor.streams['LOW_STATE'].payload['motor_state'][0]['temperature'] = 65.
        self.assertTrue(any('температура' in reason for reason in monitor.health(10.1, self.policy)))

    def test_report_does_not_infer_clock_calibration(self):
        monitor = SensorMonitor()
        for index in range(4):
            monitor.ingest('CAMERA', {'data': {}}, 10.+index*.1, index*10.)
        report = monitor.report(10.4)['streams']['CAMERA']
        self.assertAlmostEqual(report['median_hz'], 10.)
        self.assertAlmostEqual(report['source_scale_by_receive'], 100.)
        self.assertFalse(report['timing_calibrated_by_this_report'])


if __name__ == '__main__':
    unittest.main()
