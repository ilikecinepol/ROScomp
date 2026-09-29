"""Пиксельная синтетика согласованности, без метрической калибровки камеры."""
import json
import math
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from wolf_go2.consistency import ConsistencyMonitor
from wolf_go2.models import Pose


ORIENTATION = 'orientation_or_video_inconsistency'
DUPLICATE = 'duplicate_video_during_pose_motion'


def texture(seed=2026):
    rng = np.random.default_rng(seed)
    image = rng.integers(0,256,(360,640),dtype=np.uint8)
    image = cv2.GaussianBlur(image,(3,3),0)
    return cv2.cvtColor(image,cv2.COLOR_GRAY2BGR)


class ConsistencyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.image = texture()

    def series(self, monitor, image=None, pose=None, end=6.):
        result = []
        for i in range(round(end*2)+1):
            t = i*.5
            result = monitor.observe(image(t) if image else self.image,
                                     pose(t) if pose else Pose(0.,0.,0.),t)
        return result

    def test_static_scene_and_yaw_drift_reports_ambiguity_without_correction(self):
        m = ConsistencyMonitor()
        original = Pose(0.,0.,math.radians(3.))
        reasons = self.series(m,pose=lambda t: Pose(0.,0.,math.radians(t*.5)))
        self.assertIn(ORIENTATION,reasons)
        self.assertGreaterEqual(m.report['tracked_features'],40)
        self.assertLess(m.report['p95_displacement_px'],1.)
        self.assertTrue(m.report['diagnostic_only'])
        self.assertIsNone(m.report['hardware_verdict'])
        self.assertFalse(m.report['correction_applied'])
        m.observe(self.image,original,6.5)
        self.assertEqual(original.yaw,math.radians(3.))
        json.dumps(m.report,allow_nan=False)

    def test_short_window_cannot_report_orientation(self):
        m = ConsistencyMonitor()
        reasons = self.series(m,pose=lambda t: Pose(0.,0.,math.radians(t)),end=4.5)
        self.assertNotIn(ORIENTATION,reasons)

    def test_static_duplicate_without_pose_motion_is_not_warning(self):
        m = ConsistencyMonitor()
        self.assertEqual(self.series(m,end=8.),[])
        self.assertEqual(m.report['exact_duplicate_duration_s'],8.)
        self.assertFalse(m.report['pose_change_with_exact_same_frame'])

    def test_duplicate_flags_only_after_three_seconds_and_observed_motion(self):
        m = ConsistencyMonitor()
        reasons = self.series(m,pose=lambda t: Pose(t*.01,0.,0.),end=3.)
        self.assertNotIn(DUPLICATE,reasons)
        reasons = m.observe(self.image,Pose(.035,0.,0.),3.5)
        self.assertIn(DUPLICATE,reasons)
        self.assertNotIn(ORIENTATION,reasons)
        self.assertTrue(m.report['pose_change_with_exact_same_frame'])

    def test_changed_video_with_stationary_pose_does_not_flag(self):
        m = ConsistencyMonitor()
        reasons = self.series(m,image=lambda t: texture(int(t*2)+15))
        self.assertEqual(reasons,[])
        self.assertEqual(m.report['exact_duplicate_duration_s'],0.)

    def test_genuine_perspective_motion_clears_previous_warning(self):
        m = ConsistencyMonitor()
        self.assertIn(ORIENTATION,self.series(m,pose=lambda t:Pose(0.,0.,math.radians(t))))
        for i in range(1,10):
            transform = np.array([[1.,.0008*i,1.5*i],[.0005*i,1.,.4*i],[.00001*i,0.,1.]])
            moved = cv2.warpPerspective(self.image,transform,(640,360))
            reasons = m.observe(moved,Pose(0.,0.,math.radians(6.+i)),6.+i*.5)
            self.assertNotIn(ORIENTATION,reasons)
            self.assertNotIn(DUPLICATE,reasons)

    def test_wraparound_yaw_uses_shortest_angle_and_strict_threshold(self):
        m = ConsistencyMonitor()
        reasons = self.series(m,pose=lambda t:Pose(0.,0.,math.radians(179.+t/3)))
        self.assertNotIn(ORIENTATION,reasons)
        self.assertAlmostEqual(m.report['pose_yaw_change_deg'],2.)
        self.assertIn(ORIENTATION,m.observe(self.image,Pose(0.,0.,math.radians(-178.9)),6.5))

    def test_translation_limit_and_return_do_not_join_static_windows(self):
        m = ConsistencyMonitor()
        self.series(m,pose=lambda t:Pose(0.,0.,math.radians(t*.5)),end=4.)
        self.assertNotIn(ORIENTATION,m.observe(self.image,Pose(.04,0.,math.radians(3)),4.5))
        self.assertNotIn(ORIENTATION,m.observe(self.image,Pose(0.,0.,math.radians(3)),5.))
        self.assertLess(m.report['static_duration_s'],5.)

    def test_time_gap_reversal_and_duplicate_timestamp_reset_evidence(self):
        for next_t in (4.,3.,20.):
            with self.subTest(next_t=next_t):
                m = ConsistencyMonitor()
                self.series(m,pose=lambda t:Pose(0.,0.,math.radians(t)),end=4.)
                reasons = m.observe(self.image,Pose(0.,0.,math.radians(5.)),next_t)
                self.assertNotIn(ORIENTATION,reasons)
                self.assertEqual(m.report['status'],'time_discontinuity_reset')
                self.assertEqual(m.report['static_duration_s'],0.)
                self.assertEqual(m.report['exact_duplicate_duration_s'],0.)

    def test_feature_and_pixel_thresholds_are_strict(self):
        # Контролируем измеренные соответствия, проверяя границы решения;
        # реальные ORB и изображения проверяются остальными тестами.
        rng = np.random.default_rng(12)
        for count,displacement,expected in ((39,0.,False),(40,.99,True),(40,1.,False),(40,1.01,False)):
            with self.subTest(count=count,displacement=displacement):
                m = ConsistencyMonitor()
                points = np.array([(i*4.,i%7*10.) for i in range(count)],dtype=np.float32)
                descriptors = rng.integers(0,256,(count,32),dtype=np.uint8)
                frames = [(points.copy(),descriptors.copy())] + [(points+np.array([displacement,0.]),descriptors.copy()) for _ in range(12)]
                with patch.object(m,'_features',side_effect=frames):
                    reasons = self.series(m,pose=lambda t:Pose(0.,0.,math.radians(t*.5)),end=5.)
                self.assertEqual(ORIENTATION in reasons,expected)

    def test_width_normalization_matches_640_and_1280(self):
        large = cv2.resize(self.image,(1280,720),interpolation=cv2.INTER_NEAREST)
        outputs = []
        for image in (self.image,large):
            m = ConsistencyMonitor()
            outputs.append((self.series(m,image=lambda t:image,pose=lambda t:Pose(0.,0.,math.radians(t*.5))),m.report))
        self.assertEqual(outputs[0][0],outputs[1][0])
        self.assertEqual(outputs[0][1]['tracked_features'],outputs[1][1]['tracked_features'])
        self.assertEqual(outputs[0][1]['p95_displacement_px'],outputs[1][1]['p95_displacement_px'])

    def test_low_texture_never_proves_static_scene(self):
        m = ConsistencyMonitor()
        image = np.full((360,640,3),128,np.uint8)
        reasons = self.series(m,image=lambda t:image,pose=lambda t:Pose(0.,0.,math.radians(t*.5)))
        self.assertNotIn(ORIENTATION,reasons)
        self.assertLess(m.report['tracked_features'],40)

    def test_invalid_input_clears_window_and_report_is_copy(self):
        m = ConsistencyMonitor()
        self.series(m)
        self.assertEqual(m.observe(self.image,Pose(float('nan'),0.,0.),6.5),[])
        self.assertEqual(m.report['status'],'invalid_input')
        copied = m.report
        copied['reasons'].append('not_real')
        self.assertEqual(m.report['reasons'],[])
        self.assertEqual(m.observe(None,Pose(0.,0.,0.),7.),[])
        json.dumps(m.report,allow_nan=False)

    def test_valid_sparse_sampling_has_no_hidden_six_frame_requirement(self):
        m = ConsistencyMonitor()
        for t in (0.,1.25,2.5,3.75,5.):
            reasons = m.observe(self.image,Pose(0.,0.,math.radians(t*.6)),t)
        self.assertIn(ORIENTATION,reasons)

    def test_extreme_finite_pose_does_not_raise_or_make_invalid_json(self):
        m = ConsistencyMonitor()
        m.observe(self.image,Pose(0.,0.,-1e308),0.)
        m.observe(self.image,Pose(0.,0.,1e308),.5)
        json.dumps(m.report,allow_nan=False)
        m.observe(self.image,Pose(-1e308,0.,0.),1.)
        m.observe(self.image,Pose(1e308,0.,0.),1.5)
        self.assertEqual(m.report['status'],'invalid_input')
        json.dumps(m.report,allow_nan=False)


if __name__ == '__main__':
    unittest.main()
