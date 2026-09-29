import unittest
import cv2
import numpy as np
from audit_rotation_timing import image_shift, fit_alignment


class RotationTimingTests(unittest.TestCase):
    def test_small_rotation_does_not_produce_calibration(self):
        times=np.linspace(0,5,101)
        poses=np.column_stack((times,np.deg2rad(.6)*np.clip(times-2,0,1)))
        frames=[{'t':float(t),'shift_x_px':float(y*900)} for t,y in poses]
        result=fit_alignment(frames,poses)
        self.assertEqual(result['status'],'insufficient_rotation')
        self.assertEqual(result['relative_alignment_candidates'],[])

    def test_known_relative_delay_is_recovered(self):
        times=np.linspace(0,5,101)
        yaw=lambda t: .08*np.clip(t-1.5,0,1)
        poses=np.column_stack((times,yaw(times)))
        frames=[{'t':float(t),'shift_x_px':float(900*yaw(t-.2)+3)} for t in times]
        result=fit_alignment(frames,poses)
        self.assertEqual(result['status'],'relative_estimate_only')
        best=result['relative_alignment_candidates'][0]
        self.assertAlmostEqual(best['relative_shift_s'],.2)
        self.assertAlmostEqual(best['pixel_per_yaw_radian'],900)

    def test_repeated_odometry_time_is_rejected(self):
        poses=np.zeros((30,2))
        frames=[{'t':float(t),'shift_x_px':0.} for t in range(30)]
        with self.assertRaises(ValueError):fit_alignment(frames,poses)

    def test_known_image_translation(self):
        rng=np.random.default_rng(21)
        reference=cv2.GaussianBlur(rng.integers(0,256,(240,320),dtype=np.uint8),(5,5),0)
        image=cv2.warpAffine(reference,np.float32([[1,0,12],[0,1,-5]]),(320,240))
        features=cv2.goodFeaturesToTrack(reference,400,.02,8)
        result=image_shift(reference,image,features)
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result['shift_x_px'],12.,delta=.3)
        self.assertAlmostEqual(result['shift_y_px'],-5.,delta=.3)

    def test_missing_visual_correspondence_is_not_zero_motion(self):
        rng=np.random.default_rng(21)
        reference=cv2.GaussianBlur(rng.integers(0,256,(240,320),dtype=np.uint8),(5,5),0)
        features=cv2.goodFeaturesToTrack(reference,400,.02,8)
        self.assertIsNone(image_shift(reference,np.zeros_like(reference),features))
