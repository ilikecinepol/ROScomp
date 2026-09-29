"""Регрессии исследовательского детектора; без управления роботом."""
import unittest
import numpy as np
import cv2
from ramp_vision import _extract_lines, detect_ramp


class RampVisionTests(unittest.TestCase):
    def test_two_faces_of_bar_count_as_one_level(self):
        image=np.full((540,960),180,np.uint8)
        for y in (220,280,340):
            cv2.rectangle(image,(300,y),(650,y+6),40,-1)
        lines=_extract_lines(image)
        self.assertEqual(len(lines),3)

    def test_blank_is_not_ramp(self):
        self.assertEqual(detect_ramp(np.full((540,960,3),180,np.uint8))['candidates'],[])

    def test_visible_patch_does_not_authorize_motion(self):
        image=np.full((540,960,3),210,np.uint8)
        cv2.fillConvexPoly(image,np.array([[380,130],[580,130],[650,440],[310,440]]),(90,90,90))
        for y in (210,290,370):
            d=(y-130)/310
            cv2.line(image,(round(380-70*d),y),(round(580+70*d),y),(35,35,35),5)
        result=detect_ramp(image)
        self.assertTrue(result['candidates'])
        for item in result['candidates']:
            self.assertFalse(item['entrance']['verified'])
            self.assertFalse(item['motion_authorized'])
            self.assertFalse(item['full_ramp_verified'])
