import unittest
import numpy as np
from wolf_go2.striped_poles import detect_striped_poles

class StripedPoleTests(unittest.TestCase):
    def test_shadowed_stripes_require_relative_contrast(self):
        image=np.full((240,320,3),30,np.uint8)
        image[20:210,50:70]=90
        for y in (20,80,140):image[y:y+30,50:70]=(10,10,70)
        result=detect_striped_poles(image)
        self.assertEqual(len(result),1)
        self.assertFalse(result[0]['distance_verified'])
        image[50:80,50:70]=65
        image[110:140,50:70]=65
        self.assertEqual(detect_striped_poles(image),[])

    def test_shadow_gap_must_contrast_with_both_red_bands(self):
        image=np.full((240,320,3),30,np.uint8)
        image[20:210,50:70]=90
        for y in (20,80,140):image[y:y+30,50:70]=(10,10,70)
        image[80:110,50:70]=(10,10,110)
        self.assertEqual(detect_striped_poles(image),[])

    def test_stripes_found_without_claiming_distance(self):
        image=np.full((240,320,3),30,np.uint8)
        image[20:210,50:70]=230
        for y in (20,80,140):image[y:y+30,50:70]=(0,0,220)
        result=detect_striped_poles(image)
        self.assertEqual(len(result),1)
        self.assertEqual(result[0]['red_sections'],3)
        self.assertFalse(result[0]['distance_verified'])

    def test_red_board_is_not_striped_pole(self):
        image=np.zeros((240,320,3),np.uint8);image[30:210,50:90]=(0,0,220)
        self.assertEqual(detect_striped_poles(image),[])

    def test_dark_gaps_rejected(self):
        image=np.zeros((240,320,3),np.uint8)
        for y in (20,80,140):image[y:y+30,50:70]=(0,0,220)
        self.assertEqual(detect_striped_poles(image),[])

    def test_two_red_regions_do_not_prove_stripes(self):
        image=np.full((240,320,3),230,np.uint8)
        for y in (20,80):image[y:y+30,50:70]=(0,0,220)
        self.assertEqual(detect_striped_poles(image),[])
