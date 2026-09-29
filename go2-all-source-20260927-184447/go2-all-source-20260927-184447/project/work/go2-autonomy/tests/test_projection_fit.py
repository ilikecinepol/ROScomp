import copy
import unittest

from fit_projection_offset import evaluate_affine, score, session_validation


def frame(session, index, focal=.95, offset=10., width=1280):
    scale=width/1280
    predicted=[300.,640.,980.]
    observed=[(x-640)*focal+640+offset for x in predicted]
    return {'frame':f'{session}/cloud_{index:06d}.json', 'camera_candidates':3,
            'projection_features':{'image_size':[width,720],
                'centers_px':[[x*scale,200.*scale] for x in observed],
                'boxes_xyxy':[[(x-10)*scale,180*scale,(x+10)*scale,220*scale] for x in observed],
                'curves':[{'pixels':[[x*scale,200.*scale]]} for x in predicted]},
            'matches':[{'accepted':True,'image_index':i,
                        'observed_minus_projected_xy':[(observed[i]-x)*scale,0.]}
                       for i,x in enumerate(predicted)]}


class ProjectionFitTests(unittest.TestCase):
    def test_legacy_cache_requires_regeneration(self):
        f=frame('train',0)
        del f['projection_features']['boxes_xyxy']
        with self.assertRaises(ValueError):score(f,0.)

    def test_recovers_known_horizontal_transform(self):
        rows=[frame('train',i) for i in range(5)]
        result=evaluate_affine(rows,'train')
        self.assertAlmostEqual(result['candidate_focal_scale_x'],.95,places=6)
        self.assertAlmostEqual(result['candidate_offset_px_at_1280'],10.,places=5)
        self.assertFalse(result['calibration_verified'])
        self.assertFalse(result['candidate_accepted'])

    def test_held_out_data_cannot_change_fit(self):
        training=[frame('train',i) for i in range(5)]
        a=evaluate_affine(training+[frame('other',0,.9,-15)],'train')
        b=evaluate_affine(training+[frame('other',0,1.1,20)],'train')
        self.assertEqual(a['candidate_focal_scale_x'],b['candidate_focal_scale_x'])
        self.assertEqual(a['candidate_offset_px_at_1280'],b['candidate_offset_px_at_1280'])

    def test_regression_is_reported_despite_training_gain(self):
        rows=[frame('train',i,.9,40.) for i in range(5)]
        rows.append(frame('other',0,1.,0.))
        result=session_validation(rows,'train',40.,.9)
        self.assertEqual(result['held_out_regressions'],['other'])
        self.assertEqual(result['decision'],'rejected_session_regression')
        self.assertFalse(result['candidate_accepted'])

    def test_scoring_scales_with_image_and_does_not_mutate(self):
        f=frame('train',0,width=640)
        before=copy.deepcopy(f)
        self.assertEqual(score(f,10.,.95),3)
        self.assertEqual(f,before)

    def test_insufficient_training_is_rejected(self):
        with self.assertRaises(ValueError):
            evaluate_affine([frame('train',0)],'train')


if __name__=='__main__':
    unittest.main()
