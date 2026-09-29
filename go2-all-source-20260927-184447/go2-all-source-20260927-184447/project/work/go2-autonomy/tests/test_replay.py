"""Переносимые проверки replay и одна проверка существующего архива."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np

from wolf_go2.replay import replay_capture


def pose_record(t=1., yaw=0.):
    import math
    return {'topic': 'ROBOTODOM', 't': t, 'message': {'data': {'header': {'stamp': t},
        'pose': {'position': {'x': 0., 'y': 0., 'z': .3},
                 'orientation': {'x': 0., 'y': 0., 'z': math.sin(yaw/2), 'w': math.cos(yaw/2)}}}}}


def write_records(root, records, name='samples.jsonl'):
    (root/name).write_text(''.join(json.dumps(record)+'\n' for record in records), encoding='utf-8')


class ReplayTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.points = np.column_stack([np.arange(40)*.01, np.zeros(40), np.zeros(40)])

    def test_runtime_format_npz_camera_and_no_trust_promotion(self):
        np.savez_compressed(self.root/'cloud_000000.npz', points=self.points)
        image = np.zeros((180, 240, 3), dtype=np.uint8)
        image[20:110, 80:135] = (0, 140, 255)
        cv2.imwrite(str(self.root/'camera_000000.jpg'), image)
        records = [pose_record(), {'topic': 'ULIDAR_ARRAY', 't': 1.01, 'npz_file': 'cloud_000000.npz',
                  'message': {'data': {'stamp': 20., 'data': {'points': {'shape': [40, 3], 'dtype': 'float64'}}}},
                  'geometry_contract': {'geometry_frames_validated': True}},
                  {'topic': 'CAMERA', 't': 1.02, 'source_s': .01, 'image_file': 'camera_000000.jpg'}]
        write_records(self.root, records)
        before = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in self.root.iterdir()}
        report = replay_capture(self.root)
        after = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in self.root.iterdir()}
        self.assertEqual(before, after)
        self.assertTrue(report['metric_walk_blocked'])
        self.assertFalse(report['geometry_contract_verified'])
        self.assertFalse(report['motion_authorized'])
        self.assertFalse(report['pointclouds'][0]['localized'])
        self.assertEqual(report['pointclouds'][0]['metadata_pairing'], 'explicit_npz_file')
        self.assertEqual(report['pointclouds'][0]['point_count'], 40)
        self.assertTrue(any(c['kind'] == 'cone_image_candidate' for c in report['camera'][0]['pixel_candidates']))
        json.dumps(report, allow_nan=False)

    def test_probe_elapsed_and_summary_camera_records(self):
        record = pose_record(1.)
        record['elapsed_s'] = record.pop('t')
        write_records(self.root, [record])
        image = np.zeros((60, 80, 3), dtype=np.uint8)
        for index in range(2):
            cv2.imwrite(str(self.root/f'camera_{index:02d}.jpg'), image)
        summary = {'camera_files': ['camera_00.jpg', 'camera_01.jpg'], 'camera_records': [
            {'file': 'camera_00.jpg', 'elapsed_s': 1., 'pts': 900, 'time_base': '1/90000'},
            {'file': 'camera_01.jpg', 'elapsed_s': 3., 'pts': 2700, 'time_base': '1/90000'}]}
        (self.root/'summary.json').write_text(json.dumps(summary))
        report = replay_capture(self.root)
        self.assertEqual(report['files']['images_processed'], 2)
        self.assertAlmostEqual(report['timing']['CAMERA']['source_seconds_per_receive_second'], .01)
        self.assertFalse(report['timing']['CAMERA']['timing_calibrated'])
        self.assertIn('consistency_skipped', report['camera'][1])

    def test_legacy_npz_is_read_without_invented_metadata_pair(self):
        np.savez_compressed(self.root/'lidar_00.npz', data_data_data_points=self.points)
        write_records(self.root, [pose_record()])
        report = replay_capture(self.root)
        self.assertEqual(report['pointclouds'][0]['metadata_pairing'], 'unpaired_no_order_guess')
        self.assertIsNone(report['pointclouds'][0]['receive_t'])
        self.assertTrue(report['pointclouds'][0]['metric_walk_blocked'])

    def test_missing_manifest_file_is_reported(self):
        write_records(self.root, [pose_record()])
        (self.root/'summary.json').write_text(json.dumps({'lidar_files': ['lidar_00.npz', 'lidar_01.npz']}))
        np.savez_compressed(self.root/'lidar_00.npz', points=self.points)
        report = replay_capture(self.root)
        self.assertEqual(report['files']['missing'], ['lidar_01.npz'])
        self.assertEqual(report['status'], 'incomplete_capture')

    def test_path_traversal_and_windows_paths_rejected(self):
        for name in ('../outside.npz', '..\\outside.npz', '/outside.npz', 'C:\\outside.npz', 'file.npz:stream'):
            write_records(self.root, [{'topic': 'ULIDAR_ARRAY', 't': 1., 'npz_file': name}])
            with self.assertRaises(ValueError, msg=name):
                replay_capture(self.root)

    def test_summary_path_traversal_rejected(self):
        write_records(self.root, [])
        (self.root/'summary.json').write_text(json.dumps({'camera_records': [{'file': '../outside.jpg'}]}))
        with self.assertRaises(ValueError):
            replay_capture(self.root)

    def test_pickle_and_mesh_are_not_interpreted_as_native_points(self):
        write_records(self.root, [pose_record()])
        np.savez_compressed(self.root/'object.npz', points=np.array([{'point': [1., 2., 3.]}], dtype=object))
        np.savez_compressed(self.root/'mesh.npz', positions=np.arange(90, dtype=np.uint8))
        report = replay_capture(self.root)
        self.assertEqual(len(report['pointclouds']), 2)
        self.assertTrue(all('error' in item for item in report['pointclouds']))
        self.assertTrue(report['metric_walk_blocked'])

    def test_explicit_shape_mismatch_is_an_error(self):
        np.savez_compressed(self.root/'cloud.npz', points=self.points)
        write_records(self.root, [{'topic': 'ULIDAR_ARRAY', 't': 1., 'npz_file': 'cloud.npz',
            'message': {'data': {'stamp': 1., 'data': {'points': {'array_shape': [999, 3]}}}}}])
        report = replay_capture(self.root)
        self.assertIn('error', report['pointclouds'][0])
        self.assertEqual(report['status'], 'incomplete_capture')

    def test_source_duplicates_and_backwards_reported(self):
        records = [{'topic': 'ULIDAR_STATE', 'elapsed_s': t, 'message': {'data': {'stamp': stamp}}}
                   for t, stamp in ((0., 10.), (1., 10.), (2., 9.))]
        write_records(self.root, records)
        timing = replay_capture(self.root)['timing']['ULIDAR_STATE']
        self.assertEqual(timing['source_duplicates'], 1)
        self.assertEqual(timing['source_backwards'], 1)
        self.assertTrue(timing['source_scale_anomaly'])

    def test_sensor_capture_fallback_and_nan_json_output(self):
        write_records(self.root, [pose_record(1.), pose_record(2., yaw=.1)], 'sensor_capture.jsonl')
        report = replay_capture(self.root)
        self.assertAlmostEqual(report['yaw']['ROBOTODOM']['yaw_delta_deg'], 5.729577951, places=6)
        json.dumps(report, allow_nan=False)

    def test_full_saved_capture_five_clouds_fourteen_frames(self):
        root = Path(__file__).resolve().parents[3]/'outputs/go2-remote-setup/logs/20260916T112503Z'
        if not root.is_dir():
            self.skipTest('Локальная запись отсутствует')
        report = replay_capture(root)
        self.assertEqual(report['files']['npz_processed'], 5)
        self.assertEqual(report['files']['images_processed'], 14)
        self.assertEqual(report['saved_record_counts']['ROBOTODOM'], 525)
        self.assertEqual(report['saved_record_counts']['LF_SPORT_MOD_STATE'], 560)
        self.assertEqual(report['saved_record_counts']['LOW_STATE'], 561)
        self.assertAlmostEqual(report['yaw']['ROBOTODOM']['yaw_delta_deg'], -6.1546319395, places=6)
        self.assertAlmostEqual(report['timing']['CAMERA']['source_seconds_per_receive_second'], .009543837177, places=9)
        self.assertTrue(all(item['metric_walk_blocked'] and not item['localized'] for item in report['pointclouds']))
        self.assertEqual(report['errors'], [])
        json.dumps(report, allow_nan=False)


if __name__ == '__main__':
    unittest.main()
