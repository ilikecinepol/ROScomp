"""Облако сохраняется со своим контекстом, без догадок по номеру файла."""
import unittest
import numpy as np
from wolf_go2.models import Pose
from test_runtime import healthy_runtime


class SnapshotRecordingTests(unittest.TestCase):
    def test_invalid_current_stamp_does_not_inherit_previous_stamp(self):
        runtime,clock=healthy_runtime();clock.value=12.
        runtime.sensor('ULIDAR_ARRAY',{'data':{'stamp':None,'frame_id':'synthetic-world','resolution':.05,
            'data':{'points':np.zeros((30,3))}}})
        meta=dict(runtime.journal.records)['cloud_000001.json']
        self.assertIsNone(meta['cloud_source_s'])

    def test_pose_and_camera_are_bound_and_copied_at_receive(self):
        runtime,clock=healthy_runtime()
        runtime.journal.records=[]
        clock.value=11.1
        runtime.image_received=11.
        runtime.image_source=101.
        runtime.monitor.last_pose=(11.,Pose(1,2,.3,.31))
        runtime.sensor('ULIDAR_ARRAY',{'data':{'stamp':101.1,'frame_id':'synthetic-world','resolution':.05,
            'data':{'points':np.zeros((30,3))}}})
        records=dict(runtime.journal.records)
        meta=records['cloud_000001.json']
        runtime.image[:]=255
        runtime.monitor.last_pose=(12.,Pose(5,6,0))
        self.assertEqual(meta['pose']['x'],1)
        self.assertAlmostEqual(meta['pose_receive_delta_s'],.1)
        self.assertEqual(meta['camera_file'],'cloud_000001-camera.jpg')
        self.assertFalse(records[meta['camera_file']].any())
        self.assertFalse(meta['acquisition_alignment_verified'])

    def test_old_camera_not_paired(self):
        runtime,clock=healthy_runtime();clock.value=12.
        runtime.sensor('ULIDAR_ARRAY',{'data':{'stamp':102.,'frame_id':'synthetic-world','resolution':.05,
            'data':{'points':np.zeros((30,3))}}})
        meta=dict(runtime.journal.records)['cloud_000001.json']
        self.assertIsNone(meta['camera_file'])
