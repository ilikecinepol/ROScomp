"""Проверки ручного управления v2 без подключения к роботу."""
import asyncio
import itertools
import json
import math
from pathlib import Path
import tempfile
import time
import unittest
from PIL import Image
from keyboard_client import InputState
from remote_teleop import Controller, Recorder, PROFILES, velocities

ACK = {'data': {'header': {'status': {'code': 0}}}}


class InputTests(unittest.TestCase):
    def ready(self):
        s = InputState()
        s.focused = True
        s.frame_time = 10.
        s.update_status(dict(phase='ready', armed=True, token=1,
                             telemetry_age_s=0., camera_age_s=0.), 10.)
        return s

    def test_direction_without_space_and_release(self):
        s = self.ready()
        s.request_arm(10.)
        s.press('w')
        self.assertTrue(s.heartbeat(10.)['deadman'])
        s.release('w')
        self.assertFalse(s.heartbeat(10.)['deadman'])

    def test_fault_and_repeat_cannot_restart_held_key(self):
        s = self.ready()
        s.request_arm(10.)
        s.press('w')
        s.stop('связь потеряна')
        s.press('w')
        self.assertIsNone(s.request_arm(10.))
        self.assertFalse(s.keys)
        s.release('w')
        self.assertIsNotNone(s.request_arm(10.))
        self.assertFalse(s.heartbeat(10.)['deadman'])
        s.press('w')
        self.assertTrue(s.heartbeat(10.)['deadman'])

    def test_held_before_preparation_requires_release(self):
        s = self.ready()
        s.press('w')
        self.assertIsNone(s.request_arm(10.))
        s.release('w')
        self.assertIsNotNone(s.request_arm(10.))

    def test_all_profile_combinations(self):
        for profile, limits in PROFILES.items():
            for n in range(7):
                for keys in itertools.combinations('wsadqe', n):
                    v = velocities(keys, profile)
                    self.assertLessEqual(math.hypot(v['x'], v['y']), limits[0] + 1e-9)
                    self.assertLessEqual(abs(v['y']), limits[1] + 1e-9)
                    self.assertLessEqual(abs(v['z']), limits[2] + 1e-9)


class ProtocolTests(unittest.IsolatedAsyncioTestCase):
    async def test_profile_switch_stops_and_invalid_profile_rejected(self):
        calls = []
        async def send(c, p):
            calls.append(c)
            return ACK
        async def mode():
            pass
        c = Controller(send, mode, lambda: [], lambda x: None)
        c.armed = True
        await c.handle(dict(type='profile', seq=1, token=c.issue_token(), profile='fast'))
        self.assertTrue(c.armed)
        self.assertEqual(c.profile, 'fast')
        self.assertEqual(calls, [])
        await c.handle(dict(type='profile', seq=2, token=c.issue_token(), profile='unlimited'))
        self.assertEqual(c.phase, 'error')
        self.assertEqual(c.profile, 'fast')


class RecordingTests(unittest.TestCase):
    def test_explicit_boundaries_preview_and_two_runs(self):
        class Preview:
            def __init__(self): self.messages = []
            def put(self, m): self.messages.append(m)
        class Frame:
            def to_image(self): return Image.new('RGB', (640, 360))
        with tempfile.TemporaryDirectory() as folder:
            out = Path(folder)
            p = Preview()
            r = Recorder(out, p)
            r.put('telemetry', {'sample': 'before'})
            r.put('camera', {'seq': 0, 'elapsed_s': 0, 'file': 'camera/before.jpg',
                            'receive_monotonic_s': time.monotonic(), 'preview': True}, Frame())
            r.recording('record_start', 'precision')
            r.put('telemetry', {'sample': 'one'})
            r.put('maps', {'file': 'maps/one.bin'}, b'one')
            r.recording('record_stop', 'precision')
            r.put('telemetry', {'sample': 'between'})
            r.recording('record_start', 'floor')
            r.put('telemetry', {'sample': 'two'})
            r.recording('record_stop', 'floor')
            r.queue.put(None)
            r.thread.join(5)
            self.assertFalse(r.thread.is_alive())
            self.assertFalse(r.failed.is_set(), r.error)
            self.assertFalse((out / 'camera/before.jpg').exists())
            self.assertEqual(len(p.messages), 1)
            self.assertEqual(json.loads((out / 'run_001/telemetry.jsonl').read_text())['sample'], 'one')
            self.assertEqual(json.loads((out / 'run_002/telemetry.jsonl').read_text())['sample'], 'two')
            self.assertEqual((out / 'run_001/maps/one.bin').read_bytes(), b'one')
            self.assertTrue(json.loads((out / 'run_002/recording.json').read_text())['complete'])
            self.assertEqual(r.saved_recording, 'run_002')


