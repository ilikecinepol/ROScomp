"""Бортовой адаптер на поддельных кадрах/журнале. Соединений с Go2 нет.

Полный run_session не выполняется: отдельно проверяется ранний отказ до
импорта SDK. Сеть и аппаратный API не требуются.
"""
import asyncio
from fractions import Fraction
from pathlib import Path
from types import SimpleNamespace
import tempfile
import threading
import unittest
from unittest import mock

import numpy as np

from wolf_go2.models import Decision, Observation, Pose
from wolf_go2.runtime import Runtime, native_points
from test_profile import synthetic_profile


class Clock:
    def __init__(self, value=10.):
        self.value = value

    def __call__(self):
        return self.value


class FakeJournal:
    def __init__(self):
        self.failed = threading.Event()
        self.records = []

    def put(self, name, data):
        self.records.append((name, data))

    def event(self, name, **data):
        self.records.append(('event:' + name, data))


def healthy_runtime(live=False):
    profile = synthetic_profile()
    profile.geometry.update(source_kind='occupied_voxel_points', voxel_size_m=.05,
                            voxel_reference='lower_corner', localization_error_m=.01)
    for contract in profile.time_contracts.values():
        contract['offset'] = -90.
    clock = Clock()
    runtime = Runtime(profile, profile.robot_id, FakeJournal(),
                      'synthetic-boot-for-unit-test', live=live, clock=clock)
    runtime.sensor('LF_SPORT_MOD_STATE', {'data': {'stamp': 100.,
        'imu_state': {'rpy': [.01, .02, 0.]}, 'body_height': .32, 'velocity': [0., 0., 0.]}})
    runtime.sensor('LOW_STATE', {'data': {'motor_state': [{'temperature': 30.} for _ in range(12)],
        'bms_state': {'soc': 80.}}})
    runtime.sensor('ROBOTODOM', {'data': {'header': {'stamp': 100., 'frame_id': 'synthetic-world'},
        'pose': {'position': {'x': 0., 'y': 0., 'z': .32},
                 'orientation': {'x': 0., 'y': 0., 'z': 0., 'w': 1.}}}})
    runtime.sensor('ULIDAR_STATE', {'data': {'stamp': 100., 'serial_number': 'SYNTHETIC-TEST-ONLY'}})
    runtime.sensor('ULIDAR_ARRAY', {'data': {'stamp': 100., 'frame_id': 'synthetic-world',
        'resolution': .05, 'data': {'points': np.zeros((30, 3), dtype=np.float32)}}})
    runtime.monitor.ingest('CAMERA', {}, 10., 100.)
    runtime.image = np.zeros((12, 16, 3), dtype=np.uint8)
    runtime.image_received = 10.
    return runtime, clock


class FakePipeline:
    def __init__(self, clock, cost=0.):
        self.clock, self.cost = clock, cost
        self.contract = None
        self.candidates = []

    def build_observation(self, t, pose, points, epoch, contract, image, policy):
        self.contract = dict(contract)
        self.clock.value += self.cost
        return Observation(t, pose, None, localized=True, frame_epoch=epoch,
                           localization_error_m=.01)


class NativePointsTests(unittest.TestCase):
    def test_native_cloud_is_copied_and_invalid_formats_rejected(self):
        original = np.arange(12., dtype=np.float32).reshape(4, 3)
        copy = native_points({'data': {'data': {'points': original}}})
        original[0, 0] = 999.
        self.assertEqual(copy[0, 0], 0.)
        for value in ([], b'encoded-mesh', np.zeros((3, 2)), np.zeros((0, 3)),
                      np.full((3, 3), np.nan), np.full((3, 3), np.inf)):
            with self.assertRaises((ValueError, TypeError)):
                native_points({'data': {'data': {'points': value}}})


class RuntimeTests(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='wolf-go2-runtime-test-')
        self.addCleanup(self.temp.cleanup)
        self.start_file = Path(self.temp.name) / 'start.txt'

    def test_health_binds_identity_boot_frames_and_voxel_size(self):
        runtime, _ = healthy_runtime()
        self.assertEqual(runtime.health(), [])
        mutations = (
            lambda r: r.monitor.identity.update(serial_number='other-robot'),
            lambda r: setattr(r, 'boot_id', 'new-boot'),
            lambda r: r.monitor.streams['ROBOTODOM'].payload['header'].update(frame_id='other-frame'),
            lambda r: r.monitor.streams['ULIDAR_ARRAY'].payload.update(frame_id='other-frame'),
            lambda r: r.monitor.streams['ULIDAR_ARRAY'].payload.update(resolution=.10),
            lambda r: setattr(r, 'profile', None),
            lambda r: r.journal.failed.set(),
            lambda r: r.consistency_reasons.append('synthetic scene/pose mismatch'),
        )
        for mutate in mutations:
            runtime, _ = healthy_runtime()
            mutate(runtime)
            self.assertTrue(runtime.health())

    def test_invalid_cloud_latches_sensor_fault(self):
        runtime, _ = healthy_runtime()
        runtime.sensor('ULIDAR_ARRAY', {'data': {'stamp': 100.1, 'data': {'positions': [1, 2, 3]}}})
        self.assertTrue(any('ULIDAR_ARRAY' in value for value in runtime.monitor.faults))
        self.assertTrue(runtime.health())

    def test_start_token_must_match_current_runtime(self):
        runtime, _ = healthy_runtime(live=True)
        self.assertFalse(runtime.start_requested(self.start_file))
        self.start_file.write_text('wrong-session', encoding='ascii')
        self.assertFalse(runtime.start_requested(self.start_file))
        self.start_file.write_text(runtime.nonce, encoding='ascii')
        self.assertTrue(runtime.start_requested(self.start_file))
        another, _ = healthy_runtime(live=True)
        self.assertFalse(another.start_requested(self.start_file))
        self.start_file.write_text('не ascii', encoding='utf-8')
        self.assertFalse(runtime.start_requested(self.start_file))

    async def test_live_preflight_rejects_self_created_token_path_before_sdk_import(self):
        from wolf_go2.runtime import run_session
        profile = synthetic_profile()
        output = Path(self.temp.name) / 'new-session'
        args = SimpleNamespace(mode='live', execute=True, profile='synthetic-profile',
            robot_id=profile.robot_id, start_file=str(output/'start-token.txt'),
            output=str(output), sdk_dir='must-not-be-accessed', duration=1.)
        # Исполняется только отказная проверка до импорта SDK, без сети/сигналов.
        with mock.patch('wolf_go2.runtime.sys.platform', 'linux'), \
             mock.patch('wolf_go2.runtime.RobotProfile.load', return_value=profile), \
             mock.patch('wolf_go2.runtime.importlib.import_module', side_effect=AssertionError('SDK forbidden')) as sdk_import:
            with self.assertRaisesRegex(ValueError, 'вне каталога'):
                await run_session(args)
            sdk_import.assert_not_called()
        self.assertFalse(output.exists())

    async def test_observe_preserves_body_state_and_denies_wrong_identity_contract(self):
        runtime, clock = healthy_runtime()
        runtime.pipeline = FakePipeline(clock)
        observed = await runtime.observe()
        self.assertAlmostEqual(observed.roll, .01)
        self.assertAlmostEqual(observed.pitch, .02)
        self.assertAlmostEqual(observed.body_height, .32)
        self.assertEqual(observed.speed, 0.)
        self.assertIs(runtime.last_observation, observed)
        runtime.monitor.identity['serial_number'] = 'wrong'
        await runtime.observe()
        self.assertFalse(runtime.pipeline.contract['identity_verified'])

    async def test_slow_worker_cannot_refresh_old_observation(self):
        runtime, clock = healthy_runtime()
        runtime.pipeline = FakePipeline(clock, cost=.7)
        observed = await runtime.observe()
        self.assertFalse(observed.localized)
        self.assertTrue(observed.diagnostics)

    async def test_acquisition_age_and_worker_delay_are_added(self):
        for stream in ('ULIDAR_ARRAY', 'ROBOTODOM'):
            runtime, clock = healthy_runtime()
            runtime.profile.time_contracts[stream]['offset'] = -90.4
            runtime.pipeline = FakePipeline(clock, cost=.4)
            observed = await runtime.observe()
            self.assertFalse(observed.localized, stream + ': возраст 0.4с + обработка 0.4с превышают предел 0.6с')

    async def test_frame_epoch_change_during_worker_invalidates_result(self):
        runtime, clock = healthy_runtime()
        runtime.pipeline = FakePipeline(clock)
        build = runtime.pipeline.build_observation

        def changed_epoch(*args):
            result = build(*args)
            runtime.monitor.pose_epoch += 1
            return result

        runtime.pipeline.build_observation = changed_epoch
        observed = await runtime.observe()
        self.assertFalse(observed.localized)

    async def test_consistency_worker_exception_becomes_health_fault(self):
        runtime, _ = healthy_runtime()

        class BrokenConsistency:
            def observe(self, *args):
                raise ValueError('synthetic worker failure')

        runtime.consistency = BrokenConsistency()
        await runtime.diagnostics()
        self.assertTrue(any('диагностики' in reason for reason in runtime.health()))

    async def test_delayed_video_decode_keeps_capture_receipt_time(self):
        runtime, clock = healthy_runtime()
        runtime.monitor.streams.pop('CAMERA')
        runtime.image_received = -float('inf')

        class Frame:
            pts = 1000
            time_base = Fraction(1, 10)

            def to_ndarray(self, format):
                if format != 'bgr24':
                    raise AssertionError(format)
                clock.value += .7
                return np.zeros((12, 16, 3), dtype=np.uint8)

        class Track:
            delivered = False

            async def recv(self):
                if not self.delivered:
                    self.delivered = True
                    return Frame()
                runtime.ending.set()
                raise EOFError('end of synthetic stream')

        await runtime.video(Track())
        self.assertEqual(runtime.image_received, 10.)
        self.assertTrue(any('CAMERA' in reason for reason in runtime.health()))

    def fake_plan_io(self, runtime, clock):
        calls = {'begin': 0, 'step': 0}

        async def observe():
            clock.value += .1
            return Observation(clock.value, Pose(0., 0., 0.), None, localized=True)

        async def begin():
            calls['begin'] += 1

        class Mission:
            phase = 'WAIT_START'

            def step(self, obs, start=False):
                calls['step'] += 1
                self.phase = 'FINISHED'
                return Decision(phase='FINISHED', reason='synthetic mission')

        runtime.observe = observe
        runtime.mission = Mission()
        return calls, begin

    async def test_observe_mode_never_prepares_motion_even_with_matching_token(self):
        runtime, clock = healthy_runtime(live=False)
        self.start_file.write_text(runtime.nonce, encoding='ascii')
        calls, begin = self.fake_plan_io(runtime, clock)
        await runtime.plan(.15, self.start_file, begin)
        self.assertEqual(calls, {'begin': 0, 'step': 0})
        self.assertFalse(runtime.started)
        self.assertFalse(runtime.gate.armed)

    async def test_live_wrong_token_never_arms(self):
        runtime, clock = healthy_runtime(live=True)
        self.start_file.write_text('wrong', encoding='ascii')
        calls, begin = self.fake_plan_io(runtime, clock)
        await runtime.plan(.15, self.start_file, begin)
        self.assertEqual(calls, {'begin': 0, 'step': 0})
        self.assertFalse(runtime.gate.armed)

    async def test_live_requires_health_before_and_after_prepare(self):
        for when in ('before', 'during'):
            runtime, clock = healthy_runtime(live=True)
            self.start_file.write_text(runtime.nonce, encoding='ascii')
            calls, normal_begin = self.fake_plan_io(runtime, clock)
            if when == 'before':
                runtime.journal.failed.set()

            async def begin():
                await normal_begin()
                runtime.journal.failed.set()

            await runtime.plan(1., self.start_file, begin)
            self.assertEqual(calls['step'], 0)
            self.assertFalse(runtime.gate.armed)
            self.assertTrue(runtime.gate.fault)
            self.assertEqual(calls['begin'], int(when == 'during'))

    async def test_live_valid_gated_start_runs_mission_once(self):
        runtime, clock = healthy_runtime(live=True)
        self.start_file.write_text(runtime.nonce, encoding='ascii')
        calls, begin = self.fake_plan_io(runtime, clock)
        await runtime.plan(1., self.start_file, begin)
        self.assertEqual(calls, {'begin': 1, 'step': 1})
        self.assertTrue(runtime.started)
        self.assertTrue(runtime.ending.is_set())

    async def test_health_failure_after_start_cannot_advance_mission(self):
        runtime, clock = healthy_runtime(live=True)
        calls, begin = self.fake_plan_io(runtime, clock)
        runtime.started = True
        runtime.gate.arm()
        runtime.journal.failed.set()
        await runtime.plan(1., self.start_file, begin)
        self.assertEqual(calls['step'], 0)
        self.assertTrue(runtime.gate.fault)
        self.assertFalse(runtime.gate.armed)


if __name__ == '__main__':
    unittest.main()
