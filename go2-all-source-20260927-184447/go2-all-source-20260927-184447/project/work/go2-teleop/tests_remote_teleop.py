"""Проверки допуска движения на поддельном API; сети и робота здесь нет."""
import asyncio
import importlib.util
import itertools
import math
from pathlib import Path
import tempfile
import time
import unittest

spec = importlib.util.spec_from_file_location('teleop', Path(__file__).with_name('remote_teleop.py'))
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
ACK = {'data': {'header': {'status': {'code': 0}}}}


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


class MotionModeTests(unittest.TestCase):
    def response(self, mode, code=0):
        import json
        return {'data': {'header': {'status': {'code': code}}, 'data': json.dumps({'name': mode})}}

    def test_only_normal_allowed(self):
        for mode in ('normal', 'mcf'):
            self.assertEqual(m.confirmed_motion_mode(self.response(mode)), mode)

    def test_unknown_modes_rejected(self):
        for mode in ('ai', '', None, 'MCF'):
            with self.assertRaises(RuntimeError):
                m.confirmed_motion_mode(self.response(mode))

    def test_unconfirmed_and_broken_response_rejected(self):
        for response in ({}, ACK, self.response('mcf', 1)):
            with self.assertRaises(RuntimeError):
                m.confirmed_motion_mode(response)


class SafetyTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.clock = Clock()
        self.calls, self.events = [], []
        self.errors = []
        self.normal_calls = 0
        self.hang_move = False

        async def send(command, value):
            self.calls.append((command, value))
            if command == 'Move' and self.hang_move:
                await asyncio.sleep(20)
            return ACK

        async def normal():
            self.normal_calls += 1

        self.ctl = m.Controller(send, normal, lambda: self.errors, self.events.append, self.clock)
        self.seq = 0

    async def asyncTearDown(self):
        tasks = [t for t in [self.ctl.arm_task, self.ctl.move_task] if t and not t.done()]
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)

    async def command(self, kind, **values):
        self.seq += 1
        await self.ctl.handle({'type': kind, 'seq': self.seq, 'token': self.ctl.issue_token(), **values})

    async def arm(self):
        await self.command('arm')
        await self.ctl.arm_task
        self.assertTrue(self.ctl.armed)

    async def move(self):
        await self.command('input', deadman=True, keys=['w'])
        await self.ctl.tick()
        if self.ctl.move_task:
            await self.ctl.move_task

    async def test_no_normal_mode_or_move_before_arm(self):
        await self.command('input', deadman=True, keys=['w'])
        await self.ctl.tick()
        self.assertFalse(self.calls)
        self.assertEqual(self.normal_calls, 0)

    async def test_heartbeat_timeout_requires_explicit_rearm(self):
        await self.arm()
        await self.move()
        self.clock.now += 0.351
        await self.ctl.tick()
        self.assertFalse(self.ctl.armed)
        self.assertEqual(self.calls[-1][0], 'StopMove')
        before = sum(name == 'Move' for name, _ in self.calls)
        await self.move()
        self.assertEqual(before, sum(name == 'Move' for name, _ in self.calls))
        await self.arm()
        await self.move()
        self.assertGreater(sum(name == 'Move' for name, _ in self.calls), before)

    async def test_stale_token_cannot_rearm(self):
        token = self.ctl.issue_token()
        self.clock.now += 0.351
        await self.ctl.handle({'type': 'arm', 'seq': 1, 'token': token})
        self.assertFalse(self.ctl.armed)
        self.assertEqual(self.normal_calls, 0)
        self.assertEqual(self.calls[-1][0], 'StopMove')

    async def test_stale_input_stops_after_motion(self):
        await self.arm()
        await self.move()
        token = self.ctl.issue_token()
        self.clock.now += 0.351
        await self.ctl.handle({'type': 'input', 'seq': 999, 'token': token, 'deadman': True, 'keys': ['w']})
        self.assertFalse(self.ctl.armed)
        self.assertEqual(self.calls[-1][0], 'StopMove')

    async def test_eof_stops_and_blocks_all_new_arms(self):
        await self.arm()
        await self.move()
        await self.ctl.eof()
        await self.command('arm')
        self.assertFalse(self.ctl.armed)
        self.assertTrue(self.ctl.closed)
        self.assertEqual(self.normal_calls, 1)

    async def test_deadman_release_stops_but_keep_arm(self):
        await self.arm()
        await self.move()
        await self.command('input', deadman=False, keys=[])
        self.assertTrue(self.ctl.armed)
        self.assertEqual(self.calls[-1][0], 'StopMove')
        self.clock.now += 0.1
        await self.move()
        self.assertEqual(self.calls[-1][0], 'Move')

    async def test_sensor_failure_disarms_no_auto_resume(self):
        await self.arm()
        await self.move()
        self.errors.append('camera stale')
        await self.ctl.tick()
        self.assertFalse(self.ctl.armed)
        self.errors.clear()
        before = len(self.calls)
        await self.move()
        self.assertEqual(len(self.calls), before)

    async def test_send_timeout_disarms(self):
        await self.arm()
        self.hang_move = True
        started = time.monotonic()
        await self.move()
        self.assertLess(time.monotonic() - started, 0.6)
        self.assertFalse(self.ctl.armed)
        self.assertEqual(self.calls[-1][0], 'StopMove')

    async def test_watchdog_interrupts_pending_move_wait(self):
        await self.arm()
        self.hang_move = True
        await self.command('input', deadman=True, keys=['w'])
        await self.ctl.tick()
        await asyncio.sleep(0.01)
        self.clock.now += 0.4
        await self.ctl.tick()
        self.assertFalse(self.ctl.armed)
        self.assertEqual(self.calls[-1][0], 'StopMove')
        await asyncio.gather(self.ctl.move_task, return_exceptions=True)

    async def test_old_sequence_disarms(self):
        await self.arm()
        await self.move()
        await self.ctl.handle({'type': 'input', 'seq': 1, 'token': self.ctl.issue_token(), 'deadman': True, 'keys': ['w']})
        self.assertFalse(self.ctl.armed)

    async def test_stop_honoured_despite_old_sequence(self):
        await self.arm()
        await self.command('stop', seq=-10)
        self.assertFalse(self.ctl.armed)
        self.assertEqual(self.calls[-1][0], 'StopMove')

    async def test_arm_cancelled_if_heartbeat_missing_during_mode_change(self):
        blocker = asyncio.Event()
        async def normal():
            await blocker.wait()
        self.ctl.normal_mode = normal
        await self.command('arm')
        await asyncio.sleep(0)
        self.clock.now += 0.4
        await self.ctl.tick()
        blocker.set()
        await asyncio.gather(self.ctl.arm_task, return_exceptions=True)
        self.assertFalse(self.ctl.armed)
        self.assertEqual(self.ctl.phase, 'stopped')

    async def test_status_tokens_bounded(self):
        for _ in range(100):
            self.ctl.issue_token()
        self.assertEqual(len(self.ctl.tokens), 20)


class PureTests(unittest.TestCase):
    def test_all_key_combinations_stay_within_limits(self):
        for size in range(7):
            for keys in itertools.combinations('wsadqe', size):
                v = m.velocities(keys)
                self.assertLessEqual(abs(v['x']), .6 + 1e-12)
                self.assertLessEqual(abs(v['y']), .5 + 1e-12)
                self.assertLessEqual(abs(v['z']), 1.0 + 1e-12)
                self.assertLessEqual(math.hypot(v['x'], v['y']), .6 + 1e-12)

    def test_single_direction_reaches_api_limit_and_opposites_cancel(self):
        self.assertEqual(m.velocities(['w'])['x'], .6)
        self.assertEqual(m.velocities(['s'])['x'], -.6)
        self.assertEqual(m.velocities(['q'])['y'], .5)
        self.assertEqual(m.velocities(['a'])['z'], 1.0)
        self.assertEqual(m.velocities(list('wsadqe')), {'x': 0., 'y': 0., 'z': 0.})

    def test_recorder_survives_idle_and_closes(self):
        class Preview:
            def put(self, item):
                pass
        with tempfile.TemporaryDirectory() as directory:
            rec = m.Recorder(Path(directory), Preview())
            time.sleep(1.1)
            self.assertFalse(rec.failed.is_set(), rec.error)
            rec.event({'event': 'test'})
            rec.queue.put(None)
            rec.thread.join(2)
            self.assertFalse(rec.thread.is_alive())
            self.assertIn('test', (Path(directory) / 'events.jsonl').read_text(encoding='utf-8'))


if __name__ == '__main__':
    unittest.main(verbosity=2)
