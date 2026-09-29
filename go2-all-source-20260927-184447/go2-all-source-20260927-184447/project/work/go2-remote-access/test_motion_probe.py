"""Проверки ограничителя на поддельном транспорте; соединения с роботом нет."""
import asyncio
import time
import unittest
from motion_probe import run_pulse, ack_code

OK = {'data': {'header': {'status': {'code': 0}}}}

class PulseTests(unittest.IsolatedAsyncioTestCase):
    async def scenario(self, mode):
        calls, events = [], []
        abort = asyncio.Event()
        began = time.monotonic()
        if mode == 'cancelled':
            abort.set()
        def health():
            return ['bad'] if mode == 'unhealthy' or (mode == 'deterioration' and time.monotonic()-began > 0.15) else []
        async def send(cmd, param):
            calls.append((cmd, param, time.monotonic()-began))
            if mode == 'move_timeout' and cmd == 'Move':
                await asyncio.sleep(2)
            if mode == 'stop_failure' and cmd == 'StopMove':
                return {}
            await asyncio.sleep(.01)
            return OK
        result = await run_pulse(send, health, lambda kind, **kw: events.append((kind, kw)), abort)
        stops = [x for x in calls if x[0] == 'StopMove']
        moves = [x for x in calls if x[0] == 'Move']
        self.assertTrue(stops)
        self.assertFalse(any(x[2] > stops[0][2] for x in moves))
        self.assertTrue(all(x[1] == {'x': .1, 'y': 0., 'z': 0.} for x in moves))
        self.assertLessEqual(stops[0][2], 1.08)
        return result, moves, stops

    async def test_normal_deadline(self):
        result, moves, stops = await self.scenario('normal')
        self.assertGreater(len(moves), 0)
        self.assertTrue(result['stop_acknowledged'])
        self.assertGreaterEqual(stops[0][2], .99)

    async def test_initial_fault(self):
        _, moves, _ = await self.scenario('unhealthy')
        self.assertEqual(moves, [])

    async def test_cancelled(self):
        _, moves, _ = await self.scenario('cancelled')
        self.assertEqual(moves, [])

    async def test_deterioration(self):
        _, _, stops = await self.scenario('deterioration')
        self.assertLess(stops[0][2], .23)

    async def test_move_timeout(self):
        result, moves, stops = await self.scenario('move_timeout')
        self.assertEqual(len(moves), 1)
        self.assertLess(stops[0][2], .4)
        self.assertTrue(result['stop_acknowledged'])

    async def test_stop_failure(self):
        result, _, stops = await self.scenario('stop_failure')
        self.assertEqual(len(stops), 3)
        self.assertFalse(result['stop_acknowledged'])

    async def test_stop_despite_journal_fault(self):
        calls = []
        async def send(cmd, param):
            calls.append(cmd)
            return OK
        def emit(*a, **k):
            raise OSError('нет места')
        result = await run_pulse(send, lambda: ['bad'], emit, asyncio.Event())
        self.assertEqual(calls, ['StopMove'])
        self.assertTrue(result['stop_acknowledged'])

if __name__ == '__main__':
    unittest.main(verbosity=2)
