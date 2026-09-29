"""Фальшивый транспорт проверяет остановки, сроки и ответы без сети."""
import asyncio
import math
import unittest

from wolf_go2.control import CommandGate, CommandPump, acknowledged
from wolf_go2.models import Decision, Policy


def ack(code=0):
    return {'data': {'header': {'status': {'code': code}}}}


class GateTests(unittest.TestCase):
    def setUp(self):
        self.gate = CommandGate(Policy())

    def armed_move(self, issued=10.):
        self.gate.arm()
        self.gate.submit(Decision(vx=.3, phase='APPROACH'), issued)

    def test_ack_is_integer_zero_only(self):
        self.assertTrue(acknowledged(ack(0)))
        for response in (None, {}, [], True, ack(False), ack(True), ack(0.), ack('0'), ack(1)):
            self.assertFalse(acknowledged(response), response)

    def test_unarmed_gate_and_latched_fault(self):
        self.gate.submit(Decision(vx=.3), 10.)
        self.assertFalse(self.gate.command(10.).moving)
        self.gate.trip('проверка')
        with self.assertRaises(RuntimeError):
            self.gate.arm()
        self.assertFalse(self.gate.command(10.).moving)

    def test_intent_expiry_stops_and_latches(self):
        self.armed_move()
        self.assertTrue(self.gate.command(10.1).moving)
        self.assertFalse(self.gate.command(10.36).moving)
        self.assertFalse(self.gate.armed)
        self.assertIsNotNone(self.gate.fault)
        self.gate.submit(Decision(vx=.3), 10.4)
        self.assertFalse(self.gate.command(10.4).moving)

    def test_nan_bool_and_reverse_move_block(self):
        for decision in (Decision(vx=float('nan')), Decision(vx=float('inf')), Decision(vx=True), Decision(vx=-.01)):
            gate = CommandGate(Policy())
            gate.arm()
            gate.submit(decision, 10.)
            self.assertFalse(gate.command(10.).moving, decision)
            self.assertIsNotNone(gate.fault)

    def test_backwards_submission_and_nonfinite_tick(self):
        self.armed_move()
        self.gate.submit(Decision(vx=.3), 9.9)
        self.assertIsNotNone(self.gate.fault)
        gate = CommandGate(Policy())
        gate.arm()
        gate.submit(Decision(vx=.3), 10.)
        self.assertFalse(gate.command(float('nan')).moving)
        self.assertIsNotNone(gate.fault)

    def test_backwards_tick_during_fresh_intent_stops(self):
        self.armed_move()
        self.assertTrue(self.gate.command(10.15).moving)
        self.assertFalse(self.gate.command(10.10).moving)
        self.assertIsNotNone(self.gate.fault)

    def test_speed_acceleration_and_combined_speed_bounded(self):
        self.gate.arm()
        self.gate.submit(Decision(vx=100., vy=100., wz=100.), 10.)
        first = self.gate.command(10.)
        self.assertLessEqual(abs(first.vx), .25*.05+1e-9)
        previous = first
        for index in range(1, 40):
            now = 10.+index*.05
            self.gate.submit(Decision(vx=100., vy=100., wz=100.), now)
            command = self.gate.command(now)
            self.assertLessEqual(math.hypot(command.vx, command.vy), .35+1e-9)
            self.assertLessEqual(abs(command.vy), .15+1e-9)
            self.assertLessEqual(abs(command.wz), .5+1e-9)
            self.assertLessEqual(command.wz-previous.wz, .6*.05+1e-9)
            if abs(command.wz) > .2:
                self.assertLessEqual(command.vx, .16+1e-9)
            previous = command

    def test_zero_and_health_stop_do_not_decelerate_slowly(self):
        self.armed_move()
        self.gate.command(10.)
        self.gate.submit(Decision(), 10.05)
        self.assertFalse(self.gate.command(10.05).moving)
        self.gate.submit(Decision(vx=.3), 10.1)
        self.assertFalse(self.gate.command(10.1, ['нет свежей камеры']).moving)
        self.assertIn('нет свежей камеры', self.gate.fault)


class PumpTests(unittest.IsolatedAsyncioTestCase):
    def make_gate(self):
        gate = CommandGate(Policy())
        gate.arm()
        gate.submit(Decision(vx=.25, phase='APPROACH'), 10.)
        return gate

    async def test_stop_retries_then_acknowledges(self):
        calls = []
        responses = iter([ack(False), RuntimeError('offline'), ack(0)])
        async def send(command, parameter):
            calls.append((command, parameter))
            response = next(responses)
            if isinstance(response, Exception):
                raise response
            return response
        pump = CommandPump(send, self.make_gate(), lambda: [], clock=lambda: 10.)
        self.assertTrue(await pump.stop('тест'))
        self.assertEqual(calls, [('StopMove', None)]*3)
        self.assertEqual(pump.stop_attempts, 3)
        self.assertTrue(pump.stopped_ack)
        self.assertFalse(pump.gate.armed)

    async def test_failed_stop_does_not_claim_ack(self):
        calls = []
        async def send(command, parameter):
            calls.append(command)
            return ack(False)
        pump = CommandPump(send, self.make_gate(), lambda: [])
        self.assertFalse(await pump.stop('нет подтверждения'))
        self.assertEqual(calls, ['StopMove']*3)
        self.assertFalse(pump.stopped_ack)
        self.assertIsNotNone(pump.gate.fault)

    async def test_new_stop_cannot_reuse_old_acknowledgement(self):
        count = 0
        async def send(command, parameter):
            nonlocal count
            count += 1
            return ack(0 if count == 1 else 1)
        pump = CommandPump(send, self.make_gate(), lambda: [])
        self.assertTrue(await pump.stop('первая', trip=False))
        self.assertTrue(pump.stopped_ack)
        self.assertFalse(await pump.stop('вторая', trip=False))
        self.assertFalse(pump.stopped_ack)

    async def test_log_failure_cannot_prevent_stop(self):
        calls = []
        def log(*args, **kwargs):
            raise OSError('disk full')
        async def send(command, parameter):
            calls.append(command)
            return ack()
        pump = CommandPump(send, self.make_gate(), lambda: [], log=log)
        self.assertTrue(await pump.stop('журнал недоступен'))
        self.assertEqual(calls, ['StopMove'])
        self.assertTrue(pump.stopped_ack)

    async def test_move_rejected_bool_ack_trips_and_stops(self):
        calls, ending = [], asyncio.Event()
        async def send(command, parameter):
            calls.append(command)
            return ack(False) if command == 'Move' else ack()
        pump = CommandPump(send, self.make_gate(), lambda: [], clock=lambda: 10.)
        await asyncio.wait_for(pump.run(ending), 1.)
        self.assertEqual(calls.count('Move'), 1)
        self.assertIn('StopMove', calls)
        self.assertIn('Move не подтверждён', pump.gate.fault)
        self.assertTrue(pump.stopped_ack)

    async def test_move_timeout_trips_and_stops(self):
        calls, ending = [], asyncio.Event()
        async def send(command, parameter):
            calls.append(command)
            if command == 'Move':
                await asyncio.Event().wait()
            return ack()
        pump = CommandPump(send, self.make_gate(), lambda: [], clock=lambda: 10.)
        await asyncio.wait_for(pump.run(ending), 1.)
        self.assertEqual(calls.count('Move'), 1)
        self.assertIn('StopMove', calls)
        self.assertIn('TimeoutError', pump.gate.fault)

    async def test_cancel_inflight_move_still_awaits_stop(self):
        calls, move_started, ending = [], asyncio.Event(), asyncio.Event()
        async def send(command, parameter):
            calls.append(command)
            if command == 'Move':
                move_started.set()
                await asyncio.Event().wait()
            await asyncio.sleep(.005)
            return ack()
        pump = CommandPump(send, self.make_gate(), lambda: [], clock=lambda: 10.)
        task = asyncio.create_task(pump.run(ending))
        await asyncio.wait_for(move_started.wait(), .5)
        task.cancel()
        with self.assertRaises(asyncio.CancelledError):
            await task
        self.assertEqual(calls, ['Move', 'StopMove'])
        self.assertTrue(pump.stopped_ack)
        self.assertFalse(pump.gate.armed)

    async def test_pre_move_log_failure_sends_no_move(self):
        calls, ending = [], asyncio.Event()
        def log(event, **data):
            raise OSError('disk full')
        async def send(command, parameter):
            calls.append(command)
            return ack()
        pump = CommandPump(send, self.make_gate(), lambda: [], log=log, clock=lambda: 10.)
        await asyncio.wait_for(pump.run(ending), 1.)
        self.assertNotIn('Move', calls)
        self.assertIn('StopMove', calls)
        self.assertIn('журнал', pump.gate.fault)

    async def test_post_move_log_failure_stops(self):
        calls, ending = [], asyncio.Event()
        def log(event, **data):
            if event == 'response' and data.get('command') == 'Move':
                raise OSError('disk full')
        async def send(command, parameter):
            calls.append(command)
            return ack()
        pump = CommandPump(send, self.make_gate(), lambda: [], log=log, clock=lambda: 10.)
        await asyncio.wait_for(pump.run(ending), 1.)
        self.assertEqual(calls.count('Move'), 1)
        self.assertIn('StopMove', calls)
        self.assertIn('журнал', pump.gate.fault)

    async def test_health_exception_stops_before_move(self):
        calls, ending = [], asyncio.Event()
        def health():
            raise RuntimeError('sensor failed')
        async def send(command, parameter):
            calls.append(command)
            return ack()
        pump = CommandPump(send, self.make_gate(), health, clock=lambda: 10.)
        await asyncio.wait_for(pump.run(ending), 1.)
        self.assertNotIn('Move', calls)
        self.assertIn('StopMove', calls)

    async def test_expired_intent_rechecked_after_lock(self):
        calls, ending, current = [], asyncio.Event(), [10.]
        async def send(command, parameter):
            calls.append(command)
            return ack()
        pump = CommandPump(send, self.make_gate(), lambda: [], clock=lambda: current[0])
        await pump.lock.acquire()
        task = asyncio.create_task(pump.run(ending))
        await asyncio.sleep(.01)
        current[0] = 10.5
        pump.lock.release()
        await asyncio.wait_for(task, 1.)
        self.assertNotIn('Move', calls)
        self.assertIn('StopMove', calls)

    async def test_new_stop_generation_cancels_waiting_move(self):
        calls, ending = [], asyncio.Event()
        async def send(command, parameter):
            calls.append(command)
            return ack()
        gate = self.make_gate()
        pump = CommandPump(send, gate, lambda: [], clock=lambda: 10.)
        await pump.lock.acquire()
        task = asyncio.create_task(pump.run(ending))
        await asyncio.sleep(.01)
        gate.trip('внешний стоп')
        pump.lock.release()
        await asyncio.wait_for(task, 1.)
        self.assertNotIn('Move', calls)
        self.assertIn('StopMove', calls)


if __name__ == '__main__':
    unittest.main()
