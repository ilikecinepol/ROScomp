"""Задержки и приоритеты без сети и физического робота."""
import unittest
import queue
from remote_teleop import LatestOutputQueue, Controller
from keyboard_client import Outbox

class QueueTests(unittest.TestCase):
    def test_slow_consumer_receives_latest_status_before_frame(self):
        q=LatestOutputQueue()
        for n in range(1000):
            q.put_nowait(dict(type='frame',seq=n))
            q.put_nowait(dict(type='status',token=n))
        self.assertEqual(q.get(),dict(type='status',token=999))
        self.assertEqual(q.get(),dict(type='frame',seq=999))
        self.assertEqual(q.replaced['status'],999)
        q.put_nowait(None)
        self.assertIsNone(q.get())
    def test_terminal_messages_survive_video_pressure(self):
        q=LatestOutputQueue()
        q.put_nowait(dict(type='summary',complete=True))
        for n in range(100):q.put_nowait(dict(type='frame',seq=n))
        q.put_nowait(None)
        self.assertEqual(q.get()['type'],'summary')
        self.assertEqual(q.get()['seq'],99)
        self.assertIsNone(q.get())
    def test_controls_are_bounded(self):
        q=LatestOutputQueue()
        for n in range(24):q.put_nowait(dict(type='event',seq=n))
        with self.assertRaises(queue.Full):q.put_nowait(dict(type='event'))
    def test_delayed_local_movement_becomes_stop(self):
        q=Outbox()
        q.put(dict(type='input',keys=['q'],deadman=True),now=10)
        self.assertEqual(q.take(now=10.3)['type'],'stop')

class TokenTests(unittest.IsolatedAsyncioTestCase):
    async def test_stale_backlog_stops_once_and_accepts_fresh_arm(self):
        now=[100.]; calls=[]
        async def send(name,value):
            calls.append(name)
            return {'data':{'header':{'status':{'code':0}}}}
        async def normal():pass
        ctl=Controller(send,normal,lambda:[],lambda x:None,lambda:now[0])
        ctl.armed=True; ctl.phase='ready'; ctl.was_moving=True
        token=ctl.issue_token(); now[0]+=.5
        for seq in range(1,231):
            await ctl.handle(dict(type='input',seq=seq,token=token,keys=['q'],deadman=True))
        self.assertEqual(calls,['StopMove'])
        self.assertFalse(ctl.armed)
        self.assertEqual(ctl.counters['stale_commands'],230)
        await ctl.handle(dict(type='arm',seq=231,token=ctl.issue_token()))
        await ctl.arm_task
        self.assertTrue(ctl.armed)
        self.assertEqual(calls,['StopMove','StopMove'])

    async def test_delay_diagnostics_do_not_relax_expiry(self):
        now=[100.]; calls=[]
        async def send(name,value):
            calls.append(name)
            return {'data':{'header':{'status':{'code':0}}}}
        async def normal():pass
        ctl=Controller(send,normal,lambda:[],lambda x:None,lambda:now[0])
        token=ctl.issue_token();now[0]+=.5
        await ctl.handle(dict(type='input',seq=1,token=token,keys=['e'],deadman=True))
        self.assertEqual(ctl.last_token_age,.5)
        self.assertIsNone(ctl.last_accepted_seq)
        self.assertFalse(ctl.armed)
        self.assertNotIn('Move',calls)
        token=ctl.issue_token()
        await ctl.handle(dict(type='profile',seq=2,token=token,profile='ramp25'))
        self.assertEqual(ctl.last_accepted_seq,2)
        self.assertEqual(ctl.profile,'ramp25')

if __name__=='__main__':unittest.main()
