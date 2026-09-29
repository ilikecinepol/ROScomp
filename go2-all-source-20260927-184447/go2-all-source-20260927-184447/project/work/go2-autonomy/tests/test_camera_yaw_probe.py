import asyncio
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch

spec=importlib.util.spec_from_file_location('camera_yaw_probe',Path(__file__).parents[1]/'pulse-test/motion_yaw_015.py')
probe=importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)


class CameraYawProbeTests(unittest.IsolatedAsyncioTestCase):
    async def test_stale_sensors_never_send_move(self):
        calls=[]
        async def send(command,parameter):
            calls.append((command,parameter))
            return {'data':{'header':{'status':{'code':0}}}}
        result=await probe.run_pulse(send,lambda:['stale'],lambda *a,**k:None,asyncio.Event())
        self.assertEqual(result['move_attempts'],0)
        self.assertTrue(result['stop_acknowledged'])
        self.assertTrue(all(c[0]=='StopMove' for c in calls))

    async def test_stop_does_not_wait_for_stalled_move(self):
        calls=[]
        async def send(command,parameter):
            calls.append((command,parameter))
            if command=='Move':await asyncio.sleep(2)
            return {'data':{'header':{'status':{'code':0}}}}
        with patch.object(probe,'DURATION',.04):
            result=await asyncio.wait_for(probe.run_pulse(send,lambda:[],lambda *a,**k:None,asyncio.Event()),.7)
        self.assertTrue(result['stop_acknowledged'])
        self.assertEqual(calls[0],('Move',{'x':0.,'y':0.,'z':.15}))
        self.assertEqual(calls[1][0],'StopMove')
