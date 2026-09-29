import unittest,json
from types import SimpleNamespace
from unittest.mock import AsyncMock,Mock
from remote_teleop import prepare_normal_mode,velocities
class ModeTests(unittest.IsolatedAsyncioTestCase):
    async def test_readback_after_preparation(self):
        publish=AsyncMock(return_value={'data':{'header':{'status':{'code':0}},'data':json.dumps({'name':'normal'})}})
        go2=SimpleNamespace(ensure_normal_mode=AsyncMock(return_value='mcf'))
        conn=SimpleNamespace(datachannel=SimpleNamespace(pub_sub=SimpleNamespace(publish_request_new=publish)))
        emit=Mock()
        self.assertEqual(await prepare_normal_mode(go2,conn,'mode',emit),'normal')
        go2.ensure_normal_mode.assert_not_awaited()
        publish.assert_awaited_once_with('mode',{'api_id':1001})
        publish.return_value['data']['data']=json.dumps({'name':'mcf'})
        self.assertEqual(await prepare_normal_mode(go2,conn,'mode',emit),'mcf')
    def test_cabinet_sideways(self):
        self.assertEqual(velocities(['q'],'cabinet'),{'x':0.,'y':.25,'z':0.})
        self.assertEqual(velocities(['e'],'cabinet'),{'x':0.,'y':-.25,'z':0.})
