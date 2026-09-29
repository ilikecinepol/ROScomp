"""Проверка боковых клавиш и профиля рампы без робота."""
import unittest
from types import SimpleNamespace
from unittest.mock import Mock
from keyboard_client import KeyboardWindow, InputState
from remote_teleop import velocities

class SidewaysTests(unittest.TestCase):
    def test_physical_keys_through_window_and_server_mapping(self):
        for sym,code,key,sign in [('q',81,'q',1),('e',69,'e',-1),('Cyrillic_shorti',81,'q',1),('Cyrillic_u',69,'e',-1)]:
            with self.subTest(sym=sym):
                w=KeyboardWindow.__new__(KeyboardWindow)
                w.release_after={}; w.closing_since=None
                w.state=InputState(); w.state.focused=True
                w.state.frame_time=10
                w.state.update_status(dict(phase='ready',armed=True,token=1,telemetry_age_s=0,camera_age_s=0),10)
                w.state.request_arm(10)
                w.typing=Mock(return_value=False)
                w.key_press(SimpleNamespace(keysym=sym,keycode=code))
                message=w.state.heartbeat(10)
                self.assertTrue(message['deadman'])
                self.assertEqual(message['keys'],[key])
                v=velocities(message['keys'],'ramp25')
                self.assertEqual(v['y'],sign*.10)
                self.assertEqual(v['x'],0)
                self.assertEqual(v['z'],0)
    def test_ramp_forward_and_opposite_side_keys(self):
        self.assertEqual(velocities(['w'],'ramp25')['x'],.25)
        self.assertEqual(velocities(['q','e'],'ramp25')['y'],0)

if __name__=='__main__': unittest.main()
