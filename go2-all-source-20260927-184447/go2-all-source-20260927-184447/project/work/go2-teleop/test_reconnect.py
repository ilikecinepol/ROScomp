"""Переподключение: без робота, сети и графического дисплея."""
import unittest
from unittest.mock import Mock
from keyboard_client import KeyboardWindow, DemoTransport, InputState


class ReconnectTests(unittest.TestCase):
    def window(self):
        w = KeyboardWindow.__new__(KeyboardWindow)
        w.transport = DemoTransport()
        w.state = InputState()
        w.dry_run = True
        w.session_end = None
        w.reconnect_requested = False
        w.tick_paused = False
        w.closing_since = None
        for field in ('root', 'profile_box', 'reconnect_button', 'camera', 'path_var',
                      'telemetry_var', 'status_var', 'record_start_button', 'record_stop_button'):
            setattr(w, field, Mock())
        return w

    def test_request_quits_without_motion_and_double_click_is_ignored(self):
        w = self.window()
        w.state.keys.add('w')
        w.reconnect_video()
        w.reconnect_video()
        self.assertEqual(w.transport.sent, [{'type': 'quit'}])
        self.assertFalse(w.state.keys)

    def test_unconfirmed_shutdown_blocks_second_connection(self):
        w = self.window()
        old = w.transport
        old.done.set()
        self.assertFalse(w.restart_transport())
        self.assertIs(w.transport, old)

    def test_verified_reset_still_waits_for_local_transport(self):
        w = self.window()
        self.assertFalse(w.restart_transport(reset_verified=True))
        w.transport.done.set()
        self.assertTrue(w.restart_transport(reset_verified=True))
        self.assertFalse(w.state.keys)
        self.assertFalse(w.state.arm_intent)

    def test_new_session_clears_old_telemetry_recording_and_held_keys(self):
        w = self.window()
        w.transport.done.set()
        w.session_end = {'fleet_restored': True}
        w.state.keys.add('w')
        w.state.frame_time = 100
        self.assertTrue(w.restart_transport())
        self.assertFalse(w.state.arm_intent)
        self.assertFalse(w.state.keys)
        self.assertIn('w', w.state.blocked_keys)
        self.assertFalse(w.transport.sent)
        self.assertIsNone(w.transport.recording)


if __name__ == '__main__': unittest.main()
