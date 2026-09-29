"""Регрессии обновления окна: без сети и робота."""
import unittest
from unittest.mock import Mock, patch
from keyboard_client import KeyboardWindow, InputState

class VideoUITests(unittest.TestCase):
    def test_tcl_popup_does_not_need_python_widget(self):
        w = KeyboardWindow.__new__(KeyboardWindow)
        w.root = Mock()
        w.root._w = '.'
        w.profile_box = '.profile'
        w.marker_entry = '.marker'
        w.root.tk.call.return_value = '.profile.popdown.f.l'
        self.assertTrue(w.typing())
        w.root.focus_get.assert_not_called()
        w.state = InputState()
        w.closing_since = None
        w.check_focus()
        self.assertTrue(w.state.focused)
        w.root.focus_displayof.assert_not_called()

    def test_exception_stops_motion_but_keeps_refreshing(self):
        w = KeyboardWindow.__new__(KeyboardWindow)
        w.root = Mock()
        w.state = InputState()
        w.state.keys.add('w')
        w.send = Mock()
        w.status_var = Mock()
        w.tick_paused = False
        w._tick_once = Mock(side_effect=KeyError('popdown'))
        with patch('traceback.print_exc'):
            w.tick()
        self.assertFalse(w.state.keys)
        w.send.assert_called_once()
        w.root.after.assert_called_once_with(25, w.tick)

    def test_normal_refresh_schedules_once(self):
        w = KeyboardWindow.__new__(KeyboardWindow)
        w.root = Mock()
        w.tick_paused = False
        w._tick_once = Mock()
        w.tick()
        w.root.after.assert_called_once_with(25, w.tick)

if __name__ == '__main__': unittest.main()
