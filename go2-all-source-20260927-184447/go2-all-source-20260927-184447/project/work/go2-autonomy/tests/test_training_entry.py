"""Предварительная проверка не освобождает службу и не посылает движение."""
from contextlib import redirect_stdout
import io
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch
import training_entry
from wolf_go2.profile import RobotProfile


class EntryTests(unittest.TestCase):
    def test_observation_needs_no_profile_and_never_enables_motion(self):
        received=[]
        async def observe(args):
            received.append(args)
            return {'scope':'observation_only','fault':None}
        lock=SimpleNamespace(LOCK_EX=1,LOCK_NB=2,flock=lambda *a:None)
        with tempfile.TemporaryDirectory() as folder, patch.object(training_entry,'TEAM',Path(folder)), patch('sys.argv',['entry','--observe']), patch.object(training_entry,'active',return_value=False), patch.object(training_entry,'service') as service, patch.dict('sys.modules',{'fcntl':lock,'wolf_go2.runtime':SimpleNamespace(run_session=observe)}), redirect_stdout(io.StringIO()):
            self.assertEqual(training_entry.main(),0)
            self.assertEqual(received[0].mode,'observe')
            self.assertIsNone(received[0].profile)
            self.assertFalse(received[0].button_start)
            service.assert_not_called()

    def test_missing_profile_does_not_touch_services(self):
        with tempfile.TemporaryDirectory() as folder, patch.object(training_entry,'TEAM',Path(folder)), patch('sys.argv',['entry','--kind','slalom']), patch.object(training_entry,'service') as service, redirect_stdout(io.StringIO()) as output:
            self.assertEqual(training_entry.main(),2)
            service.assert_not_called()
            self.assertIn('autonomy-profile.json',output.getvalue())

    def test_unverified_profile_does_not_touch_services(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);(root/'autonomy-profile.json').write_text('{}')
            with patch.object(training_entry,'TEAM',root), patch('sys.argv',['entry','--kind','teeter']), patch.object(RobotProfile,'load',return_value=RobotProfile('test')), patch.object(training_entry,'service') as service, redirect_stdout(io.StringIO()) as output:
                self.assertEqual(training_entry.main(),2)
                service.assert_not_called()
                self.assertIn('teeter',output.getvalue())
