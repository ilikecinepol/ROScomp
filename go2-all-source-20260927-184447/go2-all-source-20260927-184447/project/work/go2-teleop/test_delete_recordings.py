"""Проверка области удаления без реальных записей и сети."""
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
from download_recordings import DELETE_HELPER, DELETE_SESSIONS

class DeleteTests(unittest.TestCase):
    def setUp(self):
        self.context={}
        exec(DELETE_HELPER,self.context)
        self.delete=self.context['delete_sessions']
    def test_only_selected_complete_sessions(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d).resolve()
            done=root/'20260924T100000Z';done.mkdir()
            (done/'run_status.json').write_text(json.dumps({'exit_code':0}))
            active=root/'20260924T110000Z';active.mkdir()
            program=root/'program.py';program.write_text('keep')
            with patch('shutil.rmtree') as remove:
                result=self.delete(root,[done.name])
                remove.assert_called_once_with(done)
                self.assertEqual(result['deleted'],[done.name])
            self.assertTrue(active.exists());self.assertTrue(program.exists())
    def test_incomplete_rejects_entire_batch_before_delete(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d).resolve()
            done=root/'20260924T100000Z';done.mkdir()
            (done/'run_status.json').write_text('{"exit_code":0}')
            active=root/'20260924T110000Z';active.mkdir()
            with patch('shutil.rmtree') as remove:
                with self.assertRaises(ValueError):self.delete(root,[done.name,active.name])
                remove.assert_not_called()
    def test_invalid_paths_never_deleted(self):
        with tempfile.TemporaryDirectory() as d:
            for name in ['..','../outside','program.py','/tmp','20260924T100000Z/child']:
                with patch('shutil.rmtree') as remove:
                    with self.assertRaises(ValueError):self.delete(Path(d).resolve(),[name])
                    remove.assert_not_called()
    def test_remote_script_compiles(self):
        compile(DELETE_SESSIONS,'remote-delete','exec')

if __name__=='__main__':unittest.main()
