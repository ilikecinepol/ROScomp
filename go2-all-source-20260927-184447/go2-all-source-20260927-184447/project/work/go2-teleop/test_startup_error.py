"""Ошибка второго запуска должна сохраняться до завершения SSH."""
import io
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from keyboard_client import SSHTransport

class StartupErrorTests(unittest.TestCase):
    def test_wrapper_error_is_visible_and_logged(self):
        with tempfile.TemporaryDirectory() as d:
            t=SSHTransport(Path(d)/'client.log',['fake-ssh'])
            msg={'type':'error','reason':'Другой сеанс управления уже запущен.'}
            t.proc=SimpleNamespace(stdout=io.StringIO(json.dumps(msg)+'\n'))
            t._reader()
            event=t.events.get_nowait()
            self.assertEqual(event['type'],'transport_error')
            self.assertEqual(event['reason'],msg['reason'])
            t.log.close()
            self.assertIn(msg['reason'],(Path(d)/'client.log').read_text(encoding='utf-8'))

if __name__=='__main__':unittest.main()
