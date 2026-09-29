"""Проверки дозагрузки без сети и робота."""
import hashlib
import io
from pathlib import Path
import tarfile
import tempfile
import unittest
from unittest.mock import patch
from download_recordings import destination, transfer


class DownloadTests(unittest.TestCase):
    def test_bad_paths(self):
        for name in ['../secret', '/etc/passwd', '20260922T122517Z/../../bad', '20260922T122517Z/C:bad']:
            with self.assertRaises(ValueError): destination(Path('records'), name)

    def test_verified_files_skipped_and_corrupt_file_replaced(self):
        name = '20260922T122517Z/events.jsonl'
        data = b'example recording'
        item = dict(path=name, size=len(data), sha256=hashlib.sha256(data).hexdigest())
        archive = io.BytesIO()
        with tarfile.open(fileobj=archive, mode='w') as t:
            entry = tarfile.TarInfo(name); entry.size = len(data)
            t.addfile(entry, io.BytesIO(data))
        class Process:
            def __init__(self, *a, **k):
                self.stdin = io.BytesIO(); self.stdout = io.BytesIO(archive.getvalue())
            def wait(self, **k): return 0
            def kill(self): pass
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            target = destination(root, name); target.parent.mkdir(); target.write_bytes(b'bad')
            with patch('download_recordings.subprocess.Popen', Process):
                transfer('192.168.11.81', 'key', root, [item], lambda x: None)
            self.assertEqual(target.read_bytes(), data)
            with patch('download_recordings.subprocess.Popen') as process:
                transfer('192.168.11.81', 'key', root, [item], lambda x: None)
                process.assert_not_called()


if __name__ == '__main__': unittest.main()
