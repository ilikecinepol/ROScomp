"""Запись и её отказы: журнал не выдаёт пропуск за успешную запись."""
import json
from pathlib import Path
import queue
import tempfile
import unittest
from unittest import mock

from wolf_go2.journal import Journal, jsonable


class JournalTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='wolf-go2-journal-test-')
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'session'
        self.journal = Journal(self.path)
        self.addCleanup(self.journal.close)

    def test_events_flush_on_close_and_nonfinite_is_explicit(self):
        self.journal.event('decision', reason='Проверка', velocity=.1,
                           unsupported=float('nan'), infinite=float('inf'))
        self.journal.close()
        records = [json.loads(line) for line in (self.path/'events.jsonl').read_text(encoding='utf-8').splitlines()]
        self.assertEqual(len(records), 1)
        self.assertEqual(records[0]['reason'], 'Проверка')
        self.assertEqual(records[0]['velocity'], .1)
        self.assertIsNone(records[0]['unsupported'])
        self.assertIsNone(records[0]['infinite'])
        self.assertFalse(self.journal.failed.is_set())
        self.assertFalse(self.journal.thread.is_alive())

    def test_media_npz_roundtrip_and_memory_accounting(self):
        import numpy as np
        points = np.arange(30, dtype=np.float32).reshape(10, 3)
        self.journal.put('cloud.npz', points)
        self.journal.close()
        with np.load(self.path/'cloud.npz', allow_pickle=False) as data:
            np.testing.assert_array_equal(data['points'], points)
        self.assertEqual(self.journal.media_bytes, 0)
        self.assertFalse(self.journal.failed.is_set())

    def test_external_path_and_existing_session_are_not_overwritten(self):
        with self.assertRaises(ValueError):
            self.journal.put('../outside.jsonl', {'unexpected': True})
        self.assertFalse((self.path.parent/'outside.jsonl').exists())
        self.journal.close()
        with self.assertRaises(FileExistsError):
            Journal(self.path)

    def test_over_budget_media_is_rejected_without_allocating_it(self):
        class Oversized:
            nbytes = 33*1024*1024
        self.journal.put('too-large.npz', Oversized())
        self.assertTrue(self.journal.failed.is_set())
        self.assertEqual(self.journal.dropped, 1)
        self.assertEqual(self.journal.media_bytes, 0)
        self.assertFalse((self.path/'too-large.npz').exists())

    def test_full_queue_reports_drop_and_releases_reserved_memory(self):
        class SmallMedia:
            nbytes = 128
        with mock.patch.object(self.journal.queue, 'put_nowait', side_effect=queue.Full):
            self.journal.put('unqueued.npz', SmallMedia())
        self.assertTrue(self.journal.failed.is_set())
        self.assertEqual(self.journal.dropped, 1)
        self.assertEqual(self.journal.media_bytes, 0)

    def test_disk_failure_is_visible_without_hanging_producer(self):
        with mock.patch.object(Path, 'open', side_effect=OSError('synthetic disk failure')):
            self.journal.event('before-disk-failure')
            self.assertTrue(self.journal.failed.wait(2.))
        self.journal.close()
        self.assertFalse(self.journal.thread.is_alive())

    def test_unknown_json_object_sets_failure_instead_of_fake_success(self):
        self.journal.put('unsupported.jsonl', {'object': object()})
        self.assertTrue(self.journal.failed.wait(2.))
        self.journal.close()
        self.assertFalse(self.journal.thread.is_alive())

    def test_telemetry_array_summary_does_not_expand_all_points(self):
        import numpy as np
        value = jsonable({'cloud': np.ones((10, 3), dtype=np.float32),
                          'count': np.int64(10), 'binary': b'abcd'})
        self.assertEqual(value['cloud'], {'shape': [10, 3], 'dtype': 'float32'})
        self.assertEqual(value['count'], 10)
        self.assertEqual(value['binary'], {'bytes': 4})


if __name__ == '__main__':
    unittest.main()
