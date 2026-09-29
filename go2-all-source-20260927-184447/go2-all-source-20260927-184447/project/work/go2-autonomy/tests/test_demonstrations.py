"""Проверка границ импорта и локальных координат примера."""
import json
import math
from pathlib import Path
import tempfile
import unittest
from wolf_go2.demonstrations import import_run, build_library


class DemonstrationsTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.folder = self.root / 'session' / 'run'
        self.folder.mkdir(parents=True)
        (self.folder / 'a.jpg').write_bytes(b'fixture')
        (self.folder / 'recording.json').write_text('{"complete":true}')
        self.rows(self.folder / 'camera.jsonl', [dict(elapsed_s=t,file='a.jpg') for t in [10,12]])
        self.rows(self.folder.parent / 'events.jsonl', [dict(elapsed_s=9,event='record_start',profile='sport')])
        records=[]
        for t,x,y in [(10,2,3),(12,2,4)]:
            records.append(dict(elapsed_s=t,topic='ROBOTODOM',message={'data':{'pose':{
                'position':dict(x=x,y=y,z=.3),
                'orientation':dict(w=math.sqrt(.5),x=0,y=0,z=math.sqrt(.5))}}}))
        records.append(dict(elapsed_s=11,topic='LF_SPORT_MOD_STATE',message={'data':{}}))
        self.rows(self.folder / 'telemetry.jsonl', records)

    def rows(self,path,rows):
        path.write_text('\n'.join(json.dumps(r) for r in rows),encoding='utf-8')

    def test_coordinates_and_no_motion_certification(self):
        result=build_library(self.root,{'examples':[dict(kind='platforms',segments=[dict(session='session',run='run')])]})
        self.assertFalse(result['autonomous_ready'])
        segment=result['examples'][0]['segments'][0]
        self.assertAlmostEqual(segment['poses'][-1][1],1)
        self.assertAlmostEqual(segment['poses'][-1][2],0)
        self.assertFalse(segment['autonomous_skill_validated'])
        self.assertEqual(segment['profile_at_start'],'sport')

    def test_escape_rejected(self):
        with self.assertRaises(ValueError):
            import_run(self.root,'..','outside')

    def test_missing_frame_rejected(self):
        (self.folder / 'a.jpg').unlink()
        with self.assertRaises(ValueError):
            import_run(self.root,'session','run')

    def test_incomplete_rejected(self):
        (self.folder / 'recording.json').write_text('{"complete":false}')
        with self.assertRaises(ValueError):
            import_run(self.root,'session','run')
