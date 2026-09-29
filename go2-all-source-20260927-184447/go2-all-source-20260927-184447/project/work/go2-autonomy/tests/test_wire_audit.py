import struct
import unittest
from wolf_go2.wire_audit import lidar_wire_stamp,WireAudit


def packet(stamp,modern=True,topic='rt/utlidar/voxel_map_compressed'):
    text=(' {"topic":"'+topic+'","data":{"stamp":'+stamp+'}}').encode()
    if modern:return struct.pack('<HHII',2,0,len(text),0)+text+b'payload'
    return struct.pack('<HH',len(text),0)+text+b'payload'


class WireAuditTests(unittest.TestCase):
    def test_exponent_precision_is_preserved(self):
        result=lidar_wire_stamp(packet('1.7905e+09'))
        self.assertEqual(result['stamp_exact'],'1.7905E+9')
        self.assertEqual(result['represented_decimal_quantum'],'100000')
        self.assertEqual(result['float_conversion_error'],'0')

    def test_old_framing_and_fraction(self):
        result=lidar_wire_stamp(packet('1790505000.123456789',False))
        self.assertEqual(result['stamp_exact'],'1790505000.123456789')
        self.assertEqual(result['represented_decimal_quantum'],'1E-9')
        self.assertNotEqual(result['float_conversion_error'],'0')

    def test_split_stamp(self):
        result=lidar_wire_stamp(packet('{"sec":123,"nanosec":42}'))
        self.assertEqual(result['stamp_exact'],{'sec':'123','nanosec':'42'})

    def test_invalid_and_other_topics_ignored(self):
        for value in (b'',b'bad',packet('42')[:15],packet('42',topic='private'),packet('true')):
            self.assertIsNone(lidar_wire_stamp(value))

    def test_original_bytes_and_result_preserved_on_journal_failure(self):
        original=packet('1.7905e+09')
        seen=[]
        class Journal:
            def put(self,*args):raise OSError('full')
        def decode(value):seen.append(value);return 'result'
        audit=WireAudit(decode,Journal())
        self.assertEqual(audit.decode(original),'result')
        self.assertIs(seen[0],original)
        self.assertEqual(audit.report()['audit_errors'],1)
        self.assertFalse(audit.report()['clock_corrected'])
