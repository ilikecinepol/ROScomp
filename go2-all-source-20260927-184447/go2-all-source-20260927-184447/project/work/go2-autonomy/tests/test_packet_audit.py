import unittest
from wolf_go2.packet_audit import PacketAudit


class AuditTests(unittest.TestCase):
    def test_predecoder_snapshot_not_changed_or_certified(self):
        class Journal:
            def put(self,name,value): self.row=value
        class Decoder:
            def decode(self,payload,meta):
                meta['stamp']=999
                return {'points':[]}
        journal=Journal();audit=PacketAudit(Decoder(),journal)
        for payload in (b'first',b'second'):
            audit.decode(payload,{'stamp':42,'frame_id':'odom'})
        self.assertEqual(journal.row['header_before_decoder']['stamp'],42)
        self.assertEqual(audit.report()['distinct_stamps'],1)
        self.assertEqual(audit.report()['distinct_payloads'],2)
        self.assertFalse(audit.report()['acquisition_age_verified'])

    def test_transport_change_does_not_prove_acquisition_freshness(self):
        class Journal:
            def put(self,*args):pass
        class Decoder:
            def decode(self,*args):return {}
        now=[0.]
        audit=PacketAudit(Decoder(),Journal(),clock=lambda:now[0])
        audit.decode(b'a',{'stamp':42})
        now[0]=.1
        audit.decode(b'b',{'stamp':42})
        self.assertEqual(audit.report()['transport_status'],'content_changes_source_time_unavailable')
        self.assertFalse(audit.report()['acquisition_age_verified'])
        now[0]=.7
        self.assertEqual(audit.report()['transport_status'],'no_recent_packets')
        audit.decode(b'b',{'stamp':42})
        self.assertEqual(audit.report()['transport_status'],'content_progress_not_observed')
        now[0]=.8
        audit.decode(b'a',{'stamp':42})
        # Даже чередование старых пакетов не доказывает свежую съёмку.
        self.assertFalse(audit.report()['acquisition_age_verified'])
