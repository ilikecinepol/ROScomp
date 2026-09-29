"""Наблюдение заголовков до декодера. Не подменяет время получения временем съёмки."""
import hashlib
import json
import time


class PacketAudit:
    def __init__(self,decoder,journal,clock=time.monotonic):
        self.decoder,self.journal=decoder,journal
        self.count=0
        self.stamps=set()
        self.payloads=set()
        self.first=None
        self.clock=clock
        self.received=None
        self.changed=None
        self.previous_digest=None
        self.changes=0

    def decode(self,payload,metadata):
        # Снимок до вызова декодера позволяет проверить, кто меняет заголовок.
        header={k:v for k,v in metadata.items() if k!='data'}
        snapshot=json.loads(json.dumps(header))
        digest=hashlib.sha256(payload).hexdigest()
        now=self.clock()
        if self.previous_digest is not None and digest!=self.previous_digest:
            self.changed=now
            self.changes+=1
        self.previous_digest=digest
        self.received=now
        self.count+=1
        if len(self.stamps)<4096:self.stamps.add(str(header.get('stamp')))
        if len(self.payloads)<4096:self.payloads.add(digest)
        if self.first is None:self.first=snapshot
        self.journal.put('packet-audit.jsonl',{'t_received':now,
            'header_before_decoder':snapshot,'payload_sha256':digest,'payload_bytes':len(payload)})
        return self.decoder.decode(payload,metadata)

    def report(self):
        now=self.clock()
        received_age=None if self.received is None else now-self.received
        change_age=None if self.changed is None else now-self.changed
        if received_age is None or not 0<=received_age<=.5:
            status='no_recent_packets'
        elif change_age is None or not 0<=change_age<=.5:
            status='content_progress_not_observed'
        elif len(self.stamps)<2:
            status='content_changes_source_time_unavailable'
        else:
            status='content_changes_time_not_calibrated'
        return {'packets':self.count,'distinct_stamps':len(self.stamps),
            'distinct_payloads':len(self.payloads),'first_header':self.first,
            'transport_status':status,'packet_age_s':received_age,
            'content_change_age_s':change_age,'content_changes':self.changes,
            'acquisition_age_verified':False}
