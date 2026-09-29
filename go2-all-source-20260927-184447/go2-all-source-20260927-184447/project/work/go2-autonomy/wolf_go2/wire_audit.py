"""Точная запись метки лидара из JSON до преобразования в float.

Не меняет SDK, пакет или шкалу времени и не разрешает движение.
"""
from decimal import Decimal
import json
import struct
import time


def lidar_wire_stamp(buffer):
    if not isinstance(buffer, bytes) or len(buffer)<4:
        return None
    first,second=struct.unpack_from('<HH',buffer)
    if (first,second)==(2,0):
        if len(buffer)<12:return None
        length=struct.unpack_from('<I',buffer,4)[0]
        start=12
    else:
        length=first
        start=4
    if not 0<length<=65536 or start+length>len(buffer):return None
    try:
        # parse_float сохраняет лексему, которую стандартный json.loads теряет.
        doc=json.loads(buffer[start:start+length].decode('utf-8'),parse_float=Decimal)
        topic=doc.get('topic','')
        if not isinstance(topic,str) or 'utlidar' not in topic:return None
        data=doc.get('data',{})
        if not isinstance(data,dict):return None
        value=data.get('stamp',data.get('header',{}).get('stamp') if isinstance(data.get('header'),dict) else None)
        if isinstance(value,dict):
            return {'topic':topic,'stamp_form':'sec_nanosec','stamp_exact':{k:str(value.get(k)) for k in ('sec','nanosec')}}
        if isinstance(value,bool) or not isinstance(value,(Decimal,int)):return None
        decimal=Decimal(value)
        if not decimal.is_finite():return None
        quantum=Decimal(10)**decimal.as_tuple().exponent
        return {'topic':topic,'stamp_form':'number','stamp_exact':str(value),
                'represented_decimal_quantum':str(quantum),
                'python_float':float(value),
                'float_conversion_error':str(abs(Decimal.from_float(float(value))-decimal))}
    except (UnicodeError,ValueError,TypeError,AttributeError,OverflowError):
        return None


class WireAudit:
    def __init__(self,decode,journal,clock=time.monotonic):
        self.decode_original,self.journal,self.clock=decode,journal,clock
        self.count=0
        self.values=set()
        self.first=None
        self.errors=0

    def decode(self,buffer):
        try:
            result=lidar_wire_stamp(buffer)
            if result is not None:
                self.count+=1
                if len(self.values)<4096:self.values.add(json.dumps(result['stamp_exact'],sort_keys=True))
                if self.first is None:self.first=result
                self.journal.put('wire-stamps.jsonl',{'received_at':self.clock(),**result})
        except Exception:
            # Диагностика не должна ломать штатный декодер SDK.
            self.errors+=1
        return self.decode_original(buffer)

    def report(self):
        return {'packets':self.count,'distinct_exact_stamps':len(self.values),
                'first':self.first,'audit_errors':self.errors,
                'source':'binary_packet_json_before_standard_float_parse',
                'clock_corrected':False,'acquisition_age_verified':False}
