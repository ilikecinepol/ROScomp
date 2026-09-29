"""Нормализация потоков и контроль их качества без исправления данных наугад."""
from collections import deque
from dataclasses import dataclass, field
import math
import statistics
from .models import Pose

def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)

def stamp_seconds(value):
    if isinstance(value, dict):
        sec, ns = value.get('sec'), value.get('nanosec')
        if not finite(sec) or not finite(ns) or not 0 <= ns < 1e9:
            return None
        return float(sec)+float(ns)*1e-9
    return float(value) if finite(value) else None

def pose_from_message(message):
    d = message.get('data', message)
    p = d.get('pose', {}).get('position', {})
    q = d.get('pose', {}).get('orientation', {})
    values = [p.get(k) for k in 'xyz']+[q.get(k) for k in 'xyzw']
    if not all(finite(v) for v in values):
        raise ValueError('Некорректная поза')
    x, y, z, w = values[3:]
    norm = math.sqrt(x*x+y*y+z*z+w*w)
    if abs(norm-1) > .02:
        raise ValueError('Кватернион не единичный')
    x,y,z,w = (v/norm for v in (x,y,z,w))
    yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    return Pose(*values[:2], yaw, values[2])

@dataclass
class Stream:
    received: float = -math.inf
    source: float | None = None
    current_source_valid: bool = False
    progressed: float = -math.inf
    count: int = 0
    duplicates: int = 0
    backwards: int = 0
    invalid: int = 0
    payload: dict = field(default_factory=dict)
    times: deque = field(default_factory=lambda: deque(maxlen=200))
    source_pairs: deque = field(default_factory=lambda: deque(maxlen=200))

class SensorMonitor:
    def __init__(self):
        self.streams = {}
        self.faults = []
        self.last_pose = None
        self.pose_epoch = 0
        self.identity = {}

    def ingest(self, name, message, received, source=None):
        if not finite(received):
            raise ValueError('Нет времени получения')
        s = self.streams.setdefault(name, Stream())
        if received < s.received:
            self.faults.append('Монотонные часы пошли назад')
            return
        d = message.get('data', message) if isinstance(message,dict) else None
        if not isinstance(d, dict):
            s.invalid += 1
            self.faults.append('Некорректный пакет: '+name)
            return
        if source is None:
            source = stamp_seconds(d.get('stamp', d.get('header', {}).get('stamp')))
        s.received, s.payload = received, d
        s.current_source_valid = source is not None and finite(source)
        if not s.current_source_valid and name in ('ROBOTODOM','LF_SPORT_MOD_STATE','ULIDAR_ARRAY','CAMERA'):
            self.faults.append('Некорректная метка времени текущего пакета: '+name)
        s.count += 1
        s.times.append(received)
        if source is not None and finite(source):
            if s.source is None or source > s.source:
                s.progressed = received
                s.source = source
            elif source < s.source:
                s.backwards += 1
            else:
                s.duplicates += 1
            s.source_pairs.append((received, source))
        if name == 'ULIDAR_STATE':
            self.identity.update({k: str(d[k]) for k in ('firmware_version','software_version','serial_number') if d.get(k)})
        if name == 'ROBOTODOM':
            try:
                p = pose_from_message(message)
                if self.last_pose:
                    old_t, old = self.last_pose
                    dt = received-old_t
                    if dt >= 0 and (math.hypot(p.x-old.x,p.y-old.y) > .12+1.0*dt or
                                   abs(p.z-old.z) > .12+.6*dt or
                                   abs(math.atan2(math.sin(p.yaw-old.yaw),math.cos(p.yaw-old.yaw))) > .12+1.3*dt):
                        self.pose_epoch += 1
                        self.faults.append('Скачок координат: нужна новая локализация')
                self.last_pose = (received, p)
            except ValueError as exc:
                s.invalid += 1
                self.faults.append(str(exc))

    def age(self, name, now):
        return now-self.streams[name].received if name in self.streams else math.inf

    def calibrated_age(self, name, now, contract):
        s = self.streams.get(name)
        if not s or not s.current_source_valid or s.source is None or not contract or contract.get('verified') is not True:
            return math.inf
        # Только установленный на текущем устройстве контракт; оценка масштаба
        # по приёму сама по себе не превращается в калибровку.
        try:
            if not finite(contract['scale']) or contract['scale'] <= 0 or not finite(contract['offset']):
                return math.inf
            age = now-(s.source*contract['scale']+contract['offset'])
            return age if finite(age) and age >= -.03 else math.inf
        except (KeyError, TypeError, ValueError):
            return math.inf

    def health(self, now, policy, required=('LF_SPORT_MOD_STATE','LOW_STATE','ROBOTODOM','CAMERA')):
        errors = list(dict.fromkeys(self.faults))
        for name in required:
            s = self.streams.get(name)
            limit = 2.0 if name == 'LOW_STATE' else policy.max_observation_age
            if not s or not 0 <= now-s.received <= limit:
                errors.append('Устаревший или отсутствующий поток '+name)
            elif name != 'LOW_STATE' and (not s.current_source_valid or s.source is None or not 0 <= now-s.progressed <= limit or s.backwards):
                errors.append('Не подтверждено продвижение времени '+name)
        d = self.streams.get('LF_SPORT_MOD_STATE', Stream()).payload
        low = self.streams.get('LOW_STATE', Stream()).payload
        try:
            r,p,_ = d['imu_state']['rpy']
            h = d['body_height']
            v = d['velocity']
            temp = [m['temperature'] for m in low['motor_state'][:12]]
            soc = low['bms_state']['soc']
            if not all(finite(x) for x in [r,p,h,soc,*v,*temp]) or len(temp)!=12 or len(v)!=3:
                raise ValueError('Неконечные/неполные данные')
            if abs(r)>math.radians(policy.max_roll_deg) or abs(p)>math.radians(policy.max_pitch_deg):
                errors.append('Наклон вне проверенных пределов')
            if not policy.minimum_body_height <= h <= policy.max_body_height:
                errors.append('Высота корпуса вне проверенных пределов')
            if soc < policy.min_battery or max(temp)>policy.max_motor_temperature:
                errors.append('Заряд/температура вне пределов')
            if math.hypot(*v[:2]) > .9:
                errors.append('Неожиданная сообщаемая скорость')
        except (KeyError, ValueError, TypeError):
            errors.append('Неполное состояние робота')
        # error_code не проверяется как булева ошибка: Sport V2 кодирует им режим.
        return list(dict.fromkeys(errors))

    def report(self, now):
        result = {}
        for name,s in self.streams.items():
            dt = [b-a for a,b in zip(s.times,list(s.times)[1:]) if b>a]
            scale = None
            if len(s.source_pairs)>2:
                a,b = s.source_pairs[0],s.source_pairs[-1]
                if b[0]>a[0]: scale=(b[1]-a[1])/(b[0]-a[0])
            result[name] = {'received':s.count, 'age_s':now-s.received,
                'median_hz':1/statistics.median(dt) if dt else None,
                'source_scale_by_receive':scale, 'duplicate_stamps':s.duplicates,
                'backward_stamps':s.backwards, 'invalid':s.invalid,
                'timing_calibrated_by_this_report':False}
        return {'streams':result,'identity':self.identity,'faults':list(dict.fromkeys(self.faults)),
                'frame_epoch':self.pose_epoch}
