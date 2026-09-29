"""Пассивная проверка альтернативных источников. Не разрешает движение."""
import time
from .sensors import stamp_seconds

SOURCES=('ULIDAR','SLAM_ODOMETRY','LIDAR_MAPPING_ODOM',
         'LIDAR_LOCALIZATION_ODOM','LIDAR_MAPPING_CLOUD_POINT',
         'LIDAR_LOCALIZATION_CLOUD_POINT','UWB_STATE')

class SourceAudit:
    def __init__(self,clock=time.monotonic):
        self.clock=clock
        self.streams={name:{'received':0,'advances':0,'duplicates':0,'backwards':0,
                           'valid_stamps':0,'header':{},'last_stamp':None} for name in SOURCES}

    def receive(self,name,message):
        state=self.streams[name]
        state['received']+=1
        state['last_receive']=self.clock()
        data=message.get('data',{}) if isinstance(message,dict) else {}
        if not isinstance(data,dict):return
        header=data.get('header',{})
        if not isinstance(header,dict):header={}
        # Массивы точек и произвольные содержимые пакетов в отчёт не копируем.
        state['header']={k:data[k] for k in ('frame_id','child_frame_id','resolution','serial_number') if isinstance(data.get(k),(str,int,float))}
        for k in ('frame_id','stamp','seq'):
            if k in header:state['header']['header_'+k]=header[k]
        stamp=stamp_seconds(data.get('stamp',header.get('stamp')))
        if stamp is None:return
        old=state['last_stamp'];state['valid_stamps']+=1
        if old is not None:
            state['advances' if stamp>old else 'duplicates' if stamp==old else 'backwards']+=1
        state['last_stamp']=stamp

    def report(self):
        now=self.clock()
        return {name:{**state,'receive_age_s':None if 'last_receive' not in state else now-state['last_receive'],
                      'acquisition_age_verified':False} for name,state in self.streams.items()}
