"""Офлайн-проверка выделения поверхностей; не выдаёт разрешений движения.

Применяет геометрические гипотезы к записи отдельно от управляющего контура.
Проверка временных меток и положения корпуса остаётся обязательной в live.
"""
import argparse
import json
import math
from pathlib import Path
import numpy as np
from scipy import ndimage
from wolf_go2.models import Policy
from wolf_go2.sensors import pose_from_message
from wolf_go2.perception import _raster, _floor, _support_grid, _profile_candidate
from wolf_go2.ramp_plane import detect_ramp


def analyze(capture):
    folder = Path(capture)
    records = [json.loads(s) for s in (folder/'samples.jsonl').read_text().splitlines()]
    poses = [r for r in records if r['topic']=='ROBOTODOM']
    maps = [r for r in records if r.get('npz_file')]
    reports = []
    for record in maps:
        pose_record = min(poses,key=lambda p:abs(p['elapsed_s']-record['elapsed_s']))
        pose = pose_from_message(pose_record['message'])
        metadata = record['message']['data']
        resolution = float(metadata['resolution'])
        with np.load(folder/record['npz_file'],allow_pickle=False) as archive:
            points = archive['data_data_data_points'].copy()
        # Обе гипотезы проверяем явно, не объявляя одну калибровкой.
        for reference,shift in [('lower_corner',[.5,.5,1.]),('center',[0.,0.,.5])]:
            cloud = points+resolution*np.array(shift)
            top,origin,xx,yy,_ = _raster(cloud,resolution)
            floor = _floor(top,xx,yy,pose,resolution)
            entry = dict(file=record['npz_file'],reference_hypothesis=reference,
                         frame=metadata['frame_id'],candidates=[])
            if floor is None:
                entry['error']='Не найдена плоскость пола'
            else:
                coef,height = floor
                dx,dy=cloud[:,0]-pose.x,cloud[:,1]-pose.y
                co,si=math.cos(pose.yaw),math.sin(pose.yaw)
                relative=np.column_stack((co*dx+si*dy,-si*dx+co*dy,
                    cloud[:,2]-(coef[0]*cloud[:,0]+coef[1]*cloud[:,1]+coef[2])))
                entry['front_ramp_plane']=detect_ramp(relative)
                _,raised = _support_grid(top,height,origin,resolution,0,Policy())
                labels,n = ndimage.label(raised,np.ones((3,3)))
                for i in range(1,n+1):
                    mask=labels==i
                    if mask.sum()<6:
                        continue
                    item=_profile_candidate(mask,top,xx,yy,pose,coef,resolution,Policy())
                    entry['candidates'].append(item)
            reports.append(entry)
    return dict(diagnostic_only=True,motion_authorized=False,
                note='Гипотезы геометрии; не профиль калибровки и не маршрут для запуска',frames=reports)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('capture');p.add_argument('output')
    args=p.parse_args()
    def clean(v):
        if isinstance(v,dict):return {k:clean(x) for k,x in v.items()}
        if isinstance(v,(list,tuple)):return [clean(x) for x in v]
        if isinstance(v,np.ndarray):return clean(v.tolist())
        if isinstance(v,np.generic):return clean(v.item())
        if isinstance(v,float) and not np.isfinite(v):return None
        return v
    result=clean(analyze(args.capture))
    with Path(args.output).open('x',encoding='utf-8') as out:
        json.dump(result,out,ensure_ascii=False,indent=2,allow_nan=False)
    print('Сохранён диагностический разбор без команд движения')
