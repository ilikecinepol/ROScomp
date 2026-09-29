"""Автономный HTML для исследования записей и кинематического прогона.

Не подключается к Pi. Измеренная карта остаётся неизменной; синтетическая
сцена явно отделена от записей. Маршрут считает бортовой plan_slalom.
"""
import base64
import json
from pathlib import Path
import numpy as np

from wolf_go2.models import Grid, Policy, Pose
from wolf_go2.perception import _raster, _floor, _support_grid
from wolf_go2.scene_audit import inspect_scene
from wolf_go2.slalom_route import plan_slalom

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT/'outputs/go2-autonomy/virtual-arena'


def grid_json(grid):
    return dict(origin=grid.origin, resolution=grid.resolution, cells=grid.cells)


def recorded_frame(path):
    meta=json.loads(path.read_text(encoding='utf-8'))
    pose=Pose(**meta['pose']); res=meta['resolution']
    with np.load(path.with_suffix('.npz')) as data:
        points=data[data.files[0]]
    top,origin,x,y,_=_raster(points+[res/2,res/2,res],res)
    floor=_floor(top,x,y,pose,res)
    if floor is None: raise ValueError('Не выделен пол: '+str(path))
    grid,_=_support_grid(top,floor[1],origin,res,0,Policy(),voxel_size=res)
    scene=inspect_scene(points,pose,{'resolution':res,'frame_id':meta['point_frame']})
    image=path.parent/meta['camera_file'] if meta.get('camera_file') else None
    return dict(grid=grid_json(grid), pose=meta['pose'], source=str(path.relative_to(ROOT)),
        received=meta['received_at'], stamp=meta['cloud_source_s'],
        camera_gap=meta.get('camera_receive_delta_s'), pose_gap=meta.get('pose_receive_delta_s'),
        photo='data:image/jpeg;base64,'+base64.b64encode(image.read_bytes()).decode() if image and image.is_file() else None,
        poles=scene.get('slalom_pole_candidates',[]), rows=scene.get('slalom_row_candidates',[]),
        plans=scene.get('slalom_route_previews',[]), body=scene.get('body_map_audit',{}))


def synthetic():
    # Свободный учебный пол и увеличенный шаг стоек: это НЕ копия полигона.
    res=.05; origin=(-2.,-3.); cells=np.zeros((120,190),dtype=int)
    yy,xx=np.indices(cells.shape); x=origin[0]+(xx+.5)*res;y=origin[1]+(yy+.5)*res
    centers=[[v,0.] for v in [0.,1.8,3.6,5.4]]
    for px,py in centers:cells[(x-px)**2+(y-py)**2<=.15**2]=1
    grid=Grid(origin,res,cells.tolist(),0,True)
    policy=Policy();plan=plan_slalom(grid,centers,.15,Pose(-1.4,0.,0.),policy,.08)
    if not plan['path']:raise RuntimeError('Учебный маршрут не рассчитан')
    path=plan['path'];pose=dict(x=path[0][0],y=path[0][1],yaw=0.,z=.3)
    return dict(name='Учебная сцена · не полигон',synthetic=True,frames=[dict(grid=grid_json(grid),pose=pose,
        source='Синтетический свободный пол, четыре стойки через 1,8 м',received=0,stamp=None,photo=None,
        poles=[dict(center_xy=p,width_xy_m=[.3,.3]) for p in centers],rows=[dict(centers_xy=centers)],
        plans=[plan],body={})])


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    datasets=[]
    for slot,title in [('slot-1724','17:24 · прежняя позиция'),('slot-1728','17:28 · прежняя позиция'),
                       ('slot-1759','17:59 · у первой стойки'),('slot-1813','18:13 · повторная проверка')]:
        paths=sorted((ROOT/'outputs/go2-autonomy/diagnostic-20260926'/slot).glob('*/cloud_*.json'))
        paths=[p for p in paths if json.loads(p.read_text(encoding='utf-8')).get('pose')]
        chosen=sorted(set([0,len(paths)//2,len(paths)-1]))
        frames=[recorded_frame(paths[i]) for i in chosen]
        datasets.append(dict(name=title,synthetic=False,frames=frames))
        print(title,len(frames),flush=True)
    datasets.append(synthetic())
    data=dict(datasets=datasets,robot=dict(length=.75,width=.4,clearance=.1,uncertainty=.08))
    text=(Path(__file__).with_name('virtual_arena_template.html')).read_text(encoding='utf-8')
    encoded=json.dumps(data,ensure_ascii=False,separators=(',',':')).replace('</',r'<\/')
    (OUT/'Карта и виртуальный прогон.html').write_text(text.replace('__ARENA_DATA__',encoded),encoding='utf-8')
    (OUT/'data.json').write_text(json.dumps(data,ensure_ascii=False),encoding='utf-8')
    print(OUT/'Карта и виртуальный прогон.html')


if __name__=='__main__':main()
