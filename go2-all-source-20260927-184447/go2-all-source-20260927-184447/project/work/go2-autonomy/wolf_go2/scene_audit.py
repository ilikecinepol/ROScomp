"""Геометрический разбор для диагностики. Не создаёт разрешение движения."""
import numpy as np
from .perception import _raster, _floor, _support_grid
from .models import Policy
from .footprint import footprint_report
from .slalom_route import plan_slalom
from .poles import pole_candidates,slalom_pole_candidates,pole_rows,exclude_body_conflicts
from .body_map_audit import body_map_audit


def inspect_monitor(points, monitor):
    """Монитор хранит пару (время получения, Pose), а не голую позу."""
    stream=monitor.streams.get('ULIDAR_ARRAY')
    return inspect_scene(points,monitor.last_pose[1] if monitor.last_pose else None,
                         stream.payload if stream else {})


def inspect_scene(points, pose, metadata):
    report={'scope':'diagnostic_only','route_verified':False,'pole_candidates':[],
            'reason':'Геометрия требует проверки; диагностические кандидаты не являются маршрутом'}
    if points is None or pose is None:
        report['reason']='Не получены облако или координаты робота'
        return report
    resolution=metadata.get('resolution')
    if not isinstance(resolution,(float,int)) or isinstance(resolution,bool) or not .02<=resolution<=.1:
        report['reason']='Нет поддерживаемого размера вокселя'
        return report
    cloud=np.asarray(points,dtype=float)
    if cloud.ndim!=2 or cloud.shape[1]!=3 or len(cloud)<30 or not np.isfinite(cloud).all():
        report['reason']='Недостаточно конечных точек облака'
        return report
    # Формат установлен просмотром нативного декодера. Совпадение систем
    # координат здесь лишь предположение для анализа, не калибровка профиля.
    surfaces=cloud+np.array([resolution/2,resolution/2,resolution])
    try:
        top,origin,x,y,_=_raster(surfaces,resolution)
        floor=_floor(top,x,y,pose,resolution)
        if floor is None:
            report['reason']='Не удалось выделить пол рядом с предполагаемой позой'
            return report
        report.update(points=len(cloud),floor_coefficients=floor[0].tolist(),
                      assumed_pose=[pose.x,pose.y,pose.yaw,pose.z],
                      point_frame=metadata.get('frame_id'),
                      pole_candidates=pole_candidates(surfaces,floor[0],resolution))
        short=slalom_pole_candidates(surfaces,floor[0],resolution)
        policy=Policy()
        short=exclude_body_conflicts(short,pose,policy,policy.max_localization_error)
        report['slalom_pole_candidates']=short
        report['slalom_row_candidates']=pole_rows(short)
        # Предварительный расчёт при явно неподтверждённой привязке координат.
        # Результат не передаётся исполнителю и не выдаёт разрешение движения.
        grid,_=_support_grid(top,floor[1],origin,resolution,0,policy,voxel_size=resolution)
        report['current_footprint']=footprint_report(grid,pose,policy,policy.max_localization_error)
        report['body_map_audit']=body_map_audit(grid,top,floor[1],pose,policy,policy.max_localization_error)
        report['slalom_route_previews']=[]
        for row in report['slalom_row_candidates'][:4]:
            radius=max(max(short[i]['width_xy_m'])/2 for i in row['pole_indices'])
            plan=plan_slalom(grid,row['centers_xy'],radius,pose,policy,policy.max_localization_error)
            report['slalom_route_previews'].append({'pole_indices':row['pole_indices'],**plan,
                'assumptions_verified':False,'route_verified':False})
    except ValueError as exc:
        report['reason']=str(exc)
    return report
