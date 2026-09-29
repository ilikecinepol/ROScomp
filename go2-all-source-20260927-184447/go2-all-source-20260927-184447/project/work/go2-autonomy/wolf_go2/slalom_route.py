"""Маршрут вокруг измеренных стоек; не выполняет команды движения."""
import math
import numpy as np
from .navigation import safe_mask, _path_clear
from .footprint import footprint_report
from .models import Pose


def plan_slalom(grid, centers, pole_radius, pose, policy, uncertainty, first_side=None, preferred_axis=None, entry_mode=None):
    from .perception import _route_on_support
    xy=np.asarray(centers,dtype=float)
    if xy.ndim!=2 or xy.shape[1]!=2 or not 3<=len(xy)<=12 or not np.isfinite(xy).all():
        raise ValueError('Нужны 3–12 конечных координат стоек')
    if not math.isfinite(pole_radius) or not 0<pole_radius<=.5:raise ValueError('Некорректный радиус основания')
    if first_side not in (None,'left','right'):raise ValueError('Неизвестная сторона обхода')
    if entry_mode not in (None,'center','flank','flank_near'):raise ValueError('Неизвестный вход')
    middle=xy.mean(0);_,_,vectors=np.linalg.svd(xy-middle,full_matrices=False)
    axis=vectors[0]
    reference=middle-[pose.x,pose.y] if preferred_axis is None else np.asarray(preferred_axis)
    if np.dot(axis,reference)<0:axis=-axis
    normal=np.array([-axis[1],axis[0]])
    order=np.argsort((xy-middle)@axis);xy=xy[order]
    along=(xy-middle)@axis
    if np.diff(along).min()<.55 or np.max(np.abs((xy-middle)@normal))>.3:
        return {'path':[],'reason':'Неоднозначный или слишком тесный ряд','variants':[]}
    mask=np.asarray(safe_mask(grid,policy,uncertainty),dtype=bool)
    clearance=math.hypot(policy.robot_length,policy.robot_width)/2+policy.clearance+uncertainty
    offset=clearance+pole_radius+2*grid.resolution
    yy,xx=np.indices(mask.shape)
    cell_xy=np.stack([grid.origin[0]+(xx+.5)*grid.resolution,grid.origin[1]+(yy+.5)*grid.resolution],axis=-1)
    projection=(cell_xy-middle)@axis
    variants=[]
    choices = [first_side] if first_side else ['left','right']
    modes=[entry_mode] if entry_mode else ['center','flank','flank_near']
    for side, entry_mode in ((side,mode) for side in choices for mode in modes):
        sign=1 if side=='left' else -1
        entry=middle+axis*(along[0]-offset)
        if entry_mode == 'flank': entry=entry+normal*offset*sign
        if entry_mode == 'flank_near':
            # Боковой вход остаётся перед первой стойкой, но не требует
            # свободного места на полную длину корпуса позади ряда.
            # Все опорные точки и проверки габарита остаются обязательными.
            entry=xy[0]-axis*(pole_radius+2*grid.resolution)+normal*offset*sign
        gates=[entry]
        gates.extend(point+normal*offset*sign*((-1)**i) for i,point in enumerate(xy))
        gates.append(middle+axis*(along[-1]+offset))
        path=[];reason='';failure=None
        for leg_index,(a,b) in enumerate(zip(gates,gates[1:])):
            lower,upper=sorted([float((a-middle)@axis),float((b-middle)@axis)])
            # Нельзя обойти весь ряд снаружи вместо пересечения каждого промежутка.
            corridor=mask&(projection>=lower-grid.resolution)&(projection<=upper+grid.resolution)
            leg=_route_on_support(grid,corridor,a,b)
            if leg is None or not _path_clear(grid,corridor.tolist(),leg):
                heading=math.atan2(b[1]-a[1],b[0]-a[0])
                failure={'leg_index':leg_index,'from_xy':a.tolist(),'to_xy':b.tolist(),
                         'start_footprint':footprint_report(grid,Pose(*a,heading),policy,uncertainty),
                         'end_footprint':footprint_report(grid,Pose(*b,heading),policy,uncertainty)}
                reason='Нет наблюдённого прохода для всего корпуса между соседними точками';break
            path.extend(leg if not path else leg[1:])
        if reason:path=[]
        length=sum(math.dist(a,b) for a,b in zip(path,path[1:])) if path else None
        variants.append({'first_side':side,'entry_mode':entry_mode,'path':path,'length_m':length,'reason':reason,
                         'blocked_leg':failure,
                         'alternating_waypoints_xy':[p.tolist() for p in gates[1:-1]]})
    available=[v for v in variants if v['path']]
    if not available:return {'path':[],'reason':'Все варианты входа заблокированы или опора неизвестна','variants':variants}
    # Подход отдельно проверяется навигацией; расстояние здесь лишь эвристика.
    best=min(available,key=lambda v:v['length_m']+math.dist((pose.x,pose.y),v['path'][0]))
    return {**best,'variants':variants,'axis_xy':axis.tolist(),'route_verified':False,
            'reason':'Геометрический путь рассчитан; идентификация полного ряда и калибровка проверяются отдельно'}
