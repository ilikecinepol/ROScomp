"""Кандидаты вертикальных стоек из облака; не подтверждение маршрута змейки."""
import numpy as np
from scipy import ndimage


def pole_candidates(points, floor_coefficients, resolution=.05):
    """Два высотных среза разделяют стойки, соединённые низкими объектами.

    Верхний срез даёт только затравку. Высоту подтверждаем точками всей
    колонки; основание, цвет и свободный путь остаются отдельными проверками.
    """
    result = _slice_candidates(points, floor_coefficients, resolution, .35)
    for candidate in _slice_candidates(points, floor_coefficients, resolution, .85):
        if not any(np.linalg.norm(np.array(candidate['center_xy'])-other['center_xy']) < .3 for other in result):
            result.append(candidate)
    return result


def _slice_candidates(points, floor_coefficients, resolution, minimum_height, min_top=.9, min_span=.55, min_levels=8, min_points=12):
    cloud=np.asarray(points,dtype=float)
    floor=np.asarray(floor_coefficients,dtype=float)
    if cloud.ndim!=2 or cloud.shape[1]!=3 or floor.shape!=(3,) or not np.isfinite(cloud).all() or not np.isfinite(floor).all():
        raise ValueError('Нужны конечные точки и плоскость пола')
    if not .02<=resolution<=.1:raise ValueError('Некорректный размер ячейки')
    heights=cloud[:,2]-(cloud[:,:2]@floor[:2]+floor[2])
    selected=(heights>=minimum_height-1e-8)&(heights<=1.7+1e-8)
    pts=cloud[selected];hs=heights[selected]
    if len(pts)<5:return []
    origin=pts[:,:2].min(axis=0)
    cells=np.floor((pts[:,:2]-origin)/resolution+1e-6).astype(int)
    size=cells.max(axis=0)+1
    if size.prod()>2000000:raise ValueError('Облако слишком большое')
    mask=np.zeros((size[1],size[0]),dtype=bool);mask[cells[:,1],cells[:,0]]=True
    labels,n=ndimage.label(mask,np.ones((3,3)))
    membership=labels[cells[:,1],cells[:,0]]
    result=[]
    for number in range(1,n+1):
        keep=membership==number
        part=pts[keep];h=hs[keep]
        if len(part)<5:continue
        width=np.ptp(part[:,:2],axis=0)+resolution
        if max(width)>.45:continue
        if minimum_height>.35:
            # Расширение ограничено силуэтом верхушки, а не соседним объектом.
            lo=part[:,:2].min(axis=0)-resolution
            hi=part[:,:2].max(axis=0)+resolution
            inside=((cloud[:,:2]>=lo)&(cloud[:,:2]<=hi)).all(axis=1)&(heights>=.35)&(heights<=1.7)
            part=cloud[inside];h=heights[inside]
            width=np.ptp(part[:,:2],axis=0)+resolution
        if len(part)<min_points:continue
        levels=len(np.unique(np.floor(h/resolution).astype(int)))
        if max(width)>.45 or np.ptp(h)<min_span-1e-8 or h.max()<min_top or levels<min_levels:continue
        center=np.median(part[:,:2],axis=0)
        result.append({'kind':'vertical_pole_candidate','center_xy':center.tolist(),
            'width_xy_m':width.tolist(),'observed_height_range_m':[float(h.min()),float(h.max())],
            'height_levels':levels,'metric_target':False,'verified':False,
            'reason':'Вертикальный узкий объект; цвет, основание и принадлежность змейке ещё не подтверждены'})
    return result


def slalom_pole_candidates(points,floor_coefficients,resolution=.05):
    """Короткие видимые участки стоек; низкий объект сам по себе не змейка."""
    # Высокий срез отделяет верхушки, когда низ соединён с соседними объектами.
    result=pole_candidates(points,floor_coefficients,resolution)
    for height in (.30,.55):
        for candidate in _slice_candidates(points,floor_coefficients,resolution,height,
                min_top=.55,min_span=.20,min_levels=4,min_points=8):
            if not any(np.linalg.norm(np.array(candidate['center_xy'])-other['center_xy'])<.3 for other in result):
                result.append(candidate)
    _mark_isolation(result,points,floor_coefficients,resolution)
    # Ближняя стойка может быть видна только у основания. Не снижаем
    # общий порог: низкие фрагменты принимаем лишь возле линии двух
    # независимо найденных высоких изолированных стоек.
    anchors=[p for p in result if p.get('row_eligible') and p['observed_height_range_m'][1]>=.55]
    partial=_slice_candidates(points,floor_coefficients,resolution,.10,
                             min_top=.20,min_span=.10,min_levels=3,min_points=8)
    _mark_isolation(partial,points,floor_coefficients,resolution)
    for candidate in partial:
        if not candidate.get('row_eligible'):continue
        xy=np.asarray(candidate['center_xy'])
        if any(np.linalg.norm(xy-other['center_xy'])<.3 for other in result):continue
        supported=False
        for i,a in enumerate(anchors):
            for b in anchors[:i]:
                start=np.asarray(a['center_xy']);end=np.asarray(b['center_xy'])
                axis=end-start;length=np.linalg.norm(axis)
                if not .55<=length<=1.8:continue
                axis/=length;offset=xy-start;along=float(offset@axis)
                if abs(float(offset@np.array([-axis[1],axis[0]])))<=.20 and -2*1.8<=along<=length+2*1.8:
                    supported=True
        if supported:
            candidate.update(partially_observed=True,
                reason='Наблюдён низкий отдельный фрагмент на линии двух высоких стоек; принадлежность требует проверки камеры')
            result.append(candidate)
    return result


def _mark_isolation(candidates,points,floor,resolution):
    """Узкая верхушка над длинной стеной остаётся кандидатом, но не стойкой ряда."""
    if not candidates:return
    cloud=np.asarray(points,dtype=float);floor=np.asarray(floor,dtype=float)
    height=cloud[:,2]-(cloud[:,:2]@floor[:2]+floor[2])
    part=cloud[(height>=.15)&(height<=1.7)]
    if not len(part):return
    origin=part[:,:2].min(0);ij=np.floor((part[:,:2]-origin)/resolution+1e-6).astype(int)
    size=ij.max(0)+1
    if size.prod()>2000000:raise ValueError('Облако слишком большое')
    mask=np.zeros((size[1],size[0]),bool);mask[ij[:,1],ij[:,0]]=True
    labels,_=ndimage.label(mask,np.ones((3,3)));membership=labels[ij[:,1],ij[:,0]]
    for candidate in candidates:
        nearby=np.linalg.norm(part[:,:2]-candidate['center_xy'],axis=1)<=.20
        components=np.unique(membership[nearby]);maximum=0.
        for component in components:
            maximum=max(maximum,float(np.max(np.ptp(part[membership==component,:2],axis=0)+resolution)))
        candidate['connected_extent_m']=maximum
        candidate['row_eligible']=bool(len(components) and maximum<=.75)
        if not candidate['row_eligible']:
            candidate['row_exclusion_reason']='Узкий фрагмент соединён с протяжённым объектом; не подтверждена отдельная стойка'


def exclude_body_conflicts(candidates, pose, policy, uncertainty=0.):
    """Не выдаёт объект внутри корпуса за очередную стойку маршрута.

    Сохраняет кандидата и всю карту: конфликт может означать ошибку позы,
    посторонний предмет или собственное отражение, причина здесь неизвестна.
    """
    if not np.isfinite(uncertainty) or uncertainty < 0:
        raise ValueError('Некорректная неопределённость позы')
    co, si = np.cos(pose.yaw), np.sin(pose.yaw)
    a, b = policy.robot_length/2+uncertainty, policy.robot_width/2+uncertainty
    result=[]
    for source in candidates:
        candidate=dict(source)
        dx,dy=np.asarray(candidate['center_xy'],dtype=float)-[pose.x,pose.y]
        wx,wy=np.asarray(candidate['width_xy_m'],dtype=float)/2
        if not np.isfinite([dx,dy,wx,wy]).all() or min(wx,wy)<0:
            raise ValueError('Некорректные размеры кандидата стойки')
        # SAT: две оси корпуса и две оси AABB наблюдаемого объекта.
        conflict=(abs(co*dx+si*dy)<=a+abs(co)*wx+abs(si)*wy and
                  abs(-si*dx+co*dy)<=b+abs(si)*wx+abs(co)*wy and
                  abs(dx)<=abs(co)*a+abs(si)*b+wx and
                  abs(dy)<=abs(si)*a+abs(co)*b+wy)
        if conflict:
            candidate['row_eligible']=False
            candidate['body_conflict']=True
            candidate['row_exclusion_reason']='Кандидат пересекает корпус по текущей позе; принадлежность стойкам не подтверждена'
        result.append(candidate)
    return result


def pole_rows(candidates):
    """Гипотезы рядов; не определяют полноту змейки и свободный путь."""
    if not 3<=len(candidates)<=64:return []
    xy=np.array([c['center_xy'] for c in candidates],dtype=float)
    if xy.shape!=(len(candidates),2) or not np.isfinite(xy).all():
        raise ValueError('Нужны конечные двумерные координаты стоек')
    rows={}
    for i in range(len(xy)):
        if candidates[i].get('row_eligible') is False:continue
        for j in range(i):
            if candidates[j].get('row_eligible') is False:continue
            axis=xy[i]-xy[j];length=np.linalg.norm(axis)
            if length<.6:continue
            axis/=length;normal=np.array([-axis[1],axis[0]])
            members=np.flatnonzero(np.abs((xy-xy[j])@normal)<=.30)
            members=np.array([k for k in members if candidates[k].get('row_eligible') is not False],dtype=int)
            if len(members)<3:continue
            center=xy[members].mean(0)
            _,_,basis=np.linalg.svd(xy[members]-center,full_matrices=False)
            axis=basis[0]
            ordered=members[np.argsort((xy[members]-center)@axis)]
            gaps=np.diff((xy[ordered]-center)@axis)
            residual=np.abs((xy[ordered]-center)@np.array([-axis[1],axis[0]]))
            if residual.max()>.30:continue
            # Разрыв не соединяем маршрутом: сохраняем только непрерывные группы.
            for group in np.split(ordered,np.flatnonzero((gaps<.55)|(gaps>1.8))+1):
                if len(group)<3:continue
                if any(candidates[int(k)].get('partially_observed') for k in group) and sum(
                        not candidates[int(k)].get('partially_observed',False) for k in group)<2:
                    continue
                group_gaps=np.diff((xy[group]-center)@axis)
                key=tuple(sorted(int(k) for k in group))
                rows[key]={'pole_indices':[int(k) for k in group],
                    'centers_xy':xy[group].tolist(),'spacing_m':group_gaps.tolist(),
                    'max_line_residual_m':float(residual.max()),'route_verified':False,
                    'reason':'Гипотеза группы стоек; полнота ряда, цвет и свободный коридор ещё не подтверждены'}
    maximal=[v for k,v in rows.items() if not any(set(k)<set(other) for other in rows)]
    return sorted(maximal,key=lambda r:(-len(r['pole_indices']),r['max_line_residual_m']))
