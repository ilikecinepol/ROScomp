"""Проверка ориентированного корпуса по карте без подмены неизвестных клеток."""
import math
from .models import Pose
from .navigation import _radius, _shape


def footprint_report(grid, pose, policy, uncertainty, extra_margin=0.):
    """Пересечение прямоугольника с квадратами карты через разделяющие оси.

    Отдельная проверка позы не разрешает поворот: для движения нужен весь swept
    путь. Внешний запас включает неопределённость локализации и clearance.
    """
    result={'clear':False,'occupied_cells':0,'unknown_cells':0,'outside_cells':0}
    try:
        _radius(policy,uncertainty)
        w,h=_shape(grid)
        if not w or not h or grid.verified is not True or not math.isfinite(grid.resolution) or grid.resolution<=0:
            return {**result,'reason':'Карта не подтверждена'}
        if not all(math.isfinite(v) for v in (pose.x,pose.y,pose.yaw,extra_margin)) or extra_margin<0:
            return {**result,'reason':'Некорректная поза или запас'}
        if any(type(v) is not int or v not in (-1,0,1) for row in grid.cells for v in row):
            return {**result,'reason':'Некорректные клетки карты'}
        margin=policy.clearance+uncertainty+extra_margin
        a,b=policy.robot_length/2+margin,policy.robot_width/2+margin
        co,si=math.cos(pose.yaw),math.sin(pose.yaw)
        hx,hy=abs(co)*a+abs(si)*b,abs(si)*a+abs(co)*b
        res=grid.resolution;r=res/2
        lo=[math.floor((v-d-o)/res)-1 for v,d,o in zip((pose.x,pose.y),(hx,hy),grid.origin)]
        hi=[math.floor((v+d-o)/res)+1 for v,d,o in zip((pose.x,pose.y),(hx,hy),grid.origin)]
        # Ограничение защищает диагностику от ошибочного масштаба карты.
        if (hi[0]-lo[0]+1)*(hi[1]-lo[1]+1)>100000:
            return {**result,'reason':'Некорректный масштаб корпуса или карты'}
        for y in range(lo[1],hi[1]+1):
            for x in range(lo[0],hi[0]+1):
                dx=grid.origin[0]+(x+.5)*res-pose.x
                dy=grid.origin[1]+(y+.5)*res-pose.y
                if abs(dx)>hx+r+1e-10 or abs(dy)>hy+r+1e-10:continue
                if abs(co*dx+si*dy)>a+r*(abs(co)+abs(si))+1e-10:continue
                if abs(-si*dx+co*dy)>b+r*(abs(co)+abs(si))+1e-10:continue
                if not 0<=x<w or not 0<=y<h:result['outside_cells']+=1
                elif grid.cells[y][x]==1:result['occupied_cells']+=1
                elif grid.cells[y][x]==-1:result['unknown_cells']+=1
        result['clear']=not any(result[k] for k in ('occupied_cells','unknown_cells','outside_cells'))
        result['reason']='Корпус помещается в наблюдаемую область' if result['clear'] else 'Корпус пересекает занятость, неизвестность или край карты'
        return result
    except (AttributeError,TypeError,ValueError,OverflowError):
        return {**result,'reason':'Некорректная геометрия'}


def swept_clear(grid, start, end, policy, uncertainty):
    """Консервативная проверка линейного перемещения и кратчайшего поворота.

    Запас между отсчётами покрывает смещение центра и дугу самого дальнего
    угла корпуса. Поэтому свободные концы не означают свободный поворот.
    """
    try:
        distance=math.hypot(end.x-start.x,end.y-start.y)
        angle=math.atan2(math.sin(end.yaw-start.yaw),math.cos(end.yaw-start.yaw))
        _radius(policy,uncertainty)
        margin0=policy.clearance+uncertainty
        radius=math.hypot(policy.robot_length/2+margin0,policy.robot_width/2+margin0)
        steps=max(1,math.ceil((distance+radius*abs(angle))/min(.025,grid.resolution/2)))
        if steps>2000:return False
        margin=(distance+radius*abs(angle))/(2*steps)
        for i in range(steps+1):
            t=i/steps
            p=Pose(start.x+(end.x-start.x)*t,start.y+(end.y-start.y)*t,start.yaw+angle*t)
            if not footprint_report(grid,p,policy,uncertainty,margin)['clear']:return False
        return True
    except (AttributeError,TypeError,ValueError,OverflowError,ZeroDivisionError):
        return False
