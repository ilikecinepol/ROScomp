"""Консервативное сопоставление проекции с областью визуального кандидата."""
import numpy as np


def curve_cost(pixels, box):
    points=np.asarray(pixels,dtype=float)
    bounds=np.asarray(box,dtype=float)
    if points.ndim!=2 or points.shape[1]!=2 or bounds.shape!=(4,):
        raise ValueError('Нужны точки Nx2 и прямоугольник xyxy')
    if not np.isfinite(points).all() or not np.isfinite(bounds).all():
        raise ValueError('Координаты должны быть конечными')
    x0,y0,x1,y1=bounds
    if x1<=x0 or y1<=y0:raise ValueError('Пустая область стойки')
    inside=(points[:,0]>=x0)&(points[:,0]<=x1)&(points[:,1]>=y0)&(points[:,1]<=y1)
    if not inside.any():return 1e6,None
    candidates=points[inside]
    center=np.array([(x0+x1)/2,(y0+y1)/2])
    delta=center-candidates
    distance=np.linalg.norm(delta,axis=1)
    best=int(np.argmin(distance))
    return float(distance[best]),delta[best].tolist()
